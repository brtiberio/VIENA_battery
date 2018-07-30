/**********************************************************************
 *   FST CAN tools --- interface
 *
 *   SerialPort class
 *   ______________________________________________________________
 *
 *   Copyright 2014 Bruno Santos <brunomanuelsantos@tecnico.ulisboa.pt>
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; version 2 of the
 *   License only.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **********************************************************************/


#include <QtSerialPort/QtSerialPort>
#include <QMutex>
#include <qmessagebox.h>


#include <unistd.h>
#include <fcntl.h>
// #include <sys/ioctl.h>
#include <thread>
#include <mutex>
#include <cerrno>

#include "main.hpp"
#include "SerialPort.hpp"
//#define DBG_SERIAL_RECEPTION

/**********************************************************************
 * Name:    SerialPort
 * Args:    -
 * Return:  -
 * Desc:    SerialPort class constructor.
 **********************************************************************/
SerialPort::SerialPort(void){
	_fd = -1;
	_status = 0;
	connect(&serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
        SLOT(handleError(QSerialPort::SerialPortError)));

	standard_ID_mode();
	return;
}


/**********************************************************************
 * Name:    ~SerialPort
 * Args:    -
 * Return:  -
 * Desc:    SerialPort class destructor.
 **********************************************************************/
SerialPort::~SerialPort(void){
	// clean exit for _worker in case it's still going at it
	ClosePort();
	return;
}


/**********************************************************************
 * Name:    HandleError
 * Args:    -
 * Return:  -
 * Desc:    SerialPort class destructor.
 **********************************************************************/

void SerialPort::handleError(QSerialPort::SerialPortError error)
{
	switch (error){
			case QSerialPort::NoError:
				break;
			case QSerialPort::DeviceNotFoundError:
				fprintf(stderr, "Unexpected Error, Device Not Found\n");
				break;
			case QSerialPort::PermissionError:
				fprintf(stderr, "Really Sorry, But you lack the permissions\n");
				break;
			case QSerialPort::OpenError:
				fprintf(stderr, "Unexpected Error while opening the port\n");
				break;
			case QSerialPort::NotOpenError:
				fprintf(stderr, "Unexpected Error is the device open?\n");
				break;
			case QSerialPort::WriteError:
				fprintf(stderr, "Error while writing, closing port...\n");
				ClosePort();
				break;
			case QSerialPort::ReadError:
				fprintf(stderr, "Error while reading, closing port...\n");
				ClosePort();
				break;
			case QSerialPort::ResourceError:
				fprintf(stderr, "Resource Error. Did you remove the device? closing port...\n");
				ClosePort();
				break;
			case QSerialPort::UnsupportedOperationError:
				fprintf(stderr, "Dont you ping of death me!!...\n");
				ClosePort();
				break;
			default:
				fprintf(stderr, "I cant handle this, closing port...\n");
				ClosePort();
				break;
		}
}

/**********************************************************************
 * Name:    OpenPort
 * Args:    const char *portname, BaudRate
 * Return:  exit status
 * Desc:    Opens and configures the serial port.
 *          Starts worker thread to read from the port.
 **********************************************************************/
int SerialPort::OpenPort(const char *portname, const qint32 Baud){

	//
	// Get file descriptor
	//
	int err = 0;
	// reopens port rather than creating another file descriptor every time OpenPort is issued
	if(_status != 0){
		ClosePort();
	}

	mutex.lock();
	// actual opening

	//_fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
	serial.setPortName(portname);
    serial.setBaudRate(Baud);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial.open(QIODevice::ReadWrite))
	    err = 1;


		// Launch worker
	// who would know?
	// http://stackoverflow.com/questions/13888453/stdthread-unresolved-overloaded-function-type-error
	if(!err){
		_status = 1;
		_worker = std::thread(&SerialPort::_readPort, this);
	}
	mutex.unlock();

	return err;
}


/**********************************************************************
 * Name:    ClosePort
 * Args:    -
 * Return:  exit status
 * Desc:    Closes serial port, if open. Returns -1 otherwise.
 **********************************************************************/
int SerialPort::ClosePort(void){

	int err = 0;

	mutex.lock();

	if(!_status){	// if open
		// mutex.unlock();	// sneaky bastard, but using only one return to avoid the deadlock is easier to read
		// return -1;
		err = -1;
	}else{

		_status = 0;
		_worker.join();

		serial.close();
	}

	mutex.unlock();

	return err;
}


/**********************************************************************
 * Name:    _readPort
 * Args:    -
 * Return:  -
 * Desc:    Independent thread to read from a serial port.
 **********************************************************************/
void SerialPort::_readPort(void){

	const unsigned int aux_buffer_size = 1024;
	const unsigned int buffer_size = 1024*4;

	// indexes for the buffer operations
	unsigned int W_caret = 0;
	unsigned int R_caret = 0;

	static unsigned char buffer[buffer_size];
	static char aux_buffer[aux_buffer_size];

	// thread loop
	while(_status){

		// impractical debug message for higher frequencies
		// #ifdef DBG_SERIAL_RECEPTION
		// 	fprintf(stderr, "[%s : %d] DBG_SERIAL_RECEPTION: Reading from serial port.\n", __FILE__, __LINE__);
		// #endif

		memset(aux_buffer, 0, aux_buffer_size);

		// read can't be blocking or thread may never quit before another one is issued!
		// call fcntl with FNDELAY
		errno = 0;

		int n = serial.read(aux_buffer,aux_buffer_size);

		if( ((errno == EBADF) || (errno == EINVAL) || (errno == EIO) || (errno == EISDIR) )&& 0){

			// TODO: this is not bei\ng triggered by pulling the USB cable, the main reason I did it
			#ifdef DBG_SERIAL_RECEPTION
				fprintf(stderr, "[%s : %d] DBG_SERIAL_RECEPTION: read() on serial port failed. Worker will exit.\n", __FILE__, __LINE__);
			#endif

			// the thread will be joined as soon as OpenPort() or ClosePort() is issued
			// I'm not guaranteeing that within this class for simplicity (for now at least)!
			emit aborting();
			return;
		}

		// this for concatenates every message in a circular buffer
		// this step is needed or messages that got cut in the read process get lost
		for(int i=0;i<n;i++){
			buffer[W_caret] = aux_buffer[i];
			W_caret = (W_caret+1)%buffer_size;
		}


		//
		// Message detection
		//

		// #ifdef DBG_SERIAL_RECEPTION
		// 	fprintf(stderr, "[%s : %d] DBG_SERIAL_RECEPTION: I'm not stuck!\n", __FILE__, __LINE__);
		// #endif

		unsigned int valid_messages = 0;

		if(n>0){
			while(1){

				// CRC and CRC's XOR are not accounted for in message_size
				unsigned int message_size = 0;

				// report/commend message
				if(_CRC_verify(buffer, R_caret, buffer_size, 1, buffer[(R_caret+1)%buffer_size]) == 0){
					message_size = 1;
				}
				// CAN message
				else if((_CRC_verify(buffer, R_caret, buffer_size, 11+_use_extID, buffer[(R_caret+11+_use_extID)%buffer_size]) == 0) &&
					((buffer[(R_caret+11+_use_extID)%buffer_size] ^ buffer[(R_caret+12+_use_extID)%buffer_size]) == 255)){
					message_size = 11+_use_extID;
				}

				// if it went further than the W_caret, break without changing R_caret
				if( ((W_caret >= R_caret) && (message_size+2 > (W_caret - R_caret))) || ((W_caret < R_caret) && (message_size+2 > (W_caret + buffer_size - R_caret))) ){
					//
					// single exit point for the while(1)
					//
					break;
				}


				//
				// Message parsing
				//

				// else if(message_size == 1){
					// TODO: general messages
				// }


				// CAN message         | toggle option |
				// FORMAT: [ 2 byte ID | 3 byte extID  | 1 byte DLC | 8 bytes of data | CRC | CRC^255 ]
				// From here it's only needed to check the correctness of the CAN message: if ID, DLC, etc. are within range.
				// If invalid, discard the first byte and try again.

				// size should vary regarding the use or not of the extID, see extended_ID_mode() and standard_ID_mode()
				else if(message_size == 11+_use_extID){

					unsigned int CAN_aux_caret = R_caret;

					// temporary CAN data
					unsigned int ID = (buffer[CAN_aux_caret]<<8) | buffer[(CAN_aux_caret+1)%buffer_size];
					unsigned int extID = 0;
					unsigned int DLC = 0;
					unsigned char data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
					bool use_extID = false;

					if(ID > 2047){					// if valid ID
						#ifdef DBG_SERIAL_RECEPTION
							fprintf(stdout, "[%s : %d] DBG_SERIAL_RECEPTION: Invalid message discarded.\n", __FILE__, __LINE__);
						#endif
						R_caret = (R_caret+1)%buffer_size;
						continue;
					}
					CAN_aux_caret = (CAN_aux_caret+2)%buffer_size;

					if(_use_extID!=0){			// if using extended ID
						extID = (buffer[CAN_aux_caret]<<16) + (buffer[(CAN_aux_caret+1)%buffer_size]<<8) + buffer[(CAN_aux_caret+2)%buffer_size];
						if(extID >= 262144){	// invalid extID
							#ifdef DBG_SERIAL_RECEPTION
								fprintf(stdout, "[%s : %d] DBG_SERIAL_RECEPTION: Invalid message discarded.\n", __FILE__, __LINE__);
							#endif
							R_caret = (R_caret+1)%buffer_size;
							continue;
						}
						use_extID = true;
						CAN_aux_caret = (CAN_aux_caret+3)%buffer_size;
					}

					DLC = buffer[CAN_aux_caret];
					if(DLC > 8){
						#ifdef DBG_SERIAL_RECEPTION
							fprintf(stdout, "[%s : %d] DBG_SERIAL_RECEPTION: Invalid message discarded.\n", __FILE__, __LINE__);
						#endif
						R_caret = (R_caret+1)%buffer_size;
						continue;
					}

					CAN_aux_caret = (CAN_aux_caret+1)%buffer_size;
					unsigned int i;
					for(i=0;i<8;i++){
						data[i] = buffer[CAN_aux_caret];
						CAN_aux_caret = (CAN_aux_caret+1)%buffer_size;
					}

					// valid CAN message!

					CANmessage msg;
					msg.set_message(ID, extID, DLC, data, use_extID);

					// the queue has to be pre-emptively cleaned or it will only grow in size
					// there's no need solving this as there would be no point in not doing so anyway
					_queueLkc.lock();
					_queue.push(msg);
					_queueLkc.unlock();

					R_caret = (R_caret+message_size+2)%buffer_size;		// index for the byte after the CRC
					valid_messages++;
				}

				// Invalid message size
				// Discard only one byte: there may be valid messages with a garbage header left by corrupted messages or desynchronization.
				else{
					#ifdef DBG_SERIAL_RECEPTION
						fprintf(stdout, "[%s : %d] DBG_SERIAL_RECEPTION: Invalid message discarded.\n", __FILE__, __LINE__);
					#endif

					R_caret = (R_caret+1)%buffer_size;
					continue;
				}
			}
		}

		if(valid_messages>0){	// if read was not 0 length
			emit new_messages(valid_messages);
		}

		// Baudrate = 460800 bit/s
		//          = 51200 byte/s		// count stop bits, etc.
		// Reading 1024 bytes at a time
		// Need >51200/1024 reads/s  ~<50 reads/s => wait <~0.02 s
		std::this_thread::sleep_for(std::chrono::milliseconds{5});
	}
	return;
}


/**********************************************************************
 * Name:    SendMessage
 * Args:    CANmessage msg
 * Return:  exit status
 * Desc:    Send CAN message to UART.
 **********************************************************************/
int SerialPort::SendCAN(CANmessage msg){

	unsigned char raw_msg[20];	// any CAN message will have a maximum of 16 bytes with extended ID and CRC included

	raw_msg[0] = msg.ID()>>8;
	raw_msg[1] = msg.ID();

	int i=2;

	if(_use_extID>0){
		for(int j=2;j>=0;j--){
			raw_msg[i] = msg.extID()>>(8*j);
			i++;
		}
	}

	raw_msg[i] = msg.DLC();
	i++;

	// full length (with CRC and CRC xor 255)
	unsigned int n = 13 + _use_extID;

	const unsigned char *data = msg.data();

	for(unsigned int j=0;j<8;j++){
		raw_msg[i] = data[j];
		i++;
	}

	raw_msg[i] = _CRC_calculate(raw_msg, 0, 20, n-2);
	raw_msg[i+1] = raw_msg[i] ^ 255;

	return _writePort(raw_msg, n);
}


/**********************************************************************
 * Name:    writePort
 * Args:    const unsigned char *data, unsigned int n
 * Return:  exit status
 * Desc:    Send byte sequence to UART.
 **********************************************************************/
int SerialPort::_writePort(const unsigned char *data, unsigned int n){

	if(!_status){
		return -1;
	}

	errno = 0;
	if(serial.write((char *)data, (qint64)n)!=n){
		return -1;
	}
	if( (errno == EBADF) || (errno == EINVAL) || (errno == EIO) || (errno == EINTR) || (errno == ENOSPC) ){

		#ifdef DBG_SERIAL_SEND
			fprintf(stdout, "[%s : %d] DBG_SERIAL_SEND: write() on serial port failed.\n", __FILE__, __LINE__);
		#endif

		return -1;
	}

	return 0;
}


/**********************************************************************
 * Name:    extended_ID_mode
 * Args:    -
 * Return:  -
 * Desc:    Change mode to use extended IDs.
 *          Value indicates the amount of bytes to parse more.
 **********************************************************************/
void SerialPort::extended_ID_mode(void){
	_use_extID = 3;		// the number of bytes of the extID
}


/**********************************************************************
 * Name:    standard_ID_mode
 * Args:    -
 * Return:  -
 * Desc:    Change mode to use the standard ID only.
 **********************************************************************/
void SerialPort::standard_ID_mode(void){
	_use_extID = 0;
}


/**********************************************************************
 * Name:    pop_message
 * Args:    -
 * Return:  -
 * Desc:    Returns and pops from internal queue one CAN message
 **********************************************************************/
std::pair<int, CANmessage> SerialPort::pop_message(void){

	std::pair<int, CANmessage> package;

	package.first = 0;

	_queueLkc.lock();
	if(!_queue.empty()){
		package.second = _queue.front();
		_queue.pop();
	}else{
		package.first = -1;
	}
	_queueLkc.unlock();

	return package;
}


/**********************************************************************
 * Name:    _CRC_calculate
 * Args:    const unsigned char *data, const unsigned int index,
 *                      const unsigned int buffersize, unsigned int n
 * Return:  CRC byte
 * Desc:    Calculates CRC for n byte data in a circular buffer from
 *          index onwards.
 **********************************************************************/
unsigned char SerialPort::_CRC_calculate(const unsigned char *data, const unsigned int index, const unsigned int buffersize, unsigned int n){

	unsigned int i, j;

	unsigned char CRC = 0b01000001;
	unsigned char DIN, IN0, IN1, IN2;

	unsigned char aux;

	for(j=0;j<n;j++){
		aux = 0b10000000;
		for(i=0;i<8;i++){		/* shift the whole byte */

			DIN = (data[(index+j)%buffersize] & aux) >> (7-i);	/* isolates the ith MSB from byte on LSB position */

			IN0 = DIN ^ ((CRC & 0b10000000)>>7);	/* DIN XOR CRC[7] */
			IN1 = (CRC & 0b00000001) ^ IN0;			/* CRC[0] XOR IN0 */
			IN2 = ((CRC & 0b00000010)>>1) ^ IN0;	/* CRC[1] XOR IN0 */

			CRC = ((CRC << 1) & 0b11111000) | IN2<<2 | IN1<<1 | IN0;	/* shifts CRC 1 bit left and assigns 3 LSBs to IN2,IN1,IN0 */

			aux = aux >> 1;		/* shifts one bit from byte */
		}
	}
	return CRC;
}


/**********************************************************************
 * Name:    _CRC_verify
 * Args:    const unsigned char *data, const unsigned int index,
 *             const unsigned int buffersize, unsigned int n,
 *             const unsigned char CRC
 * Return:  exit status
 * Desc:    Verifies if CRC is correct for n byte data in a circular
 *          buffer from index onwards.
 **********************************************************************/
int SerialPort::_CRC_verify(const unsigned char *data, const unsigned int index, const unsigned int buffersize, unsigned int n, const unsigned char CRC){

	if(_CRC_calculate(data, index, buffersize, n) == CRC){
		return 0;
	}
	return -1;
}

void SerialPort::set_message_to_queue(CANmessage msg){

    _queueLkc.lock();
    _queue.push(msg);
    _queueLkc.unlock();
    emit new_messages(1);
}
