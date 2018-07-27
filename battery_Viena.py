import numpy as np
import serial

# Initialization of structures that will be used.
msg_dict = {'ID': [None, None], 'extID': [None] * 3, 'DLC': None,
            'data': [None] * 8, 'CRC': None, 'xorCRC': None}

port = serial.Serial(
    port='/dev/ttyUSB1', baudrate=460800, parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=10)

status = 1

# Functions
def calcCRC(buffer, size=14):
    """
    calcCRC
    :param buffer:
    :return:
    """
    possibleCRC = 0b01000001  # Initial Value for CRC calc
    for j in range(0, size):
        MSB = 0b10000000
        for i in range(8):
            DIN = (buffer.tobytes()[j] & MSB) >> (7 - i)

            IN0 = DIN ^ ((possibleCRC & 0b10000000) >> 7)

            IN1 = (possibleCRC & 0b00000001) ^ IN0

            IN2 = ((possibleCRC & 0b00000010) >> 1) ^ IN0

            possibleCRC = ((possibleCRC << 1) & 0b11111000) | IN2 << 2 | IN1 << 1 | IN0

            MSB = MSB >> 1

    return possibleCRC


def checkCRC(buffer, CRC):
    """
    checkCRC
    :param buffer:
    :param CRC:
    :return:
    """
    global status, msg_dict
    if calcCRC(buffer) == int.from_bytes(CRC, 'little'):
        status += 1
        print("calcCRC deu certo")
        return 1
    else:
        return None


# TODO Make function to know if message has 16 or 13 bytes, return 0 if 13 and 1 if 16, setting size respectively.
# TODO size in line 13 to be set by this function. Values 15 from lines 78 and 80 to be replaced also be size.
def extID_use():
    """
    :return: bool value, 0 if false 1 if true
    """
    global size

    return


def writePort():
    """
    write port
    :return:
    """
    global status
    buffer = np.zeros(16, dtype=np.uint8)

    print("[+] Trying to connect to:" + port.portstr)
    if not port.is_open:
        raise ValueError("[ERROR] Port " + port.portstr + "is closed")
    print("[+] Connected")
    while status < 30:
        try:
            for pos in range(15):
                buffer[pos] = port.read(size=1)
            buffer[15] = port.read(size=1)
            writeMsg(buffer)
            if msg_dict['CRC'] == msg_dict['xorCRC']:
                print(msg_dict)
            buffer = np.roll(buffer, (0, -1))
        except KeyboardInterrupt as e:
            print('Terminated by user.\nStopping now.'.format(e))
            status = False
            break


def writeMsg(buffer):
    """
    :param buffer:
    :return:
    """

    global msg_dict
    pointer = 0
    msg_dict['ID'][0] = buffer[pointer + 1]
    msg_dict['ID'][1] = buffer[pointer]

    pointer = 2
    if extID_use() == 0:
        msg_dict.pop('extID')
    for i in range(3):
        msg_dict['extID'][i] = buffer[pointer]
        pointer += 1

    msg_dict['DLC'] = buffer[pointer]

    for i in range(8):
        msg_dict['data'][i] = buffer[pointer + i]
        pointer += 1

    msg_dict['CRC'] = calcCRC(buffer[pointer])
    msg_dict['xorCRC'] = checkCRC(buffer[pointer + 1], msg_dict['CRC'])

    # Checks if message is valid or not, no need to create a separate function
    if (msg_dict['ID'][0] > 20) or (msg_dict['ID'][1] > 47):
        raise ValueError("[ERROR] Invalid ID")

    if msg_dict['DLC'] > 8:
        raise ValueError("[ERROR] Invalid DLC")

    if extID_use() == 1:
        if (msg_dict['extID'][0] > 26) or (msg_dict['extID'][1] > 21) or (msg_dict['extID'][2] > 44):
            raise ValueError("[ERROR] Invalid extID")

    return msg_dict


writePort()
