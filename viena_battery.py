#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2018 Bruno Tibério
# Copyright (c) 2018 Diogo Neves
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import serial
import logging
import sys
import numpy as np


def crc_cal(buffer, size):
    """ Calculate crc

    Calculate crc of given buffer. The crc algorithm used is implement as described in
    `LTC6803 datasheet`_.

    .. _LTC6803 datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/680313fa.pdf

    Args:
        buffer: a list with bytes received.
    Return:
        byte: crc result.
    """
    crc = 0b01000001.to_bytes(1, 'little')  # Initial Value for CRC calc
    for j in range(0, size+1):
        aux = 0b10000000.to_bytes(1, 'little')
        for i in range(8):
            DIN = (buffer(j) & aux) >> (7 - i)

            IN0 = DIN ^ ((crc & 0b10000000) >> 7)

            IN1 = (crc & 0b00000001) ^ IN0

            IN2 = ((crc & 0b00000010) >> 1) ^ IN0

            crc = ((crc << 1) & 0b11111000) | IN2 << 2 | IN1 << 1 | IN0

            aux = aux >> 1

    return crc


def crc_check(buffer):
    """ Check crc

    Check if crc of given buffer is correct.

    Args:
        buffer: a list with bytes received.
    Return:
        bool: True if result is valid, False otherwise.
    """
    given_crc = buffer(14)
    crc_val = crc_cal(buffer, 13)
    return crc_val == given_crc


class Battery:
    _port = None
    _baudrate = None
    _connected = None
    _portObj = None
    exitFlag = None
    logger = None

    can_message = []
    buffer = np.zeros(15, dtype=np.uint8)

    def init(self, port='/dev/ttyUSB0', baudrate=460800, debug=False):
        self._port = port
        self._baudrate = baudrate
        self._connected = False
        self.exitFlag = False
        self.logger = logging.getLogger('Battery')
        if debug:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

    def logInfo(self, message=None):
        """ Log a message

        A wrap around logging with info level
        The log message will have the following structure\:
        [class name \: function name ] message
        The function name will be the caller function retrieved automatically
        by using sys._getframe(1).f_code.co_name

        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return
        self.logger.info('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def logDebug(self, message=None):
        """ Log a message

        A wrap around logging with debug level
        The log message will have the following structure\:
        [class name \: function name ] message
        The function name will be the caller function retrieved automatically
        by using sys._getframe(1).f_code.co_name

        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return

        self.logger.debug('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def begin(self):
        try:
            self._portObj = serial.Serial(port=self._port,
                                          baudrate=self._baudrate,
                                          )
            self._connected = True
        except serial.SerialException as e:
            self.logInfo(str(e))
        finally:
            return self._portObj.is_open

    def disconnect(self):
        self._portObj.close()
        self._connected = False
        return

    def read(self):
        if not self._connected:
            self.logInfo('port is not connected')
            return
        index = np.arange(1, 15)
        while not self.exitFlag:
            newByte = self._portObj.read()
            self.buffer = np.append(np.take(self.buffer, index), newByte)
            if crc_check(self.buffer):
                print(self.buffer)


def main():

    import signal
    import argparse

    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='Battery interface')
    parser.add_argument('--port', '-p', action='store', default='/dev/ttyUSB0', type=str,
                        help='serial port name', dest='port')
    parser.add_argument('--baud', action='store', default=460800, type=int,
                        help='baudrate', dest='baud')
    args = parser.parse_args()

    # ---------------------------------------------------------------------------------
    # def for signal handlers
    # ---------------------------------------------------------------------------------
    def signal_handler(signum, frame):
        if signum == signal.SIGINT:
            logging.info('Received signal INTERRUPT... exiting now')
        if signum == signal.SIGTERM:
            logging.info('Received signal TERM... exiting now')
        exitFlag = True
        return

    signal.signal(signal.SIGINT, signal_handler)
    # set up logging to file - see previous section for more details
    logging.basicConfig(level=logging.INFO,
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='battery.log',
                        filemode='w')
    # define a Handler which writes INFO messages or higher
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    battery = Battery(port=args.port, baudrate=args.baud)
    if not battery.begin():
        logging.info("Failed to connect to battery. Is port available?")
        return


if __name__ == '__main__':
    main()
