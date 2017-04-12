import serial;
import smbus;
import time;
from struct import *;

class Jy901Serial:
    def __init__(self, port, baudrate):
        self.reader = serial.Serial(port, baudrate, timeout = 1);
    def __del__(self):
        self.reader.close(); 
    def __read(cmd):
        while True:
            data = self.reader.read(11);
            k = 0;
            for b in data:
                if b == b'\x55':
                    break;
                k = k + 1;
            if k != 0:
                self.reader.read(k);
                continue;
            if data[1] != cmd:
                continue;
            else:
                return data;
    def getAngle(self):
        data = self.__read(b'\x53');
        return struct.unpack('h', data[6] + data[7])[0] / 32768.0 * 2000 * math.pi / 180;
    def getAngularVelocity(self):
        data = self.__read(b'\x52');
        return struct.unpack('h', data[6] + data[7])[0] / 32768.0 * 2000 * math.pi / 180;

class Jy901I2c:
    def __init__(self, address):
        self.bus = smbus.SMBus(1);
        self.address = address;
    def __del__(self):
        self.bus.close();
    def getAngle(self):
        data = bus.read_i2c_block_data(address, 0x3f);
        return unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 180.0 * math.pi / 180;
    def getAngularVelocity(self):
        data = bus.read_i2c_block_data(address, 0x39);
        return unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 2000 * math.pi / 180;


if __name__ == '__main__':
    #s = serial.Serial(port = '/dev/ttyUSB0', baudrate = 115200, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE);
    s = serial.Serial(port = '/dev/ttyUSB0', baudrate = 115200, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE);
    data = s.read(11);
    print(binascii.b2a_hex(data));
    k = 0;
    for d in data:
        if d == b'\x55':
            print(k);
            break;
        k = k + 1;
    print(k);
    if k != 0:
        data = s.read(k);
        print(binascii.b2a_hex(data));
        #data = binascii.b2a_hex(data);
        #print(data);
    while True:
        data = s.read(11);
        print(binascii.b2a_hex(data));
        break;
    s.close();
