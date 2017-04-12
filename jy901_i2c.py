import smbus;
from struct import *;
# sudo su -
# echo -n Y > /sys/module/i2c_bcm2708/parameters/combined
# exit
bus = smbus.SMBus(1);

address = 0x50;
data = bus.read_i2c_block_data(address, 0x3d);
x = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 180.0;
data =  bus.read_i2c_block_data(address, 0x3e);
y = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 180.0;
data = bus.read_i2c_block_data(address, 0x3f);
z = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 180.0;

data = bus.read_i2c_block_data(address, 0x37);
wx = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 2000;
data = bus.read_i2c_block_data(address, 0x38);
wy = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 2000;
data = bus.read_i2c_block_data(address, 0x39);
wz = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 2000;

data = bus.read_i2c_block_data(address, 0x34);
ax = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 16;
data = bus.read_i2c_block_data(address, 0x35);
ay = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 16;
data = bus.read_i2c_block_data(address, 0x36);
az = unpack("h", chr(data[0]) + chr(data[1]))[0] / 32768.0 * 16;

print("%f\t%f\t%f" % (x, y, z));
print("%f\t%f\t%f" % (wx, wy, wz));
print("%f\t%f\t%f" % (ax, ay, az));
