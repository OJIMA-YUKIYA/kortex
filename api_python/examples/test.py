import serial

readSerial = serial.Serial("../../../serial_out",9600)

while 1:
    line = readSerial.readline()
    print(line[0])