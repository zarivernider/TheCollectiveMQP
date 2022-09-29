import time
import serial.tools.list_ports as ps
import serial

ports = ps.comports()
for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
print("Select Port")
port = input()
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port=port,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

print("Sending message 8N1 115200")

input=1
while 1 :
    # get keyboard input
    # input = ("Please work for the love of heck")
    time.sleep(3)
        # Python 3 users
        # input = input(">> ")
    if input == 'exit':
        ser.close()
        exit()
    else:

        # ser.write(input.encode('utf-8'))
        value = 2452
        message = [0x20, int(value & 255), int(value >> 8)]
        print(message)
        ser.write(message)
        out = ''
        print("Message Sent")
        # let's wait one second before reading output (let's give device time to answer)
        # time.sleep(1)
        # while ser.inWaiting() > 0:
        #     out += ser.read(1)
            
        # if out != '':
        #     print(">>" + out)