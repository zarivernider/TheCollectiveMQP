import serial
import serial.tools.list_ports
import time

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
print("Select Port")
port = input()
ser = serial.Serial(port = port, baudrate = 115200, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
print("Port: " + port)
print("Baudrate: 115200")
print("Bytesize: 8")
print("Parity: None")
print("Stopbits: 1")

ser.isOpen()

print("Enter exit to leave")

inputs=1
while 1 :
    #inputs = raw_input()
    if inputs == 'exit':
        ser.close()
        exit()
    else:
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            # T = ser.readline().decode("utf-8")
            # print(T)
            out += ser.read(1).decode("utf-8") 

        if out != '':
            print(out)

