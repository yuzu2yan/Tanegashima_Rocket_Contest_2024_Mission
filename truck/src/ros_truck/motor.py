import pigpio
import time

DEVICE_ADDRESS = 0x20
REG_IODIRA = 0x00 # GPA I/O direction register
REG_IODIRB = 0x01 # GPB I/O direction register
REG_OLATA = 0x14 # GPA output latch register
REG_OLATB = 0x15 # GPB output latch register

# pigpio library : https://abyz.me.uk/rpi/pigpio/python.html
rised_switch = [10, 11] # forward, rear only 9
downed_switch = [9, 13] # forward, rear only 10

class Motor(object):
    def __init__(self):
        Motor.pi = pigpio.pi()
        
        Motor._device = Motor.pi.i2c_open(1, DEVICE_ADDRESS) 
        # SDA = 2, SCL = 3 => channel = 1
        Motor.pi.i2c_write_byte_data(Motor._device, REG_IODIRA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_IODIRB, 0x00)
        # # stop
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x00)
    
    def forward(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b10000001)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00001001)                                                                              
        print("forward")
        
    def back(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b01000010)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00000110)                                                                              
        print("back")
        
    def detect(self):
        print(Motor.pi.read(22))
        print(Motor.pi.read(27))
        print(Motor.pi.read(10))
        print(Motor.pi.read(9))
        print(Motor.pi.read(11))
        print(Motor.pi.read(13))
        print(Motor.pi.read(19))
        print(Motor.pi.read(26))
        
    def turn_left(self): 
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b01000001)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00001010)                                                                             
        print("turn left")
        
    def turn_right(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b10000010)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00000101)                                                                             
        print("turn right")
    
    def stop(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x00)
        print("stop") 
        
    def rising(self):
        # Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00101000)
        # while Motor.pi.read(rised_switch[0]) == 0 or Motor.pi.read(rised_switch[1]) == 0:
            # if Motor.pi.read(rised_switch[0]) == 1:
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00100000)
            # if Motor.pi.read(rised_switch[1]) == 1:
        # Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00001000) 
        # time.sleep(2)       
        # Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        
    def descending(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00010100)
        while Motor.pi.read(downed_switch[0]) == 0 or Motor.pi.read(downed_switch[1]) == 0:
            if Motor.pi.read(downed_switch[0]) == 1:
                Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00100000)
        time.sleep(2)        
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        
    def attach_para_forward(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00100000)
    
    def attach_para_rear(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00001000) 
        

if __name__ == '__main__':
    drive = Motor()
    while True:
        c = input('Enter char : ')
        if c == 'w':
            drive.forward()
        elif c == 'd':
            drive.detect()
        elif c == 'ri':
            drive.turn_right()
        elif c == 'le':
            drive.turn_left()
        elif c == 'b':
            drive.back()
        elif c == 'q':
            drive.stop()
        elif c == 'r':
            drive.rising()
        elif c == 'o':
            drive.descending()
        elif c == 'af':
            drive.attach_para_forward()
        elif c == 'ar':
            drive.attach_para_rear()
        elif c == 'z':
            break
        else:
            print('Invalid input')
