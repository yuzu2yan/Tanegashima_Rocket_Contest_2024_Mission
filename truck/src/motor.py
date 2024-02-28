import pigpio
import time

DEVICE_ADDRESS = 0x20
REG_IODIRA = 0x00 # GPA I/O direction register
REG_IODIRB = 0x01 # GPB I/O direction register
REG_OLATA = 0x14 # GPA output latch register
REG_OLATB = 0x15 # GPB output latch register

# pigpio library : https://abyz.me.uk/rpi/pigpio/python.html
# 6,7 上昇機構のモーター
# 4,5 つかむ機構のモーター
sample_switch = [9, 11]
panto_switch = [17, 27]
sepa_switch = [10, 22]

class Motor(object):
    def __init__(self):
        Motor.pi = pigpio.pi()
        for i in range(2):
            Motor.pi.set_mode(sepa_switch[i], pigpio.INPUT)
            Motor.pi.set_mode(panto_switch[i], pigpio.INPUT)
            Motor.pi.set_mode(sample_switch[i], pigpio.INPUT)
        
        Motor._device = Motor.pi.i2c_open(1, DEVICE_ADDRESS) 
        # SDA = 2, SCL = 3 => channel = 1
        Motor.pi.i2c_write_byte_data(Motor._device, REG_IODIRA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_IODIRB, 0x00)
        # # stop
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x00)
    
    def forward(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00000010)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00011001)                                                                              
        print("forward")
        
    def back(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00000001)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00100110)                                                                              
        print("back")
        
    def detect(self):
        print(Motor.pi.read(17))
        print(Motor.pi.read(27))
        print(Motor.pi.read(22))
        print(Motor.pi.read(10))
        print(Motor.pi.read(9))
        print(Motor.pi.read(11))
        
    def turn(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00000010)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00101010) 
        print("turn")
        
    
    def stop(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x00)
        print("stop")
        
    def stuck(self):
        Motor.back(self)
        time.sleep(3)
        Motor.turn_right(self)
        time.sleep(1)
        Motor.forward(self)
        time.sleep(3)
        Motor.stop(self)
        print('Finish stuck processing')
        
    def sepa_mecha(self):
        # Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        # Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x08)
        print("Separation mechanism activated")
        
    def attach_para(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x08)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x00)
        

if __name__ == '__main__':
    drive = Motor()
    while True:
        c = input('Enter char : ')
        if c == 'w':
            drive.forward()
        elif c == 'd':
            drive.detect()
        elif c == 't':
            drive.turn()
        elif c == 'b':
            drive.back()
        elif c == 'q':
            drive.stop()
        elif c == 'r':
            drive.rising_arm()
        elif c == 'o':
            drive.down_arm()
        # elif c == 'para':
        #     drive.attach_para()
        elif c == 'z':
            break
        else:
            print('Invalid input')
