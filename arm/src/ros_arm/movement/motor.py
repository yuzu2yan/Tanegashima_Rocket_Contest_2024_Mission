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
sample_switch = [19, 26]
panto_switch = [9, 10] # down, rise
sepa_switch = [13, 11] # sepa down

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
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00000110)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00000110)                                                                              
        print("forward")
        
    def back(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00001001)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00001001)                                                                              
        print("back")
        
    def turn_right(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00001010)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00000101) 
        print("turn right")
    
    def turn_left(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00000101)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0b00001010) 
        print("turn left")
        
    def detect(self):
        print(Motor.pi.read(11)) # close
        print(Motor.pi.read(13)) # sepa
        print(Motor.pi.read(10)) # down 
        print(Motor.pi.read(9)) # rising
        print(Motor.pi.read(19)) # sample
        print(Motor.pi.read(26)) # sample
        
    def arm_sep(self):
        # while Motor.pi.read(sepa_switch[0]) == 0:
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00010000)
        time.sleep(2)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        print("separated")
        
    def grabing(self):
        while Motor.pi.read(sample_switch[0]) == 0 and Motor.pi.read(sample_switch[1]) == 0:
            if Motor.pi.read(sepa_switch[1]) == 1:
                print("no hit")
                break
            Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b00100000)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        print("grabing sampling")

    def rising_arm(self):
        while Motor.pi.read(panto_switch[0]) == 0:
            Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b10000000)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        print("rised arm")
    
    def down_arm(self):
        while Motor.pi.read(panto_switch[1]) == 0:
            Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0b01000000)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        print("downed arm")
    
    def stop(self):
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATA, 0x00)
        Motor.pi.i2c_write_byte_data(Motor._device, REG_OLATB, 0x00)
        print("stop")
        

if __name__ == '__main__':
    drive = Motor()
    while True:
        c = input('Enter char : ')
        if c == 'w':
            drive.forward()
        elif c == 'b':
            drive.back()
        elif c == 'ri':
            drive.turn_right()
        elif c == 'le':
            drive.turn_left()
        elif c == 'd':
            drive.detect()
        elif c == 's':
            drive.arm_sep()
        elif c == 'g':
            drive.grabing()
        elif c == 'q':
            drive.stop()
        elif c == 'r':
            drive.rising_arm()
        elif c == 'o':
            drive.down_arm()
        elif c == 'z':
            break
        else:
            print('Invalid input')
