import serial
import time

# # Serial port parameters
serial_speed = 115200
serial_port = '/dev/tty.IMU-001-ESP32SPP' # bluetooth shield hc-06


if __name__ == '__main__':
    i = 0
    print ("conecting to serial port ...")
    ser = serial.Serial(serial_port, serial_speed, timeout=1)
    print ("sending start message")
    ser.write(b"\n")
    while True:
        # print("receiving message from esp32...")
        data = ser.readline()
        if (data != ""):
            print (data)
    ser.close()
