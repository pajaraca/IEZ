import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import serial
import time

# # Serial port parameters
serial_speed = 115200
serial_port = '/dev/tty.IMU-001-ESP32SPP' # bluetooth shield hc-06

style.use('fivethirtyeight')

fig, (accel, gyro) = plt.subplots(2)

# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)
# ax1 = fig.add_subplot(1,1,1)

def animate(i):

    tss = []
    xs = []
    ys = []
    zs = []
    lines = []

    lines = ser.readlines()

    for line in lines:
        print(line)
        if len(line) > 1:
            ts, x, y, z = line.decode().split(',')
            tss.append(int(ts))
            xs.append(float(x))
            ys.append(float(y))
            zs.append(float(z))

    fig.suptitle('Acceleration:')
    axs[0].clear()
    axs[0].plot(tss, xs, color='red')
    axs[1].clear()
    axs[1].plot(tss, ys, color='blue')
    axs[2].clear()
    axs[2].plot(tss, zs, color='green')


if __name__ == '__main__':
    i = 0
    print ("connecting to serial port ...")
    ser = serial.Serial(serial_port, serial_speed, timeout=0)
    print ("sending start message")
    ser.write(b"\n")

    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.ylim([-32000, 32000])
    plt.show()

    # while True:
    #     # print("receiving message from esp32...")
    #     data = ser.readline()
    #     if (data != ""):
    #         print (data)
    # ser.close()



#     # Another way to do it without clearing the Axis
# from itertools import count
# import pandas as pd
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# plt.style.use('fivethirtyeight')

# x_vals = []
# y_vals = []

# plt.plot([], [], label='Channel 1')
# plt.plot([], [], label='Channel 2')


# def animate(i):
#     data = pd.read_csv('data.csv')
#     x = data['x_value']
#     y1 = data['total_1']
#     y2 = data['total_2']

#     ax = plt.gca()
#     line1, line2 = ax.lines

#     line1.set_data(x, y1)
#     line2.set_data(x, y2)

#     xlim_low, xlim_high = ax.get_xlim()
#     ylim_low, ylim_high = ax.get_ylim()

#     ax.set_xlim(xlim_low, (x.max() + 5))

#     y1max = y1.max()
#     y2max = y2.max()
#     current_ymax = y1max if (y1max > y2max) else y2max

#     y1min = y1.min()
#     y2min = y2.min()
#     current_ymin = y1min if (y1min < y2min) else y2min

#     ax.set_ylim((current_ymin - 5), (current_ymax + 5))


# ani = FuncAnimation(plt.gcf(), animate, interval=1000)

# plt.legend()
# plt.tight_layout()
# plt.show()