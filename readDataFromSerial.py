# -*- coding: utf-8 -*-
"""
This script reads data from the serial port, plots a graphic with values 
measured on the fly and saves information to a variable called data_list
"""

import serial
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

data_list = [['Tempo','concentração_ppm','resistência_ohm','umidade_perc',
              'temperatura_Celsius']]

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    # Read from Serial
    data = ser.readline()
    y = data.decode()
    y = [float(i) for i in str.split(y,'|')] # converts to float list
    

    # Add x and y to lists
    time = i * 1/sampleFreqHz
    xs.append(time)
    ys.append(y[0])
    
    # saves data every minute
    if (time%30 == 0):
        print('flush')
        data_list.append([dt.datetime.today(),y[0],y[1],y[2],y[3]])


    # Limit x and y lists to 20 items
    xs = xs[-100:]
    ys = ys[-100:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    # ax.set_ylim([300, 600])
    plt.xticks(rotation=90, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('MQ135 read')
    plt.ylabel('Concentração de $CO_2$/ppm')
    plt.xlabel('Tempo/s')
    plt.grid()



# main program
plt.ion()
fig=plt.figure()
ax = fig.add_subplot(1, 1, 1)

sampleFreqHz = 2.0 

i=0
xs=list()
ys=list()
i=0
# the serial port must be changed accordingly
ser = serial.Serial('/dev/ttyUSB0',9600)
ser.close()
ser.open()


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys),
                                  interval=1000*1./sampleFreqHz)
plt.show()



    