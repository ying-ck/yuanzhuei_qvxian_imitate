import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

de = 0

def plot_cone(radius, height, plane_pos, plane_angle,l):
    global ax1
    global ax2
    fig = plt.figure()
    
    # 3D
    ax1 = fig.add_subplot(121, projection='3d')

    u = np.linspace(0, 2 * np.pi, l)
    v = np.linspace(0, 1, l)
    u, v = np.meshgrid(u, v)

    x = radius * v * np.cos(u)
    y = radius * v * np.sin(u)
    z = height * (1 - v)

    ax1.plot_surface(x, y, z)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    plane_length = 1.5 * radius
    plane_width = 1.5 * radius
    plane_height = 0.1

    plane_x = np.linspace(plane_pos[0] - plane_length, plane_pos[0] + plane_length, l)
    plane_y = np.linspace(plane_pos[1] - plane_width, plane_pos[1] + plane_width, l)
    plane_x, plane_y = np.meshgrid(plane_x, plane_y)
    plane_z = plane_pos[2] + (plane_x - plane_pos[0]) * np.tan(plane_angle[0]) + (plane_y - plane_pos[1]) * np.tan(plane_angle[1])

    ax1.plot_surface(plane_x, plane_y, plane_z, alpha=0.5)

    mask = plane_z >0 

    plane_x = plane_x[mask]
    plane_y = plane_y[mask]
    plane_z = plane_z[mask]
    plane_x = np.reshape(plane_x,(len(plane_x),1))
    plane_y = np.reshape(plane_y,(len(plane_y),1))
    plane_z = np.reshape(plane_z,(len(plane_z),1))
    ax1.set_xlim([plane_pos[0] - plane_length,plane_pos[0] + plane_length])
    ax1.set_ylim([plane_pos[1] - plane_width,plane_pos[1] + plane_width])
    ax1.set_zlim([0,height*1.3])
    # 2D
    ax2 = fig.add_subplot(122)

    intersection_x = []
    intersection_y = []
    for i in range(len(plane_x)):
        for j in range(len(plane_x[0])):
            if plane_z[i, j] <= height * (1 - np.sqrt(plane_x[i, j]**2 + plane_y[i, j]**2) / radius):
                intersection_x.append(plane_x[i, j]/np.cos(plane_angle[0]))
                intersection_y.append(plane_y[i, j]/np.cos(plane_angle[1]))

    ax2.scatter(intersection_x, intersection_y, c='b')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.legend()
    ax2.set_xlim([(plane_pos[0] - plane_length)*1.3, (plane_pos[0] + plane_length)*1.3])
    ax2.set_ylim([(plane_pos[1] - plane_width)*1.3, (plane_pos[1] + plane_width)*1.3])
    ax2.set_aspect(1)
    
    plt.show()

def update(radius, height, plane_pos, plane_angle,l):
    global ax1
    global ax2

    # 3D
    u = np.linspace(0, 2 * np.pi, l)
    v = np.linspace(0, 1, l)
    u, v = np.meshgrid(u, v)

    x = radius * v * np.cos(u)
    y = radius * v * np.sin(u)
    z = height * (1 - v)

    ax1.clear()
    ax1.plot_surface(x, y, z)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    plane_length = 1.5 * radius
    plane_width = 1.5 * radius
    plane_height = 0.1

    plane_x = np.linspace(plane_pos[0] - plane_length, plane_pos[0] + plane_length, l)
    plane_y = np.linspace(plane_pos[1] - plane_width, plane_pos[1] + plane_width, l)
    plane_x, plane_y = np.meshgrid(plane_x, plane_y)
    plane_z = plane_pos[2] + (plane_x - plane_pos[0]) * np.tan(plane_angle[0]) + (plane_y - plane_pos[1]) * np.tan(plane_angle[1])

    ax1.plot_surface(plane_x, plane_y, plane_z, alpha=0.5)

    mask = plane_z >0 

    plane_x = plane_x[mask]
    plane_y = plane_y[mask]
    plane_z = plane_z[mask]
    plane_x = np.reshape(plane_x,(len(plane_x),1))
    plane_y = np.reshape(plane_y,(len(plane_y),1))
    plane_z = np.reshape(plane_z,(len(plane_z),1))

    ax1.set_xlim([plane_pos[0] - plane_length,plane_pos[0] + plane_length])
    ax1.set_ylim([plane_pos[1] - plane_width,plane_pos[1] + plane_width])
    ax1.set_zlim([0,height*1.3])
    # 2D
    intersection_x = []
    intersection_y = []
    for i in range(len(plane_x)):
        for j in range(len(plane_x[0])):
            if plane_z[i, j] <= height * (1 - np.sqrt(plane_x[i, j]**2 + plane_y[i, j]**2) / radius):
                intersection_x.append(plane_x[i, j]/np.cos(plane_angle[0]))
                intersection_y.append(plane_y[i, j]/np.cos(plane_angle[1]))

    ax2.clear()
    ax2.scatter(intersection_x, intersection_y, c='b')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_xlim([(plane_pos[0] - plane_length)*1.3, (plane_pos[0] + plane_length)*1.3])
    ax2.set_ylim([(plane_pos[1] - plane_width)*1.3, (plane_pos[1] + plane_width)*1.3])
    ax2.set_aspect(1)

    plt.show()

def deread(ser):
    global de
    while True:
        com_input = ser.read(10)
        if com_input:
            de = str(com_input, encoding='utf-8').split('$')[1]

def drw(t):
    while True:
        ra = float(int(de)/512*np.pi)
        update(r, h, zb,[0,ra],400)
        time.sleep(t)

def ck():
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])
    cho = int(input('请输入串口号:'))
    ser = serial.Serial("COM"+str(cho), 9600)
    if ser.isOpen():
        print("打开串口 "+ser.name,"成功。")
        return ser
    else:
        print("打开串口失败。")

def tex():
    while True:
        #ser.write(input('?:').encode('utf-8'))
        de = int(input('请输入度数：'))

#ser = ck()

h = 2
r = 1
zb = [0,0,0.3]
time.sleep(1)
#threading._start_new_thread(deread,(ser,))
threading._start_new_thread(plot_cone,(r, h, zb,[0,0],400))
time.sleep(1.5)
threading._start_new_thread(drw,(0,))

while True:
    de = int(input('请输入度数：'))

#ser.close()
