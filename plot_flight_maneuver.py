#!/usr/bin/env python

# Author: Roman Bapst
# Date: 02.01.2015
# Description: This script can be used to visualize the attitude and relative
# motion of the plane from px4 flight logs

from __future__ import division
import numpy as np
from math import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import thread,sys,time
import sdlog2_dump

__author__ = "Roman Bapst"

class FlightData(object):
    def __init__(self,file_name):
        self.log_file_name = file_name
        self.csv_file_name = file_name.split('.')[0] + ".csv"
        self.header_list = []
        self.time = []
        self.x = []
        self.y = []
        self.z = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.read_data(self.csv_file_name)

        # change look of the plane here
        # VTOL orientation for RPY=[0,0,0] is nose up (+Z), dorsal fin in -X direction
        # (normal FW orientation pitched up by pi/2
        self.x_coord = np.array([0,     0,    0,   0,   0, 0, -0.2, -0.2,  0])
        self.y_coord = np.array([0,   0.5, -0.5,   0,   0, 0,    0,    0,  0])
        self.z_coord = np.array([0.5,-0.5, -0.5, 0.5, 0.7, 0, -0.1, -0.2, -0.2])

        self.INDEX = np.arange(0,len(self.time),10)
        self.sim_len = len(self.INDEX)
        self.offset = 0
        self.ref_i = 0
        self.frame = 0

    def read_data(self,filename):
        f = open(filename,'r')
        # read message header
        self.header_list = ((f.readline()).rstrip('\n')).split(',')
        header_dic = {}
        for index,item in enumerate(self.header_list):
            header_dic[item] = index
        # read data
        for line in f.readlines():
            data = line.rstrip('\n').split(',')
            self.time.append(float(data[0]))
            self.x.append(float(data[header_dic["LPOS_X"]]))
            self.y.append(float(data[header_dic["LPOS_Y"]]))
            self.z.append(float(data[header_dic["LPOS_Z"]]))
            gamma = float(data[header_dic["ATT_Roll"]])
            beta = float(data[header_dic["ATT_Pitch"]])
            alpha = float(data[header_dic["ATT_Yaw"]])
            self.roll.append(gamma)
            self.pitch.append(beta)
            self.yaw.append(alpha)
            tw = float(data[header_dic["ATT_qw"]])
            tx = float(data[header_dic["ATT_qx"]])
            ty = float(data[header_dic["ATT_qy"]])
            tz = float(data[header_dic["ATT_qz"]])
            if (tw == 0 and tx == 0 and ty == 0 and tz == 0):
                q = self.rpy_to_quat(gamma, beta, alpha)
                self.qw.append(q[0])
                self.qx.append(q[1])
                self.qy.append(q[2])
                self.qz.append(q[3])
                #print(data[header_dic["ATT_Roll"]], data[header_dic["ATT_Pitch"]], data[header_dic["ATT_Yaw"]] )
                #print(q)
            else:
                self.qw.append(tw)
                self.qx.append(tx)
                self.qy.append(ty)
                self.qz.append(tz)
                #print(data[header_dic["ATT_qw"]], data[header_dic["ATT_qx"]], data[header_dic["ATT_qy"]], data[header_dic["ATT_qz"]] )

    def quat_to_rot(self,q):
        #compute rotation matrix from quaternion
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]

        R = np.zeros((3,3))

        R[0,0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3
        R[0,1] = 2 * q1 * q2 + 2 * q0 * q3
        R[0,2] = 2 * q1 * q3 - 2 * q0 * q2
        R[1,0] = 2 * q1 * q2 - 2 * q0 * q3
        R[1,1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3
        R[1,2] = 2 * q2 * q3 + 2 * q0 * q1
        R[2,0] = 2 * q1 * q3 + 2 * q0 * q2
        R[2,1] = 2 * q2 * q3 - 2 * q0 * q1
        R[2,2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        return R

    def rot_to_quat(self, R):

        q = []
        q.append(0.5 * sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]))
        q.append(0.5 * sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]))
        q.append(0.5 * sqrt(1.0 - R[0][0] + R[1][1] - R[2][2]))
        q.append(0.5 * sqrt(1.0 - R[0][0] - R[1][1] + R[2][2]))

        return q

    def rpy_to_quat(self,roll,pitch,yaw):
        # compute quaternion from XYZ fixed Euler angles
        # RPY = gamma, beta, alpha
        #     = phi, theta, psi
        cg = cos(roll/2)
        sg = sin(roll/2)
        cb = cos(pitch/2)
        sb = sin(pitch/2)
        ca = cos(yaw/2)
        sa = sin(yaw/2)

        q = []
        q.append(cg * cb * ca + sg * sb * sa)
        q.append(sg * cb * ca - cg * sb * sa)
        q.append(cg * sb * ca + sg * cb * sa)
        q.append(cg * cb * sa - sg * sb * ca)

        return q

    def rpy_to_rot(self,roll,pitch,yaw):

        cg = cos(roll)
        sg = sin(roll)
        cb = cos(pitch)
        sb = sin(pitch)
        ca = cos(yaw)
        sa = sin(yaw)

        R = np.zeros((3,3))

        R[0,0] = ca * cb
        R[0,1] = sa * cb
        R[0,2] = -sb
        R[1,0] = ca * sb * sg - sa * cg
        R[1,1] = sa * sb * sg + ca * cg
        R[1,2] = cb * sg
        R[2,0] = ca * sb * cg + sa * sg
        R[2,1] = sa * sb * cg - ca * sg
        R[2,2] = cb * cg

        return R

    def animate(self,i):
        self.frame = self.frame + 1
        if self.frame >= self.INDEX[-2]:
            self.frame = 0
        ax.clear()
        ax.set_xlim3d([self.x[self.INDEX[self.frame]]-1.0, 1.0+self.x[self.INDEX[self.frame]]])
        ax.set_xlabel('X')
        ax.set_ylim3d([self.y[self.INDEX[self.frame]]-1.0, 1.0+self.y[self.INDEX[self.frame]]])
        ax.set_ylabel('Y')
        ax.set_zlim3d([-self.z[self.INDEX[self.frame]]-1.0, - self.z[self.INDEX[self.frame]]+1.0])
        x = []
        y = []
        z = []
        line = ax.plot(self.x_coord, self.y_coord, self.z_coord)[0]

        if (1):
            q = self.rpy_to_quat(pi/32, 0, 0)
            # q = [self.qw[self.INDEX[self.frame]],self.qx[self.INDEX[self.frame]],self.qy[self.INDEX[self.frame]],self.qz[self.INDEX[self.frame]]]
            R = self.quat_to_rot(q)
        else:
            roll = self.roll[self.INDEX[self.frame]]
            pitch = self.pitch[self.INDEX[self.frame]]
            yaw = self.yaw[self.INDEX[self.frame]]
            roll = pi/8
            pitch = 0
            yaw = 0
            R = self.rpy_to_rot(roll,pitch,yaw)

        for index,item in enumerate(self.x_coord):
            # vec = np.dot(R,[self.x_coord[index],self.y_coord[index],self.z_coord[index]])
            # x.append(vec[0])
            # y.append(vec[1])
            # z.append(vec[2])
            vec = np.dot(R,[self.x_coord[index],self.y_coord[index],self.z_coord[index]])
            x.append(vec[0] + self.x[self.INDEX[self.frame]])
            y.append(vec[1] + self.y[self.INDEX[self.frame]])
            z.append(vec[2] - self.z[self.INDEX[self.frame]])

        line.set_data(np.array(x), np.array(y))
        line.set_3d_properties(np.array(z))

        #print(self.time[self.INDEX[self.frame]])
        return line

    def user_input(self):
        print "type 'help' for help"
        while True:
            # get user input from console
            user_input = raw_input()
            command = user_input.split(' ')
            if user_input == 'help':
                self.print_help()
            elif user_input == 'time':
                print self.frame/self.sim_len*100
            elif user_input == 'reset':
                self.frame = 0
            elif command[0] == 'set':
                if command[1] == 'time':
                    desired_index = floor(int(command[2])/100*self.sim_len)
                    self.frame = desired_index
            elif user_input == 'rtf':
                frame = self.frame
                time.sleep(3)
                diff = self.time[self.frame] - self.time[frame]
                print diff/3000000
            else:
                print "unknown input command"
    def print_help(self):
        print("""Usage:
                    time: Shows momentary time in percentage
                    reset: Resets animation
                    set time <value>: Sets the time to value [in percent]""")

def _main():
    file_name = sys.argv[1]
    sys.argv = [file_name,file_name,'-f',file_name.split('.')[0]+'.csv','-t','TIME','-m','TIME','-m','ATT','-m','LPOS' ]
    sdlog2_dump._main()

    x = FlightData(file_name)
    thread.start_new_thread(x.user_input,())
    global ax

    if True:
        # Do animation
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        ax.set_xlim3d([-5.0, 1.0])
        ax.set_xlabel('X')
        ax.set_ylim3d([-1.0, 1.0])
        ax.set_xlabel('Y')
        ax.set_zlim3d([-1.0, 1.0])
        ax.set_xlabel('Z')

        line = ax.plot([-1,0,1],[-1,0,1],[-1,0,1])[0]
        line_ani = animation.FuncAnimation(fig, x.animate,1,interval=10,blit=False)
        # line_ani = animation.FuncAnimation(fig, x.animate,interval=10,blit=False)
        plt.show()

if __name__ == "__main__":
    _main()
