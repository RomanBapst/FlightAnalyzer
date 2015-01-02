#!/usr/bin/env python

# Author: Roman Bapst
# Date: 02.01.2015
# Description: This script can be used to visualize the attitude (and in future)
# also the path of a plane.

from __future__ import division
import numpy as np
from math import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import thread

class FlightData(object):
    def __init__(self):
        self.header_list = []
        self.time = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.read_data('test.csv')
        self.q = np.zeros((4,len(self.time)))
        self.compute_quaternions()
        
        self.x_coord = np.array([0,0,0,0,0,0,-0.2,-0.2,0])
        self.y_coord = np.array([0,0.5,-0.5,0,0,0,0,0,0])
        self.z_coord = np.array([0.5,-0.5,-0.5,0.5,0.7,0,-0.1,-0.2,-0.2])
        
        self.INDEX = np.arange(0,len(self.time),5)
        self.sim_len = len(self.INDEX)
        self.offset = 0
        self.ref_i = 0
        self.frame = 0
                        
    def read_data(self,filename):
        f = open(filename,'r')
        # read message header
        self.header_list = ((f.readline()).rstrip('\n')).split(',')
        # read data
        for line in f.readlines():
            data = line.rstrip('\n').split(',')
            self.time.append(float(data[0]))
            self.roll.append(float(data[1]))
            self.pitch.append(float(data[2]))
            self.yaw.append(float(data[3]))
    
    def compute_quaternions(self):
        for index,item in enumerate(self.roll):
            self.q[:,index] = self.quat_from_euler(self.yaw[index],self.pitch[index],self.roll[index]).transpose()
            
            
    def quat_from_euler(self,yaw,pitch,roll):
        q0 = cos( 0.5 * yaw ) * cos( 0.5 * pitch ) * cos( 0.5 * roll ) + sin( 0.5 * yaw ) * sin( 0.5 * pitch ) * sin( 0.5 * roll );
        q1 = cos( 0.5 * yaw ) * cos( 0.5 * pitch ) * sin( 0.5 * roll ) - sin( 0.5 * yaw ) * sin( 0.5 * pitch ) * cos( 0.5 * roll );
        q2 = cos( 0.5 * yaw ) * sin( 0.5 * pitch ) * cos( 0.5 * roll ) + sin( 0.5 * yaw ) * cos( 0.5 * pitch ) * sin( 0.5 * roll );
        q3 = sin( 0.5 * yaw ) * cos( 0.5 * pitch ) * cos( 0.5 * roll ) - cos( 0.5 * yaw ) * sin( 0.5 * pitch ) * sin( 0.5 * roll );
        
        return np.array([q0,q1,q2,q3])
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
        
            
    def animate(self,i):
        self.frame = self.frame + 1
        if self.frame >= self.INDEX[-2]:
            self.frame = 0
        ax.clear()
        ax.set_xlim3d([-1.0, 1.0])
        ax.set_xlabel('X')
        ax.set_ylim3d([-1.0, 1.0])
        ax.set_ylabel('Y')
        ax.set_zlim3d([-1.0, 1.0])
        ax.set_zlabel('Z')
        x = []
        y = []
        z = []
        line = ax.plot(self.x_coord, self.y_coord, self.z_coord)[0]
        R = self.quat_to_rot(self.q[:,self.INDEX[self.frame]])
        for index,item in enumerate(self.x_coord):
            vec = np.dot(R,[self.x_coord[index],self.y_coord[index],self.z_coord[index]])
            x.append(vec[0])
            y.append(vec[1])
            z.append(vec[2])
            
        line.set_data(np.array(x), np.array(y))
        line.set_3d_properties(np.array(z))
        return line
    
    def rot_by_quat(self,v,q):
        v_ext = np.array([0,v[0],v[1],v[2]])
        q_inv = list(q)
        q_inv[1:4] = [-item for item in q[1:4]]
        h1 = self.ham_prod(q,v_ext)
        h2 = self.ham_prod(h1,q_inv)
        return h2
        
        
    def ham_prod(self,x,y):
        out = []
        out.append(x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3])
        out.append(x[0]*y[1] + x[1]*y[0] + x[2]*y[3] - x[3]*y[2])
        out.append(x[0]*y[2] - x[1]*y[3] + x[2]*y[0] + x[3]*y[1])
        out.append(x[0]*y[3] + x[1]*y[2] - x[2]*y[1] + x[3]*y[0])
        return np.array(out)
    
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
            else:
                print "unknown input command"
    def print_help(self):
        print("""Usage:
                    time: Shows momentary time in percentage
                    reset: Resets animation
                    set time value: Sets the time to value [in percent]""")
        

        

x = FlightData()
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
    
    # Set up formatting for the movie files
    #Writer = animation.FFMpegWriter()
        
    line_ani = animation.FuncAnimation(fig, x.animate,blit=False,interval=10)
    plt.show()
                                  
    #line_ani.save('test1.mp4',writer=Writer)
