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
import thread,sys,time,os.path
import sdlog2_dump

__author__ = "Roman Bapst"

class FlightData(object):
    def __init__(self,file_name):
        self.log_file_name = file_name
        self.csv_file_name = file_name.split('.')[0] + ".csv"
        self.header_list = []
        self.time = []
        self.origin = [0, 0, 0]
        self.x = []
        self.y = []
        self.z = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.qw_des = []
        self.qx_des = []
        self.qy_des = []
        self.qz_des = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.read_data(self.csv_file_name)

        # change look of the plane here
        # VTOL orientation for RPY=[0,0,0] is nose up (-Z), dorsal fin in -X direction
        # (normal FW orientation pitched up by pi/2
        self.prop_rad = 0.1
        self.arm_length = 0.5
        self.x_coord = np.array([self.arm_length*cos(pi/4), self.arm_length*cos(pi/4), 0, 0.1, 0, -self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4), 0, -self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4), 0, self.arm_length*cos(pi/4),self.arm_length*cos(pi/4),self.arm_length*cos(pi/4), 0,0])
        self.y_coord = np.array([-self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4), 0, 0, 0, -self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4),-self.arm_length*cos(pi/4),0, self.arm_length*cos(pi/4),self.arm_length*cos(pi/4),self.arm_length*cos(pi/4), 0, self.arm_length*cos(pi/4),self.arm_length*cos(pi/4),self.arm_length*cos(pi/4), 0,0])
        self.z_coord = -np.array([0.1,0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0.1,0,0,0,0.1,0,0,0.1])

        # Cartesian axes (body frame, x forward, y right, z down)
        # self.x_coord = np.array([0, 1, 0, 0, 0,  0])
        # self.y_coord = np.array([0, 0, 0, 1, 0,  0])
        # self.z_coord = np.array([0, 0, 0, 0, 0, -1])
        self.animation_state = 'run'
        self.INDEX = np.arange(0,len(self.time),1)
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
            
            try:
                self.qw_des.append(float(data[header_dic["ATSP_qw"]]))
                self.qx_des.append(float(data[header_dic["ATSP_qx"]]))
                self.qy_des.append(float(data[header_dic["ATSP_qy"]]))
                self.qz_des.append(float(data[header_dic["ATSP_qz"]]))
            except KeyError:
                #quaternion setpoint not logged yet
                self.qw_des.append(0)
                self.qx_des.append(0)
                self.qy_des.append(0)
                self.qz_des.append(0)
            
    
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
                # verified correct for RPY values in sdlog2
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
        R[0,1] = 2 * q1 * q2 - 2 * q0 * q3
        R[0,2] = 2 * q1 * q3 + 2 * q0 * q2
        R[1,0] = 2 * q1 * q2 + 2 * q0 * q3
        R[1,1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3
        R[1,2] = 2 * q2 * q3 - 2 * q0 * q1
        R[2,0] = 2 * q1 * q3 - 2 * q0 * q2
        R[2,1] = 2 * q2 * q3 + 2 * q0 * q1
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
        R[0,1] = ca * sb * sg - sa * cg
        R[0,2] = ca * sb * cg + sa * sg
        R[1,0] = sa * cb
        R[1,1] = sa * sb * sg + ca * cg
        R[1,2] = sa * sb * cg - ca * sg
        R[2,0] = -sb
        R[2,1] = cb * sg
        R[2,2] = cb * cg

        return R

    def animate(self,i):
        if self.animation_state == 'run':
            self.frame = self.frame + 1
            if self.frame >= self.INDEX[-2]:
                self.frame = 0
        zsign = 1
        if self.frame >= self.sim_len:
            self.frame = 0
            print("looping")
        ax.clear()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        position = [self.x[self.INDEX[self.frame]],self.y[self.INDEX[self.frame]],zsign*self.z[self.INDEX[self.frame]]]
        dspan = 2
        if (dspan > 4):
            for i, pos in enumerate(position):
                if (abs(pos - self.origin[i]) > dspan/2):
                    self.origin[i] = position[i]
                    if (i==2):
                        self.origin[i] *= zsign
        else:
            for i, pos in enumerate(position):
                self.origin[i] = position[i]
                if (i==2):
                    self.origin[i] *= zsign
        ax.set_xlim3d([self.origin[0]-dspan/2, self.origin[0]+dspan/2])
        ax.set_ylim3d([self.origin[1]-dspan/2, self.origin[1]+dspan/2])
        ax.set_zlim3d([zsign*self.origin[2]-dspan/2, zsign*self.origin[2]+dspan/2])
        #ax.invert_zaxis()
        x = []
        y = []
        z = []
        x_des = []
        y_des = []
        z_des = []
        line = ax.plot(self.x_coord, self.y_coord, zsign*self.z_coord)[0]
        line_des = ax.plot(self.x_coord, self.y_coord, zsign*self.z_coord)[0]

        if (1):
            # verified correct for RPY values in sdlog2
            # q = self.rpy_to_quat(pi/8, 0, 0)
            # q = self.rpy_to_quat(0, pi/8, 0)
            # q = self.rpy_to_quat(0, 0, pi/8)
            q = [self.qw[self.INDEX[self.frame]],self.qx[self.INDEX[self.frame]],self.qy[self.INDEX[self.frame]],self.qz[self.INDEX[self.frame]]]
            R = self.quat_to_rot(q)
            q_des = [self.qw_des[self.INDEX[self.frame]],self.qx_des[self.INDEX[self.frame]],self.qy_des[self.INDEX[self.frame]],self.qz_des[self.INDEX[self.frame]]]
            R_des = self.quat_to_rot(q_des)
        else:
            # verified correct for RPY values in sdlog2
            roll = self.roll[self.INDEX[self.frame]]
            pitch = self.pitch[self.INDEX[self.frame]]
            yaw = self.yaw[self.INDEX[self.frame]]
            R = self.rpy_to_rot(roll,pitch,yaw)
            # R = self.rpy_to_rot(pi/8,0,0)
            R = self.rpy_to_rot(0,pi/8,0)
            # R = self.rpy_to_rot(0,0,pi/8)

        for index,item in enumerate(self.x_coord):
            vec = np.dot(R,[self.x_coord[index],self.y_coord[index],self.z_coord[index]])
            x.append(vec[0] + self.x[self.INDEX[self.frame]])
            y.append(vec[1] + self.y[self.INDEX[self.frame]])
            z.append(vec[2] + zsign * self.z[self.INDEX[self.frame]])
            vec_des = np.dot(R_des,[self.x_coord[index],self.y_coord[index],self.z_coord[index]])
            x_des.append(vec_des[0] + self.x[self.INDEX[self.frame]])
            y_des.append(vec_des[1] + self.y[self.INDEX[self.frame]])
            z_des.append(vec_des[2] + zsign*self.z[self.INDEX[self.frame]])      

        line.set_data(np.array(x), np.array(y))
        line.set_3d_properties(np.array(z))
        line_des.set_data(np.array(x_des), np.array(y_des))
        line_des.set_3d_properties(np.array(z_des))
        
        return [line, line_des]
     
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
            elif user_input == 'p':
                self.animation_state = 'paused'
            elif user_input == 'r':
                self.animation_state = 'run'
            elif user_input == '+' and self.animation_state == 'paused':
                self.frame = self.frame + 1
            elif user_input == '-' and self.animation_state == 'paused':
                self.frame = self.frame - 1
            elif user_input == 'sec':
                print (self.time[self.INDEX[self.frame]] - self.time[0])/(1e6)
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
    #only parse log if the csv does not exist yet
    if not os.path.exists(file_name.split('.')[0] + '.csv'):
        sys.argv = [file_name,file_name,'-f',file_name.split('.')[0]+'.csv','-t','TIME','-m','TIME','-m','ATT','-m','LPOS','-m','ATSP' ]
        sdlog2_dump._main()

    x = FlightData(file_name)
    thread.start_new_thread(x.user_input,())
    global ax

    if True:
        # Do animation
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        ax.view_init(elev=-170)
        ax.set_xlim3d([-1.0, 1.0])
        ax.set_xlabel('X')
        ax.set_ylim3d([-1.0, 1.0])
        ax.set_xlabel('Y')
        ax.set_zlim3d([-1.0, 1.0])
        ax.set_xlabel('Z')
        line = ax.plot([-1,0,1],[-1,0,1],[-1,0,1])[0]
        line_des = ax.plot([-1,0,1],[-1,0,1],[-1,0,1])[0]
        lines = [line,line_des]
        line_ani = animation.FuncAnimation(fig, x.animate,interval=10,blit=False)
        #line_ani.save('movie.mp4',fps=30)
        
        
        try:
            plt.show()
        except:
            sys.exit()

if __name__ == "__main__":
    _main()
