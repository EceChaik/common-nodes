#!/usr/bin/env python

import argparse
import matplotlib.pyplot as plt
import numpy as np

WARNING = "\033[93m[WARNING] "


def main():
    uav = np.loadtxt('uav.txt', delimiter=",", skiprows=1)
    arm = np.loadtxt('arm.txt', delimiter=",", skiprows=1)    
    force = np.loadtxt('force.txt', delimiter=",", skiprows=1)    
    control = np.loadtxt('control.txt', delimiter=",", skiprows=1)    

    show_uav = 1
    show_robot = 1
    show_force = 1
    show_control = 1

    if(show_uav):
        figU, axsU = plt.subplots(1,3)
        figU.canvas.set_window_title('uav')

        axsU[0].plot(uav[:, 0], uav[:, 1], color="red", linestyle="dashed")
        axsU[0].set_title("X")

        axsU[1].plot(uav[:, 0], uav[:, 2], color="green", linestyle="dashed")
        axsU[1].set_title("Y")

        axsU[2].plot(uav[:, 0], uav[:, 3], color="blue", linestyle="dashed")
        axsU[2].set_title("Z")

    if(show_robot):        
        figR, axsR = plt.subplots(2, 1)
        figR.canvas.set_window_title('robot')
        
        axsR[0].plot(arm[:, 0], arm[:, 1], color="red", linestyle="dashed")
        axsR[0].set_title("q1")

        axsR[1].plot(arm[:, 0], arm[:, 2], color="green", linestyle="dashed")
        axsR[1].set_title("q2")

    if(show_force):
        figF, axsF = plt.subplots(1,3)
        figF.canvas.set_window_title('force')

        axsF[0].plot(force[:, 0], force[:, 1], color="red", linestyle="dashed")
        axsF[0].set_title("fX")

        axsF[1].plot(force[:, 0], force[:, 2], color="green", linestyle="dashed")
        axsF[1].set_title("fY")

        axsF[2].plot(force[:, 0], force[:, 3], color="blue", linestyle="dashed")
        axsF[2].set_title("fZ")

    if(show_control):
        figC, axsC = plt.subplots(2,2)
        figC.canvas.set_window_title('force')

        axsC[0, 0].plot(control[:, 0], control[:, 1], color="red", linestyle="dashed")
        axsC[0, 0].set_title("T")

        axsC[0, 1].plot(control[:, 0], control[:, 2], color="green", linestyle="dashed")
        axsC[0, 1].set_title("roll")

        axsC[1, 0].plot(control[:, 0], control[:, 3], color="green", linestyle="dashed")
        axsC[1, 0].set_title("pitch")

        axsC[1, 1].plot(control[:, 0], control[:, 4], color="blue", linestyle="dashed")
        axsC[1, 1].set_title("yaw_r")

    plt.show()

if __name__ == "__main__":
    main()






