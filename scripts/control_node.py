#!/usr/bin/env python3
# nodo encargado de actuar sobre la velocidad de cada rueda para convertirla en 
# velocidad lineal y angular segun la lectura de un topic

import rospy
import time
from std_msgs.msg import Float32
from diff_chaser.msg import velocity_cmd

#libreria para pilotar el jetbot
from custom_jetbot import Robot, Motor

class controller():
    def __init__(self):
        # nos suscribimos al topic que ordenara la velocidad
        self.img_sub=rospy.Subscriber('/diff/cmd_vel', velocity_cmd, self.get_cmd)
        # creamos las variables que extraeremos de la orden
        self.vel_lin=0
        self.vel_ang=0
        # creamos un objeto tipo robot para manejar el jetbot
        self.robot=Robot()

    def vel_control(self):
        # calculamos la velocidad de cada motor
        vel_d = self.vel_lin + self.vel_ang/2
        vel_i = self.vel_lin - self.vel_ang/2 
        
        #actuamos sobre el robot. la propia libreria se encarga de saturar la orden en caso de ser necesario
        
        self.robot.set_motors(vel_i,vel_d)

        return True

    def get_cmd(self, data):
        #guardamos en variables locales los datos
        self.vel_lin=data.lineal
        self.vel_ang=data.angular
        self.vel_control()

if __name__ == "__main__":
    rospy.init_node('control_node')
    rospy.loginfo('CONTROL_NODE: Node started.')
    controlador = controller()
    rospy.loginfo('CONTROL_NODE: Waiting for orders.')
    rospy.spin()
