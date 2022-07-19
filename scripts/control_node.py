#!/usr/bin/env python3
# nodo encargado de actuar sobre la velocidad de cada rueda para convertirla en 
# velocidad lineal y angular segun la lectura de un topic

import rospy
import time
from std_msgs.msg import Float32
from jetbot_color_chaser.msg import velocity_cmd

#libreria para pilotar el jetbot
from custom_jetbot import Robot, Motor

class controller():
    def __init__(self):
        # inicializamos parametros
        self.max_vel=rospy.get_param('~max_wheel_vel',default=1)
        # nos suscribimos al topic que ordenara la velocidad
        self.img_sub=rospy.Subscriber('/jetbot/cmd_vel', velocity_cmd, self.get_cmd)
        # creamos un objeto tipo robot para manejar el jetbot
        self.robot=Robot()

    def vel_control(self, vel_lin, vel_ang):
        # calculamos la velocidad de cada motor
        vel_d = vel_lin + vel_ang/2
        vel_i = vel_lin - vel_ang/2 
        
        #actuamos sobre el robot. la propia libreria se encarga de saturar la orden en caso de ser necesario
        
        self.robot.set_motors(vel_i,vel_d)

        return True

    def get_cmd(self, data):
        # calculamos la velocidad de cada motor
        vel_d = data.lineal + data.angular/2
        vel_i = data.lineal - data.angular/2 
        
        #actuamos sobre el robot. la propia libreria se encarga de saturar la orden en caso de ser necesario
        self.robot.set_motors(vel_i,vel_d)

if __name__ == "__main__":
    rospy.init_node('control_node')
    rospy.loginfo('CONTROL_NODE: Node started.')
    controlador = controller()
    rospy.loginfo('CONTROL_NODE: Waiting for orders.')
    rospy.spin()
