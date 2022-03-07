#!/usr/bin/env python
# nodo encargado de actuar sobre la velocidad de cada rueda para convertirla en 
# velocidad lineal y angular segun la lectura de un topic

import rospy
import time
from std_msgs.msg import Float32
from jetbot_color_chaser.msg import velocity_cmd


class controller():
    def __init__(self):
        # inicializamos parametros
        self.max_vel=rospy.get_param('max_wheel_vel',default=10)
        # nos suscribimos al topic que ordenara la velocidad
        self.img_sub=rospy.Subscriber('/jetbot/cmd_vel', velocity_cmd, self.get_cmd)
        # creamos las variables que extraeremos de la orden
        self.vel_lin=0
        self.vel_ang=0
        # creamos los publishers e inicializamos los mensajes
        self.pub_vel_d = rospy.Publisher('robot/vel_der', Float32 , queue_size=10)
        self.pub_vel_i = rospy.Publisher('robot/vel_izq', Float32 , queue_size=10)
        self.act_vel_d = Float32()
        self.act_vel_i = Float32()

    def vel_control(self):
        # calculamos la velocidad de cada motor
        vel_d = self.vel_lin + self.vel_ang/2
        vel_i = self.vel_lin - self.vel_ang/2 
        
        
        #publicamos la accion debidamente saturada
        self.pub_vel_d.publish(max(0,min(self.act_vel_d,self.max_vel)))
        self.pub_vel_i.publish(max(0,min(self.act_vel_i,self.max_vel)))

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
