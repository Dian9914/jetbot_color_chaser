#!/usr/bin/env python3
# nodo encargado de actuar sobre la velocidad de cada rueda para convertirla en 
# velocidad lineal y angular segun la lectura de un topic

import rospy

# se importan los mensajes que se van a publicar y recibir
from std_msgs.msg import Float32
from jetbot_color_chaser.msg import velocity_cmd

#libreria para pilotar el jetbot
from custom_jetbot import Robot, Motor

class controller():
    # Clase encargada de controlar el robot, recibiendo comandos de velocidad por un topic, traduciendolos a inputs para los motores izquierdo y derecho del robot y dando las ordenes de actuacion correspondientes
    def __init__(self):
        # leyemos parametros del .launch
        self.max_vel=rospy.get_param('~max_wheel_vel',default=1)
        self.verbose=rospy.get_param('~verbose',default=True)
        # nos suscribimos al topic que ordenara la velocidad, programando un callback para manejar las ordenes entrantes
        self.img_sub=rospy.Subscriber('/jetbot/cmd_vel', velocity_cmd, self.process_cmd)
        # creamos un objeto tipo robot para manejar el jetbot
        self.robot=Robot()

    def process_cmd(self, data):
        # calculamos la velocidad de cada motor
        vel_d = data.lineal + data.angular/2
        vel_i = data.lineal - data.angular/2 
        
        #actuamos sobre el robot, saturando la señal antes
        vel_d = max(-self.max_vel,min(vel_d,self.max_vel))
        vel_i = max(-self.max_vel,min(vel_i,self.max_vel))
        
        if self.verbose: 
            print("CONTROL NODE: Left wheel speed: %1.1f\t Right wheel speed: %1.1f" % (vel_i, vel_d))
            
        self.robot.set_motors(vel_i,vel_d)

if __name__ == "__main__":
    # Se inicializa el nodo de control en ROS y se notifica en pantalla.
    rospy.init_node('control_node')
    rospy.loginfo('CONTROL_NODE: Node started.')
    # Instanciamos el objeto controlador y esperamos por ordenes en un bucle
    controlador = controller()
    rospy.loginfo('CONTROL_NODE: Waiting for orders.')
    rospy.spin()
