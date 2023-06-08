import rospy
import simplecamera
import cv2
import argparse
import time

#cargamos las librerias para manejar el mando
import Gamepad

#importamos los mensajes
from jetbot_color_chaser.msg import camera_data, velocity_cmd

# funcion para leer los argumentos que se le dan por consola a python
def parse_args():
    parser = argparse.ArgumentParser(description='Node used for gamepad teleoperation')
    parser.add_argument('--control', default= 'gamepad', help ='Switch between "gamepad" or "keyboard" for the teleoperation control. Deprecated.')
    args = parser.parse_args()
    return args

def main():
    # guardamos los argumentos en la variable arg
    args = parse_args()

    if args.control=="gamepad":
        # Gamepad settings
        gamepadType = Gamepad.Gamepad
        button = 2
        joystickSpeed = 2
        joystickSteering = 3
        pollInterval = 0.1

        # esperamos a que el gamepad se conecte y este disponible
        if not Gamepad.available():
            print('Please connect your gamepad...')
            while not Gamepad.available():
                time.sleep(1.0)
        gamepad = gamepadType()
        print('Gamepad connected')

        # Inicializamos las variables relacionadas con el gamepad
        global running, lin_vel, ang_vel
        running = True
        lin_vel = 0.0
        ang_vel = 0.0

        # Creamos un callback que se ejecute cada vez que se pulse el boton
        # En este caso el callback hara que el robot gire sobre sí mismo
        def buttonPressed():
            lin_vel = 0.0
            ang_vel = 0.8


        # Inicia el hilo encargado de actualizar los valores del gamepad
        gamepad.startBackgroundUpdates()

        # Registramos el callback del boton
        gamepad.addButtonPressedHandler(button, buttonPressed)



    # inicializamos el nodo de ROS
    rospy.init_node('teleoperation_node')

    # creamos el publisher para publicar la accion de control
    action_pub=rospy.Publisher('/jetbot/cmd_vel', velocity_cmd, queue_size=10)

    # en un bucle while se iran leyendo las instrucciones por teclado o por gamepad y se ira mostrando la imagen que ve la camara del jetbot
    while not rospy.is_shutdown():
        #reseteamos la accion de control
        lin_vel = 0.0
        ang_vel = 0.0

        if args.control=="gamepad":
            lin_vel = -gamepad.axis(joystickSpeed)*0.5
            ang_vel = gamepad.axis(joystickSteering)*0.3

        #finalmente, publicamos la accion de control
        vel_action=velocity_cmd()
        vel_action.lineal=lin_vel
        vel_action.angular=ang_vel
        action_pub.publish(vel_action)

    gamepad.disconnect()

if __name__ == '__main__':
    main()