import rospy
import simplecamera
import cv2
import argparse


#importamos los mensajes
from jetbot_color_chaser.msg import camera_data, velocity_cmd

# funcion para leer los argumentos que se le dan por consola a python
def parse_args():
    parser = argparse.ArgumentParser(description='Useful script used to calibrate the color detection.')
    parser.add_argument('--streaming', default= 'gstreamer', help ='Switch between "gstreamer" or "opencv" for the visualization of the camera.')
    args = parser.parse_args()
    return args

def main():
    # guardamos los argumentos en la variable arg
    args = parse_args()

    # inicializamos el nodo de ROS
    rospy.init_node('teleoperation_node')

    # creamos el publisher para publicar la accion de control
    action_pub=rospy.Publisher('/jetbot/cmd_vel', velocity_cmd, queue_size=10)

    if args.streaming=='opencv':
        # creamos la ventana en la que se mostrara lo que ve el robot
        win_camera = cv2.namedWindow('Camera_Image')
    else:
        # creamos la pipeline de gstreamer que usaremos para streamear el video
        out_send = cv2.VideoWriter("appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.73.255 port=5000",cv2.CAP_GSTREAMER,0, 5, (320,240), True)
        if not out_send.isOpened():
            print('VideoWriter not opened')
            exit(0)

    # creamos el habdler de la camara
    cap = simplecamera.start_camera(pre_proc=False)

    #variable que cuenta cuantos frames han fallado de forma sucesiva, de modo que si fallan 20 frames seguidos se cierre el programa y se libere el handler de la camara
    fail_counter = 0

    # en un bucle while se iran leyendo las instrucciones por teclado o por gamepad y se ira mostrando la imagen que ve la camara del jetbot
    while not rospy.is_shutdown():
        #reseteamos la accion de control
        lin_vel = 0
        ang_vel = 0

        #capturamos la imagen
        re, img = cap.read()

        #si la lectura de imagen falla, imprimimos un error y esperamos al siguiente ciclo
        if not re:
            fail_counter = fail_counter + 1
            if fail_counter>=20: break
            rospy.logerror('IMAGE_NODE: Error getting image')
            rate.sleep()
            continue
        fail_counter = 0

        if args.streaming=='opencv':
            # mostramos la imagen
            cv2.imshow('Camera_Image', img)
        else:
            #mandamos la imagen por la pipeline de gstreamer
            out_send.write(img)

    
        # finalmente, esperamos 10 milisegundos para esperar un input del teclado. Segun la tecla pulsada se tomaran directas acciones
        k = cv2.waitKey(10) & 0xFF

        if k == 113 or k == 27: #esc o q: termina la ejecucion del programa y lo cierra
            break
        if k == ord('w'): #w: hacia delante
            lin_vel=lin_vel+0.25
        if k == ord('s'): #s: hacia atras
            lin_vel=lin_vel-0.25
        if k == ord('a'): #a: hacia la izquierda
            ang_vel=ang_vel+0.25
        if k == ord('d'): #d: hacia la derecha
            ang_vel=ang_vel-0.25

        #finalmente, publicamos la accion de control
        vel_action=velocity_cmd()
        vel_action.lineal=lin_vel
        vel_action.angular=ang_vel
        action_pub.publish(vel_action)

    cap.release()
    out_send.release()

if __name__ == '__main__':
    main()