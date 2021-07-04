#!/usr/bin/env python
# nodo encargado de obtener los datos de imagen y extraer informacion
# acerca de los objetos encontrados

#importamos mensajes
from diff_chaser.msg import camera_data, color_pose
from sensor_msgs.msg import Image

import rospy

#cv2 y numpy para tratado de imagen, cvbridge para leer las imagenes
import cv2
import numpy as np 
from cv_bridge import CvBridge

# time para controlar el tiempo de procesamiento de imagen
import time

class image_processing():
    def __init__(self):
        #definicion de los thresholds para la deteccion de colores
        self.low_thresh_red = np.array([150, 0, 0])
        self.high_thresh_red = np.array([255, 50, 50])
        self.low_thresh_green = np.array([0, 150, 0])
        self.high_thresh_green = np.array([50, 255, 50])
        self.low_thresh_blue = np.array([0, 0, 150])
        self.high_thresh_blue = np.array([50, 50, 255])

        # objeto para poder convertir los mensajes image de ROS a un array 
        # legible por opencv
        self.bridge = CvBridge()

        self.image_flag = False #bandera que marca la primera lectura de imagen

        # fragmento de codigo que lee el rosparam "enable_verbose", que se pasara en el fichero .launch
        if rospy.has_param('~enable_verbose'):
            self.enable_verbose = rospy.get_param('~enable_verbose')
        else:
            self.enable_verbose = True

        self.img_sub=rospy.Subscriber('/robot/imagen',Image, self.read_img)
        self.img_pub=rospy.Publisher('/robot/cv_image',Image, queue_size=10)
        self.data_pub=rospy.Publisher('/diff/camera_data',camera_data, queue_size=10)

        self.procesed_data = camera_data()

    def read_img(self, data):
        self.image_data = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        #la imagen esta espejada, cosa que no nos interesa
        self.image_data = cv2.flip(self.image_data, 1)
        self.image_flag = True

    def findCenter(self, img, low_thresh, high_thresh):
        #metodo que localiza un objeto de un color dado dentro del threshold y calcula su centro geometrico y su area
        #normalmente se filtraria previamente la imagen, pero en este caso no es necesario porque es una camara virtual

        # obtenemos una mascara que solo abarque los objetos del color de interes
        mask = cv2.inRange(img, low_thresh, high_thresh)

        # Obtenemos los contornos de cada objeto visible en la mascara
        (cnts ,_) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        
        # Seleccionamos el contorno de mayor area como el contorno que seguira nuestro robot
        # es decir, en caso de encontrar varios objetos de color, seguira al mas grande
        if cnts: 
            max_area=0
            cnt_max=cnts[0]
            for cnt in cnts:
                area = cv2.contourArea(cnt)
                if max_area<area:
                    max_area=area
                    cnt_max=cnt
            cnt=cnt_max
            cnt_area=max_area
        else:
            return -1, -1

        #finalmente calculamos los momentos, sabiendo que el momento 10 entre el 00 nos da el centro
        #geometrico de la figura en el eje horizontal
        M = cv2.moments(cnt)

        try:
            cX = int(M["m10"] / M["m00"])
        except ZeroDivisionError:
            cX = -1

        return cX, cnt_area

    def process_data(self):
        #metodo que lee la imagen cada 0.2s, procesa las posiciones de los objetos y las publica en un topic
        rate=rospy.Rate(5)
        #primero esperamos a tener senial de imagen
        while not self.image_flag:
            rospy.loginfo('IMAGE_NODE: Waiting for camera feed.')
            rate.sleep()

        #una vez tenemos senial, empezamos a publicar la informacion tratada
        rospy.loginfo('IMAGE_NODE: Started publishing data.')
        while not rospy.is_shutdown():
            #usamos time para controlar el tiempo que se tarda entre ejecuciones de codigo
            start=time.time()
            #guardamos la imagen que queremos tratar para evitar que se nos sobreescriba con otra
            #durante el proceso
            img=self.image_data
            #buscamos objetos de los 3 colores que queremos tratar en este sistema
            (c_r,area_r)=self.findCenter(img,self.low_thresh_red,self.high_thresh_red)
            (c_g,area_g)=self.findCenter(img,self.low_thresh_green,self.high_thresh_green)
            (c_b,area_b)=self.findCenter(img,self.low_thresh_blue,self.high_thresh_blue)
            #finalmente calculamos el tiempo empleado
            end=time.time()
            time_elapsed=end-start

            #si el verbose esta activado, imprimira un resumen de lo obtenido
            if self.enable_verbose:
                print('RED:\t center: %d \t area: %d'%(c_r,area_r))
                print('GREEN:\t center: %d \t area: %d'%(c_g,area_g))
                print('BLUE:\t center: %d \t area: %d'%(c_b,area_b))
                print('Elapsed time: %f'%time_elapsed)
                print('----------------')

            #construimos el mensaje que se comunicara al central node y lo publicamos
            self.procesed_data.red.center=int(c_r)
            self.procesed_data.red.area=int(area_r)
            self.procesed_data.green.center=int(c_g)
            self.procesed_data.green.area=int(area_g)
            self.procesed_data.blue.center=int(c_b)
            self.procesed_data.blue.area=int(area_b)
            self.data_pub.publish(self.procesed_data)

            #anhadimos informacion al frame actual para publicarlo
            if c_r>=0:
                img = cv2.circle(img, (c_r,128/2), radius=5, color=(255, 255, 255), thickness=-1)
                img = cv2.circle(img, (c_r,128/2), radius=3, color=(255, 0, 0), thickness=-1)

            if c_g>=0: 
                img = cv2.circle(img, (c_g,128/2), radius=5, color=(255, 255, 255), thickness=-1)
                img = cv2.circle(img, (c_g,128/2), radius=3, color=(0, 255, 0), thickness=-1)

            if c_b>=0: 
                img = cv2.circle(img, (c_b,128/2), radius=5, color=(255, 255, 255), thickness=-1)
                img = cv2.circle(img, (c_b,128/2), radius=3, color=(0, 0, 255), thickness=-1)

            image_message = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
            self.img_pub.publish(image_message)

            #finalmente, suspendemos el codigo con sleep. Como indicamos al objeto rate que deseabamos
            #5Hz, se intentara mantener dormido durante el tiempo necesario para mantener la frecuencia
            rate.sleep()


if __name__ == "__main__":
    #iniciamos el nodo, lo indicamos en consola y inicializamos el objeto
    rospy.init_node('image_node')
    rospy.loginfo('IMAGE_NODE: Node started.')
    obj = image_processing()
    #iniciamos el procesado de datos
    obj.process_data()