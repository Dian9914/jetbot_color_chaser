#!/usr/bin/env python
# nodo encargado de obtener los datos de imagen y extraer informacion
# acerca de los objetos encontrados

#importamos mensajes
from jetbot_color_chaser.msg import camera_data
from sensor_msgs.msg import Image

import rospy

#cv2 y numpy para tratado de imagen, cvbridge para leer las imagenes
import cv2
import numpy as np 
from cv_bridge import CvBridge
import json

# time para controlar el tiempo de procesamiento de imagen
import time

# libreria para manejar la camara instalada en el jetbot
import simplecamera

# funcion para segmentar imagenes en casos en los que los thresholds tienen que partirse en dos
def segmentate(hsv,lower_hsv,higher_hsv):
    if higher_hsv[0] > 179:
        mask1 = cv2.inRange(hsv,lower_hsv,np.array([179, higher_hsv[1], higher_hsv[2]]))
        mask2 = cv2.inRange(hsv,np.array([0, lower_hsv[1], lower_hsv[2]]),np.array([higher_hsv[0]-179, higher_hsv[1], higher_hsv[2]]))
        mask = cv2.bitwise_or(mask1,mask2)
    else:
        mask = cv2.inRange(hsv,lower_hsv,higher_hsv)

    return mask

class image_processing():
    def __init__(self):

        rospy.loginfo('IMAGE_NODE: Reading config file.')
        # ---lectura del archivo de configuracion---
        with open('/home/jetbot/catkin_ws/src/jetbot_color_chaser/config/parameters.json', 'r') as f:
            data=json.load(f)
        rospy.loginfo('IMAGE_NODE: Config file successfully read.')
        
        #definicion de los thresholds para la deteccion de colores
        self.low_thresh_red = np.array(data['red']['lower_hsv'])
        self.high_thresh_red = np.array(data['red']['higher_hsv'])
        self.low_thresh_blue = np.array(data['blue']['lower_hsv'])
        self.high_thresh_blue = np.array(data['blue']['higher_hsv'])
        self.low_thresh_green = np.array(data['green']['lower_hsv'])
        self.high_thresh_green = np.array(data['green']['higher_hsv'])

        #---obtencion de parametros de ROS, definidos en el fichero launch---
        #flag que define si se aplicara un postprocesamiento a la imagen capturada
        self.post_proc = rospy.get_param('~post_proc',default=False)
        #flag que define si se muestra informacion por pantalla sobre el programa
        self.enable_verbose = rospy.get_param('~enable_verbose', default=True)
        #flag que define si se publica la imagen procesada en un topic
        self.enable_vis = rospy.get_param('~enable_visualization', default=True)

        #---inicializacion de los publishers de ROS---
        #publica la imagen en un topic para permitir visualizar el resultado del procesamiento en programas como rviz
        if self.enable_vis:
            self.img_pub=rospy.Publisher('/jetbot/cv_image',Image, queue_size=10)   
        #publica la informacion extraida de la imagen
        self.data_pub=rospy.Publisher('/jetbot/camera_data',camera_data, queue_size=10)

        #---iniciacion de variables y objetos---
        #inicializacion de la variable donde guardamos la informacion extraida de la imagen, que publicaremos en el topic "camera_data"
        self.procesed_data = camera_data()

        # kernel a usar en los metodos morfologicos
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

        # objeto para poder convertir los mensajes image de ROS a un array 
        # legible por opencv
        self.bridge = CvBridge()
        
        # inicializamos la camara para capturar imagenes
        self.cap = simplecamera.start_camera()

    def findCenter(self, img, low_thresh, high_thresh):
        #metodo que localiza un objeto de un color dado dentro del threshold y calcula su centro geometrico y su area
        # obtenemos una mascara que solo abarque los objetos del color de interes
        mask = segmentate(img, low_thresh, high_thresh)
        
        # aplicamos metodos morfologicos para limpiar la mascara
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

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
        rate=rospy.Rate(10)

        #variable que cuenta cuantos frames han fallado de forma sucesiva, de modo que si fallan 20 frames seguidos se cierre el programa y se libere el handler de la camara
        fail_counter = 0

        #asumimos que el el handle de la camara se ha creado sin problemas, por lo que comenzamos a procesar la imagen
        rospy.loginfo('IMAGE_NODE: Started publishing data.')
        while not rospy.is_shutdown():
            #primero leemos la imagen
            re, img = self.cap.read()
            
            #si la lectura de imagen falla, imprimimos un error y esperamos al siguiente ciclo
            if not re:
                fail_counter = fail_counter + 1
                if fail_counter>=20: break
                rospy.logerror('IMAGE_NODE: Error getting image')
                rate.sleep()
                continue
            fail_counter = 0
                
            #usamos time para controlar el tiempo que se tarda entre ejecuciones de codigo, solo si el verbose esta activo
            if self.enable_verbose: start=time.time()
            
            
            # si esta activado el post-procesado de la imagen, hemos de filtrarla a fin de eliminar el ruido en la medida de lo posible.
            # Utilizamos un filtro de medianas
            if self.post_proc: img=cv2.medianBlur(img,3)

            # a continuacion buscamos objetos de los 3 colores que queremos tratar en este sistema.
            # usaremos el espacio de color HSV para el thresholding
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # se usa el metodo findcenter para procesar cada color de forma individual
            (c_r,area_r)=self.findCenter(img,self.low_thresh_red,self.high_thresh_red)
            (c_g,area_g)=self.findCenter(img,self.low_thresh_green,self.high_thresh_green)
            (c_b,area_b)=self.findCenter(img,self.low_thresh_blue,self.high_thresh_blue)
            

            #construimos el mensaje que se comunicara al central node y lo publicamos
            self.procesed_data.red.center=int(c_r)
            self.procesed_data.red.area=int(area_r)
            self.procesed_data.green.center=int(c_g)
            self.procesed_data.green.area=int(area_g)
            self.procesed_data.blue.center=int(c_b)
            self.procesed_data.blue.area=int(area_b)
            self.data_pub.publish(self.procesed_data)
            
            
            # a continuacion se construye la imagen que se publicara para obtener una visualizacion del resultado del procesamiento de la imagen. Esta nueva imagen marcara los objetos detectados para cada color
            if self.enable_vis:
                img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
                height = img.shape[0]
                mid_img = int(round(height/2))
                #anhadimos informacion al frame actual para publicarlo
                if c_r>=0:
                    img = cv2.circle(img, (c_r,mid_img), radius=5, color=(255, 255, 255), thickness=-1)
                    img = cv2.circle(img, (c_r,mid_img), radius=3, color=(255, 0, 0), thickness=-1)

                if c_g>=0: 
                    img = cv2.circle(img, (c_g,mid_img), radius=5, color=(255, 255, 255), thickness=-1)
                    img = cv2.circle(img, (c_g,mid_img), radius=3, color=(0, 255, 0), thickness=-1)

                if c_b>=0: 
                    img = cv2.circle(img, (c_b,mid_img), radius=5, color=(255, 255, 255), thickness=-1)
                    img = cv2.circle(img, (c_b,mid_img), radius=3, color=(0, 0, 255), thickness=-1)

                # se construye el mensaje usando cvbridge y se publica al topic
                image_message = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
                self.img_pub.publish(image_message)

            #si el verbose esta activado, imprimira un resumen de lo obtenido y el tiempo que se tardo en obtener la informacion
            if self.enable_verbose:
                #imprimimos la informacion extraida de la imagen
                print('IMAGE_NODE: Red location data:\t center: %d \t area: %d'%(c_r,area_r))
                print('IMAGE_NODE: Green location data:\t center: %d \t area: %d'%(c_g,area_g))
                print('IMAGE_NODE: Blue location data:\t center: %d \t area: %d'%(c_b,area_b))
                #calculamos e imprimimos el tiempo empleado en el procesamiento
                end=time.time()
                time_elapsed=end-start
                print('IMAGE_NODE: Elapsed time: %f'%time_elapsed)

            
            #finalmente, suspendemos el codigo con sleep. Como indicamos al objeto rate que deseabamos
            #5Hz, se intentara mantener dormido durante el tiempo necesario para mantener la frecuencia
            rate.sleep()

            
        # cuando el bucle se termine, es decir, cuando se cierre ros o haya ocurrido algun error, se liberara el handler de la camara
        self.cap.release()


if __name__ == "__main__":
    #iniciamos el nodo, lo indicamos en consola y inicializamos el objeto
    rospy.init_node('image_node')
    rospy.loginfo('IMAGE_NODE: Node started.')
    obj = image_processing()
    #iniciamos el procesado de datos
    obj.process_data()
