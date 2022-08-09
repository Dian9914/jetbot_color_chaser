#!/usr/bin/env python
# nodo encargado de obtener los datos de imagen y extraer informacion
# acerca de los objetos encontrados

#importamos mensajes
from jetbot_color_chaser.msg import camera_data, color_pose
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
        #---obtencion de parametros de ROS, definidos en el fichero launch---
        # fragmento de codigo que lee el rosparam "enable_verbose", que se pasara en el fichero .launch
        self.enable_verbose = rospy.get_param('~enable_verbose', default=False)

        #---inicializacion de publishers y subscribers de ROS---
        # la imagen capturada por la camara simulada se sube al topic /robot/imagen, por lo que nos suscribimos
        self.img_sub=rospy.Subscriber('/robot/imagen',Image, self.read_img)

        # publicamos en dos topic, uno con la imagen y otro con los datos extraidos de la imagen
        self.img_pub=rospy.Publisher('/jetbot/cv_image',Image, queue_size=10)
        self.data_pub=rospy.Publisher('/jetbot/camera_data',camera_data, queue_size=10)


        #---iniciacion de variables y objetos---
        #definicion de los thresholds para la deteccion de colores, en simulacion siempre son los mismos
        self.low_thresh_red = np.array([-10*1.8, 150, 150])
        self.high_thresh_red = np.array([10*1.8, 255, 255])
        self.low_thresh_green = np.array([25*1.8, 150, 150])
        self.high_thresh_green = np.array([40*1.8, 255, 255])
        self.low_thresh_blue = np.array([55*1.8, 150, 150])
        self.high_thresh_blue = np.array([70*1.8, 255, 255])

        #definicion de los umbrales para calcular la distancia al objetivo
        self.area_umb0=rospy.get_param('~area_far', default=20)  #numero de pixeles que ocupa el objeto cuando esta lejos
        self.area_umb1=rospy.get_param('~area_mid', default=100) #numero de pixeles que ocupa el objeto cuando esta a media distancia
        self.area_umb2=rospy.get_param('~area_near', default=400) #numero de pixeles que ocupa el objeto cuando esta cerca
        

        # kernel a usar en los metodos morfologicos
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

        # objeto para poder convertir los mensajes image de ROS a un formato legible por opencv
        self.bridge = CvBridge()

        self.image_flag = False #bandera que marca la primera lectura de imagen

    # callback que se llama cada vez que recibimos una imagen. Se encarga de preprocesar la imagen para facilitar su procesamiento posterior con opencv.
    def read_img(self, data):
        raw_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        #la imagen esta espejada, cosa que no nos interesa
        raw_image = cv2.flip(raw_image, 1)
        self.image_data = raw_image
        self.image_flag = True

    def findCenter(self, img, low_thresh, high_thresh):
        #metodo que localiza un objeto de un color dado dentro del threshold y calcula su centro geometrico y su area
        # obtenemos una mascara que solo abarque los objetos del color de interes
        mask = cv2.inRange(img, low_thresh, high_thresh)
        
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
        #metodo que lee la imagen cada 0.1s, procesa las posiciones de los objetos y las publica en un topic
        rate=rospy.Rate(10)

        #primero esperamos a tener senial de imagen
        while not self.image_flag:
            rospy.loginfo('IMAGE_NODE: Waiting for camera feed.')
            rate.sleep()

        #una vez tenemos senial, empezamos a publicar la informacion tratada
        rospy.loginfo('IMAGE_NODE: Started publishing data.')
        while not rospy.is_shutdown():
            #primero leemos la imagen
            img=self.image_data
                
            #usamos time para controlar el tiempo que se tarda entre ejecuciones de codigo
            start=time.time()
            
            # primero hemos de filtrar la imagen para eliminar ruido en la medida de lo posible.
            # Utilizamos un filtro de medianas
            img=cv2.medianBlur(img,3)
            #buscamos objetos de los 3 colores que queremos tratar en este sistema.
            # usaremos HSV para el thresholding, por lo que hay que cambiar el espacio de color de rgb a hsv
            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            # buscamos un objeto de cada color, llamando a findCenter una vez por color
            #rojo
            (c_r,area_r)=self.findCenter(img,self.low_thresh_red,self.high_thresh_red)
            #verde
            (c_g,area_g)=self.findCenter(img,self.low_thresh_green,self.high_thresh_green)
            #azul
            (c_b,area_b)=self.findCenter(img,self.low_thresh_blue,self.high_thresh_blue)

            #finalmente calculamos a que distancia esta el objeto de la camara, usando una escala del 0 (cerca) al 3 (lejos)
            #empezamos con el rojo
            if area_r < self.area_umb0:
                dis_r = 3
            elif area_r >= self.area_umb0 and area_r < self.area_umb1:
                dis_r = 2
            elif area_r >= self.area_umb1 and area_r < self.area_umb2:
                dis_r = 1
            elif area_r >= self.area_umb2:
                dis_r = 0
            # verde
            if area_g < self.area_umb0:
                dis_g = 3
            elif area_g >= self.area_umb0 and area_g < self.area_umb1:
                dis_g = 2
            elif area_g >= self.area_umb1 and area_g < self.area_umb2:
                dis_g = 1
            elif area_g >= self.area_umb2:
                dis_g = 0
            #azul
            if area_b < self.area_umb0:
                dis_b = 3
            elif area_b >= self.area_umb0 and area_b < self.area_umb1:
                dis_b = 2
            elif area_b >= self.area_umb1 and area_b < self.area_umb2:
                dis_b = 1
            elif area_r >= self.area_umb2:
                dis_r = 0

            #una vez se tienen los datos de los objetos, construimos el mensaje que se comunicara al central node y lo publicamos
            # primero se crea el mensaje que se enviara por el topic /jetbot/camera_data, del tipo camera_data(), un tipo personalizado
            procesed_data = camera_data()
            procesed_data.red.center=int(c_r)
            procesed_data.red.area=int(area_r)
            procesed_data.red.distance=int(dis_r)
            procesed_data.green.center=int(c_g)
            procesed_data.green.area=int(area_g)
            procesed_data.green.distance=int(dis_g)
            procesed_data.blue.center=int(c_b)
            procesed_data.blue.area=int(area_b)
            procesed_data.blue.distance=int(dis_b)
            self.data_pub.publish(procesed_data)
            
            
            # a continuacion se construye la imagen que se publicara para obtener una visualizacion del resultado del procesamiento de la imagen. Esta nueva imagen marcara los objetos detectados para cada color
            img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
            height = img.shape[0]
            mid_img = int(round(height/2))
            if c_r>=0:
                img = cv2.circle(img, (c_r,mid_img), radius=5, color=(255, 255, 255), thickness=-1)
                img = cv2.circle(img, (c_r,mid_img), radius=3, color=(255, 0, 0), thickness=-1)

            if c_g>=0: 
                img = cv2.circle(img, (c_g,mid_img), radius=5, color=(255, 255, 255), thickness=-1)
                img = cv2.circle(img, (c_g,mid_img), radius=3, color=(0, 255, 0), thickness=-1)

            if c_b>=0: 
                img = cv2.circle(img, (c_b,mid_img), radius=5, color=(255, 255, 255), thickness=-1)
                img = cv2.circle(img, (c_b,mid_img), radius=3, color=(0, 0, 255), thickness=-1)

            image_message = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
            self.img_pub.publish(image_message)

            #finalmente, suspendemos el codigo con sleep. Como indicamos al objeto rate que deseabamos
            #10Hz, se intentara mantener dormido durante el tiempo necesario para mantener la frecuencia
	        #finalmente calculamos el tiempo empleado
            end=time.time()
            time_elapsed=end-start
            #si el verbose esta activado, imprimira un resumen de lo obtenido
            if self.enable_verbose:
                print('IMAGE_NODE: Red data:\t center: %d \t area: %d \t distance: %d'%(c_r,area_r,dis_r))
                print('IMAGE_NODE: Green data:\t center: %d \t area: %d \t distance: %d'%(c_g,area_g,dis_g))
                print('IMAGE_NODE: Blue data:\t center: %d \t area: %d \t distance: %d'%(c_b,area_b,dis_b))
                print('IMAGE_NODE: Elapsed time: %f'%time_elapsed)
            rate.sleep()


if __name__ == "__main__":
    #iniciamos el nodo, lo indicamos en consola y inicializamos el objeto
    rospy.init_node('image_node')
    rospy.loginfo('IMAGE_NODE: Node started.')
    obj = image_processing()
    #iniciamos el procesado de datos
    obj.process_data()
