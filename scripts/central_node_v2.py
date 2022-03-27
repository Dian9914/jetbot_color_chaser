#!/usr/bin/env python
# nodo encargado de tomar decisiones en base a los datos de la imagen,
# actuar acorde sobre los actuadores y ofrecer un servicio al usuario
# para alterar el comportamiento del sistema durante el funcionamiento

from time import sleep
import rospy
from math import floor

#importamos los mensajes
from jetbot_color_chaser.msg import camera_data, velocity_cmd

#importamos los servicio
from jetbot_color_chaser.srv import chase_conf, start_srv

class central_node():
    def __init__(self):
        # parametros de seguimiento 
        self.max_lin = rospy.get_param('~max_lin_vel', default=1)    #velocidad lineal maxima a la que se persiguen objetos
        self.max_ang = rospy.get_param('~max_ang_vel', default=1)      #velocidad angular maxima a la que se persiguen objetos
        self.target_color = 'r'  #color que se persigue, de serie es rojo, todos los demas se evitan

        #ademas aniadimos una bandera para indicar cuando queremos que el control actue
        self.control_flag = False
        
        #umbrales para la construccion del mapa
        self.theta_umb4=rospy.get_param('~camera_width',default=256)
        self.theta_umb0=round(self.theta_umb4/5)
        self.theta_umb1=self.theta_umb0*2
        self.theta_umb2=self.theta_umb0*3
        self.theta_umb3=self.theta_umb0*4
        self.rho_umb0=rospy.get_param('~area_far',default=500) #numero de pixeles que ocupa el objeto cuando esta lejos
        self.rho_umb1=rospy.get_param('~area_mid',default=2000) #numero de pixeles que ocupa el objeto cuando esta a media distancia
        self.rho_umb2=rospy.get_param('~area_near',default=5000) #numero de pixeles que ocupa el objeto cuando esta cerca
        
        #margen que le damos a la zona en la que consideramos "aceptable" el obstaculo
        self.control_margin=rospy.get_param('~area_margin',default=2000)

        #para que el robot tenga "memoria", es necesario que recuerde las posiciones anteriores. Como 
        #situacion inicial supondremos que el objetivo esta en algun lugar a nuestra derecha muy cerca nuestra,
        #de modo que el robot gire sobre si mismo
        self.target_rho_ant = self.rho_umb2 
        self.target_theta_ant = self.theta_umb4 + 30
        
    def process_data(self, data):
        #primero leemos la informacion referente al color que deseamos seguir y la procesamos para enviar la senial de control
        # esto se realiza cada vez que recibimos imagenes de la camara, es decir, con una frecuencia de unos 10 Hz

        #si un objeto esta en el borde, es probable que no estemos observando la totalidad de sus pixeles.
        #para evitar este problema, los objetos en el borde se consideraran desaparecidos, a costa de perder field of vision
        #Por tanto, si los objetos estan en el borde, sus parametros pasan a valer -1, -1, como si hubieran sido perdidos
        if data.red.center>self.theta_umb4-4 or data.red.center<5:
            red_area=-1
            red_center=-1
        else:
            red_area=data.red.area
            red_center=data.red.center
        if data.green.center>self.theta_umb4-4 or data.green.center<5:
            green_area=-1
            green_center=-1
        else:
            green_area=data.green.area
            green_center=data.green.center
        if data.blue.center>self.theta_umb4-4 or data.blue.center<5:
            blue_area=-1
            blue_center=-1
        else:
            blue_area=data.blue.area
            blue_center=data.blue.center
            
        
        # dependiendo del parametro "target_color", nuestro objetivo y nuestros obstaculos seran distintos
        # realmente nos orientamos con coordenadas cartesianas y, como tal, definimos la posicion de los objetos
        # con dos parametros, theta y rho.
        # theta se define como la posicion en x del centro geometrico del objeto, parametro que nos obtenemos directamente
        # de camera data
        # rho lo obtenemos a partir del area que ocupa el objeto en la imagen. Suponemos que cuando un objeto ocupa 500 pixeles de nuestra
        # imagen, este objeto esta lo mas cerca que consideramos aceptable, por lo que la distancia a la que esta un objeto de nosotros se
        # define como 500 - el area que ocupa. Cuando estemos muy cerca, esa distancia sera 0. Cuando estemos lejos, se acercara a 500
        if self.target_color == 'r':
            self.target_theta = red_center
            self.target_rho = red_area
            self.obs1_theta = green_center
            self.obs1_rho = green_area
            self.obs2_theta = blue_center
            self.obs2_rho = blue_area
        elif self.target_color == 'g':
            self.target_theta = green_center
            self.target_rho = green_area
            self.obs1_theta = red_center
            self.obs1_rho = red_area
            self.obs2_theta = blue_center
            self.obs2_rho = blue_area
        elif self.target_color == 'b':
            self.target_theta = blue_center
            self.target_rho = blue_area
            self.obs1_theta = green_center
            self.obs1_rho = green_area
            self.obs2_theta = red_center
            self.obs2_rho = red_area

        # una vez tenemos la informacion, comenzamos el control
        # si hay algun obstaculo en nuestro encuadre debemos tenerlo en cuenta para decidir la trayectoria.
        # se considerara siempre el obstaculo mas cercano, a no ser que el objetivo este antes, en ese caso se supone que no hay obstaculos. 
        # Para esquivarlo, construimos un mapa local
        # el mapa local es una cuadricula que separa el espacio que podemos observar en la camara y que nos ayuda a tomar decisiones para esquivar
        # en el se marcaran como 1 las zonas intransitables
        # el mapa cubre el campo de vision de la camara y dos zonas que representan aquello que no podemos ver, pero que sabemos
        # que se encuentra en esa zona. Esa zona solo se usa para perseguir mejor al objetivo.
        # Cuando perdemos de vista al objetivo, suponemos que esta en esa zona
        # vitual fuera de la vision de nuestra camara para perseguirle de forma optima

        '''
        zona virtual| visible |zona virtual
        self.map=[[0|0,0,0,0,0|0],
                  [0|0,0,0,0,0|0],
                  [0|0,0,0,0,0|0],
                  [0|0,0,0,0,0|0],
                  [0|0,0,X,0,0|0]]
                        / \
                         |
                posicion del robot
    
        '''
        self.map=[[0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0]]

        # si hemos perdido de vista al objetivo, consultamos la ultima localizacion conocida y buscamos por ahi
        if self.target_theta==-1 and self.target_theta_ant!=-1:
            if self.target_theta_ant<self.theta_umb2:
                self.target_theta=-30
            else:
                self.target_theta=self.theta_umb4+30
            self.target_rho=self.target_rho_ant

        #primero registramos la posicion de nuestro objetivo en el mapa local.
        # los parametros theta se discretizan definiendo unos umbrales segun la definicion de la camara, que en este caso va de 0 a 127
        #  En el caso de que no conozcamos la posicion concreta
        # lo guardaremos en las posiciones 0 o 6, correspondientes a las zonas virtuales de nuestro mapa que no podemos ver con la camara
        t_theta_map = int(floor(self.target_theta/self.theta_umb0)+1)
        if t_theta_map<0: t_theta_map=0
        if t_theta_map>6: t_theta_map=6

        #tambien discretizamos las distancias entre 3 umbrales. Esta es una division no lineal, por lo que debemos usar if-else
        if self.target_rho<self.rho_umb0:
            t_rho_map=4  #objeto pequeno, ergo esta lejos
        elif self.target_rho>=self.rho_umb0 and self.target_rho<self.rho_umb1:
            t_rho_map=3
        elif self.target_rho>=self.rho_umb1 and self.target_rho<self.rho_umb2:
            t_rho_map=2  
        elif self.target_rho>=self.rho_umb2:
            t_rho_map=1  #objeto grande, ergo esta cerca

        # ahora hemos de comprobar si existe un obstaculo y que ademas es el obstaculo mas cercano
        if (self.obs1_rho<self.target_rho) and (self.obs1_theta!=-1) and (self.obs1_rho<self.obs2_rho):
            self.obs_rho=self.obs1_rho
            self.obs_theta=self.obs1_theta
        elif (self.obs2_rho<self.target_rho) and (self.obs2_theta!=-1) and (self.obs1_rho>self.obs2_rho):
            self.obs_rho=self.obs1_rho
            self.obs_theta=self.obs1_theta
        else:
            self.obs_rho=-1
            self.obs_theta=-1

        if self.obs_rho!=-1:
            #si hay un obstaculo, hemos de esquivarlo. Para esquivarlo, primero lo registramos en nuestro mapa local
            if self.obs_rho<self.rho_umb0:
                obs_rho_map=3
            elif self.obs_rho>=self.rho_umb0 and self.obs_rho<self.rho_umb1:
                obs_rho_map=2
            elif self.obs_rho>=self.rho_umb1:
                obs_rho_map=1

            obs_theta_map = int(floor(self.obs_theta/self.theta_umb0)+1)
            if obs_theta_map<2: obs_theta_map=2
            if obs_theta_map>4: obs_theta_map=4

            #para asegurarnos de esquivar bien el obstaculo, suponemos los obstaculos mas anchos de lo que son en realidad
            self.map[obs_rho_map][obs_theta_map]=1
            self.map[obs_rho_map][obs_theta_map-1]=1
            self.map[obs_rho_map][obs_theta_map+1]=1
            if obs_rho_map==1: #si esta en el centro lo consideramos aun mas ancho
                self.map[obs_rho_map][obs_theta_map-2]=1
                self.map[obs_rho_map][obs_theta_map+2]=1
        

        self.map[t_rho_map][t_theta_map]=2 #marcamos el objetivo en el mapa
        self.map[0]=[0,0,0,0,0,0,0]
        start = (0, 3) #el punto 0,3 se supone que es donde esta el robot situado en nuestro mapa local
        end = (t_rho_map, t_theta_map)  #este punto es donde esta el objetivo

        if self.obs_theta==-1 and self.obs_rho>self.target_rho:
            #si no se encuentran obstaculos en el camino, directaemente seguimos al objetivo, sin necesidad de llamar al planificador
            ref_pos=self.target_theta
            ref_dist=self.target_rho
        else:
            #si hay obstaculos o ninguno nos bloquea completamente el camino, planeamos el camino para esquivar
            #para ello hacemos una sencilla serie de comparaciones
            #nuestro medidor de como de bueno es un camino es lo mucho que nos aleja del objetivo en huecos del mapa. 
            #Esto se mide con la metrica dist to target, cuantas casillas hay que girar respecto al objetivo. 
            #Se inicializa con un valor alto
            dist_to_target=10
            #para cada fila del mapa, buscamos que hayan obstaculos
            for row in range(len(self.map)):
                if any(self.map[row]):
                    #si encontramos obstaculos, miramos columna a columna de la fila buscando elementos
                    for col in range(len(self.map[row])):
                        #si encontramos el objetivo antes que otra cosa, simplemente lo seguimos
                        if self.map[row][col]==2:
                            tray=col
                            break
                        #si encontramos un hueco libre por el que pasar, comprobamos como de bueno es y si es el mejor
                        #encontrado hasta la fecha, lo guardamos como la nueva trayectoria a seguir
                        if self.map[row][col]==0 and abs(t_theta_map-col)<dist_to_target:
                            dist_to_target=abs(t_theta_map-col)
                            tray=col
                    break


            # a partir de la posicion del mapa indicada por el planificador, obtenemos la posicion a la que hemos de girar
            ref_pos=self.theta_umb0*tray-30


            if (self.map[1][3]==1 and self.map[1][4]==1) and (self.map[1][2]==1):
                #si tenemos un obstaculo bloqueandonos completamente, nos paramos mientras seguimos girando en busca del objetivo
                ref_dist=0
            else:
                #si la via esta libre, seguimos avanzando hacia el objetivo
                ref_dist=self.target_rho
                
            # el error en posicion se calcula teniendo en cuenta que el robot debe mover el centro de su vision a donde la referencia le marca
            err_pos=(self.theta_umb4/2)-ref_pos

        # finalmente almacenamos los valores obtenidos del objetivo para usarlos en la siguiente iteracion
        self.target_theta_ant=self.target_theta
        self.target_rho_ant=self.target_rho
        
        # a continuacion hemos de calcular la accion del control
        
        # si el objeto esta a una distancia dentro de un cierto rango, el robot se para.
        print(ref_dist)
        print(self.rho_umb2-self.control_margin)
        print(self.rho_umb2+self.control_margin)
        if ref_dist >= self.rho_umb2-self.control_margin and ref_dist <= self.rho_umb2+self.control_margin:
            err_dist=0
            lin_vel=0
            print('Objetivo cerca')
        else:
            lin_vel=((self.rho_umb2-ref_dist)/ref_dist)*self.max_lin
            print('objetivo lejos',(self.rho_umb2-ref_dist), ((self.rho_umb2-ref_dist)/ref_dist))

        if abs(err_pos)<(self.theta_umb4/32):
            print('objetivo centrado')
            ang_vel=0
        elif err_pos>0:
            print('objetivo a la derecha')
            ang_vel=err_pos/(self.theta_umb4/2)*self.max_ang
        elif err_pos<0:
            print('objetivo a la izquierda')
            ang_vel=err_pos/(self.theta_umb4/2)*self.max_ang

        print(ref_dist)
        print(self.map)
                

        #finalmente, publicamos la accion de control debidamente saturada
        vel_action=velocity_cmd()
        vel_action.lineal=max(-self.max_lin,min(self.max_lin,lin_vel))
        vel_action.angular=max(-self.max_ang,min(self.max_ang,ang_vel))

        print(lin_vel)
        #solo se publicara la accion de control si la bandera esta alzada, es decir
        #si le hemos comunicado al nodo por medio de un servicio que queremos que persiga al objeto
        #si no, simplemente se detiene
        if self.control_flag:
            self.action_pub.publish(vel_action)
        else:
            vel_action.angular=0.0
            vel_action.lineal=0.0
            self.action_pub.publish(vel_action)

    def start_chase(self, request):
        #este es el handler del servicio que inicia la marcha
        #este servicio permite iniciar la persecucucion con el color que elijamos
        self.control_flag=request.start
        if self.control_flag:
            rospy.loginfo('CENTRAL_NODE: Started actuaction.')
        else:
            rospy.loginfo('CENTRAL_NODE: Actuaction ended.')


        if request.color:
            if request.color == 'r' or request.color == 'g' or request.color == 'b':
                # si es un color valido, se cambia el parametro
                self.target_color = request.color  
            else:
                # si no es valido, se imprime un mensaje de error y no se cambia nada
                rospy.logerr('CENTRAL_NODE: Unrecogniced color input.') 
                return False 

        return True

    def change_params(self, request):
        # este es el handler del servicio de configuracion. Cambia los parametros definidos en la llamada al servicio

        # si se ha especificado un nuevo color, se evalua la expresion
        if request.color:
            if request.color == 'r' or request.color == 'g' or request.color == 'b':
                # si es un color valido, se cambia el parametro
                self.target_color = request.color  
            else:
                # si no es valido, se imprime un mensaje de error y no se cambia nada
                rospy.logerr('CENTRAL_NODE: Unrecogniced color input.') 
                return False 

        # con los limites es mas simple, si son superiores a 0.1 se consideran validos y se cambian
        if request.limits.lineal > 0.1:
            self.max_lin = request.limits.lineal
        
        if request.limits.angular > 0.1:
            self.max_ang = request.limits.angular

        #finalmente se imprime en consola un aviso de que los parametros se han cambiado
        rospy.loginfo('CENTRAL_NODE: Params changed.\nCurrent params are:\ncolor: %s\tlinvel limit: %.2f\tangvel limit: %.2f',self.target_color, self.max_lin, self.max_ang)

        return True

    def start_node(self):
        # creamos el servicio que permite cambiar los parametros de seguimiento
        self.param_service=rospy.Service('/jetbot/config',chase_conf, self.change_params)

        # creamos el servicio que permite iniciar la accion
        self.param_service=rospy.Service('/jetbot/start', start_srv, self.start_chase)

        # creamos el publisher para publicar la accion de control
        self.action_pub=rospy.Publisher('/jetbot/cmd_vel', velocity_cmd, queue_size=10)

        # nos suscribimos al topic con los datos extraidos de la camara
        self.img_sub=rospy.Subscriber('/jetbot/camera_data', camera_data, self.process_data)

if __name__ == "__main__":
    # creamos el nodo e inicializamos todo
    rospy.init_node('central_node')
    rospy.loginfo('CENTRAL_NODE: Node started.')
    obj = central_node()
    obj.start_node()
    rospy.spin()
