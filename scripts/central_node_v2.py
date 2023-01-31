#!/usr/bin/env python
# nodo encargado de tomar decisiones en base a los datos de la imagen,
# actuar acorde sobre los actuadores y ofrecer un servicio al usuario
# para alterar el comportamiento del sistema durante el funcionamiento

from time import sleep
import rospy
from math import floor
import json

#importamos los mensajes
from jetbot_color_chaser.msg import camera_data, velocity_cmd

#importamos los servicio
from jetbot_color_chaser.srv import chase_conf, start_srv

class centralNode():
    def __init__(self):
        # --- Lectura de parametros ---
        # parametro que nos indica si estamos en simulacion o no
        self.sim_mode = rospy.get_param('~sim', default=False)
        self.verbose = rospy.get_param('~enable_verbose', default=False)

        # parametros de seguimiento 
        self.max_lin = rospy.get_param('~max_lin_vel', default=1)    #velocidad lineal maxima a la que se persiguen objetos
        self.max_ang = rospy.get_param('~max_ang_vel', default=1)      #velocidad angular maxima a la que se persiguen objetos
        self.target_color = 'r'  #color que se persigue, de serie es rojo, todos los demas se evitan
        self.ref_area = rospy.get_param('~area_ref', default=3000) 
        
        #umbrales para la creacion del mapa
        self.umb4_theta=rospy.get_param('~camera_width',default=256)
        self.umb0_theta=round(self.umb4_theta/5)
        self.umb1_theta=self.umb0_theta*2
        self.umb2_theta=self.umb0_theta*3
        self.umb3_theta=self.umb0_theta*4

        #margen que le damos a la zona en la que consideramos "aceptable" el obstaculo
        self.ref_margin=rospy.get_param('~area_margin',default=2000)

        # --- Declaracion de variables ---
        #para que el robot tenga "memoria", es necesario que recuerde las posiciones anteriores. Como 
        #situacion inicial supondremos que el objetivo esta en algun lugar a nuestra derecha muy lejos,
        #de modo que el robot gire sobre si mismo
        self.target_theta_ant = self.umb2_theta
        self.target_dist_ant = 3
        self.target_area_ant = 1

        # bandera para indicar cuando queremos que el control actue
        self.control_flag = False
        # bandera para indicar que el target se ha perdido. asumimos que no empezamos viendo el target
        self.target_lost = True
        
    def processData(self, data):
        #primero leemos la informacion referente al color que deseamos seguir y la procesamos para enviar la senial de control
        # esto se realiza cada vez que recibimos imagenes de la camara, es decir, con una frecuencia de unos 10 Hz

        #si un objeto esta en el borde, es probable que no estemos observando la totalidad de sus pixeles.
        #para evitar este problema, los objetos en el borde se consideraran desaparecidos, a costa de perder field of vision
        #Por tanto, si los objetos estan en el borde, sus parametros pasan a valer -1, -1, como si hubieran sido perdidos
        if data.red.center>self.umb4_theta-4 or data.red.center<5:
            red_area=-1
            red_center=-1
            red_dist=-1
        else:
            red_area=data.red.area
            red_center=data.red.center
            red_dist=data.red.distance
        if data.green.center>self.umb4_theta-4 or data.green.center<5:
            green_area=-1
            green_center=-1
            green_dist=-1
        else:
            green_area=data.green.area
            green_center=data.green.center
            green_dist=data.green.distance
        if data.blue.center>self.umb4_theta-4 or data.blue.center<5:
            blue_area=-1
            blue_center=-1
            blue_dist=-1
        else:
            blue_area=data.blue.area
            blue_center=data.blue.center
            blue_dist=data.blue.distance
            
        
        # dependiendo del parametro "target_color", nuestro objetivo y nuestros obstaculos seran distintos
        # realmente nos orientamos con coordenadas cartesianas y, como tal, definimos la posicion de los objetos
        # con dos parametros, theta, area y distancia.
        # theta se define como la posicion en x del centro geometrico del objeto, parametro que nos obtenemos directamente
        # de camera data
        # area lo obtenemos a partir del area que ocupa el objeto en la imagen y se usa para obtener la distancia del objeto a la camara. A menos area tenga un objeto, mas lejos esta. En principio usamos siempre la distancia, expresada en una escala del 0 al 3, pero el area es util en ocasiones en las que se requiere mas preecision para comparar distancias
        if self.target_color == 'r':
            target_theta = red_center
            target_area = red_area
            target_dist = red_dist
            item1_theta = green_center
            item1_area = green_area
            item1_dist = green_dist
            item2_theta = blue_center
            item2_area = blue_area
            item2_dist = blue_dist
        elif self.target_color == 'g':
            target_theta = green_center
            target_area = green_area
            target_dist = green_dist
            item1_theta = red_center
            item1_area = red_area
            item1_dist = red_dist
            item2_theta = blue_center
            item2_area = blue_area
            item2_dist = blue_dist
        elif self.target_color == 'b':
            target_theta = blue_center
            target_area = blue_area
            target_dist = blue_dist
            item1_theta = red_center
            item1_area = red_area
            item1_dist = red_dist
            item2_theta = green_center
            item2_area = green_area
            item2_dist = green_dist

        # una vez tenemos la informacion, comenzamos el control
        # si hay algun obstaculo en nuestro encuadre debemos tenerlo en cuenta para decidir la trayectoria.
        # se considerara siempre el obstaculo mas cercano, suponiendo que los obstaculos estan suficientemente separados entre si
        # Para esquivarlo, construimos un mapa local
        # el mapa local es una cuadricula que separa el espacio que podemos observar en la camara y que nos ayuda a tomar decisiones para esquivar
        # en el se marcaran con un 1 las zonas intransitables y con un 2 el objetivo
        # el mapa cubre el campo de vision de la camara y dos zonas que representan aquello que no podemos ver, pero que sabemos
        # que se encuentra en esa zona.
        # Cuando perdemos de vista al objetivo, suponemos que esta en esa zona

        '''
                posicion del robot
                         |
                        \ /
                         X
            map=[[0|0,0,0,0,0|0],      CERCA
                  [0|0,0,0,0,0|0],      
                  [0|0,0,0,0,0|0],      
                  [0|0,0,0,0,0|0]]      LEJOS
        zona virtual| visible |zona virtual
                         
    
        '''
        map=[[0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0]]

        # si hemos perdido de vista al objetivo, consultamos la ultima localizacion conocida y buscamos por ahi
        if target_theta==-1 and self.target_theta_ant!=-1:
            self.target_lost = True
            if self.target_theta_ant < (self.umb4_theta / 2):
                target_theta= - self.umb0_theta/2
            else:
                target_theta= self.umb4_theta + self.umb0_theta/2
            target_dist=3
            target_area=1
        else:
            self.target_lost = False

        #primero registramos la posicion de nuestro objetivo en el mapa local.
        # los parametros theta se discretizan definiendo unos umbrales segun la definicion de la camara,
        #  En el caso de que no conozcamos la posicion concreta
        # lo guardaremos en las posiciones 0 o 6, correspondientes a las zonas virtuales de nuestro mapa que no podemos ver con la camara
        t_x = int(floor(target_theta/self.umb0_theta)+1)
        if t_x<0: t_x=0
        if t_x>6: t_x=6
        # la y queda definida por la distancia
        t_y = target_dist

        # ahora hemos de comprobar si existe un obstaculo y que ademas es el obstaculo mas cercano
        if (item1_area>target_area) and (item1_theta!=-1) and (item1_area>=item2_area):
            obs_area=item1_area
            obs_dist=item1_dist
            obs_theta=item1_theta
        elif (item2_area>target_area) and (item2_theta!=-1) and (item1_area<item2_area):
            obs_area=item2_area
            obs_dist=item2_dist
            obs_theta=item2_theta
        else:
            obs_area=-1
            obs_dist=-1
            obs_theta=-1

        # si existe obstaculo, lo registramos enn el mapa
        if obs_area!=-1:
            obs_y = obs_dist

            obs_x = int(floor(obs_theta/self.umb0_theta)+1)
            if obs_x<2: obs_x=1
            if obs_x>4: obs_x=5

            #para asegurarnos de esquivar bien el obstaculo, suponemos los obstaculos mas anchos de lo que son en realidad
            map[obs_y][obs_x]=1
            map[obs_y][obs_x-1]=1
            map[obs_y][obs_x+1]=1
            if obs_x==3 and obs_y==0: #si esta en el centro o cerca lo consideramos aun mas ancho
                map[obs_y][obs_x-2]=1
                map[obs_y][obs_x+2]=1
        

        map[t_y][t_x]=2 #marcamos el objetivo en el mapa

        # decidimos el camino a seguir
        if (obs_theta==-1) or (obs_area < target_area):
            #si no se encuentran obstaculos en el camino o el objetivo esta mas creca, directaemente seguimos al objetivo, sin necesidad de llamar al planificador
            ref_vel_ang=target_theta
            ref_vel_lin=target_area
        else:
            #si hay obstaculos o ninguno nos bloquea completamente el camino, planeamos el camino para esquivar
            #para ello hacemos una sencilla serie de comparaciones
            #nuestro medidor de como de bueno es un camino es lo mucho que nos aleja del objetivo en huecos del mapa. 
            #Esto se mide con la metrica coste, cuantas casillas hay que girar respecto al objetivo. 
            #Se inicializa con un valor alto
            cost=10
            #Tambien se inicializa una variable que define el camino a seguir. Si no se encuentra camino, el robot debera girar en circulos
            tray=6
            #para cada fila del mapa, buscamos que hayan obstaculos
            for row in range(t_y):
                if any(map[row]):
                    #si encontramos obstaculos, miramos columna a columna de la fila buscando elementos
                    for col in range(len(map[row])):
                        #si encontramos un hueco libre por el que pasar, comprobamos como de bueno es y si es el mejor
                        #encontrado hasta la fecha, lo guardamos como la nueva trayectoria a seguir
                        if map[row][col]==0 and abs(t_x-col)<cost:
                            cost=abs(t_x-col)
                            tray=col
                    break

            # si el target esta perdido, el giro debe ser lento, por lo que limitamos las trayectorias
            if self.target_lost:    
                if tray < 3: tray = 2
                else: tray = 4

            # a partir de la posicion del mapa indicada por el planificador, obtenemos la posicion a la que hemos de girar
            ref_vel_ang=self.umb0_theta*tray - self.umb0_theta/2

            

            

            if (map[0][3]==1) and (map[0][4]==1) and (map[0][2]==1):
                #si tenemos un obstaculo bloqueandonos completamente, nos paramos mientras seguimos girando en busca del objetivo
                ref_vel_lin=0
            elif tray<2 or tray>4 or self.target_lost:
                #si un obstaculo nos obliga a girar o el target esta perdido, el movimiento se hara a menor velocidad
                ref_vel_lin=obs_area
            else:
                #si la via esta libre, seguimos avanzando hacia el objetivo
                ref_vel_lin=target_area
                
        # el error en posicion se calcula teniendo en cuenta que el robot debe mover el centro de su vision a donde la referencia le marca
        err_pos=(self.umb4_theta/2)-ref_vel_ang

        # finalmente almacenamos los valores obtenidos del objetivo para usarlos en la siguiente iteracion
        self.target_theta_ant=target_theta
        self.target_area_ant=target_area
        
        # a continuacion hemos de calcular la accion del control
        
        # si el objeto esta a una distancia dentro de un cierto rango, el robot se para.
        if ref_vel_lin >= self.ref_area-self.ref_margin and ref_vel_lin <= self.ref_area+self.ref_margin or ref_vel_lin==0:
            lin_vel=0
            if self.verbose: print('CENTRAL_NODE: Objetivo cerca o bloqueado')
        else:
            lin_vel=((self.ref_area-
            )/ref_vel_lin)
            if self.verbose: print('CENTRAL_NODE: Objetivo lejos')

        # si el objetivo esta, de forma aproximada, en el centro de nuestra vision, no giramos.
        if abs(err_pos)<10:
            if self.verbose: print('CENTRAL_NODE: Trayectoria recta')
            ang_vel=0
        elif err_pos>0:
            if self.verbose: print('CENTRAL_NODE: Trayectoria a la izquierda')
            ang_vel=err_pos/(self.umb4_theta/2)*self.max_ang
        elif err_pos<0:
            if self.verbose: print('CENTRAL_NODE: Trayectoria a la derecha')
            ang_vel=err_pos/(self.umb4_theta/2)*self.max_ang

        if self.verbose:
            print("CENTRAL_NODE: Mapa generado:")
            print(map[3])
            print(map[2])
            print(map[1])
            print(map[0])
                

        #finalmente, publicamos la accion de control debidamente saturada
        vel_action=velocity_cmd()
        vel_action.lineal=max(-self.max_lin,min(self.max_lin,lin_vel))
        vel_action.angular=max(-self.max_ang,min(self.max_ang,ang_vel))

        #solo se publicara la accion de control si la bandera esta alzada, es decir
        #si le hemos comunicado al nodo por medio de un servicio que queremos que persiga al objeto
        if self.control_flag:
            self.action_pub.publish(vel_action)
        else:
            vel_action.angular=0.0
            vel_action.lineal=0.0
            self.action_pub.publish(vel_action)

    def startChase(self, request):
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

    def changeParams(self, request):
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

    def startNode(self):
        # creamos el servicio que permite cambiar los parametros de seguimiento
        self.param_service=rospy.Service('/jetbot/config',chase_conf, self.changeParams)

        # creamos el servicio que permite iniciar la accion
        self.param_service=rospy.Service('/jetbot/start', start_srv, self.startChase)

        # creamos el publisher para publicar la accion de control
        self.action_pub=rospy.Publisher('/jetbot/cmd_vel', velocity_cmd, queue_size=10)

        # nos suscribimos al topic con los datos extraidos de la camara
        self.img_sub=rospy.Subscriber('/jetbot/camera_data', camera_data, self.processData)

if __name__ == "__main__":
    # creamos el nodo e inicializamos todo
    rospy.init_node('central_node')
    rospy.loginfo('CENTRAL_NODE: Node started.')
    obj = centralNode()
    obj.startNode()
    rospy.spin()
