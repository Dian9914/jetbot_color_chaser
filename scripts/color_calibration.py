#!/usr/bin/env python3

import cv2
import numpy as np
import json
import os.path as osp
import os
import argparse
import time

# libreria para manejar la picam instalada en el jetbot
import simplecamera

# funcion para leer los argumentos que se le dan por consola a python
def parse_args():
    parser = argparse.ArgumentParser(description='Useful script used to calibrate the color detection.')
    parser.add_argument('--mode', default= 'jetbot', help ='Switch between "jetbot" mode or "opencv" mode to access the camera.')
    parser.add_argument('--out_dir', default='./config/', help='Directory to save the output')
    parser.add_argument('--post_proc', default=False, help='Post process the image ussing filters. This is deprecated because the pipeline already preprocess the image.')
    args = parser.parse_args()
    return args


def segmentate(hsv,lower_hsv,higher_hsv):
    if higher_hsv[0] > 179:
        mask1 = cv2.inRange(hsv,np.array(lower_hsv, dtype=np.int64),np.array([int(179), int(higher_hsv[1]), int(higher_hsv[2])]))
        mask2 = cv2.inRange(hsv,np.array([int(0), int(lower_hsv[1]), int(lower_hsv[2])]),np.array([int(higher_hsv[0]-179), int(higher_hsv[1]), int(higher_hsv[2])]))
        mask = cv2.bitwise_or(mask1,mask2)
    else:
        mask = cv2.inRange(hsv,lower_hsv,higher_hsv)

    return mask

# funcion callback que se ejecuta cada vez que se hace un click izquierdo en algun punto de la imagen. Esta funcion avisa de que se ha clickado
# y devuelve la posicion del cursor en el momento del click
def register_click(event,x,y,flags,param):
    global lclick
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print('click registered')
        mouseX,mouseY = x,y
        lclick=True
        print(lclick)

def main():
    global lclick
    global mouseX,mouseY
    # guardamos los argumentos en la variable arg
    args = parse_args()

    # si no existe la carpeta especificada para guardar el output, la creamos
    if not osp.isdir(args.out_dir):
                os.mkdir(args.out_dir)

    # inicializamos la camara, con simplecamera si estamos en el jetbot o directamente con opencv si esta disponible
    if args.mode=='jetbot':
        cap = simplecamera.start_camera()
    else: 
        cap = cv2.VideoCapture(0)
    print("Camera initialized!")

    # creamos las ventanas y asignamos el callback para el uso del raton
    win_sliders = cv2.namedWindow('Selector')
    win_img = cv2.namedWindow('Pixel_Selection')
    win_output = cv2.namedWindow('Segmentation')
    cv2.setMouseCallback('Pixel_Selection',register_click)

    # inicializamos variables
    lclick = False
    hsv_values = list()
    h_values = list()
    v_values = list()
    s_values = list()

    parameters = dict()
    parameters['red'] = dict()
    parameters['blue'] = dict()
    parameters['green'] = dict()

    # creamos las trackbar y les asignamos un callback
    # funcion callback que se ejecuta cada vez que tocamos una trackbar. No hace nada
    def nothing(useless):
        pass

    cv2.createTrackbar('LowH', 'Selector', 0, 255, nothing)
    cv2.createTrackbar('HighH', 'Selector', 255, 255, nothing)
    cv2.createTrackbar('LowS', 'Selector', 0, 255, nothing)
    cv2.createTrackbar('HighS', 'Selector', 255, 255, nothing)
    cv2.createTrackbar('LowV', 'Selector', 0, 255, nothing)
    cv2.createTrackbar('HighV', 'Selector', 255, 255, nothing)

    # kernel a usar en los metodos morfologicos usados para limpiar la mascara usada
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))

    while True:
        # tomamos lectura de la camara. "re" es una bandera que nos indica si hemos obtenido la imagen correctamente o no. Si "re" es False, 
        # imprimiremos un mensaje de error
        re, img = cap.read()
        if not re:
            print('Error with image capture.')
            break


        #obtenemos los umbrales de segmentacion. Para ello se pueden usar dos metodos: modificar las trackbars a mano o 
        # clicar en la imagen en puntos cuyo color queramos mantener en la segmentacion. El programa esta pensado para usar ambos metodos de forma complementaria,
        # utilizando primero los clicks para seleccionar el objeto a grosso modo y luego ajustando con las trackbars para mayor precision
        lower_hsv = np.array([cv2.getTrackbarPos('LowH','Selector'),cv2.getTrackbarPos('LowS','Selector'),cv2.getTrackbarPos('LowV','Selector')])
        higher_hsv = np.array([cv2.getTrackbarPos('HighH','Selector'),cv2.getTrackbarPos('HighS','Selector'),cv2.getTrackbarPos('HighV','Selector')])
        if lclick:
            print(lclick)
            lclick = False
            print(hsv[mouseY,mouseX])
            click_values = hsv[mouseY,mouseX]
            print(click_values)
            if click_values[0] < 50: click_values[0]=180 + click_values[0]
            print(click_values[0])

            hsv_values.append(click_values)
            h_values = [item[0] for item in hsv_values]
            s_values = [item[1] for item in hsv_values]
            v_values = [item[2] for item in hsv_values]
            if len(h_values)>1: 
                lower_hsv = np.array([min(h_values),min(s_values),min(v_values)])
                higher_hsv = np.array([max(h_values),max(s_values),max(v_values)])
                cv2.setTrackbarPos('LowH', 'Selector', min(h_values)) 
                cv2.setTrackbarPos('HighH', 'Selector', max(h_values)) 
                cv2.setTrackbarPos('LowS', 'Selector', min(s_values)) 
                cv2.setTrackbarPos('HighS', 'Selector', max(s_values)) 
                cv2.setTrackbarPos('LowV', 'Selector', min(v_values)) 
                cv2.setTrackbarPos('HighV', 'Selector', max(v_values))    
        


        if args.post_proc:
            # apilamos un filtro para limpiar el ruido en la imagen
            img=cv2.medianBlur(img,5)


        # convertimos a hsv para la segmentacion
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # obtenemos la mascara de segmentacion

        mask = segmentate(hsv,lower_hsv,higher_hsv)

        # aplicamos metodos morfologicos para limpiar la mascara
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


        # mostramos la imagen capturada en la ventana de "Img"
        cv2.imshow('Pixel_Selection', img)


        # aplicamos la mascara a la imagen capturada y la mostramos en la ventana "Out"
        img = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow('Segmentation', img)


        # finalmente, esperamos 100 milisegundos para esperar un input del teclado. Segun la tecla pulsada se tomaran directas acciones
        k = cv2.waitKey(50) & 0xFF


        if k == 113 or k == 27: #esc o q: termina la ejecucion del programa y lo cierra
            break

        if k == 110: #n: guarda en el diccionario "parametros" el area del objeto segmentado actual, asignando el valor al que ocupa un objeto cercano
            area= np.sum(np.sum(mask))/255
            print(f'Near objects have an area of {area}')
            parameters['near_area']=area
            
        if k == 109: #m: guarda en el diccionario "parametros" el area del objeto segmentado actual, asignando el valor al que ocupa un objeto a medio rango
            area= np.sum(np.sum(mask))/255
            print(f'Mid-range objects have an area of {area}')
            parameters['mid_area']=area

        if k == 102: #f: guarda en el diccionario "parametros" el area del objeto segmentado actual, asignando el valor al que ocupa un objeto lejano
            area= np.sum(np.sum(mask))/255
            print(f'Far objects have an area of {area}')
            parameters['far_area']=area

        if k == 114: #r: guarda en el diccionario "parametros" los umbrales de segmentacion del color rojo
            print('Red saved!')
            parameters['red']['lower_hsv']=lower_hsv.tolist()
            parameters['red']['higher_hsv']=higher_hsv.tolist()
            hsv_values = list()
            h_values = list()
            v_values = list()
            s_values = list()

        if k == 103: #g: guarda en el diccionario "parametros" los umbrales de segmentacion del color verde
            print('Green saved!')
            parameters['green']['lower_hsv']=lower_hsv.tolist()
            parameters['green']['higher_hsv']=higher_hsv.tolist()
            hsv_values = list()
            h_values = list()
            v_values = list()
            s_values = list()
            
        if k == 98: #b: guarda en el diccionario "parametros" los umbrales de segmentacion del color azul
            print('Blue saved!')
            parameters['blue']['lower_hsv']=lower_hsv.tolist()
            parameters['blue']['higher_hsv']=higher_hsv.tolist()
            hsv_values = list()
            h_values = list()
            v_values = list()
            s_values = list()

        if k == 115: #s: guarda el diccionario "parametros" como un archivo json para que el sistema principal lo lea directamente
            print(parameters)
            with open(osp.join(args.out_dir,'parameters.json'), 'w') as outf:
                json.dump(dict(parameters), outf)
            parameters = dict()

        if k == 8: #backspace: borra el ultimo punto de color sobre el que hayamos clicado
            if len(hsv_values)>0:
                hsv_values.pop()
                h_values.pop()
                v_values.pop()
                s_values.pop()
                if len(hsv_values)>1:
                    lower_hsv = np.array([min(h_values),min(s_values),min(v_values)])
                    higher_hsv = np.array([max(h_values),max(s_values),max(v_values)])
                    cv2.setTrackbarPos('LowH', 'Selector', min(h_values)) 
                    cv2.setTrackbarPos('HighH', 'Selector', max(h_values)) 
                    cv2.setTrackbarPos('LowS', 'Selector', min(s_values)) 
                    cv2.setTrackbarPos('HighS', 'Selector', max(s_values)) 
                    cv2.setTrackbarPos('LowV', 'Selector', min(v_values)) 
                    cv2.setTrackbarPos('HighV', 'Selector', max(v_values)) 
                else:
                    cv2.setTrackbarPos('LowH', 'Selector', 0) 
                    cv2.setTrackbarPos('HighH', 'Selector', 255) 
                    cv2.setTrackbarPos('LowS', 'Selector', 0) 
                    cv2.setTrackbarPos('HighS', 'Selector', 255) 
                    cv2.setTrackbarPos('LowV', 'Selector', 0) 
                    cv2.setTrackbarPos('HighV', 'Selector', 255)

    # Liberamos el handle de la camara al terminar el programa
    cap.release()
            

if __name__ == '__main__':
    main()



	
	
