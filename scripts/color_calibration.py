#!/usr/bin/env python

import cv2
from time import sleep
import numpy as np
# libreria para manejar la picam instalada en el jetbot
import simplecamera
import json

# inicializamos la camara
cap = simplecamera.start_camera()
# capturamos imagen

win_sliders = cv2.namedWindow('Selector')
win_img = cv2.namedWindow('Img')


def nothing(useless):
    pass


cv2.createTrackbar('LowH', 'Selector', 0, 255, nothing)

cv2.createTrackbar('HighH', 'Selector', 0, 255, nothing)

cv2.createTrackbar('LowS', 'Selector', 0, 255, nothing)

cv2.createTrackbar('HighS', 'Selector', 0, 255, nothing)

cv2.createTrackbar('LowV', 'Selector', 0, 255, nothing)

cv2.createTrackbar('HighV', 'Selector', 0, 255, nothing)

# kernel a usar en los metodos morfologicos
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))

thresholds = list()


while True:
    re, img = cap.read()

    if not re:
        print('Error with image capture.')
        continue
        
    img=cv2.medianBlur(img,5)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_hsv = np.array([cv2.getTrackbarPos('LowH','Selector'),cv2.getTrackbarPos('LowS','Selector'),cv2.getTrackbarPos('LowV','Selector')])
    higher_hsv = np.array([cv2.getTrackbarPos('HighH','Selector'),cv2.getTrackbarPos('HighS','Selector'),cv2.getTrackbarPos('HighV','Selector')])

    mask = cv2.inRange(hsv,lower_hsv,higher_hsv)
    # aplicamos metodos morfologicos para limpiar la mascara
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    img = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow('Img', img)

    suma= np.sum(np.sum(mask))/255
    print suma

    k = cv2.waitKey(100) & 0xFF
    if k == 113 or k == 27: #esc o q
        break

    if k == 115: #s
        with open('thresholds.json', 'w') as outf:
            json.dump(thresholds, outf)
        thresholds = list()

    if k == 114: #r
        thresholds.append({
            'lower_hsv': lower_hsv,
            'higher_hsv': higher_hsv,
            'color': 'red'
        })

    if k == 103: #g
        thresholds.append({
            'lower_hsv': lower_hsv,
            'higher_hsv': higher_hsv,
            'color': 'green'
        })
        
    if k == 98: #b
        thresholds.append({
            'lower_hsv': lower_hsv,
            'higher_hsv': higher_hsv,
            'color': 'blue'
        })

''' RED: 0-10, 100-185, 90-255
GREEN: 70-85, 150-255, 75-175, 
BLUE: 100-120, 85-175, 50-220, 15000-5000-2000'''




	
	
