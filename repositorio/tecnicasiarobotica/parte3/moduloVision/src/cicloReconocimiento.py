# -*- coding: utf-8 -*-

import numpy as np
import sys
import VisioLib
import pygame
import Image
import cv
from VisioLib import *

NUMERO_MARCADORES = 4
PATH_MARCADORES = '../marcas/'

try:
    selcam = sys.argv[1]
except Exception, error:
    selcam = 0

# Genera la base de datos de marcadores
 
marcadores = []

for i in range(NUMERO_MARCADORES):
    kp = np.load(PATH_MARCADORES + 'KEYP_IMG0' + str(i) + '.npy')
    desc = np.load(PATH_MARCADORES + 'DESC_IMG0' + str(i) + '.npy')
    #ruta, etiqueta, keypoints, descriptores
    marcadores.append([PATH_MARCADORES + str(i) + 'f.jpg', 'Marcador ' + str(i),kp,desc])

if selcam == 0:
    # Obtiene los puntos de interés, y sus descriptores, a través de la cámara
    camara = cv.CreateCameraCapture(0)
    for i in range(5):
        imagen = cv.QueryFrame(camara)
else:
    imagen = cv.LoadImageM('../marcas/' + sys.argv[1] + 'f.jpg')   

tamImagen = cv.GetSize(imagen)
    
    #Crea la estructura de datos para la imagen (relamente crea 2, una RGB y otra en 
    #escala de grises.
imagenRGB = cv.CreateImage(tamImagen, cv.IPL_DEPTH_8U, 3)
imagenGRIS = cv.CreateImage(tamImagen,cv.IPL_DEPTH_8U, 1)
    #Convierte la captura a formato CV RGB y lo guarda en imagenRGB
cv.CvtColor(imagen,imagenRGB,cv.CV_BGR2RGB)          
    #Convierte la captura a escala de grises y lo guarda en imagenGRIS
cv.CvtColor(imagen,imagenGRIS,cv.CV_BGR2GRAY)

try: #Libera el dispositivo de captura.
    del(camara)
except Exception, error:
    pass

    #Extrae los puntos SURF
(puntosInteres, descriptores) = cv.ExtractSURF(imagenGRIS, None, cv.CreateMemStorage(), (0,5000,4,1))

descriptoresObservados = np.array(descriptores)

#print len(descriptoresObservados), len(marcadores[0][3])

#El siguiente paso es establecer un orden de preferencia en los marcadores almacenadas
#según la semejanza aproximada con la observación
score = [0 for i in range(NUMERO_MARCADORES)]

#dMin = 999999999.0
#dMin2 = 999999998.0
#masCercano = -1
#masCercano2 = -1

factorSemejanza = 0.95

for nmar in range(NUMERO_MARCADORES): #Para cada marcador almacenado
    for ndesc in range(len(marcadores[nmar][3])): #Para cada descriptor de la marca.
        rank = []
        for nosb in range(len(descriptoresObservados)):
            d = distanciaDescriptores(marcadores[nmar][3][ndesc],descriptoresObservados[nosb])
            """print len(marcadores[nmar][3][ndesc]),len(descriptoresObservados[nosb])
            if d < dMin:
                masCercano = nosb
                dMin = d 
            elif d < dMin2:
                masCercano2 = nosb
                dMin2 = d"""
            rank = rank + [d] 
        rank.sort()
        dMin= rank[0]
        dMin2 = rank[1]
        dMax = rank[len(rank)-1]
        #print dMin,dMin2,dMin/dMin2,dMin2*factorSemejanza
        #if dMin < factorSemejanza*dMin2:
        #    score[nmar] = score[nmar]+1+1-abs(dMin2-dMin)/dMin2
        if dMin/dMax < factorSemejanza*dMin2/dMax:
            score[nmar] = score[nmar]+1#+2-abs(dMin2-dMin)/dMin2
            
print score;


    #Convierte la imagen RGB a otra en formato PIL (a través de string)

imagenPIL = Image.fromstring('RGB', cv.GetSize(imagenRGB),
                                 imagenRGB.tostring())

#Muestra descriptores obtenidos frente a los almacenados
#print "Introduce el número de máscara presentada a la cámara"
#nm = int(raw_input())

imagenpg = pygame.image.frombuffer(imagenPIL.tostring(),imagenPIL.size,imagenPIL.mode)
for ((x, y), laplacian, size, dir, hessian) in puntosInteres:
    radio = size*1.2/9.*2
    #print "radioOld: ", int(radio)
    color = (255, 0, 0)
    if radio < 3:
        radio = 2.0
        color = (0, 255, 0)
    #print "radioNew: ", int(radio)
    pygame.draw.circle(imagenpg, color, (x, y), radio, 2.0)
"""
for k in range(len(marcadores[nm][2])):
    #print marcadores[nm][2][k][0],marcadores[nm][2][k][1]
    (x,y) = (marcadores[nm][2][k][0],marcadores[nm][2][k][1])
    color = (0, 0, 255)
    radio = 2
    pygame.draw.circle(imagenpg, color, (x, y), radio, 2)
"""

pygame.init()
ventana = pygame.display.set_mode(tamImagen)
pygame.display.set_caption('Comparación de descriptores')
superficieDibujo = pygame.display.get_surface()

superficieDibujo.blit(imagenpg,(0,0))
pygame.display.flip()

#Bucle de espera

terminado = False
while not terminado:
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            terminado = True
