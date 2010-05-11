# -*- coding: utf-8 -*-

import pygame
import Image
import cv
import time

camara = cv.CreateCameraCapture(0)

pygame.init()

ventana = pygame.display.set_mode((640,480))
ventana2 = pygame.display.set_mode((640,480))

pygame.display.set_caption('Captura desde cámara')

screen = pygame.display.get_surface()

for i in range(5):
    imagen = cv.QueryFrame(camara)
    
tamImagen = cv.GetSize(imagen)

#Crea la estructura de datos para la imagen (relamente crea 2, una RGB y otra en 
#escala de grises.
imagenRGB = cv.CreateImage(tamImagen, cv.IPL_DEPTH_8U, 3)
imagenGRIS = cv.CreateImage(tamImagen,cv.IPL_DEPTH_8U, 1)

#Convierte la captura a formato CV RGB y lo guarda en imagenRGB
cv.CvtColor(imagen,imagenRGB,cv.CV_BGR2RGB)
#Convierte la captura a escala de grises y lo guarda en imagenGRIS
cv.CvtColor(imagen,imagenGRIS,cv.CV_BGR2GRAY)

#Convierte la imagen RGB a otra en formato PIL (a través de string)

imagenPIL = Image.fromstring('RGB', cv.GetSize(imagenRGB),
                                 imagenRGB.tostring())

try:
    del(camara)
except Exception, error:
    print error 

#Convierte la imagen PIL a otra en formato PYGAME

imagenpg = pygame.image.frombuffer(imagenPIL.tostring(),imagenPIL.size,imagenPIL.mode)

#Extrae los puntos SURF
(puntos, descriptores) = cv.ExtractSURF(imagenGRIS, None, cv.CreateMemStorage(), (0,3000,3,4))
  
#Añade una representación de los puntos SURF a la imagen antes de
#mostrarla

# DRAW KEYPOINT
for ((x, y), laplacian, size, dir, hessian) in puntos:
    radio = size*1.2/9.*2
    #print "radioOld: ", int(radio)
    color = (255, 0, 0)
    if radio < 3:
        radio = 2
        color = (0, 255, 0)
    #print "radioNew: ", int(radio)
    pygame.draw.circle(imagenpg, color, (x, y), radio, 2)

#Añade la imagen a la superficie de dibujo de pygame

screen.blit(imagenpg,(0,0))

#Actualiza superficie de dibujo.

pygame.display.flip()

#Bucle de espera

terminado = False
while not terminado:
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            terminado = True
