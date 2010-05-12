# -*- coding: utf-8 -*-

import numpy as np
import pygame
import Image
import cv

NUMERO_MARCADORES = 4
PATH_MARCADORES = '../marcas/'

# Genera la base de datos de marcadores
 
marcadores = []

for i in range(NUMERO_MARCADORES):
    kp = np.load(PATH_MARCADORES + 'KEYP_IMG0' + str(i) + '.npy')
    desc = np.load(PATH_MARCADORES + 'DESC_IMG0' + str(i) + '.npy')
    #ruta, etiqueta, keypoints, descriptores
    marcadores.append([PATH_MARCADORES + str(i) + 'f.jpg', 'Marcador ' + str(i),kp,desc])

# Obtiene los puntos de interés, y sus descriptores, a través de la cámara

camara = cv.CreateCameraCapture(0)

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

try: #Libera el dispositivo de captura.
    del(camara)
except Exception, error:
    print error 

    #Extrae los puntos SURF
(puntosInteres, descriptores) = cv.ExtractSURF(imagenGRIS, None, cv.CreateMemStorage(), (0,3000,3,4))

    #Convierte la imagen RGB a otra en formato PIL (a través de string)

imagenPIL = Image.fromstring('RGB', cv.GetSize(imagenRGB),
                                 imagenRGB.tostring())


#Muestra descriptores obtenidos frente a los almacenados
print "Introduce el número de máscara presentada a la cámara"
nm = int(raw_input())

imagenpg = pygame.image.frombuffer(imagenPIL.tostring(),imagenPIL.size,imagenPIL.mode)
for ((x, y), laplacian, size, dir, hessian) in puntosInteres:
    radio = size*1.2/9.*2
    #print "radioOld: ", int(radio)
    color = (255, 0, 0)
    if radio < 3:
        radio = 2
        color = (0, 255, 0)
    #print "radioNew: ", int(radio)
    pygame.draw.circle(imagenpg, color, (x, y), radio, 2)

for k in range(len(marcadores[nm][2])):
    #print marcadores[nm][2][k][0],marcadores[nm][2][k][1]
    (x,y) = (marcadores[nm][2][k][0],marcadores[nm][2][k][1])
    color = (0, 0, 255)
    radio = 2
    pygame.draw.circle(imagenpg, color, (x, y), radio, 2)
    
"""for (x, y,a,b) in marcadores[nm][2]:
    radio = 2
    #print "radioOld: ", int(radio)
    color = (0, 0, 255)
    print x,y
    #pygame.draw.circle(imagenpg, color, (x, y), radio, 2)"""

pygame.init()
ventana = pygame.display.set_mode((640,480))
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
