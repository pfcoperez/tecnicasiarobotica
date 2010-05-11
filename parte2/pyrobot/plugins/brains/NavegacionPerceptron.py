#! /usr/bin/python
# -*- coding: utf-8 -*- 

#    This file is part of PyNavegacion.
#
#    PyNavegacion is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Foobar is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pablo Francisco Pérez Hidalgo (pfcoperez@gmail.com)
#
# Este módulo de navegación o "Brain" utilizará una red 
# neuronal tipo "Perceptrón" con 9 entradas:
#         
#         *sonar[0]
#         *sonar[1]
#         *sonar[2]
#         *sonar[3]
#         *sonar[4]
#         *sonar[5]
#         *sonar[6]
#         *sonar[7]
#         *Teta (Desviación de encabezado respecto objetivo)
#         *b
#
# Al ser monocapa, tendrá una única salida comprendida en [-1,1], intervalo
# en el cual se normalizan giros desde los 360º hacia la derecha hasta (en el caso
# de -1) hasta 360º hacia la izquierda (en el caso de 1).
#
from pyrobot.brain import Brain  
from Perceptron import *

import random
import math
   

class NavegaEntrena(Brain):  
   

   def ruta(self):
      if len(self.rutasUsadas) == len(self.testPath):
         self.rutasUsadas = []
      x = int(random.uniform(0,len(self.testPath)))
      if x >= len(self.testPath):
         x = len(self.testPath)-1
      while x in self.rutasUsadas:
         x = int(random.uniform(0,len(self.testPath)-1))
      self.rutasUsadas + [x]
      return x

   def calculaVelocidadAvance(self,vgiro):
      return 0.1+0.9*(1-abs(vgiro))

   def setup(self):
      self.rutasUsadas = []
      #self.flag = True
      self.maxTicksPermitidos = 200
      #self.vt = 0.5 #Velocidad de translación
      self.distMin = 0.3 #Distancia del robot al objetivo para que se considere
      # que este ha llegado a la meta.
      self.desv_tipica = 0.6
     
      self.bufferTicks = 0

      self.N = 35 #Tamaño de la población (el primer individuo es el 0) 
      self.tasaMutantes = 0.9  #Proporción de la población que muta \in [0,1]

      self.pesos = [-0.25830320437434307, 1.160526501403915, -0.15551005292679421, 1.2578377038143795, -0.55707372041035363, -2.9552247878192888, -1.6809527155003046, -2.2014578204055359, -1.5535152460077812, 1.2828000172631631]

      #Pares ORIGEN-DESTINO para pruebas:

      self.testPath = [((0.30,0.30,0),(2.80,2.80,0)),((2.80,0.70,0),(0.60,2.0,0)),((1.6,0.3,0),(1.84,2.9,0)),((2.9,1.75,0),(0.55,1.75,0)),((1.67,0.6,0.0),(1.67,2.8,0.0))]
      self.npath = 4
      self.posInicial = (self.testPath[self.npath][0][0], self.testPath[self.npath][0][1], self.testPath[self.npath][0][2])
      self.robot.simulation[0].setPose(0,self.posInicial[0],self.posInicial[1],self.posInicial[2])
      self.calculadorNeuronal = PerceptronMonocapa(self.pesos,w2)


   def calculaPos(self):
      x = self.posInicial[0] - self.robot.y #Conocimiento del robot...
      y = self.posInicial[1] + self.robot.x #... sobre su posición.
      t = (self.posInicial[2] + self.robot.th+90)%360
      return (x, y, t)

   def step(self):
   
      if self.bufferTicks > self.maxTicksPermitidos:
         print "Ha superado recorrido máximo"
         self.engine.pleaseStop()
      elif self.robot.stall == 1:
         print "CHOQUE"
         self.engine.pleaseStop()
      else:
         self.bufferTicks = 1+self.bufferTicks
         x, y, t = self.calculaPos()
         distancia = +math.sqrt((x-self.testPath[self.npath][1][0])**2+(y-self.testPath[self.npath][1][1])**2)
         #print "Distancia a objetivo> "  + str(distancia)
         #print "Pos = (" + str(x) + ", " + str(y) + ")  Obj = (" + str(self.testPath[self.npath][1][0]) + ", " + str(self.testPath[self.npath][1][1]) + ")"
         if distancia <= self.distMin: #Prepara siguientes iteraciones
         #En lo que respecta al control del robot, hay que pararlo
            print "Objetivo Alcanzado> Origen->" + str(self.testPath[self.npath][0][0:1]) + " Destino->" + str(self.testPath[self.npath][1][0:1])
               #self.robot.move(0,0)
            #En lo que respecta al entrenamiento, debe gestionarlo
            self.engine.pleaseStop()
         else: #Utiliza red neuronal para decidir giro
            teta = t-(360.0/(2.0*math.pi))*(math.atan2((self.testPath[self.npath][1][1]-y),(self.testPath[self.npath][1][0]-x))%(2*math.pi))
            #print "Theta: " + str(teta) + "º"; 
            sensores = [self.robot.sonar[0][i].distance() for i in range(0,8)]
            sensoresNorm = sensores
            #sensoresNorm = [sensores[i]/max(sensores) for i in range(0,8)]
            velGiro = self.calculadorNeuronal.transferencia(sensoresNorm+[teta/360.0]);
            #print "V GIRO Neuronal: " + str(velGiro)
            #print velGiro
            #velGiro = 0.9*velGiro #Así evito giros muy bruscos
            self.robot.move(self.calculaVelocidadAvance(velGiro), velGiro)
     
def INIT(engine):  
   assert (engine.robot.requires("range-sensor") and
           engine.robot.requires("continuous-movement"))
   return NavegaEntrena('NavegaEntrena', engine)  

