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
#         *sonar[6]
#         *sonar[7]
#         *Teta (Desviación de encabezado respecto objetivo)
#
# Al ser monocapa, tendrá una única salida comprendida en [-1,1], intervalo
# en el cual se normalizan giros desde los 360º hacia la derecha hasta (en el caso
# de -1) hasta 360º hacia la izquierda (en el caso de 1).
#
# La condición de parada estará programada al margen de la red neuronal.
# Este fichero buscará el vector de pesos que mejor se ajuste al entrenamiento
# utilizando un algoritmo genético con operador de muerte y de mutación 
# sin uso de elitismo.
#
# En esta aproximación, lo que se hará es asignar mayor probabilidad de 
# mutación a aquellos individuos con peor fitness. 
#
# En esta versión se incluye elitismo implementado de la siguiente forma:
#  En cada generación de nueva población, antes de mutar todos, aquellos
#  individuos que no son el de mayor fitness serán reemplazados por 
#  valores aleatorios.
from pyrobot.brain import Brain  
from Perceptron import *

import random
import math
   

class NavegaEntrena(Brain):  

   def setup(self):
      #self.flag = True
      self.maxTicksPermitidos = 200
      self.vt = 0.6 #Velocidad de translación
      self.distMin = 0.2 #Distancia del robot al objetivo para que se considere
                         # que este ha llegado a la meta.
      self.desv_tipica = 4.5
      self.N = 12 #Tamaño de la población (el primer individuo es el 0) 
      self.tasaMutantes = 0.125 #Porcentaje de la población que muta \in [0,1]
      self.poblacion = [[random.uniform(-1,1) for i in range(0,9)]
                        for individuo in range(0,self.N)]# Por ahora                                                          # Wi en [-1,1]
      self.fitness_vector = [0.0 for i in range(0,self.N)] #-1 es un fitness sin determinar
      #self.ticks_vector = [0 for i in range(0,self.N)]
      self.g = 200 #Número máximo de generaciones en algoritmo evolutivo.
      self.ga = 0 #Número de generación actual
      #Pares ORIGEN-DESTINO para pruebas:
      self.testPath = [((0.30,0.30,0),(2.80,2.80,0)),((2.80,0.70,0),(0.60,2.85,0)),((1.6,0.6,0),(1.84,2.9,0)),((2.9,1.75,0),(0.55,1.75,0))]
      #Prepara primer cálculo de fitting
      self.npath = 0;
      self.bufferTicks = [0, 0, 0, 0]
      self.ni = 0 #Número de individuo cuyo fitness se está calculando, 
                  #si 'ni' == -1 entonces es que no se está calculando fitness
      self.posInicial = (self.testPath[self.npath][0][0], self.testPath[self.npath][0][1], self.testPath[self.npath][0][2])
      self.robot.simulation[0].setPose(0,self.posInicial[0],self.posInicial[1],self.posInicial[2])
      self.calculadorNeuronal = PerceptronMonocapa(self.poblacion[self.ni],w1)
      

   def generaNuevaPoblacion(self):
      if max(self.fitness_vector) != 0:
         #Busca el individuo con mejor fitness
         mejorFit = max(self.fitness_vector)
         for i in range(0,self.N):
            if self.fitness_vector[i] == mejorFit:
               mejorIndividuo = i;
         #Sustituye a los demas por aleatorios
         for i in range(0,self.N):
            if i != mejorIndividuo or mejorFir <= 0:
               self.poblacion[i] = [random.uniform(-1,1) for j in range(0,self.n)]
         #Realiza cálculos para mutaciones
         fn = [self.fitness_vector[i]/max(self.fitness_vector) for i in range(0,self.N)]
      else:
         fn = [0.0 for i in range(0,self.N)]
      #fn = [self.fitness_vector[i]/max(self.fitness_vector) for i in range(0,self.N)]
      nfn = [1-fn[i] for i in range(0,self.N)]
      probMutacion = [nfn[i]/sum(nfn) for i in range(0,self.N)]#Contine la 
      #longitud de los intervalos diana según la probabilidad de seleccionar
      #cada individuo.

      #Ahora prepara los subintervalos de lanzamiento aleatorio.
      acul = [sum(probMutacion[0:i]) for i in range(0,self.N+1)]
      #print "ACUL" + str(acul)
      #Mutamos tantos fenotipos como indique la tasa de mutación.
      #print "FITNESS:" + str(self.fitness_vector)
      for j in range(0,int(math.ceil(self.tasaMutantes*self.N))):
         mutante = -1
         x = random.uniform(0,1); #Genera número aleatorio
         #y vemos en que subintervalo entre 0 y 1 ha caido:
         if x == 0:
            #print "X: " + str(x) + ", acul: " + str(acul[i-1])
            mutante = 0 # El mutante es el primer individuo de la población
         else:
            for i in range(1,self.N):
               #print "X: " + str(x) + ", acul: " + str(acul[i-1])
               if x > acul[i-1] and x <= acul[i]:
                  mutante = i #El muntante es el individuo 'i'
                  i = self.N+1
            if mutante == -1 and x > acul[self.N-1]:
               mutante = self.N-1
         #Ya se conoce que individuo debe mutar, ahora se muta
         #modidicando su código de pesos neuronales.
         print "El fitness del mutante (" + str(mutante) + ") antes de la mutación era: " + str(self.fitness_vector[mutante])
         for i in range(0,9):
            self.poblacion[mutante][i] = self.poblacion[mutante][i] + random.gauss(0,self.desv_tipica)
      #Se ha generado la nueva población, reinicia ticks
      self.ticks_vector = [0 for i in range(0,self.N)]   
      print "NUEVA POBLACIÓN GENERADA (" + str(float(self.ga)/float(self.g)*100.0) + "%)"
      self.ga = self.ga +1 

   def controlIteraciones(self,choque):
      if choque: #Se ha terminado de probar un individuo
         #x = self.posInicial[0] + self.robot.x #Conocimiento del robot...
         #y = self.posInicial[1] + self.robot.y #... sobre su posición.
         #t = self.posInicial[2] + self.robot.th
         #teta = t-(360.0/(2.0*math.pi))*math.atan2((self.testPath[self.npath][1][1]-y),(self.testPath[self.npath][1][0]-x))
         #self.fitness_vector[self.ni] = 0.001*max(self.bufferTicks)+0.01*(1-teta/360.0);
         self.fitness_vector[self.ni] = 0;
         self.ni = self.ni+1
         if self.ni == self.N: #Se ha caracterizado a toda la población y hay que ir a siguiente generación
            self.ni = -1
         else:
            self.npath = 0;
            self.bufferTicks = [0, 0, 0, 0]
            self.posInicial = (self.testPath[self.npath][0][0], self.testPath[self.npath][0][1], self.testPath[self.npath][0][2])
            self.robot.simulation[0].setPose(0,self.posInicial[0],self.posInicial[1],self.posInicial[2])
            self.calculadorNeuronal.modPesos(self.poblacion[self.ni])
      elif self.npath == 3: #Se ha terminado de probar un individuo
         x = self.posInicial[0] + self.robot.x #Conocimiento del robot...
         y = self.posInicial[1] + self.robot.y #... sobre su posición.
         t = self.posInicial[2] + self.robot.th
         teta = t-(360.0/(2.0*math.pi))*math.atan2((self.testPath[self.npath][1][1]-y),(self.testPath[self.npath][1][0]-x))
         self.fitness_vector[self.ni] = 1/max(self.bufferTicks)+0.05*(1-teta/360.0);
         self.ni = self.ni+1
         if self.ni == self.N: #Se ha caracterizado a toda la población y hay que ir a siguiente generación
            self.ni = -1
         else:
            self.npath = 0;
            self.bufferTicks = [0, 0, 0, 0]
            self.posInicial = (self.testPath[self.npath][0][0], self.testPath[self.npath][0][1], self.testPath[self.npath][0][2])
            self.robot.simulation[0].setPose(0,self.posInicial[0],self.posInicial[1],self.posInicial[2])
            self.calculadorNeuronal.modPesos(self.poblacion[self.ni])
      else: #Hay que probar otra ruta, pone el robot en la nueva posición inicial y marca su nuevo objetivo
         self.npath = self.npath+1; 
         self.posInicial = (self.testPath[self.npath][0][0], self.testPath[self.npath][0][1], self.testPath[self.npath][0][2])
         self.robot.simulation[0].setPose(0,self.posInicial[0],self.posInicial[1],self.posInicial[2])

   def step(self):
      if self.ni >= 0: #Si se está calculando fitness
         if self.bufferTicks[self.npath] > self.maxTicksPermitidos:
            print "Ha superado recorrido máximo"
            self.controlIteraciones(False)
         if self.robot.stall == 1:
            self.controlIteraciones(True) #El parámetro con valor True indica que se ha chocado
         else:
            self.bufferTicks[self.npath] = 1+self.bufferTicks[self.npath]
            x = self.posInicial[0] + self.robot.x #Conocimiento del robot...
            y = self.posInicial[1] + self.robot.y #... sobre su posición.
            t = self.posInicial[2] + self.robot.th
            distancia = math.sqrt((x-self.testPath[self.npath][1][0])**2+(y-self.testPath[self.npath][1][1])**2)
            if distancia <= self.distMin: #Prepara siguientes iteraciones
            #En lo que respecta al control del robot, hay que pararlo
               print "Objetivo Alcanzado> Origen->" + str(self.testPath[self.nPath][0][0:1]) + " Destino->" + str(self.testPath[self.nPath][1][0:1])
               self.robot.move(0,0)
            #En lo que respecta al entrenamiento, debe gestionarlo
               self.controlIteraciones(False) #El parámetro es Falso porque no se ha chocado
            else: #Utiliza red neuronal para decidir giro
               teta = t-(360.0/(2.0*math.pi))*math.atan2((self.testPath[self.npath][1][1]-y),(self.testPath[self.npath][1][0]-x))
               #print "Theta: " + str(teta) + "º"; 
               sensores = [self.robot.sonar[0][i].distance() for i in range(0,8)]
               sensoresNorm = [sensores[i]/max(sensores) for i in range(0,8)]
               velGiro = self.calculadorNeuronal.transferencia(sensoresNorm+[teta/360.0]);
               #print "V GIRO Neuronal: " + str(velGiro)
               velGiro = 0.5*velGiro #Así evito giros muy bruscos
               self.robot.move(self.vt, velGiro)
      else: #Gestión de evolución de la población
         if self.ga < self.g: #Si no se ha alcanzado el número máximo de generaciones, 
                              #sigue iterando sin importar si ya se consiguió llegar
                              #al objetivo debido a que interesa optimizar.
            self.generaNuevaPoblacion()
             #Prepara primer cálculo de fitting
            self.npath = 0;
            self.bufferTicks = [0, 0, 0, 0]
            self.ni = 0 #Número de individuo cuyo fitness se está calculando, 
                  #si 'ni' == -1 entonces es que no se está calculando fitness
            self.posInicial = (self.testPath[self.npath][0][0], self.testPath[self.npath][0][1], self.testPath[self.npath][0][2])
            self.robot.simulation[0].setPose(0,self.posInicial[0],self.posInicial[1],self.posInicial[2])
            self.calculadorNeuronal.modPesos(self.poblacion[self.ni])
         else:
            #Busca el individuo con mejor fitness
            mejorFit = max(self.fitness_vector)
            for i in range(0,self.N):
               if self.fitness_vector[i] == mejorFit:
                  mejorIndividuo = i;
            print "ENTRENAMIENTO FINALIZADO"
            print "------------------------"
            print "Los pesos neuronales del mejor individuo, que ha tenido fitness=" + str(mejorFit) + " son:"
            print self.poblacion[mejorIndividuo]
            self.engine.pleaseStop()
               
      #if self.flag:
      #   print "Población inicial:"
      #   print "------------------"
      #   for individuo in range(0,self.N):
      #      print "Individuo " + str(individuo) + " > " + str(self.poblacion[individuo])
      #   self.flag = False
      #print self.engine.robot.sonar[0][0].value
      #self.robot.move(translation, rotate)

def INIT(engine):  
   assert (engine.robot.requires("range-sensor") and
           engine.robot.requires("continuous-movement"))
   return NavegaEntrena('NavegaEntrena', engine)  

