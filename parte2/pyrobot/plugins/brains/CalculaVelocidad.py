# -*- coding: utf-8 -*-b

from pyrobot.brain import Brain  
from time import time
from math import sqrt

class CalculaVelocidad(Brain):  

   def setup(self):
      self.recorrido = 0.0
      self.t_ini = time()
      self.t_fin = 0.0
      self.x_ant = self.robot.x
      self.y_ant = self.robot.y
      self.vel = 0.5
      self.ticks = 0
  
   # Give the front two sensors, decide the next move  
   def determineMove(self, front, left, right):  
      #if front < 0.3:  
      #   return(0, 0)
      #else:    
      return(self.vel, 0.0) 

   def step(self):  
      front = min([s.distance() for s in self.robot.range["front"]])
      left = min([s.distance() for s in self.robot.range["left-front"]])
      right = min([s.distance() for s in self.robot.range["right-front"]])
      translation, rotate = self.determineMove(front, left, right)
      if self.robot.stall:
         self.t_fin = time()
         print "Pared alcanzada, los datos recogidos son: "
         print "Distancia recorrida>" + str(self.recorrido)
         print "Tiempo invertido>" + str(self.t_fin - self.t_ini) + " segundos"
         print "Velocidad en línea recta> " + str(self.vel)
         print "Velocidad de recorrido>" + str(self.recorrido/(self.t_fin - self.t_ini)) 
         print "TICKS>" + str(self.ticks)
         self.engine.pleaseStop()
      else:
         dr = sqrt((self.robot.x-self.x_ant)**2+(self.robot.y-self.y_ant)**2)
         self.x_ant = self.robot.x
         self.y_ant = self.robot.y
         self.recorrido = self.recorrido + dr;
         self.robot.move(translation, rotate)
         self.ticks = self.ticks + 1

def INIT(engine):  
   assert (engine.robot.requires("range-sensor") and
           engine.robot.requires("continuous-movement"))
   return CalculaVelocidad('CalculaVelocidad', engine)  
