#! /usr/bin/python
# -*- coding: utf-8 -*- 

#    This file is part of PyNavegacion.
#
#    PyNavegacion is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    PyNavegacion is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with PyNavegacion.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pablo Francisco Pérez Hidalgo (pfcoperez@gmail.com)
import math

def w1(excitacion):
    return 1/(1+math.exp(-excitacion))

def w2(excitacion):
    return math.tanh(excitacion)

class PerceptronMonocapa:
    #Constructor que inicia un perceptrón con tantas
    #entradas como pesos tenga el vector de entrada 'pesos',
    #dicho perceptrón usará como función de activación la dada por w.
    def __init__(self,pesos,w):
        self.pesos = pesos;
        self.activacion = w;

    def getPesos(self):
        return self.pesos

    def modPesos(self,npesos):
        self.pesos = npesos

    def transferencia(self,entradas):
        excitacion = 0.0;
        for i in range(0,min(len(entradas),len(self.pesos)-1)):
            excitacion = excitacion + float(entradas[i])*float(self.pesos[i])
        excitacion = excitacion + float(self.pesos[len(self.pesos)-1]) #Hay que sumar el peso del 
        #término de grado 0 del polinomio
        activ = self.activacion(excitacion)
        #Realiza normalización sobre intervalo [-1,1]:
        c = -1.0;
        d = 1.0;
        if self.activacion == w1:
            a = 0.0;
            b = 1.0;
        elif self.activacion == w2:
            a = -1.0;
            b = 1.0;
        return (d-c)/(b-a)*(activ-a)+c
        
        




    
