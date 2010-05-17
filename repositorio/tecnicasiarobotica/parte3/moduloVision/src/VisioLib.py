# -*- coding: utf-8 -*-
'''
Created on 14/05/2010

@author: Pablo Francisco Pérez Hidalgo
'''
import numpy as np
from math import sqrt

def distanciaDescriptores(da,db):
    #return sqrt(np.sum(da*db))
    return abs(np.sum(da*db))
