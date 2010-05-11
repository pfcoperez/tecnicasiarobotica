# -*- coding: utf-8 -*- 
import math 

from occupancyGrid import OccupancyGrid
from robotUtils import getAbsAngle, normalizeAngle2, normalizeAngle, \
                       calculateHypotenuse
from markerToolkit import MarkerToolkit
from Perceptron import *
import time

class Navigator(OccupancyGrid):

  NO_FORWARD = 0
  SLOW_FORWARD = 0.05
  MED_FORWARD = 0.3 #0.5
  FULL_FORWARD = 0.5 #1.0

  NO_TURN = 0
  MED_LEFT = 0.5
  HARD_LEFT = 1.0
  MED_RIGHT = -0.5
  HARD_RIGHT = -1.0

  def __init__(self, cols, rows, widthMM, heightMM,
               #the x,y and th of the current location can be passed
               currentRow = -1, currentCol = -1, currentTh= 0,
               goalRow = -1, goalCol = -1, gridUpdate = "",
               image=None, grid=None, gridResize=1.0,showIter=0, title=""):

    self.centros = []
    self.markerToolkit = MarkerToolkit()

    self.x_localizeMM = 0
    self.y_localizeMM = 0
    self.thr_localize = 0
    self.x_last_marker_localizeMM = 0
    self.x_last_marker_robotMM = 0
    self.y_last_marker_localizeMM = 0
    self.y_last_marker_robotMM = 0
    self.thr_last_marker_localize = 0
    self.thr_last_marker_robot = 0

    self.resetMarkers()


    OccupancyGrid.__init__(self, cols=cols, rows=rows, 
                widthMM=widthMM, heightMM=heightMM,
                currentRow=currentRow, currentCol=currentCol, 
                currentTh=currentTh,
                goalRow=goalRow, goalCol=goalCol,
                image=image, grid=grid, gridResize=gridResize, 
		showIter=showIter, title=title)
    
    self.distanceToWall = [[self.infinity for col in range(self.cols)] for row in range(self.rows)]
    
    #Todos los bloques ocupados forman parte de una pared y por tanto
    #se les asigna distancia 0 a las paredes.
    for row in range(self.rows):
        for col in range(self.cols):
            if self.grid[row][col] > self.threshhold:
                self.distanceToWall[row][col] = 0;
  
    #Inicia perceptrón con los pesos calculados en la parte1. Si no se desea usar, comentar 
    #esta sección de código.

    self.pesos = [-0.25830320437434307, 1.160526501403915, -0.15551005292679421, 1.2578377038143795, -0.55707372041035363, -2.9552247878192888, -1.6809527155003046, -2.2014578204055359, -13.0, 0.0]

    self.calculadorNeuronal = PerceptronMonocapa(self.pesos,w2)
    
                
  def findPath(self, event=None):
      print("FIND PATH:",self.getGoalRow(),self.getGoalCol());
      if (self.getCurrentRow() < 0 or self.getCurrentCol() < 0):
         raise Exception('NoPathExists', 'current location unknown')
      if (self.getGoalCol() < 0 or self.getGoalRow() < 0):
         raise Exception('NoPathExists', 'goal location unknown')
      if self.grid[self.getGoalRow()][self.getGoalCol()] > self.threshhold:
         raise Exception('NoPathExists', 'goal is in unattainable location')
      if self.debug: print "Finding path..."
    
      self.calculateAllCosts(goalCol=self.getGoalCol(),
                             goalRow=self.getGoalRow(),
                             maxIters=self.rows*self.cols)
      (pathGrid,pathList) = self.getPath(row=self.getCurrentRow(),
                                         col=self.getCurrentCol())
      #self.getPath(row=self.getCurrentRow(),
      #             col=self.getCurrentCol())
      
      #return
      if pathGrid:
         print "Done!"
         self.redraw(self.distanceToWall, path=pathGrid)
         #self.redraw(self.segmentacion,pathGrid,stepByStep)
      else:
         raise Exception('NoPathExists', 'maximum interation limit exceded')
      return pathList

  def distanciaManhattan(self,i,j,l,m):
    return abs(i-l)+abs(j-m)

  def distanciaEuclidea(self,i,j,l,m):
    return math.sqrt((i-l)**2+(i-m)**2)

  def costeF(self,G,i,j):
    #H = self.distanciaManhattan(self.getGoalRow(),self.getGoalCol(),i,j)
    H = max(self.distanciaEuclidea(self.getGoalRow(),self.getGoalCol(),i,j),
            self.distanciaManhattan(self.getGoalRow(),self.getGoalCol(),i,j))
    return G+H

  def costesFGH(self,n,G):
    return (self.costeF(G,self.centros[n][0],self.centros[n][1]),
                     G, self.distanciaManhattan(self.getGoalRow(),self.getGoalCol(),
                                           self.centros[n][0],self.centros[n][1]))

  def aEstrella(self,MA,n):
    #print n, self.nodoObjetivo
    if n == self.nodoObjetivo:
      return [n]
    nnodos = len(self.centros)
    listaAbierta = set([n])
    almacenFGH = [(0,0,0) for i in range(nnodos)]
    almacenFGH[n] = self.costesFGH(n,0)
    listaCerrada = set()
    padres = [0 for i in range(nnodos)]
    camino = []
    g = 0
    alcanzado = False
    while not alcanzado and len(listaAbierta) > 0:
      candidato = -1
      iterok = False
      minimo = self.infinity
      for k in range(nnodos):
        if self.costeF(g,self.centros[k][0], self.centros[k][1]) < minimo and k in listaAbierta and self.grid[self.centros[k][0]][self.centros[k][1]] < self.threshhold:
          minimo = self.costeF(g,self.centros[k][0], self.centros[k][1])
          candidato = k
      camino = camino + [candidato]
      g = almacenFGH[candidato][2]
      listaAbierta.remove(candidato)
      listaCerrada.add(candidato)
      for k in range(nnodos):        
        if candidato <> k and (MA[candidato][k] > 0 or MA[k][candidato] > 0): #Si es nodo adyacente
          if not k in listaCerrada and self.grid[self.centros[k][0]][self.centros[k][1]] < self.threshhold:    #Si el adyacente no está en la lista cerrada ni es pared
            iterok = True 
            if not k in listaAbierta: #Si no estaba en la lista abierta.
              listaAbierta.add(k)
              if k == self.nodoObjetivo:
                alcanzado = True
              padres[k] = candidato
              almacenFGH[k] = self.costesFGH(k,g+1)
            else: #Si ya estaba en la lista abierta.
              if g+1 < almacenFGH[k][2]: #Si es mejor el camino actual que el por el que se llegó a k
                padre[k] = candidato
                almacenFGH[k] = self.costesFGH(k,self.centros[k][1])
      if not iterok:
        return []
    if alcanzado:          
      return camino+[self.nodoObjetivo]
    else:
      return []
              
      
      
    
    

  def dijkstra(self,MA,n):
    #MA = Matriz de adyacencia del grafo.
    #n = nodo origen
    nnodos = len(self.centros)
    retorno = [[0 for j in range(nnodos)] for i in range(nnodos)]
    L = [self.infinity for i in range(nnodos)] 
    L[n] = 0
    S = set(range(nnodos))
    while len(S) > 1:
      minimo = self.infinity
      candidato = -1
      for p in range(nnodos):
        if p in S and L[p] < minimo:
          minimo = L[p]
          candidato = p
      for q in range(nnodos):
        if q in S:
          if MA[candidato][q] > 0 or MA[q][candidato] > 0:
            peso = 1
          else:
            peso = self.infinity
          if min(L[q],L[candidato]+peso) == L[candidato]+peso and min(L[q],L[candidato]+peso) < self.infinity:
            retorno[candidato][q] = peso 
          L[q] = min(L[q],L[candidato]+peso)
      S.remove(candidato)
   
    return retorno
          

  def getPath(self, row, col):
    pathGrid = [[0 for c in range(self.cols)] for r in range(self.rows)]
    pathList = []
    
      #El primer paso para construir el grafo de adyacencia entre habitaciones
      #para ello, se consideran adyacentes aquellas habitaciones que tienen
      #un punto adyacente con otro de otra habitación (dos puntos
      #adyacentes que en la matriz "segmentacion" tienen etiquetas diferentes.
    nnodos = len(self.centros) #Número de nodos (habitaciones)
    MA = [[0 for j in range(nnodos)] for i in range(nnodos)] #Inicializa matriz de adyacencia
    MAF = [[(0,0) for j in range(nnodos)] for i in range(nnodos)] #Puntos frontera en cada adyacencia
    for i in range(self.rows):
      for j in range(self.cols):
        if self.grid[i][j] < self.threshhold:
          eti = self.segmentacion[i][j]
          if i > 0 and i < self.rows-1 and j > 0 and j < self.cols-1:
            for l in range(-1,2):
              for m in range(-1,2):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i>0 and i < self.rows-1 and j == self.cols-1:
            for l in range(-1,2):
              for m in range(-1,1):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i > 0 and i < self.rows-1 and j == 0:
            for l in range(-1,2):
              for m in range(0,2):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i == self.rows-1 and j > 0 and j < self.cols-1:
            for l in range(-1,1):
              for m in range(-1,2):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i == 0 and j > 0 and j < self.cols-1:
            for l in range(0,2):
              for m in range(-1,2):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i == 0 and j == 0:
            for l in range(0,2):
              for m in range(0,1):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i == 0 and j == self.cols-1:
            for l in range(0,2):
              for m in range(-1,1):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i == self.rows-1 and j == 0:
            for l in range(-1,1):
              for m in range(0,2):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 
          elif i == self.rows-1 and j == self.cols-1:
            for l in range(-1,1):
              for m in range(-1,1):
                if self.segmentacion[i+l][j+m] <> eti and self.grid[i+l][j+m] < self.threshhold:
                  MA[eti][self.segmentacion[i+l][j+m]] = 1 
                  MAF[eti][self.segmentacion[i+l][j+m]] = (i,j) 

    #Una vez obtenido el grafo (su matriz de adyacencia), se busca el camino topológico
    #más corto desde el origen hasta la habitación objetivo, para ello, se ha utilizado
    #el algoritmo A*
    caminoCentros = self.aEstrella(MA,self.segmentacion[row][col])
    if len(caminoCentros) == 0:
      return [[],[]]
    for k in range(1,len(caminoCentros)):
      frontera = MAF[caminoCentros[k-1]][caminoCentros[k]]
      i = self.centros[caminoCentros[k]][0]
      j = self.centros[caminoCentros[k]][1]
      pathGrid[i][j] = 1
      pathGrid[frontera[0]][frontera[1]] = 1
      pathList.append(frontera)
      pathList.append((i,j))
    return (pathGrid,pathList)

  def clasificaPuntosSegmentacion(self,pi,pj):
      i = pi
      j = pj
      encontrado = False
      
      while not encontrado and i < self.rows and j < self.cols:
        ci = i
        cj = j
        if i > 0 and i < self.rows-1 and j > 0 and j < self.cols-1:
          maximo = 0
          for l in range(-1,2):
            for m in range(-1,2):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i>0 and i < self.rows-1 and j == self.cols-1:
          maximo = 0
          for l in range(-1,2):
            for m in range(-1,1):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i > 0 and i < self.rows-1 and j == 0:
          maximo = 0
          for l in range(-1,2):
            for m in range(0,2):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i == self.rows-1 and j > 0 and j < self.cols-1:
          maximo = 0
          for l in range(-1,1):
            for m in range(-1,2):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i == 0 and j > 0 and j < self.cols-1:
          maximo = 0
          for l in range(0,2):
            for m in range(-1,2):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i == 0 and j == 0:
          maximo = 0
          for l in range(0,2):
            for m in range(0,1):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i == 0 and j == self.cols-1:
          maximo = 0
          for l in range(0,2):
            for m in range(-1,1):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i == self.rows-1 and j == 0:
          maximo = 0
          for l in range(-1,1):
            for m in range(0,2):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
        elif i == self.rows-1 and j == self.cols-1:
          maximo = 0
          for l in range(-1,1):
            for m in range(-1,1):
              if self.distanceToWall[i+l][j+m] > maximo and self.grid[i+l][j+m] < self.threshhold:
                maximo = self.distanceToWall[i+l][j+m]
                ci = i+l
                cj = j+m
          if ci == i and cj == j:
            encontrado = True
          i = ci
          j = cj
      
      if not [i,j] in self.centros:
        self.centros = self.centros + [[i,j]]
      return (i,j,maximo)
    
  def calculateAllCosts(self, goalCol, goalRow, maxIters):
      """
      Versión simplificada del algoritmo "watershed".

      When an occupancy probability is above some threshold, assume
      that the cell is occupied.
      """
      
      #if (goalCol == self.lastGoalDistCol and
      #    goalRow == self.lastGoalDistRow):
      #   return
      #startTime = time.time()
      print("Calculando todas las distancias a paredes")
      
      self.distanceToWall = [[self.infinity for col in range(self.cols)] for row in range(self.rows)]
      self.segmentacion = [[0 for j in range(self.cols)] for i in range(self.rows)]
    
      #Todos los bloques ocupados forman parte de una pared y por tanto
      #se les asigna distancia 0 a las paredes.
      for row in range(self.rows):
          for col in range(self.cols):
              if self.grid[row][col] > self.threshhold:
                  self.distanceToWall[row][col] = 0;
            
      if not self.inRange(goalRow, goalCol):
         raise Exception('goalOutOfMapRange')
     

      for iter in range (maxIters):
	 valuesChanged = 0
	 if (self.showIterations):
            print "Mostrando resultado tras la iteración:> ",iter
            self.redraw(self.distanceToWall, stepByStep=1)
	    self.frame.update()
            #time.sleep(10)
         
         #print "ITERACION: ", iter
         for row in range(self.rows):
            for col in range(self.cols):
               for i in [-1,0,1]:
                  for j in [-1,0,1]:
                     if self.inRange(row+i, col+j):
                        if self.grid[row][col] > self.threshhold:
                           self.distanceToWall[row][col] = 0
                        else:
                           if abs(i) == 0 and abs(j) == 0:
                              d = 0.00
                           elif abs(i) == 1 and abs(j) == 1:
                             if self.tooTight(row, col, i, j):
                               d = self.infinity
			     else:
                               d = 1.41
                             #d = 1.41
                           else:
                              d = 1.00                              
                           #adj = self.distanceToWall[row+i][col+j] + self.grid[row+i][col+j] + d
                           adj = self.distanceToWall[row+i][col+j] + d
                           #print adj<self.distanceToWall[row][col]
                           #print "ADJ: ", adj, "ANT: ", self.distanceToWalll[row][col]
                           #if self.distanceToWalll[row][col] != 0:
                           #print "IT: ",adj < self.distanceToWall[row][col]
			   if adj < self.distanceToWall[row][col]:
			     valuesChanged += 1
                           self.distanceToWall[row][col] = min(self.distanceToWall[row][col], adj)
      
         if valuesChanged == 0:
	   #endTime = time.time()
	   print "Distancias a paredes calculadas en %d iterationes." % iter
	   #print "Tiempo transcurrido %0.3f ms." % ((endTime-startTime)*1000.0)
	   break
      
      self.centros = [] #Puntos representantes de cada habitacion encontrada.
      
      buff = []
      contador = 0.1
      anterior = self.segmentacion[i][j] = self.clasificaPuntosSegmentacion(0,0)[2]
      for i in range(self.rows):
        for j in range(self.cols):
          if self.grid[i][j] < self.threshhold:
            buff = buff + [(i, j, self.clasificaPuntosSegmentacion(i,j))]
          #self.segmentacion[buff[0]][buff[1]] = 1
          """if self.grid[i][j] < self.threshhold:
            buff = self.clasificaPuntosSegmentacion(i,j)[2]
            self.segmentacion[i][j] = self.clasificaPuntosSegmentacion(i,j)[2]
            if buff != anterior:
              contador = contador + 0.1
              anterior = buff"""

      self.numeracionCentros = [[0 for j in range(self.cols)] for i in range(self.rows)] 
      for i in range(len(self.centros)):
        self.numeracionCentros[self.centros[i][0]][self.centros[i][1]] = i

      for i in range(len(buff)):
        self.segmentacion[buff[i][0]][buff[i][1]] = self.numeracionCentros[buff[i][2][0]][buff[i][2][1]] 
        if buff[i][0] == self.getGoalRow() and buff[i][1] == self.getGoalCol():
          self.nodoObjetivo = self.numeracionCentros[buff[i][2][0]][buff[i][2][1]]

      SegmentacionPintada = [[float(self.segmentacion[i][j])/float(len(self.segmentacion)-1)*1.5 for j in range(self.cols)] for i in range(self.rows)]
      for i in range(len(self.centros)):
        SegmentacionPintada[self.centros[i][0]][self.centros[i][1]] = 0
      self.redraw(SegmentacionPintada, stepByStep=1)
      self.frame.update()
      #time.sleep(10)

  def resetMarkers(self):
      self.markers = {}
      self.markerLoc = {}
      self.markerAngle = {}

  # we use the same angle convention used in the pyrobot simulator
  # angles are expressed in radians, 0 is north and + angles rotate
  # counter-clockwise

  def placeMarker(self,row,col,name,angle=0,redraw=0):
      self.markers[name] = (row,col)
      self.markerLoc[(row,col)] = name
      self.markerAngle[name] = angle
      if (redraw):
        self.redrawTrig.set(1);

  def placeRelativeMarker(self,name,relXOffM,relYOffM,relTh,redraw=1):

      # the -1's are to change the perspective.
      newMarkerInfo = self.getPosRelativeToKnown(self.getCurrentRow()*
                                                 self.rowScaleMM,
                                                 self.getCurrentCol()*
						 self.colScaleMM,
					         self.getCurrentTh(),
                                                 relXOffM, 
                                                 relYOffM*-1, 
						 relTh*-1)

      print ("placing new marker ",name,newMarkerInfo[0],
             newMarkerInfo[1],newMarkerInfo[4])
             
      self.placeMarker(row=newMarkerInfo[0], col=newMarkerInfo[1],
                       name=name, angle=newMarkerInfo[4], redraw=1)


  def getPosRelativeToKnown(self, knownRowMM, knownColMM, knownAngle, 
                             relXOffM, relYOffM, relTh):

     #print "relXOffM",relXOffM,"relYOffM",relYOffM,"relTh",relTh,\
     #      "atan",math.atan(relYOffM/relXOffM),"knownAngle",knownAngle

     toKnownDistMM = calculateHypotenuse(relXOffM*1000, relYOffM*1000)
     toKnownAngle = knownAngle+relTh

     #print "toKnownAngle",toKnownAngle,"toKnownDistMM",toKnownDistMM,\
     #      "cos-row",math.cos(toKnownAngle),"sin-col",math.sin(toKnownAngle)
     #print "rowMMDiff ", (toKnownDistMM * math.cos(toKnownAngle)), \
     #      "colMMDiff ", (toKnownDistMM * math.sin(toKnownAngle))

     relRowMM = knownRowMM - (toKnownDistMM * math.cos(toKnownAngle))
     relColMM = knownColMM - (toKnownDistMM * math.sin(toKnownAngle))
     
     relRow = int(round(relRowMM/self.rowScaleMM))
     relCol = int(round(relColMM/self.colScaleMM))

     return (relRow, relCol, relRowMM, relColMM,
             (knownAngle+math.pi+relTh)%(math.pi*2))

  def hasMarker(self,row,col):
      try:
        return self.markerLoc.has_key((row,col))
      except:
        return False 

  def getMarkerRow(self,name):
     return self.markers[name][0]

  def getMarkerCol(self,name):
     return self.markers[name][1]

  def markerExists(self,name):
     return self.markers.has_key(name)

  def printMarker(self,row,col):
     if self.hasMarker(row=row,col=col):
        return "%s-%.2f" % (self.markerLoc[(row,col)],
	                   self.markerAngle[self.markerLoc[(row,col)]])
     else:
        return ""

  def drawCell(self,row,col,matrix,maxval,path,stepByStep,removeBelow=0):
  # see the occupancyGrid redraw comment for more info on the coordinate
  # system.

      OccupancyGrid.drawCell(self,row,col,matrix,maxval,path,stepByStep,
                             removeBelow)
      x = col
      y = row

      if self.hasMarker(row=row,col=col):
         self.canvas.create_text((x + .5) * self.colScale,
				 (y + .5) * self.rowScale,
				 tag = 'label',
				 text=self.printMarker(row=row,col=col),
				 anchor="center",
				 fill='orange')

  def setRobotLocation(self,robotXMM,robotYMM,robotTh):

    self.x_localizeMM = robotXMM
    self.y_localizeMM = robotYMM
    self.thr_localize = robotTh

    self.setCurrent(row=int(round(self.x_localizeMM/self.rowScaleMM)),
                      col=int(round(self.y_localizeMM/self.colScaleMM)),
                      th=self.thr_localize)

  def updateRobotLocation(self,robot):
    # get the list of markers in view
    markers = self.markerToolkit.getMarkerData(robot)
    # print "I found markers: ",markers 
   
    # if we can see at least one marker 
    if (len(markers) >= 1):

       # print "localizing using seen markers"
       # we look for the closest marker of those found, we will localize
       # upon that (assuming less error in measurements the closer the
       # marker)
       closestMarker = -1
       for index in range(len(markers)):
          if (self.markerExists(markers[index][3]) and
              (closestMarker == -1 or
               # we assume that negative distance markers are errors
               (markers[closestMarker][0] > markers[index][0] and
                markers[index][0] > 0))):
            closestMarker = index

       if (closestMarker >= 0):
          # print "Localizing using closest marker: ",markers[closestMarker]
   
	  # localize sets the current position and orientation of the robot in
	  # the grid and returns that position as well as more precise MM based
	  # location information which we will use internally.  (remember x=row
	  # y=col)
          coord_robot = self.localize(markerName=markers[closestMarker][3],
                                        relXOffM=markers[closestMarker][0],
                                        relYOffM=markers[closestMarker][1],
                                        relTh=markers[closestMarker][2])
   
          self.x_localizeMM = coord_robot[0]
          self.y_localizeMM = coord_robot[1]
	  self.thr_localize = coord_robot[2]

          # temporarily change the robot's unit system to meters
	  # it seems as though a simulated robot doesn't have a units
	  # attribute??
	  try:
	    oldUnits = robot.units
	  except AttributeError:
	    oldUnits = "METERS"
          robot.units = "METERS"

          # We keep track of where we think that we are and where the robot
          # thinks that it is to be able to localize without artoolkit
          # in future passes.
          self.x_last_marker_localizeMM = self.x_localizeMM
          self.x_last_marker_robotMM = robot.x*1000
          self.y_last_marker_localizeMM = self.y_localizeMM 
          self.y_last_marker_robotMM = robot.y*1000
          self.thr_last_marker_localize = self.thr_localize
          self.thr_last_marker_robot = robot.thr

          robot.units = oldUnits

       # Lastly, we look through all the seen markers and if there are
       # any that are NEW we place them in the map (remember that we
       # can't localize upon a new marker that is why this comes at the
       # end) Note, if we start out with an empty map this will
       # place the marker based upon the assumed starting 'current'
       # location.

       for index in range(len(markers)):
          if (not self.markerExists(markers[index][3])):
            self.placeRelativeMarker(name=markers[index][3],
                                       relXOffM=markers[index][0],
                                       relYOffM=markers[index][1],
				       relTh=markers[index][2])

    # otherwise we localize using odometry
    else: 
	# print "Localizing with odometry"
        # temporarily change the robot's unit system to meters
	# it seems as though a simulated robot doesn't have a units
	# attribute so we try and if not there we just add it.
	try:
	  oldUnits = robot.units
	except AttributeError:
	  oldUnits = "METERS"
        robot.units = "METERS"

        xDiffMM = (robot.x*1000) - self.x_last_marker_robotMM
        yDiffMM = (robot.y*1000) - self.y_last_marker_robotMM
        thDiff = self.thr_last_marker_robot - self.thr_last_marker_localize
	# print "diffs",xDiffMM,yDiffMM,thDiff

        # we need to remember that the robot's odometry was set when the 
	# robot started and thus its coordinate system could be very different,
	# most likely even rotated with respect to the map's coordinates.
        # also there is a sign difference between the map's Y and the i
        # y given by the odometry.

    	self.x_localizeMM = (self.x_last_marker_localizeMM -
                             (math.cos(thDiff)*xDiffMM + 
                              -1*math.sin(thDiff)*yDiffMM))
	self.y_localizeMM = (self.y_last_marker_localizeMM +
                             math.sin(thDiff)*xDiffMM +
                             -1*math.cos(thDiff)*yDiffMM)
	self.thr_localize = (robot.thr - 
                             self.thr_last_marker_robot + 
                             self.thr_last_marker_localize)
        # print "new location",self.x_localizeMM,self.y_localizeMM,self.thr_localize

        self.setCurrent(row=int(round(self.x_localizeMM/self.rowScaleMM)),
                          col=int(round(self.y_localizeMM/self.colScaleMM)),
                          th=self.thr_localize)

        robot.units = oldUnits

  def localize(self, markerName, relXOffM, relYOffM, relTh):

   # We are localizing using a marker in the field of vision of the robot.
   # relXOffM, relYOffM are the legs of the triangle made from the straight
   # line from the robot to the marker (with the base of the right triangle
   # along the marker + relTh).  relXOffM is along the 'heading' line' and
   # relYOffM is at 90 degrees to the heading line

     try:
       # assume that the marker is in the center of the cell. (I have
       # left this as .45 to prevent it from rounding 'up' in other
       # places which causes problems with alignemnt.
       markerRowMM = (self.getMarkerRow(markerName)+.45)*self.rowScaleMM
       markerColMM = (self.getMarkerCol(markerName)+.45)*self.colScaleMM
       markerAngle = self.markerAngle[markerName]
     except KeyError:
       # we don't have info regarding that marker??
       return (-1,-1,-1)

     #the return of getPosRelativeToKnown is (row,col,rowMM,colMM,th) 
     coord_robot = self.getPosRelativeToKnown(markerRowMM,
                                              markerColMM,
					      markerAngle,
                                              relXOffM, 
                                              relYOffM, relTh)

     #print " marker - row ",self.getMarkerRow(markerName)," col ",\
     #      self.getMarkerCol(markerName)," th ", markerAngle
     #print "        - rowMM ",markerRowMM," colMM ", markerColMM
     #print " robot - row ",coord_robot[0]," col ",coord_robot[1]," th ", \
     #      coord_robot[4]
     #print "        - rowMM ",coord_robot[2]," colMM ", coord_robot[3] 

     self.setCurrent(row=coord_robot[0],col=coord_robot[1],
                     th=coord_robot[4])
     self.redrawTrig.set(1);

     # return the location in MM and the th
     return (coord_robot[2], coord_robot[3], coord_robot[4])

  def calculaVelocidadAvance(self,vgiro):
    return 0.75*(0.1+0.9*(1-abs(vgiro)))

  def determineMove(self, subgoal, sonar):
     # note that we assume that the row and col given are subgoals, this
     # algorithm can NOT deal with walls or obstacles.

     subgoalRow, subgoalCol = subgoal

     subgoalXmm = subgoalRow*self.rowScaleMM
     subgoalYmm = subgoalCol*self.colScaleMM
     

     angleToGoal = math.atan2((subgoalYmm-self.y_localizeMM)*-1,
                              (subgoalXmm-self.x_localizeMM)*-1)
                             
     robotAngleDiff = -normalizeAngle2(angleToGoal-normalizeAngle2(self.thr_localize))
     print "DIFF:  ", robotAngleDiff

     sensores = sonar
     #sensores = [2.0*float(sonar[k])/float(max(sonar)) for k in range(len(sonar))]
     velgiro = self.calculadorNeuronal.transferencia(sensores+[robotAngleDiff/(2.0*math.pi)]);
     velavan = self.calculaVelocidadAvance(velgiro)
     return(velavan,velgiro)

  """
  def determineMove(self, subgoal):
     # note that we assume that the row and col given are subgoals, this
     # algorithm can NOT deal with walls or obstacles.

     subgoalRow, subgoalCol = subgoal

     subgoalXmm = subgoalRow*self.rowScaleMM
     subgoalYmm = subgoalCol*self.colScaleMM
     
     # remember that in pyrobot th increases as the robot turns
     # COUNTER-CLOCKWISE

     # we start by calculating the difference between the current
     # heading (orientation angle) of the robot and a straight line to
     # the subgoal
     angleToGoal = math.atan2((subgoalYmm-self.y_localizeMM)*-1,
                              (subgoalXmm-self.x_localizeMM)*-1)
                             
     robotAngleDiff = normalizeAngle(angleToGoal-self.thr_localize)

     #print "Current: ",self.x_localizeMM, self.y_localizeMM, self.thr_localize
     #print "Subgoal: ",subgoalXmm, subgoalYmm
     #print "angleToGoal",angleToGoal
     #print "robotAngleDiff: ",robotAngleDiff

     # if the subgoal isn't within an arc of size pi/4 centered in the
     # front of the robot we do a hard turn
     if (robotAngleDiff > math.pi/8 and robotAngleDiff <= math.pi):
        print "hard left"
        return(self.NO_FORWARD, self.HARD_LEFT)
     elif (robotAngleDiff > math.pi and 
           robotAngleDiff < (math.pi*2 - (math.pi/8))):
        print "hard right"
        return(self.NO_FORWARD, self.HARD_RIGHT)

     # when within the pi/4 arc we turn less the closer we are to
     # having it straight ahead
     elif (robotAngleDiff > 0 and robotAngleDiff <= math.pi/8):
        print "med-forward, variable to the left"
        return(self.MED_FORWARD, robotAngleDiff/math.pi/8)
     elif (robotAngleDiff >= (math.pi*2 - (math.pi/8)) and
           robotAngleDiff < math.pi*2):
        print "med-forward, variable to the right"
        return(self.MED_FORWARD, (robotAngleDiff-math.pi*2)/math.pi/8)

     # if it is straight ahead to just go straight
     else:
        print "forward"
        return(self.FULL_FORWARD,self.NO_TURN)"""



  def mergeOtherGridInfo(self, otherGrid):

      # find two common markers
      commonKeys = []
      for key in self.markers.keys():
        if (otherGrid.markerExists(key)):
          commonKeys.append(key)

      if (len(commonKeys) < 2):
        print "I can not find two common markers, merge failed!"
        return

      # for now we just use the first two common markers found
      thisGridAnchor1Row = self.getMarkerRow(commonKeys[0])
      thisGridAnchor1Col = self.getMarkerCol(commonKeys[0])
      thisGridAnchor2Row = self.getMarkerRow(commonKeys[1])
      thisGridAnchor2Col = self.getMarkerCol(commonKeys[1])
      otherGridAnchor1Row = otherGrid.getMarkerRow(commonKeys[0]) 
      otherGridAnchor1Col = otherGrid.getMarkerCol(commonKeys[0])
      otherGridAnchor2Row = otherGrid.getMarkerRow(commonKeys[1])
      otherGridAnchor2Col = otherGrid.getMarkerCol(commonKeys[1])

      return OccupancyGrid.mergeOtherGridInfo(self,otherGrid,
                                     thisGridAnchor1Row = thisGridAnchor1Row,
				     thisGridAnchor1Col = thisGridAnchor1Col,
				     thisGridAnchor2Row = thisGridAnchor2Row,
				     thisGridAnchor2Col = thisGridAnchor2Col,
				     otherGridAnchor1Row = otherGridAnchor1Row,
				     otherGridAnchor1Col = otherGridAnchor1Col,
				     otherGridAnchor2Row = otherGridAnchor2Row,
				     otherGridAnchor2Col = otherGridAnchor2Col)
  
  def redraw(self, matrix = None, path = None, stepByStep=0, onlyDirty=0):
    # The origin of the tkinter coordinate system is at the top left with
    # x increasing along the top edge and y increasing along the left
    # edge:
    #
    #   0,0 ---- +x
    #    |
    #    |
    #   +y
    #
    # This means that x corresponds to the column number and y to the row
    # number.
    
    if matrix == None:
      matrix = self.grid
    maxval = 0.0
    for row in range(self.rows):
      for col in range(self.cols):
        if matrix[row][col] != self.infinity:
          maxval = max(matrix[row][col], maxval)
    if maxval < 1:
      maxval = 1
    if onlyDirty:
      tmpDirty = self.getAndResetDirtyCells()
      while len(tmpDirty) > 0:
        (row,col) = tmpDirty.pop(0)
        self.drawCell(row=row,col=col,matrix=matrix,
                      maxval=maxval,path=path,stepByStep=stepByStep,
                      removeBelow=1)
    else:
      self.canvas.delete("cell")
      self.canvas.delete("label")
      for row in range(self.rows):
        for col in range(self.cols):
          self.drawCell(row=row,col=col,matrix=matrix,
                        maxval=maxval,path=path,
                        stepByStep=stepByStep)

    if (self.getCurrentRow() >= 0 and self.getCurrentCol() >= 0):
      self.canvas.create_text((self.getCurrentCol() + .5) * self.colScale,
                              (self.getCurrentRow() + .5) * self.rowScale,
                              tag = 'cell',
                              text="Current", fill='green')
    if (self.getGoalCol() >= 0 and self.getGoalRow() >= 0):
      self.canvas.create_text((self.getGoalCol() + .5) * self.colScale,
                              (self.getGoalRow() + .5) * self.rowScale,
                              tag = 'cell',
                              text="Goal", fill='green')

    for k in range(len(self.centros)):
      i = self.centros[k][0]
      j = self.centros[k][1]
      self.canvas.create_text((j + .5) * self.colScale,
                              (i + .5) * self.rowScale,
                              tag = 'cell',
                              text="Centro "+str(k), fill='blue')

  def drawCell(self,row,col,matrix,maxval,path,stepByStep,removeBelow=0):
    # see the redraw comment for more info on the coordinate system.
    x = col
    y = row
    
    if removeBelow:
      self.canvas.addtag_overlapping("toDelete",
                                     x * self.colScale,
                                     y * self.rowScale,
                                     (x + 1) * self.colScale,
                                     (y + 1) * self.rowScale)
      self.canvas.delete("toDelete")

    if self.background==None or path or stepByStep:
      self.canvas.create_rectangle(x * self.colScale,
                                   y * self.rowScale,
                                   (x + 1) * self.colScale,
                                   (y + 1) * self.rowScale,
                                   width = 0,
                                   fill=self.color(matrix[row][col], maxval),
                                   tag = "cell")
#         sys.__stdout__.write("drawing Cell %s,%s with %f/%f\n" % \
#                              (col,row,matrix[row][col],maxval))
    else:
      self.canvas.create_rectangle(x * self.colScale,
                                   y * self.rowScale,
                                   ((x + 1) * self.colScale)-1,
                                   ((y + 1) * self.rowScale)-1,
                                   fill="",
                                   width=1.0,
                                   outline=self.color(matrix[row][col], maxval),
                                   tag = "cell")

    if path and path[row][col] == 1:
      self.canvas.create_rectangle((x + .25) * self.colScale,
                                   (y + .25) * self.rowScale,
                                   (x + .75) * self.colScale,
                                   (y + .75) * self.rowScale,
                                   width = 0,
                                   fill = "red",
                                   tag = "cell")

