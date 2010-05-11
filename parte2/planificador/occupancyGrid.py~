from tkmap import TkMap
import robotUtils

import Tkinter
from PIL import Image, ImageTk
import time
import math
import sys
import sets
import threading

class OccupancyGrid(TkMap):
   """
   GUI for visualizing an occupancy grid style map.

   The mouse can be used to change occupancy values:
   Press the left button for 1.0
   Press the middle button for 0.5
   Press the right button  for 0.0

   Certain keys provide other funtionality:
   Press 'S' to set the current location.
   Press 'G' to set the goal cell of the path.
   Press 'P' to plan the path.
   Press '2' to double the size of the occupancy grid.
   Press 'Q' to quit.
   """
   def __init__(self, 
                # the size of the space in MM, and the number of cells
                cols, rows, widthMM, heightMM, 
                #the x,y and th of the current location can be passed
                currentRow = -1, currentCol = -1, currentTh= 0,
		# the goal cal also be passed
                goalRow = -1, goalCol = -1, 
		# you can pass a BW image or a grid as the default data
		image=None, grid=None, gridUpdate = "",
		# when passing an image you can resize it before showing
		# it on the screen
		gridResize=1.0,showIter=0, title=""):

      menu = [('File',[['Load map...',self.myLoadMap],
                       # ['Save map...',self.saveMap],
                       ['Exit',self.destroy]
                       ]),
              ]
      keybindings = [ ("<B1-Motion>", self.increaseCell),
                      ("<B2-Motion>", self.middleCell),
                      ("<B3-Motion>", self.decreaseCell),
                      ("<Button-1>", self.increaseCell),
                      ("<Button-2>", self.middleCell),
                      ("<Button-3>", self.decreaseCell),
                      ("<KeyPress-p>", self.findPath),
                      ("<KeyPress-s>", self.setCurrentFromEvent),
                      ("<KeyPress-g>", self.setGoalFromEvent),
                      ("<KeyPress-d>", self.printGrid),
                      ("<KeyPress-2>", self.setDouble),
		      # problem with number of arguments to destroy it
		      # seems to be passing an event as well which makes
		      # 1 to many
                      # ("<KeyPress-q>", self.destroy)
                      ]

      # the default window width and height in pixels
      (frameWidth,frameHeight) = (cols*5,rows*5)

      self.infinity = 1e5000
      self.bigButNotInfinity = 5000
      self.background = None
      self.showIterations = showIter
      self.gridUpdate = gridUpdate

      # make a Tkinter variable which will allow redraw from another
      # thread.
      self.redrawTrig = Tkinter.IntVar()

      # mode = "r" (call observer when variable is read by someone), "w"
      # (call when variable is written by someone), or "u" (undefined
      # call when the variable is deleted).
      self.redrawTrig.trace("w",self.redrawTrigCallback)

      if image != None:
	try:
          self.image = Image.open(image)
          self.image = self.image.convert('L')
          frameWidth,frameHeight = self.image.size
	  if gridResize != 1.0:
	    frameWidth = int(round(frameWidth*gridResize))
  	    frameHeight = int(round(frameHeight*gridResize))
  	    self.image = self.image.resize((frameWidth,frameHeight),
	                                   Image.BICUBIC)
#	   print "using image size = %d %d with cols,rows %d,%d" % \
#                (frameWidth,frameHeight,cols,rows)
        except:
	  self.image = None
	  print "Error opening image file ",image," I will ignore it."
      else:
	  self.image = None

      TkMap.__init__(self, cols=cols, rows=rows,
                     frameWidth=frameWidth, frameHeight=frameHeight, 
                     widthMM=widthMM, heightMM=heightMM,
                     title="Occupancy Grid "+title, menu=menu, 
                     keybindings=keybindings)

#      print "M/cell scale: col = %s, row %s " % (self.colScaleMM,self.rowScaleMM)

      self.threshhold = 0.3
      self.distanceToGoal= [[self.infinity for col in range(self.cols)]
                             for row in range(self.rows)]
      self.lastGoalDistCol = -1
      self.lastGoalDistRow = -1

      self.setGoal(row=goalRow,col=goalCol)
      self.setCurrent(row=currentRow,col=currentCol,th=currentTh)

      self.setupDirtyCells()

      if self.image != None:
      # build a grid using the pixels in an image file.
	self.background = ImageTk.PhotoImage(self.image,master=self)
	self.canvas.create_image(0,0, anchor="nw", image=self.background)

        mapData = list(self.image.getdata())
	map = [[-1 for col in range(cols)] for row in range(rows)]
	for i in range(self.rows):
	   for j in range(self.cols):
	      for subI in range(int(self.rowScale)):
	         for subJ in range(int(self.colScale)):
		    # only look in the cell until you find a non-white
		    # pixel and then move to the next cell to save
		    # scanning the entire image.
	            if mapData[self.calculatePos((i*self.rowScale)+subI,
		                                 (j*self.colScale)+subJ,
					         self.frameWidth)] < 200:
		       map[i][j] = 1
		       break
	         if map[i][j]>0:
	            break
	      else:
	         map[i][j] = 0
        self.setGrid(map)
	self.redraw(map)

      if grid != None:
        self.setGrid(grid)
	self.redraw(grid)

   def calculatePos(self, i, j, width):
      return int(math.floor(i)*width+j)

   def myLoadMap(self):
      TkMap.loadMap(self)
      self.redraw(self.grid)

   def increaseCell(self, event):
      self.canvas.focus_set()
      clickCol,clickRow = self.eventToCell(event)
      self.grid[clickRow][clickCol] = 1.0
      print "Maximizing value in cell (%d,%d)." % (clickRow,clickCol)
      self.redraw(self.grid)

   def middleCell(self, event):
      self.canvas.focus_set()
      clickCol,clickRow = self.eventToCell(event)
      self.grid[clickRow][clickCol] = 0.5
      print "Putting 0.5 in cell (%d,%d)." % (clickRow,clickCol)
      self.redraw(self.grid)

   def decreaseCell(self, event):
      self.canvas.focus_set()
      clickCol,clickRow = self.eventToCell(event)
      self.grid[clickRow][clickCol] = 0.0
      print "Minimizing value in cell (%d,%d)." % (clickRow,clickCol)
      self.redraw(self.grid)

   def eventToCell(self,event):
     return (int(round(event.x/self.colScale)),
             int(round((event.y - 15)/self.rowScale)))

   # Overloaded:
   def changeSize(self, event = 0):
      self.frameWidth = self.winfo_width() - 2
      self.frameHeight = self.winfo_height() - 30 # with menu
      self.canvas.configure(width = self.frameWidth, height = self.frameHeight)
      self.reScale()
      try:
         self.redraw(self.grid)
      except:
         pass

   def color(self, value, maxvalue):
      if value == self.infinity:
         return "brown"
      value = 1.0 - value / maxvalue
      color = "gray%d" % int(value * 100.0)
      return color

   def setDouble(self, event):
      self.rows *= 2
      self.cols *= 2
      self.reScale()
      self.distanceToGoal= [[self.infinity for col in range(self.cols)]
                            for row in range(self.rows)]
      self.grid= [[0.0 for col in range(self.cols)]
                  for row in range(self.rows)]
      self.redraw(self.grid)

   def setGoalFromEvent(self, event):
      clickCell = self.eventToCell(event)
      self.setGoal(row=clickCell[1],col=clickCell[0])
      self.redraw(self.grid)

   def setGoal(self, row, col):
     self.goal = (row,col)

   def getGoalRow(self):
     return self.goal[0]

   def getGoalCol(self):
     return self.goal[1]

   def printGrid(self, event):
      print "Printing Current grid to stderr."
      sys.__stderr__.write("[\n")
      for i in range(self.rows):
        sys.__stderr__.write(" [ ")
        for j in range(self.rows):
	  if (j > 0):
            sys.__stderr__.write(", ")
          sys.__stderr__.write("%s" % self.grid[i][j])
	if (i < (self.rows-1)):
          sys.__stderr__.write(" ],\n")
	else:
          sys.__stderr__.write(" ]\n")
      sys.__stderr__.write("]\n")

   def setCurrent(self, row, col, th=0):
     self.current = (row,col,th)
     self.redrawTrig.set(1);

   def getCurrentRow(self):
     return self.current[0]

   def getCurrentCol(self):
     return self.current[1]

   def getCurrentTh(self):
     return self.current[2]

   def setCurrentFromEvent(self, event):
     clickCell = self.eventToCell(event)
     self.setCurrent(row=clickCell[1],col=clickCell[0],th=0)
     print ("current is now",self.current)
     self.redraw(self.grid)

   def findPath(self, event=None):
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
      if pathGrid:
         print "Done!"
         self.redraw(self.distanceToGoal, pathGrid)
      else:
         raise Exception('NoPathExists', 'maximum interation limit exceded')
      return pathList

   def setupDirtyCells(self):
     self.dirtyCells = []
     self.dirtyLock = threading.Lock()

   def addDirtyCells(self,row,col):
     if (col < self.cols and row < self.rows):
       self.dirtyLock.acquire()
       self.dirtyCells.append((row,col))
       self.dirtyLock.release()

   def getAndResetDirtyCells(self):
     self.dirtyLock.acquire()
     tmpDirty = list(sets.Set(self.dirtyCells))
     self.dirtyCells = []
     self.dirtyLock.release()
     return tmpDirty

   def redrawTrigCallback(self, name, index, mode):
     self.redraw(self.grid)
     self.redrawTrig.set(0);

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
				      fill = "blue",
				      tag = "cell")

   def tooTight(self, row, col, i, j):
      """ Check to see if you aren't cutting a corner next to an obstacle.
          The assumption is made that the absolute value of both i and j
	  is the same, i.e, that the robot is considering the
	  possibility of moving in a diagonal line.  The idea is to
	  check the two adjacent regions which will be partially crossed
	  while the diagonal movement is made.  """
      return self.distanceToGoal[row + i][col] == self.infinity or \
                 self.distanceToGoal[row][col + j] == self.infinity


   def calculateAllCosts(self, goalCol, goalRow, maxIters):
      """
      Path planning algorithm is based on the value iteration algorithm
      given by Thrun et al. in the chapter 'Map learning and high-speed
      navigation in Rhino' from the book 'Artificial Intelligence and
      Mobile Robots' edited by Kortenkamp, Bonasso, and Murphy.

      Made two key changes to the algorithm given.
      1. When an occupancy probability is above some threshold, assume
         that the cell is occupied and set its value for search to
         infinity.
      2. When iterating over all cells to update the search values, add
         in the distance from the current cell to its neighbor.  Cells
         which are horizontal or vertical from the current cell are
         considered to be a distance of 1, while cells which are diagonal
         from the current cell are considered to be a distance of 1.41.
      """

      if (goalCol == self.lastGoalDistCol and
          goalRow == self.lastGoalDistRow):
         return

      startTime = time.time()
      print "Calculating all costs given the current goal."
      self.distanceToGoal=[[self.infinity for col in range(self.cols)]
                   for row in range(self.rows)]
      if not self.inRange(goalRow, goalCol):
         raise Exception('goalOutOfMapRange')
      self.distanceToGoal[goalRow][goalCol] = 0.0

      for iter in range (maxIters):
	 valuesChanged = 0
	 if (self.showIterations):
            print "Displaying result after iteration:",iter
            self.redraw(self.distanceToGoal, stepByStep=1)
	    self.frame.update()
            time.sleep(1)
         for row in range(self.rows):
            for col in range(self.cols):
               for i in [-1,0,1]:
                  for j in [-1,0,1]:
                     if self.inRange(row+i, col+j):
                        if self.grid[row][col] > self.threshhold:
                           self.distanceToGoal[row][col] = self.infinity
                        else:
                           if abs(i) == 0 and abs(j) == 0:
                              d = 0.00
                           elif abs(i) == 1 and abs(j) == 1:
                              if self.tooTight(row, col, i, j):
                                 d = self.infinity
			      else:
                                 d = 1.41
                           else:
                              d = 1.00
                           adj = self.distanceToGoal[row+i][col+j] + self.grid[row+i][col
+j] + d
			   if (adj < self.distanceToGoal[row][col]):
			     valuesChanged += 1
                           self.distanceToGoal[row][col] = min(self.distanceToGoal[row][col], adj)

         if valuesChanged == 0:
	   endTime = time.time()
	   print "Value iteration converged after %d iterations." % iter
	   print "Time elapsed %0.3f ms." % ((endTime-startTime)*1000.0)
	   break

   def getPath(self, row, col):
      pathGrid = [[0 for c in range(self.cols)] for r in range(self.rows)]
      pathList = []
      steps = 0
      while (not(self.distanceToGoal[row][col] == 0.0)):
         pathGrid[row][col] = 1
         pathList.append((row,col))
         bestStep = self.bigButNotInfinity
         nextRow = -1
         nextCol = -1
         for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
               if self.inRange(row+i, col+j):
                  if self.distanceToGoal[row+i][col+j] < bestStep and \
                         not self.tooTight(row, col, i, j):
                     bestStep = self.distanceToGoal[row+i][col+j]
                     nextRow = row+i
                     nextCol = col+j
         if nextRow == -1:
            raise Exception('NoPathExists')
         steps += 1
         row = nextRow
         col = nextCol
      pathGrid[row][col] = 1
      pathList.append((row,col))
#      print "Path is %d steps" % steps
      return (pathGrid,pathList)

   """
   -------------------------------------------------------------------------
   getProb: compute the probability that a given location is NOT occupied
            including previous readings.  This selects the update
	    formula from amongst those implemented.
   -------------------------------------------------------------------------
     prevVal: get probability from previous iterations
     certainty: factor of certainty, -1 = throw out, otherwise it reduces
                or adds a bit depending on whether it is a hit or not.
   """
   def getProb(self, prevVal, newInfo=1):
     if (self.gridUpdate == "Thrun93"):
       return self.getProbThrun(prevVal, newInfo)
     elif (self.gridUpdate == "Konolige"):
       return self.getProbKonolige(prevVal, newInfo)
     else:
       return self.getProbMatthiesElfes(prevVal, newInfo)

   def getProbMatthiesElfes(self, prevVal, probOcc=1):

      # Calculation based upon Matthies and Elfes 1988: "Integration of
      # Sonar and Stereo Range Data Using a Grid-Based Representation",
      # a Bayesian approach:
      # (assume Occ(xy)=1-Empty(xy)=Map(xy))

      # inverse probability of old probability
      prevValInv = 1 - prevVal
      # inverse of the current probability, 
      probEmpty = 1 - probOcc

      return( (probOcc * prevVal) / 
              ( (probOcc * prevVal) + (probEmpty * prevValInv) ) )
   
   def getProbThrun(self, prevVal, probOcc=1, certainty=0):
      
      # calculation from Thrun's '93 paper "Exploration and Model
      # Building in Mobile Robot Domains"

      # I am assuming than occ(xy) = 1-Empty(xy)=Map(xy)!!!
      # inverse probability of old probability
      prevValInv = 1 - prevVal
      # inverse of the current probability
      probEmpty = 1 - probOcc

      return (1 - math.pow((1 +((probOcc/probEmpty)*(prevValInv/prevVal))),-1))
   
   
   def getProbKonolige(self, prevVal, probOcc=1, certainty=0):

      # Calculation based upon Konolige '97 "Improved occupancy grids
      # for Map Building".

      # I am assuming than occ(xy) = 1-Empty(xy)=Map(xy)!!!
      # inverse probability of old probability
      prevValInv = 1 - prevVal
      # inverse of the current probability, 
      probEmpty = 1 - probOcc

      return( math.log((prevValInv*probOcc),10) * 
      		math.log((prevVal*probEmpty),10))	

   ### check if a given cell (yPos, xPos) has any known neighbor cell
   def hasKnownNeighborCell(self, yPos, xPos) :
     for y in range(yPos - 1, yPos + 2) :
	for x in range(xPos - 1, xPos + 2) :
	  if (self.grid[y][x] < 0.2 and x != xPos and y != yPos) :
	    return True
     return False

   # funcion to calculate the transformation between this grid and
   # another, we assume that anchor passed as 'anchor1' will be the axis
   # of the rotation.
   def calculateTransformation(self, oGrid, otherGridAnchor1Row,
                               otherGridAnchor1Col, otherGridAnchor2Row,
			       otherGridAnchor2Col, thisGridAnchor1Row,
			       thisGridAnchor1Col, thisGridAnchor2Row,
			       thisGridAnchor2Col):

      # marker1 will be the axis of the rotation so we start by
      # calculating its translation.
      translationRow = otherGridAnchor1Row - thisGridAnchor1Row
      translationCol = otherGridAnchor1Col - thisGridAnchor1Col

      # to calculate the angle of rotation we first apply the
      # translation to marker2 (we don't have to calculate the
      # translation of marker1 because we already know that it is the
      # same as the other grid's marker1 location so we just use that)
      thisGridAnchor2TransRow = thisGridAnchor2Row + translationRow 
      thisGridAnchor2TransCol = thisGridAnchor2Col + translationCol

      thisAngle = robotUtils.getAbsAngle(centerX=otherGridAnchor1Row,
				   centerY=otherGridAnchor1Col,
				   headingX=thisGridAnchor2TransRow, 
				   headingY=thisGridAnchor2TransCol)

      otherAngle = robotUtils.getAbsAngle(centerX=otherGridAnchor1Row,
				   centerY=otherGridAnchor1Col,
				   headingX=otherGridAnchor2Row, 
				   headingY=otherGridAnchor2Col)

      angle = otherAngle-thisAngle

      return (translationRow,translationCol,angle,
              otherGridAnchor1Row,otherGridAnchor1Col)

   # given a point in another grid and all the transformation
   # information calculate the corresponding point in this grid.
   def getRelativePosInOtherGrid (self, row, col, oGrid, rowOffset, colOffset, 
                                  angle, axisRow, axisCol):

      #first we translate the point
      translatedCellRow = row + rowOffset
      translatedCellCol = col + colOffset

      # row = cos(angle)(x1 - x0) - sin(angle)(y1 - y0) + x0
      # with (x0,y0) the axis and (x1,y1) the cell to translate

      row = ((math.cos(angle)*(translatedCellRow - axisRow)) -
     	     (math.sin(angle)*(translatedCellCol - axisCol)) + axisRow)

      # col = sin(angle)(x1 - x0) + cos(angle)(y1 - y0) + y0
      # with (x0,y0) the axis and (x1,y1) the cell to translate
      col = ((math.sin(angle)*(translatedCellRow - axisRow)) +
   	     (math.cos(angle)*(translatedCellCol - axisCol)) + axisCol)

      return (int(round(row)),int(round(col)))

   """
   This method extracts all the information from another grid to include
   it in this grid.  It starts by looking for two common markers in this
   and the other grid.  If 2 can not be found it fails, otherwise it
   uses these markers to calculate the transformation between the grids
   and then combines the information using this transformation and the
   grid update algorithms.
   """
   def mergeOtherGridInfo(self, otherGrid, otherGridAnchor1Row,
                          otherGridAnchor1Col, otherGridAnchor2Row,
                          otherGridAnchor2Col, thisGridAnchor1Row,
                          thisGridAnchor1Col, thisGridAnchor2Row,
                          thisGridAnchor2Col):

      res = otherGrid.calculateTransformation(oGrid = self, 
                          thisGridAnchor1Row = otherGridAnchor1Row,
                          thisGridAnchor1Col = otherGridAnchor1Col, 
                          thisGridAnchor2Row = otherGridAnchor2Row,
                          thisGridAnchor2Col = otherGridAnchor2Col, 
                          otherGridAnchor1Row = thisGridAnchor1Row,
                          otherGridAnchor1Col = thisGridAnchor1Col, 
                          otherGridAnchor2Row = thisGridAnchor2Row,
                          otherGridAnchor2Col = thisGridAnchor2Col)

      for row in range(otherGrid.rows):
         for col in range(otherGrid.cols):

            # Note: the return is row,col 
	    cellTransf = otherGrid.getRelativePosInOtherGrid(row=row, col=col,
	                  oGrid=self, rowOffset=res[0], colOffset=res[1], 
			  angle=res[2], axisRow=res[3], axisCol=res[4])
	    
	    cellTransfRow = cellTransf[0]
	    cellTransfCol = cellTransf[1]

	    # avoiding invalid grid locations... (dsager, 2009-05-22)
	    if (cellTransfRow >=0 and cellTransfRow < self.rows and 
	        cellTransfCol >=0 and cellTransfCol < self.cols):

  	      myCurrentValue = self.getGridLocation(row=cellTransfRow,
	                                            col=cellTransfCol,
						    absolute=1)
 	      otherCurrentValue = otherGrid.getGridLocation(row=row,
	                                                    col=col,
							    absolute=1)

	      newProb = self.getProb(myCurrentValue, otherCurrentValue)

	      # the new value will be the result of applying the grid
	      # update algorithm to the values of the two cells.
 	      self.setGridLocation(col=cellTransfCol,
		                   row=cellTransfRow,
				   value=newProb,label=0,absolute=1)

if __name__ == '__main__':

###########################
# ----> Test two smaller grids
###########################
#   g = OccupancyGrid(cols=5,
#                     rows=5,
#		      widthMM=50,
#		      heightMM=50,
#		      image='map-images/5x5world.gif',
#		      showIter=1,
#		      gridResize = 30)
#   g = OccupancyGrid(cols=10,
#                     rows=30,
#		      widthMM=50,
#		      heightMM=50,
#		      image='map-images/10x30world.gif',
#		      showIter=0,
#		      gridResize = 30)

###########################
# ----> Test floor scan grid
###########################
# floor-scan-scaled.png 500x1004 pixels
# map scale information:
#   width 152 pixels = 782cm
#   height 227 pixels = 1120cm
   g = OccupancyGrid(currentCol=4,currentRow=4,
                     goalCol=4, goalRow=46,
		     cols=64,  # approx 40cm each
		     rows=124, # approx 40cm each
		     widthMM=float(25724),  # map width in mm
		     heightMM=float(49537), # map height in mm
		     image='map-images/lab-map-scaled.png',
		     gridResize = 0.90)
   g.application = 1
   g.mainloop()
