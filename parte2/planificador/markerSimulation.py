"""
A wrapper for the TkSimulator to add markers to the simulator
"""

import math
from pyrobot.simulators.pysim import TkSimulator, Segment
from robotUtils import getAbsAngle, normalizeAngle2, calculateHypotenuse

class MarkerSimulation(TkSimulator):

   CAMERA_FIELD_OF_VIEW = 40*(math.pi/180)
   MARKER_VIEWABLE_ANGLE = 60*(math.pi/180)

   def __init__(self, dimensions, offsets, scale, root = None, run = 1):
        TkSimulator.__init__(self, dimensions=dimensions,
	                     offsets=offsets, scale=scale, root=root,
			     run=run)
	self.resetMarkers()

   def redraw(self):
      # first we call the redraw of the super-class
      TkSimulator.redraw(self)
      # we now add our markers
      for (markerX,markerY) in self.markerLoc.keys():
         markerName = self.markerLoc[(markerX,markerY)]
	 markerAngle = self.markerAngle[markerName]

	 tipX = markerX+(.25*math.cos(markerAngle))
	 tipY = markerY+(.25*math.sin(markerAngle))
	 rightX = markerX+(.25*math.cos(markerAngle+(math.pi/2)))
	 rightY = markerY+(.25*math.sin(markerAngle+(math.pi/2)))
	 leftX = markerX+(.25*math.cos(markerAngle+(math.pi)))
	 leftY = markerY+(.25*math.sin(markerAngle+(math.pi)))

	 self.canvas.create_polygon([(self.scale_x(tipX),self.scale_y(tipY)),
	                             (self.scale_x(leftX),self.scale_y(leftY)),
	                             (self.scale_x(rightX),self.scale_y(rightY))],
	                            tag="line", fill='green')
	 self.canvas.create_text(self.scale_x(markerX),
	                         self.scale_y(markerY), text=markerName,
				 fill='red')

   def dumpMarkers(self):
      print self.markers

   def resetMarkers(self):
      self.markers = {}
      self.markerLoc = {}
      self.markerAngle = {}
      self.redraw()

   def addMarker(self,xM,yM,name,angle=0):
      self.markers[name] = (xM,yM)
      self.markerLoc[(xM,yM)] = name
      self.markerAngle[name] = angle
      self.redraw()
 
   def getVisibleMarkers(self): # !!! ,robot):
      robot = self.robots[0]
      foundMarkers = []
      # print "robot", robot._gx, robot._gy, robot._ga

      for (markerX,markerY) in self.markerLoc.keys():

         markerName = self.markerLoc[(markerX,markerY)]
	 markerAngle = self.markerAngle[markerName]
         # print "marker", markerName, markerX, markerY, markerAngle

	 distanceToMarker = calculateHypotenuse(robot._gx-markerX,
	                                        robot._gy-markerY)

         # get the angle to the marker, and its relation to the
	 # orientation of the robot
         angleToMarker = getAbsAngle(centerX=robot._gx, centerY=robot._gy, 
	                             headingX=markerX, headingY=markerY)

	 cameraAngleDiff = normalizeAngle2(angleToMarker-robot._ga+math.pi)

	 # we do the same but this time from the perspective of the
	 # marker 
         angleFromMarker = getAbsAngle(centerX=markerX, centerY=markerY,
	                               headingX=robot._gx, headingY=robot._gy)

	 markerViewDiff = normalizeAngle2(angleFromMarker-markerAngle+math.pi)

	 # print angleToMarker,cameraAngleDiff,",",angleFromMarker,markerViewDiff

	 # see if the marker is in the camera's field of view
         # CAMERA_FIELD_OF_VIEW/2 = 0.35
	 # and that the marker is at a viewable angle from the robot
	 # MARKER_VIEWABLE_ANGLE/2 = 0.52
	 if (cameraAngleDiff < self.CAMERA_FIELD_OF_VIEW/2 and
	     cameraAngleDiff > self.CAMERA_FIELD_OF_VIEW/2*-1 and
	     markerViewDiff < self.MARKER_VIEWABLE_ANGLE/2 and
	     markerViewDiff > self.MARKER_VIEWABLE_ANGLE/2*-1):

   	    # see if that line is clear, i.e., doesn't intersect any walls
   	    seg = Segment((robot._gx, robot._gy), (markerX, markerY))
   	    clear = True
   	    for w in self.world:
   	       if (w.intersects(seg)):
		  # print "in the way", w
                  clear = False
   	          break
   
            if (clear):
	       # print "I can see ",markerName
	       relXOffM = math.cos(cameraAngleDiff)*distanceToMarker
	       relYOffM = math.sin(cameraAngleDiff)*distanceToMarker
	       foundMarkers.append([relXOffM,relYOffM,
	                            markerViewDiff,markerName])

      return foundMarkers
