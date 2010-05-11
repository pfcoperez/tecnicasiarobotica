from pyrobot.brain import Brain

from navigator import Navigator

class BrainTestNavigator(Brain):

  def setup(self):

     #################################
     # We start with a map built from a to-scale image of the floor-plan
     # of the location of the robot.  Then we localize in that 'perfect'
     # representation of the space.  (The location of the 'real world'
     # marker in the map was determined manually by placing the marker
     # on an easily identifiable feature of the image, like a pillar.)
     #################################
     # cropped map (only the downstairs lab area)
     ################################
     self.nav = Navigator(cols=26,  # approx 40cm each
  		       rows=38, # approx 40cm each
  		       widthMM=float(10383),  # map width in mm
  		       heightMM=float(15184), # map height in mm
     # lab-map-crop-scaled.png has 202x308 pixels
     # map scale information:
     #   xScale 0.0514 (m/pixel), yScale 0.0493 (m/pixel)
  		       image='map-images/lab-map-crop-scaled.png',
  		       gridResize = 2)   
     self.nav.placeMarker(row=29,col=6,name="A",angle=0, redraw=1)
     self.nav.placeMarker(row=19,col=13,name="B",angle=3.14, redraw=1)
     self.nav.setGoal(row=4,col=22)
################################
# full map (entire floor of the downstairs lab area)
################################
#     self.nav = Navigator(cols=64,  # approx 40cm each
#  		       rows=124, # approx 40cm
#  		       widthMM=float(25724),  # map width in mm 
#  		       heightMM=float(49537), # map height in mm 
## lab-map-scaled.png has 500x1004 pixels
## map scale information:
##   width 152 pixels = 782cm
##   height 227 pixels = 1120cm
#  		       image='map-images/lab-map-scaled.png',
#  		       imageResize = 0.90)   
#     self.nav.placeMarker(row=30,col=6,name="A",angle=0, redraw=1)
#     self.nav.placeMarker(row=19,col=14,name="B",angle=3.14, redraw=1)
#     self.nav.setGoal(row=4,col=22)
################################
     # then we use update the robot's location
     self.nav.updateRobotLocation(self.robot)
     try:
       self.pathList = self.nav.findPath()
     except:
       pass

  def step(self):

    # then we use update the robot's location
    self.nav.updateRobotLocation(self.robot)

    # if we have already arrived at the next subgoal we remove it and
    # move on to the next.
    if (self.pathList[0] == (self.nav.getCurrentRow(),self.nav.getCurrentCol())):
      print "I reached old subgoal",self.pathList[0]
      del self.pathList[0]

    print "current Location",self.nav.getCurrentRow(),self.nav.getCurrentCol()
    print "next subgoal",self.pathList[0]

    translation, rotate = self.nav.determineMove(self.pathList[0])
    self.robot.move(translation,rotate)
 
def INIT(engine):
  assert (engine.robot.requires("range-sensor") and
	  engine.robot.requires("continuous-movement"))

  # If we are allowed (for example you can't in a simulation), enable
  # the motors.
  try:
    engine.robot.position[0]._dev.enable(1)
  except AttributeError:
    pass

  return BrainTestNavigator('BrainTestNavigator', engine)
