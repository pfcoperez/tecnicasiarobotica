import math

# some generic utility functions

def normalizeAngle(angle):
   # make sure that the angle is between 0 and 2*pi
   return (angle % (math.pi*2))

def normalizeAngle2(angle):
   # make sure that the angle is between -pi and pi
   newAngle = (angle % (math.pi*2))
   if (newAngle > math.pi):
      return -1*((math.pi*2)-newAngle)
   else:
      return newAngle

def calculateHypotenuse(x,y):
   return math.sqrt(math.pow(x,2)+math.pow(y,2))

def getAbsAngle(centerX, centerY, headingX, headingY):
  # remember that the y axix is in the opposite direction from that
  # expected, thus the -1.
  return math.atan2(headingX-centerX,(headingY-centerY)*-1)

