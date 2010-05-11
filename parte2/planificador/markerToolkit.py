try:
  import locateMarkers
  artoolkitAvailable = True
except ImportError:
  artoolkitAvailable = False

class MarkerToolkit:
  """
  Wrapper for the locateMarkers library (generated from C by swig)
  """

  def __init__(self):
  # Start up Artoolkit
    if (artoolkitAvailable):
      locateMarkers.setupARToolKit()

  # a sample return:
  # [0.76380476990703006, 0.19719272472233135, 0.68975951645812705, 'A']
  #  relativeXOffsetM, relativeYOffsetM, relTh, markerName

  def getMarkerData(self, robot):
     if (artoolkitAvailable):
        return unpackArtoolkitData(locateMarkers.getData())
     else:
        return eval(robot.simulation[0].eval("self.getVisibleMarkers()"))

  def unpackArtoolkitData(self,res):
  # Unroll the lisp-like structure returned by artoolkit into an
  # array.

    nowIndex = 0
    markers = []

    if (len(res) > 0):
      data = res.pop(0)
      res = res.pop(0)
      res = res.pop(0)
  
      while (len(data) > 0 ):
         markers.append(data)
         # remove the directory info and the patt. from the pattern name
         # before returning it
         index = markers[nowIndex][3].find("patt.")
         markers[nowIndex][3] = markers[nowIndex][3][index+5:len(markers[nowIndex][3])].upper()
         nowIndex=nowIndex+1
         if (len(res) > 0):
           data = res.pop(0)
         else:
           data = []
         if (len(res) > 0):
           res = res.pop(0)
         if (len(res) > 0):
           res = res.pop(0)

    return markers
