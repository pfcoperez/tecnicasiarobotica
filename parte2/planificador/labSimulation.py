"""
A simulation of the lab space in the basement of the faculty.
"""

from markerSimulation import MarkerSimulation

from pyrobot.simulators.pysim import TkPioneer, \
     PioneerFrontSonars, PioneerFrontLightSensors

def INIT():
    # (width, height), (offset x, offset y), scale:
    # why 40.357554??
    sim = MarkerSimulation((500,700), (25,650), 40.357554)  
    # we are using pixels from the map, so we need to multiply by the
    # map's scale to get meters, also y is the reverse of what we really
    # want.
    xMax = 209
    yMax = 304
    xScale = 0.0514 # m/pixel
    yScale =  0.0493 # m/pixel

    # remember addBox(x1, y1, x2, y2)
    # All outer walls 7 thick.
    # top outer wall
    sim.addBox(0*xScale, (yMax-0)*yScale, 209*xScale, (yMax-7)*yScale)
    # left outer wall
    sim.addBox(0*xScale, (yMax-7)*yScale, 7*xScale, (yMax-297)*yScale)
    # right outer wall
    sim.addBox(202*xScale, (yMax-7)*yScale, 209*xScale, (yMax-297)*yScale)
    # bottom outer wall
    sim.addBox(0*xScale, (yMax-297)*yScale, 209*xScale, (yMax-304)*yScale)

    # inner walls - horizonal - left column top to bottom
    sim.addBox(8*xScale, (yMax-69)*yScale, 82*xScale, (yMax-73)*yScale)
    sim.addBox(8*xScale, (yMax-126)*yScale, 82*xScale, (yMax-130)*yScale)
    sim.addBox(8*xScale, (yMax-185)*yScale, 78*xScale, (yMax-189)*yScale)
    sim.addBox(8*xScale, (yMax-242)*yScale, 82*xScale, (yMax-246)*yScale)
    # inner walls - horizonal - right column top to bottom
    sim.addBox(126*xScale, (yMax-69)*yScale, 201*xScale, (yMax-73)*yScale)
    sim.addBox(126*xScale, (yMax-131)*yScale, 201*xScale, (yMax-135)*yScale)
    sim.addBox(126*xScale, (yMax-195)*yScale, 201*xScale, (yMax-199)*yScale)
    sim.addBox(126*xScale, (yMax-256)*yScale, 201*xScale, (yMax-260)*yScale)
    # inner walls - vertical - 1st column top to bottom
    sim.addBox(78*xScale, (yMax-73)*yScale, 82*xScale, (yMax-108)*yScale)
    sim.addBox(78*xScale, (yMax-148)*yScale, 82*xScale, (yMax-225)*yScale)
    sim.addBox(78*xScale, (yMax-263)*yScale, 82*xScale, (yMax-297)*yScale)
    # inner walls - vertical - 2st column top to bottom
    sim.addBox(102*xScale, (yMax-7)*yScale, 106*xScale, (yMax-71)*yScale)
    # inner walls - vertical - 3st column top to bottom
    sim.addBox(126*xScale, (yMax-73)*yScale, 130*xScale, (yMax-114)*yScale)
    sim.addBox(126*xScale, (yMax-153)*yScale, 130*xScale, (yMax-195)*yScale)
    sim.addBox(126*xScale, (yMax-217)*yScale, 130*xScale, (yMax-256)*yScale)
    # pillar
    sim.addBox(102*xScale, (yMax-151)*yScale, 107*xScale, (yMax-157)*yScale)

    # add the markers to the simulation
    # 7.7 pixels per col, 8.1 pixels per row
    # A is in row=29,col=6; x=46 y=235; we then center on the cell
    # B is in row=19,col=13; x=100 y=154
    sim.addMarker(50*xScale, (yMax-238)*yScale, "A", 0)
    sim.addMarker(104*xScale, (yMax-158)*yScale, "B", 3.14)

    # addRobot(port, robot)
    # robot(name, x, y, th, bounding Xs, bounding Ys, color (opt))
    # the erratic robot is 40cm long and 37 cm wide (18cm high)
    sim.addRobot(60000, TkPioneer("BlueErratic",
				  # +x right, +y up, +th counter-clock
                                  2.5, 5, 3.14,
                                  ((.185, .185, -.185, -.185),
                                   (.2, -.2, -.2, .2)),
				   color="blue"))
    # add some sensors:
    sim.robots[0].addDevice(PioneerFrontSonars())
    sim.robots[0].addDevice(PioneerFrontLightSensors())
    return sim
