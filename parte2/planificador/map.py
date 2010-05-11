# pyrobot/map/__init__.py

class Map:
    """ Basic class for robot maps"""

    def __init__(self, cols, rows, widthMM, heightMM):
        """ Constructor """

        self.cols = cols
        self.rows = rows
        self.widthMM = widthMM
        self.heightMM = heightMM
        self.originMM = self.widthMM / 2.0, self.heightMM / 2.0
        self.colScaleMM = self.widthMM / float(self.cols)
        self.rowScaleMM = self.heightMM / float(self.rows)
        self.reset()

    def reset(self, value = 0.5):
        self.grid = [[value for col in range(self.cols)]
                     for row in range(self.rows)]
        self.label = [['' for col in range(self.cols)]
                      for row in range(self.rows)]

    def getLabel(self, row, col):
        return label[row][col]

    def getAbsFromRel(self,row,col):
        return int(round((self.originMM[1] - row) / self.rowScaleMM)), \
               int(round((self.originMM[0] + col) / self.colScaleMM))

    def setGridLocation(self, row, col, value, label = None, absolute = 0):
        if( absolute == 0 ):
            modRow,modCol = self.getAbsFromRel(row,col)
        else:
            modRow = row
            modCol = col

        if self.inRange(modRow, modCol):
            # if hit was already detected, leave it alone!
            ##if self.grid[modRow][modCol] != 1.0:
            self.grid[modRow][modCol] = value
            if label != None:
                self.label[modRow][modCol] = "%s" % label
        else:
	    pass 
            # print "INVALID SET GRID LOCATION (%d,%d)" % (modRow,modCol)

    def getGridLocation(self, row, col, absolute = 0 ):
        if( absolute == 0 ):
            modRow,modCol = self.getAbsFromRel(row,col)
        else:
            modRow = row
            modCol = col

        if self.inRange(modRow, modCol):
            return( self.grid[modRow][modCol] )
        else:
	    pass
            # print "INVALID GET GRID LOCATION (%d,%d)" % (modRow,modCol)
            return( -1 )
        
    def inRange(self, row, col):
        return row >= 0 and row < self.rows and col >= 0 and col < self.cols

    def display(self, m = None):
        if m == None: m = self.grid
        for row in range(self.rows):
            for col in range(self.cols):
                print "%8.2f" % m[row][col],
            print
        print "-------------------------------------------------"

    def setGrid(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.colScaleMM = self.widthMM / float(self.cols)
        self.rowScaleMM = self.heightMM / float(self.rows)
        self.label = [['' for col in range(self.cols)]
                      for row in range(self.rows)]

    def validateGrid(self):
        print "Validating Grid: Checking bounds (%d, %d)..." % \
              (self.rows, self.cols),
        for r in range(self.rows):
            for c in range(self.cols):
                assert(self.inRange(r, c))
        print "done!"
                
if __name__ == '__main__':
    print "Testing Map()..."
    map = Map(cols=8, rows=10, widthMM=500, heightMM=1000)
    map.display()
    map.reset()
    map.display()
    print "Setting Grid location..."
    map.setGridLocation(col=200, row=450, value=1.0, label="A")
    print "setting (9,7) to 1.0"
    map.setGridLocation(col=7, row=9, value=1.0, absolute=1)
    print "get (9,7)", map.getGridLocation(col=7,row=9,absolute=1)
    map.validateGrid()
    print "Setting Grid to new size..."
    map.setGrid( [[0, 0, 0],
                  [0, 1, 0],
                  [0, 0, 0],
                  [1, 0, 0]] )
    map.validateGrid()
    map.display()
    print "All done!"
