'''
Utility Functions that are utilized within the VTKVisualizer
'''

import vtk
import numpy as np
import asyncio
import binascii
import random
import zlib
import os
from vtk.util import numpy_support as ns
from utils import *

#----------------------------------------------------------------------#
def CreateSubVolumes(volumeBounds=None, numSubV=32):
  if volumeBounds == None:
    print("No Volume to Subdivide, returning")
    return

  subVolumes = []
  
  xLen = abs(volumeBounds[1] - volumeBounds[0])
  yLen = abs(volumeBounds[3] - volumeBounds[2])
  zLen = abs(volumeBounds[5] - volumeBounds[4])
  
  x_incr = xLen / numSubV**(1/3)
  y_incr = yLen / numSubV**(1/3)
  z_incr = zLen / numSubV**(1/3)
  
  x_vals, y_vals, z_vals, = [],  [], []
  
  iterating = True
  x = volumeBounds[0]
  while(iterating):
    x_vals.append(x)
    if x >= volumeBounds[1]:
      iterating = False
    x += x_incr
  
  iterating = True
  y = volumeBounds[2]
  while(iterating):
    y_vals.append(y)
    if y >= volumeBounds[3]:
      iterating = False
    y += y_incr
  
  iterating = True
  z = volumeBounds[4]
  while(iterating):
    z_vals.append(z)
    if z >= volumeBounds[5]:
      iterating = False
    z += z_incr
    
  for i in range(len(x_vals)):
    if (x_vals[i] == x_vals[-1]):
      break
    xmin = x_vals[i]
    xmax = x_vals[i + 1]
    
    for j in range(len(y_vals)):
      if (y_vals[j] == y_vals[-1]):
        break
      ymin = y_vals[j]
      ymax = y_vals[j + 1]
      
      for k in range(len(z_vals)):
        if (z_vals[k] == z_vals[-1]):
          break
        zmin = z_vals[k]
        zmax = z_vals[k + 1]
        
        subVolumes.append([xmin, ymin, zmin, xmax, ymax, zmax])
  
  return subVolumes
 
#----------------------------------------------------------------------#
def CreateSubVolumes2(volumeBounds=None, numSubV=[10, 10, 10]):
  if volumeBounds == None:
    print("No Volume to Subdivide, returning")
    return

  subVolumes = []
  
  xLen = abs(volumeBounds[1] - volumeBounds[0])
  yLen = abs(volumeBounds[3] - volumeBounds[2])
  zLen = abs(volumeBounds[5] - volumeBounds[4])
  
  x_incr = xLen / numSubV[0]
  y_incr = yLen / numSubV[1]
  z_incr = zLen / numSubV[2]
  
  x_vals, y_vals, z_vals, = [],  [], []
  
  iterating = True
  x = volumeBounds[0]
  while(iterating):
    x_vals.append(x)
    if x >= volumeBounds[1]:
      iterating = False
    x += x_incr
  
  iterating = True
  y = volumeBounds[2]
  while(iterating):
    y_vals.append(y)
    if y >= volumeBounds[3]:
      iterating = False
    y += y_incr
  
  iterating = True
  z = volumeBounds[4]
  while(iterating):
    z_vals.append(z)
    if z >= volumeBounds[5]:
      iterating = False
    z += z_incr
    
  for i in range(len(x_vals)):
    if (x_vals[i] == x_vals[-1]):
      break
    xmin = x_vals[i]
    xmax = x_vals[i + 1]
    
    for j in range(len(y_vals)):
      if (y_vals[j] == y_vals[-1]):
        break
      ymin = y_vals[j]
      ymax = y_vals[j + 1]
      
      for k in range(len(z_vals)):
        if (z_vals[k] == z_vals[-1]):
          break
        zmin = z_vals[k]
        zmax = z_vals[k + 1]
        
        subVolumes.append([xmin, ymin, zmin, xmax, ymax, zmax])
  
  return subVolumes

#----------------------------------------------------------------------#
def background(f):
  def wrapped(*args, **kwargs):
    return asyncio.get_event_loop().run_in_executor(None, f, *args, \
      **kwargs)
  
  return wrapped

#----------------------------------------------------------------------#
@background
def ExtractPointsToOctree(index, volume, cellIds, octree):
  if (index % 100000 == 0.0):
    print("\tExtracting Points " + str(index))

  p = np.array(volume.GetPoint(cellIds.GetId(index))) * UP_SCALE_FACTOR
  p = p.astype(int)
  args = floatsToString(p)
  
  octree.InsertNextPoint(args)

#----------------------------------------------------------------------#
@background
def ExtractPointsForOctree(index, volume, cellIds, octree):
  if (index % 100000 == 0.0):
    print("\tExtracting Points " + str(index))

  p = np.array(volume.GetPoint(cellIds.GetId(index))) * UP_SCALE_FACTOR
  p = p.astype(int)
  
  octree.InsertPoint(p[0], p[1], p[2])

#----------------------------------------------------------------------#
@background
def UpdatePoints(i, points, color, newPoints, newColors):
  if (i % 1000 == 0.0):
    print("\tExtracting Points " + str(i))

  newPoints.InsertNextPoint(points[Ids.GetId(i+1)])
  newColors.InsertNextTuple(color[Ids.GetId(i+1)])
  
  return np.delete(points, (Ids.GetId(i+1)), axis=0), \
    np.delete(color, (Ids.GetId(i+1)), axis=0)

#----------------------------------------------------------------------#
@background
def writeDataToFile(outFile=None, index=None):
  if index and index % 100000 == 0.0:
    print(index)
  
  if not index and outFile == None:
    print("The parameter conditions were not met, please check")
    return
    
  p = POINTS[index][0:3]
  c = COLORS[index][3:]
  
  args = p + c
  
  string = ','.join(str(i) for i in args) + '\n'
  outFile.write(string)

#----------------------------------------------------------------------#
def IsPointWithinBounds(index, volume, subVol):
  p = volume.GetPoint(index)
  if ( subVol[0] <= p[0] <= subVol[3] and
       subVol[1] <= p[1] <= subVol[4] and
       subVol[2] <= p[2] <= subVol[5] ):
    return p[0], p[1], p[2]
  else:
    return None

#----------------------------------------------------------------------#
def ReadVTK(filename):
  fileExtension = filename.split('.')[-1]
  
  reader = None
  
  if (fileExtension == 'vtp'):
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(filename)
    reader.Update()
  
  if (fileExtension == 'vti'):
    reader = vtk.vtkXMLImageDataReader()
    reader.SetFileName(filename)
    reader.Update()

  if not reader:
    print('Filetype - {0} - was not recognized')
    return None
  
  return reader

#----------------------------------------------------------------------#
def getPointDataArrays(subVol, POINTS, COLORS):
  bounds = [float()] * 6
  bounds[0] = subVol[0]
  bounds[1] = subVol[3]
  bounds[2] = subVol[1]
  bounds[3] = subVol[4]
  bounds[4] = subVol[2]
  bounds[5] = subVol[5]
  
  a = POINTS[np.where((POINTS.T[0] >= bounds[0]) & (POINTS.T[0] <= bounds[1]))]
  a = a[np.where((a.T[1] >= bounds[2]) & (a.T[1] <= bounds[3]))]
  a = a[np.where((a.T[2] >= bounds[4]) & (a.T[2] <= bounds[5]))]
  
  if len(a) == 0.0:
    print("There are no points to store, move to next sub volume")
    return None
    
  b = COLORS[a.T[3].astype(int)]
  
  data = (a, b)
  
  return data

#----------------------------------------------------------------------#
def getPointData(locator, subVol, POINTS, COLORS):
  
  # insert region point cloud data in the BST
  regionPtIds = vtk.vtkIdTypeArray()
  locator.FindPointsInArea(subVol, regionPtIds)
  
  Ids = ns.vtk_to_numpy(regionPtIds)
  p = POINTS[Ids]
  c = COLORS[Ids]
  
  if len(p) == 0.0:
    print("There are no points to store, move to next sub volume")
    return None
  
  return (p, c)

#----------------------------------------------------------------------#
def WriteBinaryOutput(outFileName="", dataArray=None):
  if not dataArray or outFileName == "":
    print('Empty Dataset')
    return
  
  np.savetxt(outFileName, dataArray)

#----------------------------------------------------------------------#
def LoadBinaryFile(inputFileName="", dataArray=None):
  import os
  if not os.path.exists(inputFileName):
    print("Input File Does NOT Exist")
    return
  
  return np.load(inputFileName)

#----------------------------------------------------------------------#
def fastInverseSqrt(number):
  threehalfs = 1.5
  x2 = number * 0.5
  y = number
  
  packed_y = struct.pack('f', y)
  i = struct.unpack('i', packed_y)[0]
  i = 0x5f3759df - (i >> 1)
  packed_i = struct.pack('i', i)
  y = struct.unpack('f', packed_i)[0]
  
  y = y * (threehalfs - (x2 * y * y))
  return y

#----------------------------------------------------------------------#
def CalculateFrustumPlanes(vtkMatrix4x4):
  planes = [float()] * 24
  
  matrix = np.zeros([4, 4])
  for i in range(4):
    for j in range(4):
      matrix[i][j] = vtkMatrix4x4.GetElement(i, j)
  
  mT = matrix
  
  planes[0] = mT[3][0] - mT[0][0]
  planes[1] = mT[3][1] - mT[0][1]
  planes[2] = mT[3][2] - mT[0][2]
  planes[3] = mT[3][3] - mT[0][3]

  t = fastInverseSqrt(planes[0] * planes[0] + planes[1] * planes[1] + planes[2] * planes[2]);

  planes[0] *= t;
  planes[1] *= t;
  planes[2] *= t;
  planes[3] *= t;

  planes[4] = mT[3][0] + mT[0][0]
  planes[5] = mT[3][1] + mT[0][1]
  planes[6] = mT[3][2] + mT[0][2]
  planes[7] = mT[3][3] + mT[0][3]

  t = fastInverseSqrt(planes[4] * planes[4] + planes[5] * planes[5] + planes[6] * planes[6]);

  planes[4] *= t;
  planes[5] *= t;
  planes[6] *= t;
  planes[7] *= t;

  planes[8] = mT[3][0] - mT[1][0]
  planes[9] = mT[3][1] - mT[1][1]
  planes[10] = mT[3][2] - mT[1][2]
  planes[11] = mT[3][3] - mT[1][3]

  t = fastInverseSqrt(planes[8] * planes[8] + planes[9] * planes[9] + planes[10] * planes[10]);

  planes[8] *= t;
  planes[9] *= t;
  planes[10] *= t;
  planes[11] *= t;

  planes[12] = mT[3][0] + mT[1][0]
  planes[13] = mT[3][1] + mT[1][1]
  planes[14] = mT[3][2] + mT[1][2]
  planes[15] = mT[3][3] + mT[1][3]

  t = fastInverseSqrt(planes[12] * planes[12] + planes[13] * planes[13] + planes[14] * planes[14]);

  planes[12] *= t;
  planes[13] *= t;
  planes[14] *= t;
  planes[15] *= t;

  planes[16] = mT[3][0] - mT[2][0]
  planes[17] = mT[3][1] - mT[2][1]
  planes[18] = mT[3][2] - mT[2][2]
  planes[19] = mT[3][3] - mT[2][3]

  t = fastInverseSqrt(planes[16] * planes[16] + planes[17] * planes[17] + planes[18] * planes[18]);

  planes[16] *= t;
  planes[17] *= t;
  planes[18] *= t;
  planes[19] *= t;

  planes[20] = mT[3][0] + mT[2][0]
  planes[21] = mT[3][1] + mT[2][1]
  planes[22] = mT[3][2] + mT[2][2]
  planes[23] = mT[3][3] + mT[2][3]

  t = fastInverseSqrt(planes[20] * planes[20] + planes[21] * planes[21] + planes[22] * planes[22]);

  planes[20] *= t;
  planes[21] *= t;
  planes[22] *= t;
  planes[23] *= t;
  
  return planes


