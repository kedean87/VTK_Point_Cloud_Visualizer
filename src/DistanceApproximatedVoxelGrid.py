import vtk
# import pcl
import numpy as np

from utils import *
from VTKShaderPresets import *
from scipy.spatial.distance import euclidean
from vtk.util import numpy_support as ns

class DistanceApproximatedVoxelGrid(VTKShaders):
  def __init__(self, handleVTKCloud):
    self.vtkShaders = VTKShaders()
    self.handleVTKCloud = handleVTKCloud
  
  def execute(self, obj, event):
    if self.handleVTKCloud.frustum:
      self.traverse(self.handleVTKCloud.root)
  
  def getDistance(self, volumeCenter, windowViewPosition):
    return euclidean(volumeCenter, windowViewPosition)
  
  def getVolumeCenter(self, bounds):
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    x = (xmin + xmax) / 2.0
    y = (ymin + ymax) / 2.0
    z = (zmin + zmax) / 2.0
    
    return (x, y, z)
  
  def convertToVTKBounds(self, bounds):
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    return [xmin, xmax, ymin, ymax, zmin, zmax]
  
  def getData(self, data=None, subV=None, newPoints=None, newColors=None, locator=None):
    Id = locator.FindClosestPoint(self.getVolumeCenter(subV))
    delta = [float()]*3
    
    if not vtk.vtkMath.PointIsWithinBounds(data.GetPoint(Id), self.convertToVTKBounds(subV), delta):
      return
      
    newPoints.InsertNextPoint(data.GetPoint(Id))
    newColors.InsertNextTuple(data.GetPointData().GetScalars().GetTuple(Id))
  
  # ~ def voxelGrid(self, data=None, numSubV=10000, locator=None):
    # ~ subVolumes = CreateSubVolumes(data.GetBounds(), numSubV=numSubV)
    
    # ~ if not subVolumes:
      # ~ return None
    
    # ~ newPoints = vtk.vtkPoints()
    # ~ newColors = vtk.vtkUnsignedCharArray()
    # ~ newColors.SetNumberOfComponents(3)
    # ~ newColors.SetName("RGB")
    
    # ~ res = list(map(lambda i: self.getData(data, i, newPoints, newColors, locator), subVolumes))
    
    # ~ pd = vtk.vtkPolyData()
    # ~ pd.SetPoints(newPoints)
    # ~ pd.GetPointData().SetScalars(newColors)
    
    # ~ vgf = vtk.vtkVertexGlyphFilter()
    # ~ vgf.SetInputData(pd)
    # ~ vgf.Update()
    
    # ~ return vgf.GetOutput()

  def randomSample(self, rootNode=None, preset=None, factor=3):
    if factor == 0.0 or factor < 0.0:
      factor = 1

    data = rootNode.data.bstPolyData
    points = ns.vtk_to_numpy(data.GetPoints().GetData())[::factor]
    colors = ns.vtk_to_numpy(data.GetPointData().GetScalars())[::factor]
    
    #x_ = points.T[0][::factor]
    #y_ = points.T[1][::factor]
    #z_ = points.T[2][::factor]
    
    #r_ = colors.T[0][::factor]
    #g_ = colors.T[1][::factor]
    #b_ = colors.T[2][::factor]
    
    #p = np.concatenate([x_.reshape(len(x_), 1), y_.reshape(len(y_), 1), z_.reshape(len(z_), 1)], axis=1)
    #c = np.concatenate([r_.reshape(len(r_), 1), g_.reshape(len(g_), 1), b_.reshape(len(b_), 1)], axis=1)
    
    # convert data from numpy array to vtkPoints
    vtkPoints = vtk.vtkPoints()
    vtkPoints.SetData(ns.numpy_to_vtk(points, deep=True))
    
    # convert color data from numpy array to vtkUnsignedCharArray
    vtkColorArray = ns.numpy_to_vtk(colors, array_type=vtk.VTK_UNSIGNED_CHAR)
    vtkColorArray.SetName("RGB")
    
    # create a new polydata to store points, vertices, and color
    newPd = vtk.vtkPolyData()
    newPd.SetPoints(vtkPoints)
    newPd.GetPointData().SetScalars(vtkColorArray)
    
    # this vertex filter creates cell data for the polygonal output
    vgf = vtk.vtkVertexGlyphFilter()
    vgf.SetInputData(newPd)
    vgf.Update()
    
    rootNode.data.bstActor.SetMapper(self.getMapper(rootNode, vgf.GetOutput(), preset))

#----------------------------------------------------------------------#
  def voxelGrid(self, rootNode=None, preset=None, numSubV=1000):
    data = rootNode.data.bstPolyData
    locator = rootNode.data.pLocator
    
    volumeBounds = [float()]*6
    data.GetBounds(volumeBounds)
    
    if volumeBounds == None:
      print("No Volume to Subdivide, returning")
      return
  
    vertices = vtk.vtkCellArray()
    points = vtk.vtkPoints()
    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    colors.SetName("RGB")
    
    xLen = abs(volumeBounds[1] - volumeBounds[0])
    yLen = abs(volumeBounds[3] - volumeBounds[2])
    zLen = abs(volumeBounds[5] - volumeBounds[4])
    
    x_incr = xLen / numSubV**(1/3)
    y_incr = yLen / numSubV**(1/3)
    z_incr = zLen / numSubV**(1/3)
    
    if x_incr == 0.0 or y_incr == 0.0 or z_incr == 0.0:
      return
    
    x_vals, y_vals, z_vals, = [],  [], []
    
    x = volumeBounds[0]
    x_vals .append(x)
    x += x_incr
    
    for i in range(int(np.round(xLen / x_incr))):
      x_vals.append(x)
      x += x_incr
    
    y = volumeBounds[2]
    y_vals .append(y)
    y += x_incr
    
    for i in range(int(np.round(yLen / y_incr))):
      y_vals.append(y)
      y += y_incr
    
    z = volumeBounds[4]
    z_vals .append(z)
    z += z_incr
    
    for i in range(int(np.round(zLen / z_incr))):
      z_vals.append(z)
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
          
          bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
          ids = vtk.vtkIdTypeArray()
          locator.FindPointsInArea(bounds, ids)
          
          if ids.GetNumberOfTuples() == 0:
            continue
          
          Id = locator.FindClosestPoint(((xmax+xmin)/2., (ymax+ymin)/2., (zmax+zmin)/2.))
          newId = points.InsertNextPoint(data.GetPoint(Id))
          colors.InsertNextTuple(data.GetPointData().GetScalars().GetTuple(Id))
          vertices.InsertNextCell(1)
          vertices.InsertCellPoint(newId)
          
    pd = vtk.vtkPolyData()
    pd.SetPoints(points)
    pd.SetVerts(vertices)
    pd.GetPointData().SetScalars(colors)
    
    rootNode.data.bstActor.SetMapper(self.getMapper(rootNode, pd, preset))
  
  def getSlope(self, x1, y1, x2, y2):
    return (y2-y1)/(x2-x1)
  
  def computeSamplingFactor(self, distance, min_factor=1, max_factor=50, distance_scale=100.0):
    """
    Smoothly increases the sampling stride with distance.
    distance_scale ~ maximum distance where full resolution is desired
    """
    normalized = np.clip(distance / distance_scale, 0, 1)
    factor = min_factor + (max_factor - min_factor) * (normalized ** 2)  # quadratic smooth curve
    return int(factor)

  def computeSplatScale(self, distance, base=0.02, growth=0.005, min_scale=0.003, max_scale=0.15):
    """
    Increase splat size smoothly with distance.
    base: splat size for nodes very close to camera
    growth: how much splat size increases per unit distance
    """
    scale = base + growth * distance
    return float(np.clip(scale, min_scale, max_scale))

  
  def traverse(self, rootNode):
    if not rootNode:
        return

    for child in [rootNode.left, rootNode.right]:
        if not child:
            continue
        self.traverse(child)
        
        if child.data:
            cam = self.handleVTKCloud.renderer.GetActiveCamera()
            normal = [0.0, 0.0, 0.0]
            cam.GetEyePlaneNormal(normal)
            cam_pos = np.array(cam.GetPosition())
            ref_point = cam_pos + 1 * np.array(normal)
            
            dist = self.getDistance(child.data.bstPolyData.GetCenter(), ref_point)
            
            splat_scale = self.computeSplatScale(dist)
            sample_factor = self.computeSamplingFactor(dist)
            
            # Optional fade for distant nodes
            opacity = np.clip(1.0 - (dist / 10.0)**0.5, 0.2, 1.0)
            child.data.bstActor.GetProperty().SetOpacity(opacity)

            if sample_factor > 1:
                self.randomSample(child, preset=[self.handleVTKCloud.current_vtk_shader, splat_scale], factor=sample_factor)
            else:
                child.data.bstActor.SetMapper(
                    self.getMapper(child, child.data.bstPolyData, [self.handleVTKCloud.current_vtk_shader, splat_scale])
                )
          
  def getMapper(self, rootNode, newData, shaderPresetIndex):
    pg_mapper = vtk.vtkPointGaussianMapper()
    pg_mapper.SetInputData(newData)
    pg_mapper.SetScaleArray('radius')
    pg_mapper.SetScalarVisibility(True)
    pg_mapper.SetScaleFactor(shaderPresetIndex[1])
    pg_mapper.EmissiveOff()
    pg_mapper.SetSplatShaderCode(self.vtkShaders.Shaders[shaderPresetIndex[0]][0])
    
    return pg_mapper
