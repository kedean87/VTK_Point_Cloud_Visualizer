from BinarySearchTree import *
from VTKShaderPresets import *
from BSTPointCloud import *
from FilterOccludedPoints import *
from NodeVisbilitiy import *

class UpdateFrustum():
  def __init__(self, handleVTKCloud):
    self.handleVTKCloud = handleVTKCloud
  
  def execute(self, obj, event):
    aspect = self.handleVTKCloud.renderer.GetAspect()
    aspectRatio = aspect[0] / aspect[1]
    planes = [float()]*24
    self.handleVTKCloud.renderer.GetActiveCamera().GetFrustumPlanes(aspectRatio, planes)
    
    # print(planes)
    vtkPlanes = vtk.vtkPlanes()
    vtkPlanes.SetFrustumPlanes(planes)
    
    # create a frustum source from those planes
    fs = vtk.vtkFrustumSource()
    fs.SetPlanes(vtkPlanes)
    fs.Update()
    
    self.handleVTKCloud.planes = fs.GetPlanes()
    self.handleVTKCloud.frustum = fs.GetOutput()
