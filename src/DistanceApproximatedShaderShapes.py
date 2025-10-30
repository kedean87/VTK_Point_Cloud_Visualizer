import vtk
from VTKShaderPresets import *
from scipy.spatial.distance import euclidean

class DistanceApproximatedShaderShapes(VTKShaders):
  def __init__(self, handleVTKCloud):
    self.vtkShaders = VTKShaders()
    self.handleVTKCloud = handleVTKCloud
  
  def execute(self, obj, event):
    if self.handleVTKCloud.frustum:
      self.traverse(self.handleVTKCloud.root)
  
  def getDistance(self, volumeCenter, windowViewPosition):
    return euclidean(volumeCenter, windowViewPosition)
  
  def traverse(self, rootNode):
    children = [rootNode.left, rootNode.right]
    for child in children:
      if child:
        self.traverse(child)
        
        if child.data:
          windowPose = self.handleVTKCloud.renderer.GetActiveCamera().GetPosition()
          dist = self.getDistance(child.data.bstPolyData.GetCenter(), windowPose)
          
          if dist < 200:
            child.data.bstActor.SetMapper(self.getMapper(child, 1))
          else:
            child.data.bstActor.SetMapper(self.getMapper(child, 3))
          
  def getMapper(self, rootNode, shaderPresetIndex):
    pg_mapper = vtk.vtkPointGaussianMapper()
    pg_mapper.SetInputData(rootNode.data.bstPolyData)
    pg_mapper.SetScaleArray('radius')
    pg_mapper.SetScalarVisibility(True)
    pg_mapper.SetScaleFactor(self.vtkShaders.Shaders[shaderPresetIndex][1])
    pg_mapper.EmissiveOff()
    pg_mapper.SetSplatShaderCode(self.vtkShaders.Shaders[shaderPresetIndex][0])
    
    return pg_mapper
