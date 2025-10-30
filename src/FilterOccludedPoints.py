from BinarySearchTree import *
from VTKShaderPresets import *
from BSTPointCloud import *
from Frustum import *
from NodeVisbilitiy import *

class FilterOccludedPoints(VTKShaders):
  def __init__(self, handleVTKCloud):
    self.handleVTKCloud = handleVTKCloud
    self.shaders = VTKShaders()
  
  def executeProc(self, nodeId):
    p = mp.Process(target=self.ExtractVisiblePoints,
          args=(search(self.handleVTKCloud.root, int(nodeId[0])),)
        )
    p.start()
  
  def execute(self, obj, event):
    if self.handleVTKCloud.frustum:# and self.handleVTKCloud.priorityQueue:
      res = list(map(lambda i: self.ExtractVisiblePoints(
        search(self.handleVTKCloud.root, int(i[0]))), 
        np.asarray(self.handleVTKCloud.priorityQueue[0:150])))
  
  def isVisible(self, rootNode, camera_position, point):
    p2 = point
    
    Ids = vtk.vtkIdList()
    rootNode.data.locator.FindCellsAlongLine(camera_position, p2, 0.0, Ids)
    
    numIds = Ids.GetNumberOfIds()
    if numIds > 1:
      print(numIds)
      rootNode.data.visiblePointIds.append(Ids.GetId(0))
  
  def ExtractVisiblePoints(self, rootNode):
    delta = [0.0001] * 3
    if vtk.vtkMath.BoundsIsWithinOtherBounds(rootNode.bounds,
      self.handleVTKCloud.frustum.GetBounds(), delta):
      return
    
    svp = vtk.vtkSelectVisiblePoints()
    svp.SetInputData(rootNode.data.bstPolyData)
    svp.SetRenderer(self.handleVTKCloud.renderer)
    svp.Update()
    
    rootNode.data.bstActor.SetMapper(self.setMapper(svp.GetOutput(),
        self.shaders.Shaders[CURRENT_VTK_SHADER]))
  
  def setMapper(self, polyData, vtkShaderPreset):
    pg_mapper = vtk.vtkPointGaussianMapper()
    pg_mapper.SetInputData(polyData)
    pg_mapper.SetScaleArray('radius')
    pg_mapper.SetScalarVisibility(True)
    pg_mapper.SetScaleFactor(vtkShaderPreset[1])
    pg_mapper.SetEmissive(False)
    pg_mapper.SetSplatShaderCode(vtkShaderPreset[0])
    
    return pg_mapper
  
  def TraverseVisibleNodes(self, rootNode):
    self.ExtractVisiblePoints(rootNode)
