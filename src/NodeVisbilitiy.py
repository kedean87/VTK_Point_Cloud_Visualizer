from BinarySearchTree import *
from VTKShaderPresets import *
from BSTPointCloud import *
from FilterOccludedPoints import *
from Frustum import *

class UpdateNodeVisibility():
  def __init__(self, handleVTKCloud):
    self.handleVTKCloud = handleVTKCloud
  
  def execute(self, obj, event):
    if self.handleVTKCloud.frustum:
      self.handleVTKCloud.priorityQueue = []
      self.handleVTKCloud.updatePriorityQueue(self.handleVTKCloud.root, 
        self.handleVTKCloud.frustum.GetCenter(), 
        self.handleVTKCloud.planes)
      self.handleVTKCloud.sortPriorityQueue()
