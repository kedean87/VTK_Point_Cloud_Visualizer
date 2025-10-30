'''
  Illuscio VTK Visualizer
  
  needed:
    pip3 install vtk
    (everything else should be accounted for)
  
  how to run:
    * all 3 files: visualizer, utils, and BinarySearchTree need to be
      in the same directory in order to successfully run (obviously the
      structure will be changed; it's a minimal implementation)
      
    python (or .exe) visualizer
  
'''

from BinarySearchTree import *
from VTKShaderPresets import *
from BSTPointCloud import *
# from OctreePointCloud import *
# from Octree import *
from FilterOccludedPoints import *
from Frustum import *
from NodeVisbilitiy import *
from FramesPerSecond import *
from DistanceApproximatedShaderShapes import *
from DistanceApproximatedVoxelGrid import *

import vtk
import h5py
import numpy as np
import asyncio
import binascii
import random
import zlib
import os, sys
import struct
import socket

import multiprocessing as mp
#from joblib import Parallel, delayed
#from numba import jit
#from numba.typed import List
from vtk.util import numpy_support as ns
from utils import *
from scipy.spatial import distance

global CURRENT_VTK_SHADER
CURRENT_VTK_SHADER = 2

global POINTS
POINTS = None

global COLORS
COLORS = None


########################################################################
class HandleVTKCloud():
########################################################################
# FUNCTIONS
#----------------------------------------------------------------------#
  def __init__(self,  
               renderer,
               debug=False,
               client_server=False,
               filename="",
               server='localhost', 
               address=10000, 
               maxNumPoints=1.5e6, 
               timeout=30):
    # assign the filename so we can track what's read in, along with
    # the directory structure it came from
    self.filename = filename
    
    # set up an OpenGLRenderer
    self.renderer = renderer
    
    # some current, irrelevant arguments... will be useful for client/
    # server operations
    self.debug = debug
    self.client_server = client_server
    self.timeout = timeout
    
    # create tcp/ip socket
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # bind the socket to the port
    # self.sock.bind((server, address))
    print(sys.stderr, "Staring On {0}, port {1}".format(server, address))
    
    # open the socket to listen, and wait for a connection
    # self.sock.listen(1)
    
    # tracker(s)
    self.pointCount = 0
    self.maxNumPoints = maxNumPoints
    
    # read in the point cloud data file
    print("Reading File")
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(filename)
    reader.Update()
    
    # save the reader, and polygonal input data
    self.PolyDataReader = reader
    self.PolyData = self.PolyDataReader.GetOutput()
    
    # instantiate variables for priority list, and SubVolumes (or node
    # regions)
    self.priorityQueue = None
    self.subVolumes = None
    self.frustum = None
    self.planes = None
    self.iter = 0
    
    # create Binary Search Tree
    self.root = self.InitializeTree()

#----------------------------------------------------------------------#
  def isVisible(self, treeNode, fsBounds, vtkPlanes):
    # check if the current tree node is within the bounds of the frustum
    delta = [0.0000] * 3
    if vtk.vtkMath.BoundsIsWithinOtherBounds(treeNode.bounds, fsBounds, delta):
      treeNode.data.bstActor.SetVisibility(True)
      return
      
    if self.checkIntersection(treeNode.bounds, fsBounds):
      # if yes, set the visibility to True
      # self.clipFunction(treeNode, vtkPlanes)
      treeNode.data.bstActor.SetVisibility(True)
      return
    else:
      # if not, set the visbility to False
      treeNode.data.bstActor.SetVisibility(False)
      return

#----------------------------------------------------------------------#
  def checkIntersection(self, bounds1, bounds2):
    # check whether or not the bounds are overlapping one anotherr
    isOverlapping = (bounds1[0] <= bounds2[1] or bounds2[0] <= bounds2[1] \
                 and bounds1[2] <= bounds2[3] or bounds2[2] <= bounds2[3] \
                 and bounds1[4] <= bounds2[5] or bounds2[5] <= bounds2[4])
    return isOverlapping
  
  def subExecute(self, nodeIndex):
    # select the current node and check if it exists and contains data
    cN = search(self.root, nodeIndex)
    if not cN or cN.empty:
      return
    
    # when the points get to a certain increment, render the polydatas
    # renders a little faster
    if self.pointCount % 1.5e5 == 0:
      self.obj.GetRenderWindow().Render()
    
    # this represents a points budget that won't allow the visualizer
    # to render more than them.
    if self.pointCount >= 1e8:
      self.pointCount = 0
      return
    
    # ~ # check if the current node is visible and assign visibilit (or
    # ~ # not) to the polydata actor
    self.isVisible(cN, self.frustum.GetBounds(), self.planes)
    self.pointCount += cN.data.bstPolyData.GetNumberOfPoints()

#----------------------------------------------------------------------#
  def execute(self, obj, event):
    if self.frustum:
      self.obj = obj
      Ids = np.array(self.priorityQueue).astype(np.int32)
      res = list(map(lambda i: self.subExecute(int(i[0])), Ids[0:50000]))
      
      obj.GetRenderWindow().Render()

#----------------------------------------------------------------------#
  def modify_points_global(self, points):
    global POINTS
    POINTS = points

#----------------------------------------------------------------------#
  def reset_points_global(self):
    global POINTS
    POINTS = None

#----------------------------------------------------------------------#
  def modify_colors_global(self, colors):
    global COLORS
    COLORS = colors
    
#----------------------------------------------------------------------#
  def reset_colors_global(self):
    global POINTS
    POINTS = None

#----------------------------------------------------------------------#
  def getDistanceToCamera(self, rootNode, fsCenter):
    center = [float()] * 3
    rootNode.data.bstPolyData.GetCenter(center)
    
    return distance.euclidean(fsCenter, center)

#----------------------------------------------------------------------#
  def extractSelectedFrustum(self, rootNode, vtkPlanes):
    esf = vtk.vtkExtractSelectedFrustum()
    esf.SetInputData(rootNode.data.bstPolyData)
    esf.SetFrustum(vtkPlanes)
    esf.Update()
    
    if esf.GetOutput().GetNumberOfPoints() > 0:
      rootNode.data.pg_mapper.SetInputData(esf.GetOutput())
    else:
      rootNode.data.pg_mapper.SetInputData(rootNode.data.bstPolyData)

#----------------------------------------------------------------------#
  def extractVisiblePoints(self, rootNode):
    svf = vtk.vtkSelectVisiblePoints()
    svf.SetInputData(rootNode.data.bstPolyData)
    svf.SetRenderer(self.renderer)
    svf.Update()
    
    if svf.GetOutput().GetNumberOfPoints() > 0:
      rootNode.data.pg_mapper.SetInputData(svf.GetOutput())
    else:
      rootNode.data.pg_mapper.SetInputData(rootNode.data.bstPolyData)
  
  def clipFunction(self, rootNode, vtkPlanes):
    clip = vtk.vtkClipPolyData()
    clip.SetInputData(rootNode.data.bstPolyData)
    clip.SetClipFunction(vtkPlanes)
    clip.InsideOutOn()
    clip.Update()
    
    if clip.GetOutput().GetNumberOfPoints() > 0:
      rootNode.data.pg_mapper.SetInputData(esclipf.GetOutput())
    else:
      rootNode.data.pg_mapper.SetInputData(rootNode.data.bstPolyData)

#----------------------------------------------------------------------#
  def sortPriorityQueue(self):
    if len(self.priorityQueue) == 0:
      return
    
    q = np.array(self.priorityQueue)
    
    # this sorts the list based upon the distance from the node to the
    # center of the frustum
    self.priorityQueue = q[np.argsort(q[:, 1])]

#----------------------------------------------------------------------#
  def updatePriorityQueue(self, rootNode, fsCenter, vtkPlanes):
    # create a list to iterate through (binary node)
    children = [ rootNode.left, rootNode.right ]
    
    # iterate through the node's children
    for child in children:
      if child:
        # if there are still children to iterate through, do so
        self.updatePriorityQueue(child, fsCenter, vtkPlanes)
        
        # if the node has data, calculate the distance and append it to
        # the priority list
        if child.data:
          #inScene = self.is_point_in_frustum(child.data.center, vtkPlanes)
          
          #if inScene:
          dist2 = self.getDistanceToCamera(child, fsCenter)
          self.priorityQueue.append([child.Id,
                dist2])
  
  def is_point_in_frustum(self, point, vtkPlanes):
    # planes is a list of vtkPlane objects or the frustum planes from vtkFrustumSource
    for i in range(vtkPlanes.GetNumberOfPlanes()):
        plane = vtkPlanes.GetPlane(i)
        if plane.EvaluateFunction(point) < 0:
            return False
    return True
  
  def updateVisibleNodes(self, obj, rootNode, fsBounds):
    # create a list to iterate through (binary node)
    children = [ rootNode.left, rootNode.right ]
    
    # iterate through the node's children
    for child in children:
      if child:
        # if there are still children to iterate through, do so
        self.updateVisibleNodes(obj, child, fsBounds)
        
        if child.data:
          # when the points get to a certain increment, render the polydatas
          # renders a little faster
          if self.pointCount % 1e5 == 0:
            obj.GetRenderWindow().Render()
          
          # this represents a points budget that won't allow the visualizer
          # to render more than them.
          if self.pointCount >= 3e6:
            self.pointCount = 0
            return
          
          # check if the current node is visible and assign visibilit (or
          # not) to the polydata actor
          self.isVisible(child, fsBounds)
          self.pointCount += child.data.bstPolyData.GetNumberOfPoints()

#----------------------------------------------------------------------#
  def InitializeTree(self):
    # convert the points from vtk to numpy, and create an indices to
    # concatenate to the points array
    points = ns.vtk_to_numpy(self.PolyData.GetPoints().GetData())
    Ids = np.arange(start=0, stop=len(points), step=1).reshape(len(points), 1)
    
    # convert the colors from vtk to numpy
    colors = ns.vtk_to_numpy(self.PolyData.GetPointData().GetScalars())
    
    # update the global access variables
    self.modify_points_global(np.concatenate([points, Ids], axis=1))
    self.modify_colors_global(colors)
    
    # delete unused variables
    del points
    del Ids
    del colors
    
    pl = vtk.vtkOctreePointLocator()
    pl.SetDataSet(self.PolyData)
    pl.BuildLocator()
    
    # reorder the bounds so they match necessary function requirements
    bounds = [float()]*6
    bounds[0] = POINTS.T[0].min()
    bounds[1] = POINTS.T[0].max()
    bounds[2] = POINTS.T[1].min()
    bounds[3] = POINTS.T[1].max()
    bounds[4] = POINTS.T[2].min()
    bounds[5] = POINTS.T[2].max()
    
    # subVolume the point cloud in order to create subVolume (node)
    # regions
    print("Creating SubVolumes for Binary Search Tree nodes")
    self.subVolumes = CreateSubVolumes(volumeBounds=bounds, numSubV=1000)
    
    def getVolumeCenter(bounds):
      xmin, ymin, zmin, xmax, ymax, zmax = bounds
      x = (xmin + xmax) / 2.0
      y = (ymin + ymax) / 2.0
      z = (zmin + zmax) / 2.0
      return (x, y, z)
    
    distances = []
    for v in self.subVolumes:
      vCenter = getVolumeCenter(v)
      distances.append(
        vtk.vtkMath.Distance2BetweenPoints(vCenter, self.PolyData.GetCenter())
      )
    
    Ids = np.arange(start=0, stop=len(self.subVolumes), step=1)
    array = np.array(list(zip(distances, Ids)))
    subVIds = array[np.argsort(array[:, 0])].T[1].astype(np.int32)
    
    # set flag saying root doesn't exist
    isRoot = False
    
    print("Creating/Updating Divisible Search Tree")
    # iterate through the subvolumes and update BST
    for subVId in subVIds:
      # get current subvolume
      print("SubVolume - {0}".format(subVId))
      subVol = self.subVolumes[subVId]
      
      bounds = [
        subVol[0], subVol[3],
        subVol[1], subVol[4],
        subVol[2], subVol[5]
      ]
      
      # extract the data, if it exists
      print("\tExtract Point Data within subVolume")
      data = getPointData(pl, bounds, POINTS, COLORS)
      
      # insert region point cloud data in the BST
      if isRoot:
        if data == None:
          root = insert(root, subVId, subVol, data)
          print('\t  num points = {0}'.format(data))
        else:
          bstPC = BSTPointCloud(data[0][:, 0:3], data[1], CURRENT_VTK_SHADER)
          root = insert(root, subVId, subVol, bstPC)
          print('\t  num points = {0}'.format(len(data[0])))
        continue
      
      if data == None:
        root = TreeNode(subVId, subVol, data)
        print('\t  num points = {0}'.format(data))
      else:
        bstPC = BSTPointCloud(data[0][:, 0:3], data[1], CURRENT_VTK_SHADER)
        root = TreeNode(subVId, subVol, bstPC)
        print('\t  num points = {0}'.format(len(data[0])))
      
      isRoot = True
    
    # iterate through the tree and make sure to add every actor that
    # contains data to the renderer
    for nodeId in range(len(self.subVolumes)):
      cN = search(root, nodeId)
      if not cN or cN.empty:
        continue
      
      self.renderer.AddActor(cN.data.bstActor)
      
    return root
  
  def viz_generate_octree(self):
    print("Reading from H5")
    self.hf = h5py.File("C:/Users/Kevin/Illuscio/data/Engine_Room/data.h5","r")
    
    unique_colors = self.hf['PointData/Colors'][:]
    self.unique_colors = unique_colors.reshape(int(unique_colors.size/3), 3)
    self.points_and_color_indices = self.hf['PointData/ushort'][:]
    self.node_info = self.hf['FieldData/Nodes/Info'][:]
    self.node_names = self.hf['FieldData/Nodes/Names'][:]
    
    tree = Octree(0)
    root = Node('r', 0, None)
    
    octree_status = tree.generateOctree(
      p_root=root, 
      level=0,
      DEBUG=1,
      node_names=self.node_names,
      node_info=self.node_info
    )
    self.root = root
    
    print(self.root.children)
    

########################################################################
class KeyBoardInterrupt(vtk.vtkInteractorStyleTrackballCamera, VTKShaders):
########################################################################
# FUNCTIONS
#----------------------------------------------------------------------#
  def __init__(self, HandleCloud):
    self.cloudInfo = HandleCloud
    self.setW = False
    self.setP = False
    self.emissive = False
    self.metallic = False
    self.rayCasting = False
    self.parallelProjection = False
    self.shaders = VTKShaders()
    
    # self.AddObserver('MiddleButtonPressEvent', self.middle_button_press_event)
    self.selected_mapper = vtk.vtkDataSetMapper()
    self.selected_actor = vtk.vtkActor()
  
  def middle_button_press_event(self, obj, event):
    colors = vtk.vtkNamedColors()

    # Get the location of the click (in window coordinates)
    pos = self.GetInteractor().GetEventPosition()

    picker = vtk.vtkPointPicker()
    picker.SetTolerance(0.0005)

    # Pick from this location.
    picker.Pick(pos[0], pos[1], 0, self.cloudInfo.renderer)

    world_position = picker.GetPickPosition()
    print(f'Point id is: {picker.GetPointId()}')
    
    props = self.GetInteractor().GetPicker().GetProp3Ds()
    props.InitTraversal()
    
    for i in range(props.GetNumberOfItems()):
      prop = props.GetNextProp3D()
      print("Prop: " + str(prop))

#----------------------------------------------------------------------#
  def modify_shader_global(self):
    global CURRENT_VTK_SHADER
    CURRENT_VTK_SHADER += 1

#----------------------------------------------------------------------#
  def reset_shader_global(self):
    global CURRENT_VTK_SHADER
    CURRENT_VTK_SHADER = 0

#----------------------------------------------------------------------#
  def modify_execute_counter(self):
    global EXECUTE_COUNTER
    EXECUTE_COUNTER += 1

#----------------------------------------------------------------------#
  def reset_execute_counter(self):
    global EXECUTE_COUNTER
    EXECUTE_COUNTER = 0

#----------------------------------------------------------------------#
  def modifyBSTNodeActor(self, rootNode, vtkShaderPreset):
    children = [rootNode.left, rootNode.right]
    for child in children:
      if child:
        self.modifyBSTNodeActor(child, vtkShaderPreset)
        
        if child.data:
          child.data.bstActor.SetMapper(self.getMapper(child, vtkShaderPreset))
  
  def modifyBSTNodeActorEdges(self, rootNode):
    children = [rootNode.left, rootNode.right]
    for child in children:
      if child:
        self.modifyBSTNodeActorEdges(child)
        
        if child.data:
          if child.data.bstEdgeVisibility == True:
            child.data.bstActor.GetProperty().SetEdgeVisibility(False)
            child.data.bstEdgeVisibility = False
          else:
            child.data.bstActor.GetProperty().SetEdgeVisibility(True)
            child.data.bstEdgeVisibility = True

#----------------------------------------------------------------------#
  def modifyRayCasting(self, rootNode):
    children = [rootNode.left, rootNode.right]
    for child in children:
      if child:
        self.modifyRayCasting(child)
        
        if child.data:
          self.checkRayCasting(child)
  
  def checkRayCasting(self, rootNode):
    if not self.rayCasting:
      self.rayCasting = True
      
      rootNode.data.bstActor.SetVisibility(False)
      self.cloudInfo.renderer.AddVolume(rootNode.data.volume)
      
    else:
      self.rayCasting = False
      
      rootNode.data.bstActor.SetVisibility(True)
      self.cloudInfo.renderer.RemoveVolume(rootNode.data.volume.GetProperty())
      
#----------------------------------------------------------------------#
  def modifyMetallic(self, rootNode):
    children = [rootNode.left, rootNode.right]
    for child in children:
      if child:
        self.modifyMetallic(child)
        
        if child.data:
          self.checkMetallicStatus(child)
          
#----------------------------------------------------------------------#
  def checkMetallicStatus(self, rootNode):
    if not self.metallic:
      self.metallic = True
      rootNode.data.bstActor.SetMapper(self.setMapperMetallic(rootNode))
      rootNode.data.bstActor.GetProperty().SetInterpolationToPBR()
      rootNode.data.bstActor.GetProperty().SetColor(2.,4.,4.)
      rootNode.data.bstActor.GetProperty().RenderPointsAsSpheresOn()
      rootNode.data.bstActor.GetProperty().SetPointSize(0.165)
      rootNode.data.bstActor.GetProperty().SetMetallic(1)
      rootNode.data.bstActor.GetProperty().SetRoughness(0.95)
    else:
      self.metallic = False
      rootNode.data.bstActor.GetProperty().SetInterpolationToFlat()
      rootNode.data.bstActor.GetProperty().RenderPointsAsSpheresOff()
      rootNode.data.bstActor.GetProperty().SetMetallic(0)
  
#----------------------------------------------------------------------#
  def modifyEmissivity(self, rootNode, vtkShaderPreset, emissive):
    children = [rootNode.left, rootNode.right]
    for child in children:
      if child:
        self.modifyEmissivity(child, vtkShaderPreset, emissive)
        
        if child.data:
          child.data.bstActor.SetMapper(self.setMapperEmissive(child, vtkShaderPreset, emissive))
            

#----------------------------------------------------------------------#
  def setMapperEmissive(self, rootNode, vtkShaderPreset, emissive):
    pg_mapper = vtk.vtkPointGaussianMapper()
    pg_mapper.SetInputData(rootNode.data.bstPolyData)
    pg_mapper.SetScaleArray('radius')
    pg_mapper.SetScaleFactor(self.shaders.Shaders[vtkShaderPreset][1])
    
    if emissive:
      pg_mapper.EmissiveOn()
    else:
      pg_mapper.EmissiveOff()
      
    pg_mapper.SetSplatShaderCode(self.shaders.Shaders[vtkShaderPreset][0])
    
    return pg_mapper

#----------------------------------------------------------------------#
  def setMapperMetallic(self, rootNode):
    if self.metallic:
      points = ns.vtk_to_numpy(rootNode.data.bstPolyData.GetPoints().GetData())
      x, y, z = points.T[0], points.T[1], points.T[2]
      
      rgrid = vtk.vtkRectilinearGrid()
      rgrid.SetDimensions(len(x), len(y), len(z))
      rgrid.SetXCoordinates(ns.numpy_to_vtk(x, array_type=vtk.VTK_DOUBLE))
      rgrid.SetYCoordinates(ns.numpy_to_vtk(y, array_type=vtk.VTK_DOUBLE))
      rgrid.SetZCoordinates(ns.numpy_to_vtk(z, array_type=vtk.VTK_DOUBLE))
      rgrid.GetPointData().SetScalars(
        rootNode.data.bstPolyData.GetPointData().GetScalars()
        )
      
      mapper = vtk.vtkFixedPointVolumeRayCastMapper()
      mapper.SetInputData(rgrid)
      mapper.SetScalarModeToUsePointData()
      
      return mapper
    else:
      pg_mapper = vtk.vtkPointGaussianMapper()
      pg_mapper.SetInputData(rootNode.data.bstPolyData)
      pg_mapper.SetScaleArray('radius')
      pg_mapper.SetScalarVisibility(True)
      pg_mapper.SetScaleFactor(self.shaders.Shaders[vtkShaderPreset][1])
      pg_mapper.SetSplatShaderCode(self.shaders.Shaders[vtkShaderPreset][0])
      
      return pg_mapper

  def getMapper(self, rootNode, vtkShaderPreset):
    pg_mapper = vtk.vtkPointGaussianMapper()
    pg_mapper.SetInputData(rootNode.data.bstPolyData)
    pg_mapper.SetScaleArray('radius')
    pg_mapper.SetScalarVisibility(True)
    pg_mapper.SetScaleFactor(self.shaders.Shaders[vtkShaderPreset][1])
    pg_mapper.EmissiveOff()
    pg_mapper.SetSplatShaderCode(self.shaders.Shaders[vtkShaderPreset][0])
    
    return pg_mapper

#----------------------------------------------------------------------#
  def moveForward(self):
    camera = self.cloudInfo.renderer.GetActiveCamera()
    
    cam = camera.GetPosition()
    focal = camera.GetFocalPoint()
    
    vec = [0, 0, 2]
    newCam = [0, 0, 0]
    newFocal = [0, 0, 0]
    
    vtk.vtkMath.Subtract(focal, cam, vec)
    vtk.vtkMath.Normalize(vec)
    vtk.vtkMath.Add(cam, vec, newCam)
    vtk.vtkMath.Add(focal, vec, newFocal)
    
    camera.SetPosition(newCam)
    camera.SetFocalPoint(newFocal)
    
    self.cloudInfo.renderer.ResetCameraClippingRange()

#----------------------------------------------------------------------#
  def moveBackward(self):
    camera = self.cloudInfo.renderer.GetActiveCamera()
    
    cam = camera.GetPosition()
    focal = camera.GetFocalPoint()
    
    vec = [0, 0, 2]
    newCam = [0, 0, 0]
    newFocal = [0, 0, 0]
    
    vtk.vtkMath.Subtract(focal, cam, vec)
    vtk.vtkMath.Normalize(vec)
    vtk.vtkMath.Subtract(cam, vec, newCam)
    vtk.vtkMath.Subtract(focal, vec, newFocal)
    
    camera.SetPosition(newCam)
    camera.SetFocalPoint(newFocal)
    
    self.cloudInfo.renderer.ResetCameraClippingRange()

#----------------------------------------------------------------------#
  def moveRight(self):
    camera = self.cloudInfo.renderer.GetActiveCamera()
    
    cam = camera.GetPosition()
    focal = camera.GetFocalPoint()
    
    up = [0, 2, 0]
    vec = [0, 0, 2]
    newCam = [0, 0, 0]
    newFocal = [0, 0, 0]
    
    vtk.vtkMath.Subtract(focal, cam, vec)
    vec[1] = 0
    
    vtk.vtkMath.Normalize(vec)
    vtk.vtkMath.Cross(vec, up, vec)
    vtk.vtkMath.Add(cam, vec, newCam)
    vtk.vtkMath.Add(focal, vec, newFocal)
    
    camera.SetPosition(newCam)
    camera.SetFocalPoint(newFocal)
    
    self.cloudInfo.renderer.ResetCameraClippingRange()

#----------------------------------------------------------------------#
  def moveLeft(self):
    camera = self.cloudInfo.renderer.GetActiveCamera()
    
    cam = camera.GetPosition()
    focal = camera.GetFocalPoint()
    
    up = [0, 2, 0]
    vec = [0, 0, 2]
    newCam = [0, 0, 0]
    newFocal = [0, 0, 0]
    
    vtk.vtkMath.Subtract(focal, cam, vec)
    vec[1] = 0
    
    vtk.vtkMath.Normalize(vec)
    vtk.vtkMath.Cross(vec, up, vec)
    vtk.vtkMath.Subtract(cam, vec, newCam)
    vtk.vtkMath.Subtract(focal, vec, newFocal)
    
    camera.SetPosition(newCam)
    camera.SetFocalPoint(newFocal)
    
    self.cloudInfo.renderer.ResetCameraClippingRange()

#----------------------------------------------------------------------#
  def execute(self, obj, event):
    self.setW = False
    self.setA = False
    self.setS = False
    self.setD = False
    self.setP = False
    self.setT = False
    self.setH = False
    self.setM = False
    
    if obj.GetKeySym() == 'Q':
      print(sys.stderr, "Pressed key: %s" % obj.GetKeySym())
      self.cloudInfo.sock.close()
      obj.GetRenderWindow().Finalize()
      obj.TerminateApp()
    
    if obj.GetKeySym() == 'h':
      # traverse the binary search tree and update the actors to utilize
      # a mapper with the current vtk shader preset code
      if self.emissive:
        self.emissive = False
      else:
        self.emissive = True
      
      self.modifyEmissivity(self.cloudInfo.root, CURRENT_VTK_SHADER, self.emissive)
      
      if not self.setH:
        obj.SetKeySym("")
        self.setH = True
    
    if obj.GetKeySym() == 'm':
      print('m')
      # traverse the binary search tree and update the actors to utilize
      # a mapper with the current vtk shader preset code
      self.modifyRayCasting(self.cloudInfo.root)
      
      if not self.setM:
        obj.SetKeySym("")
        self.setM = True
    
    if obj.GetKeySym() == 't':
      # this if / else is used to cycle through the shader presets and
      # globally modify the according variables
      if CURRENT_VTK_SHADER == len(self.shaders.Shaders) - 1:
        self.reset_shader_global()
      else:
        self.modify_shader_global()
      
      # traverse the binary search tree and update the actors to utilize
      # a mapper with the current vtk shader preset code
      self.modifyBSTNodeActor(self.cloudInfo.root, CURRENT_VTK_SHADER)
      
      if not self.setT:
        obj.SetKeySym("")
        self.setT = True
    
    if obj.GetKeySym() == 'w':
      # this if / else is used to cycle through the shader presets and
      # globally modify the according variables
      self.moveForward()
      
      if not self.setW:
        obj.SetKeySym("")
        self.setW = True
    
    if obj.GetKeySym() == 'a':
      # this if / else is used to cycle through the shader presets and
      # globally modify the according variables
      self.moveLeft()
      
      if not self.setA:
        obj.SetKeySym("")
        self.setA = True
    
    if obj.GetKeySym() == 's':
      # this if / else is used to cycle through the shader presets and
      # globally modify the according variables
      self.moveBackward()
      
      if not self.setS:
        obj.SetKeySym("")
        self.setS = True
    
    if obj.GetKeySym() == 'd':
      # this if / else is used to cycle through the shader presets and
      # globally modify the according variables
      self.moveRight()
      
      if not self.setD:
        obj.SetKeySym("")
        self.setD = True
      
    if obj.GetKeySym() == 'p':
      # this if / else is used to toggle through parallel / not parallel
      # projection visualization
      if not self.parallelProjection:
        self.parallelProjection = True
      else:
        self.parallelProjection = False
      
      self.cloudInfo.renderer.GetActiveCamera().SetParallelProjection(self.parallelProjection)
      
      if not self.setP:
        obj.SetKeySym("")
        self.setP = True

#----------------------------------------------------------------------#
# MAIN FUNCTION
#----------------------------------------------------------------------#
def main():
  # create the vtk Camera
  camera = vtk.vtkCamera()
  camera.SetPosition(3.7785712206194617, -4.559214892204117, 5.534146050157985)
  camera.SetFocalPoint(0.14291489124298096, -0.21075290441513062, 1.278286635875702)
  camera.SetViewUp(-0.16687639596331583, -0.757374824457846, -0.6312967952934194)
  camera.SetViewAngle(30.0)
  #camera.SetThickness(450.0)
  
  fxaaOptions = vtk.vtkFXAAOptions()
  fxaaOptions.SetSubpixelBlendLimit(0.55)
  fxaaOptions.SetUseHighQualityEndpoints(0)
  
  # create the vtk Rendererr
  renderer = vtk.vtkOpenGLRenderer()
  renderer.SetBackground(0.1, 0.4, 0.4)
  renderer.SetActiveCamera(camera)
  renderer.SetUseFXAA(1)
  renderer.SetFXAAOptions(fxaaOptions)
  renderer.SetUseDepthPeeling(1)
  renderer.SetUseHiddenLineRemoval(1)
  renderer.SetUseShadows(1)
  renderer.SetLightFollowCamera(1)
  renderer.SetUseDepthPeeling(1)
  renderer.SetMaximumNumberOfPeels(3)
  renderer.SetNearClippingPlaneTolerance(0.1)
  renderer.SetUseSphericalHarmonics(0)
  
  # set sorting style for the culler
  culler = vtk.vtkFrustumCoverageCuller()
  culler.SetSortingStyleToBackToFront()
  culler.SetMaximumCoverage(1000.0)
  culler.SetMinimumCoverage(0.0)
  renderer.AddCuller(culler)
  
  irradiance = renderer.GetEnvMapIrradiance()
  irradiance.SetIrradianceStep(0.3)
  
  
  # set up visualizer screen width and height
  screenWidth = 2400
  screenHeight = int(screenWidth * 0.515151)
  
  # create a render window and add the renderer
  renderWindow = vtk.vtkRenderWindow()
  renderWindow.AddRenderer(renderer)
  renderWindow.SetSize(screenWidth, screenHeight)
  renderWindow.SetWindowName("ILLUSCIO - (medium) Engine Room")
  
  # create the interactor
  areaPicker = vtk.vtkAreaPicker()
  renderWindowInteractor = vtk.vtkRenderWindowInteractor()
  renderWindowInteractor.SetRenderWindow(renderWindow)
  renderWindowInteractor.SetPicker(areaPicker)
  renderWindowInteractor.SetTimerDuration(50)
  renderWindowInteractor.Initialize()
  
  FpsDisplay = FpsObserver(renderer)
  
  # read in the data and create the BST to handle the point data and
  # set callbacks to the classes to handle the point cloud manipulation
  handleCloud = HandleVTKCloud(renderer=renderer,
    filename="data/example.vtp",)
  frustum = UpdateFrustum(handleCloud)
  queue = UpdateNodeVisibility(handleCloud)
  occlFilter = FilterOccludedPoints(handleCloud)
  distApproxShaderShapes = DistanceApproximatedShaderShapes(handleCloud)
  distApproxVoxelGrid = DistanceApproximatedVoxelGrid(handleCloud)
  style = KeyBoardInterrupt(handleCloud)
  
  renderWindowInteractor.SetInteractorStyle(style)
  renderWindowInteractor.AddObserver('TimerEvent', handleCloud.execute)
  renderWindowInteractor.AddObserver('TimerEvent', frustum.execute)
  renderWindowInteractor.AddObserver('TimerEvent', queue.execute)
  #renderWindowInteractor.AddObserver('TimerEvent', distApproxShaderShapes.execute)
  #renderWindowInteractor.AddObserver('TimerEvent', distApproxVoxelGrid.execute)
  renderWindowInteractor.AddObserver('KeyPressEvent', style.execute)
  # renderWindowInteractor.AddObserver('TimerEvent', occlFilter.execute)
  timerId = renderWindowInteractor.CreateRepeatingTimer(1)
  
  # render and start the program
  renderWindow.Render()
  renderWindowInteractor.Start()

if __name__ == "__main__":
    main()
