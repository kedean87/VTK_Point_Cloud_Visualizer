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
CURRENT_VTK_SHADER = 0

global POINTS
POINTS = None

global COLORS
COLORS = None

import heapq
import numpy as np

class BoundingBoxManager:
    def __init__(self, renderer):
        self.renderer = renderer
        self.actors = {}
        self.visible = False

    def updateBoxes(self, visible_nodes):
        """Show boxes for visible_nodes, hide others, without recreating."""
        current_ids = set(id(n[1]) for n in visible_nodes)

        # Hide actors no longer visible
        for nid, actor in list(self.actors.items()):
            if nid not in current_ids:
                actor.SetVisibility(False)
                del self.actors[nid]

        # Ensure actors for visible nodes exist and are visible
        for node_entry in visible_nodes:
            node = node_entry[1]
            nid = id(node)
            if nid not in self.actors:
                actor = self.createBoundingBoxActor(node.data.bstPolyData.GetBounds(),
                    color=(node.level*7, node.level*15, node.level*30))
                self.renderer.AddActor(actor)
                self.actors[nid] = actor
            self.actors[nid].SetVisibility(self.visible)

    def createBoundingBoxActor(self, bounds, color=(1, 0, 0)):
        cube = vtk.vtkCubeSource()
        cube.SetBounds(bounds)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetRepresentationToWireframe()
        actor.GetProperty().SetLineWidth(2)
        actor.SetVisibility(False)
        return actor

    def toggleVisibility(self, visible_flag: bool):
        """Externally toggle bounding box visibility."""
        self.visible = visible_flag
        for actor in self.actors.values():
            actor.SetVisibility(visible_flag)



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
    # self.priority_queue = LODPriorityQueue()
    self.lod_sigma = 50.0
    self.lod_depth_scale = 0.5
    self.max_visible_nodes = 50  # adaptive limit; can adjust by distance or FPS
    self.last_cam_pos = None

    # assign the filename so we can track what's read in, along with
    # the directory structure it came from
    self.filename = filename

    # set up an OpenGLRenderer
    self.renderer = renderer
    
    self.bounding_boxes = False
    self.actors = {}

    self.boundingBoxManager = BoundingBoxManager(self.renderer)

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
    self.priorityQueue = []
    self.subVolumes = None
    self.frustum = None
    self.planes = None
    self.iter = 0

    self.current_vtk_shader = CURRENT_VTK_SHADER
    
    # --- during initialization ---
    self.points_count_text = vtk.vtkTextActor()
    self.points_count_text.SetPosition(0, 60)  # pixels from bottom-left (adjust for top-left)
    self.points_count_text.GetTextProperty().SetFontSize(50)
    self.points_count_text.GetTextProperty().SetColor(1.0, 1.0, 1.0)  # white text
    self.renderer.AddActor2D(self.points_count_text)
    
    self.node_count_text = vtk.vtkTextActor()
    self.node_count_text.SetPosition(0, 120)  # pixels from bottom-left (adjust for top-left)
    self.node_count_text.GetTextProperty().SetFontSize(50)
    self.node_count_text.GetTextProperty().SetColor(1.0, 1.0, 1.0)  # white text
    self.renderer.AddActor2D(self.node_count_text)

    # create Binary Search Tree
    self.root = self.InitializeTree()

#----------------------------------------------------------------------#
  
  def get_current_visible_points_count(self):
    """
    Returns the total number of points currently visible in the renderer.
    Uses vtkPolyData from each visible actor.
    """
    actors = self.renderer.GetActors()
    actors.InitTraversal()
    total_points = 0

    for _ in range(actors.GetNumberOfItems()):
        actor = actors.GetNextActor()
        if actor.GetVisibility():
            mapper = actor.GetMapper()
            if mapper:
                input_data = mapper.GetInput()
                if input_data and hasattr(input_data, 'GetNumberOfPoints'):
                    total_points += input_data.GetNumberOfPoints()
    return total_points

#----------------------------------------------------------------------#
  def execute(self, obj, event):
    # print(f"Actors in renderer: {self.renderer.GetActors().GetNumberOfItems()}")

    if not self.frustum or not self.priorityQueue:
        return

    self.obj = obj
    self.sortPriorityQueue()

    current_ids = set(id(n[1]) for n in self.priorityQueue)

    # Hide actors no longer visible
    for nid, actor in list(self.actors.items()):
        if nid not in current_ids:
            actor.SetVisibility(False)
            del self.actors[nid]
    
    # Ensure actors for visible nodes exist and are visible
    for node_entry in self.priorityQueue:
        node = node_entry[1]
        nid = id(node)
        if nid not in self.actors:
            self.actors[nid] = node.data.bstActor
            self.actors[nid].SetVisibility(True)
    
    if self.bounding_boxes:
        self.boundingBoxManager.toggleVisibility(True)
        self.boundingBoxManager.updateBoxes(self.priorityQueue)
    else:
        self.boundingBoxManager.toggleVisibility(False)

    self.obj.GetRenderWindow().Render()
    
    # Update true current visible points count
    self.pointCount = self.get_current_visible_points_count()
    print(f"Current visible point count: {self.pointCount}")
    
    # update the text actor
    self.points_count_text.SetInput(f"Points: {self.pointCount}")
    self.node_count_text.SetInput(f"Nodes: {len(self.priorityQueue)}")
    
    self.priorityQueue.clear()

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
  def sortPriorityQueue(self):
    if len(self.priorityQueue) == 0:
      return

    q = np.array(self.priorityQueue)

    # this sorts the list based upon the distance from the node to the
    # center of the frustum
    self.priorityQueue = list(q[np.argsort(q[:, 0])])

#----------------------------------------------------------------------#
  def InitializeTree(self):
    """
    Build a spatial LOD binary tree:
     - recursively median-split along the longest axis
     - assign sequential integer Ids to nodes so existing search()/insert() usage still works
     - create BSTPointCloud at each node containing either a downsampled set (for higher-level nodes)
       or the full (or near-full) point set for leaf nodes
     - add actors to renderer for nodes that have data
    """

    # --- prepare numpy points and colors ---
    points = ns.vtk_to_numpy(self.PolyData.GetPoints().GetData())            # Nx3
    Ids = np.arange(len(points)).reshape(len(points), 1)                     # Nx1
    colors = ns.vtk_to_numpy(self.PolyData.GetPointData().GetScalars())     # Nx3 or Nx4

    # keep global access
    self.modify_points_global(np.concatenate([points, Ids], axis=1))  # Nx4
    self.modify_colors_global(colors)

    P = POINTS
    C = COLORS

    # compute overall bounds
    bounds = [
        float(P[:,0].min()), float(P[:,0].max()),
        float(P[:,1].min()), float(P[:,1].max()),
        float(P[:,2].min()), float(P[:,2].max())
    ]

    MAX_DEPTH = 6
    MIN_POINTS_LEAF = 15000
    BASE_PARENT_SAMPLE = 0.5
    MIN_SAMPLE_POINTS = 8000

    next_id = [0]
    def gen_id():
        i = next_id[0]
        next_id[0] += 1
        return i

    # helper: mask points in bounds
    def mask_points_in_bounds(ptarr, b):
        xmin, xmax, ymin, ymax, zmin, zmax = b
        mask = (
            (ptarr[:,0] >= xmin) & (ptarr[:,0] <= xmax) &
            (ptarr[:,1] >= ymin) & (ptarr[:,1] <= ymax) &
            (ptarr[:,2] >= zmin) & (ptarr[:,2] <= zmax)
        )
        return mask

    # helper: build BSTPointCloud
    def build_bst_pointcloud(indices, depth):
        if indices.size == 0:
            return None

        coords = P[indices, 0:3]
        orig_indices = P[indices, 3].astype(np.int32)

        frac = BASE_PARENT_SAMPLE * (0.5 ** depth)
        frac = min(1.0, max(frac, MIN_SAMPLE_POINTS / max(len(indices), 1)))
        sample_n = int(round(frac * len(indices)))
        sample_n = max(1, min(sample_n, len(indices)))

        if len(indices) <= MIN_POINTS_LEAF:
            sample_idx = np.arange(len(indices))
        else:
            rng = np.random.default_rng(seed=42 + int(depth))
            sample_idx = rng.choice(len(indices), size=sample_n, replace=False)

        sampled_coords = coords[sample_idx]
        sampled_orig = orig_indices[sample_idx]

        try:
            sampled_colors = C[sampled_orig]
        except Exception:
            sampled_colors = np.zeros((sampled_coords.shape[0], 3), dtype=np.uint8)

        return BSTPointCloud(sampled_coords, sampled_colors, self.current_vtk_shader)

    # recursive subdivision: **points assigned to only one child**
    def subdivide(bounds_in, indices, depth=0):
        node_id = gen_id()
        if indices.size == 0:
            node = TreeNode(node_id, bounds_in, None, level=depth)
            node.empty = True
            return node

        bstpc = build_bst_pointcloud(indices, depth)
        node = TreeNode(node_id, bounds_in, bstpc, level=depth)
        node.empty = (bstpc is None)

        if depth >= MAX_DEPTH or len(indices) <= MIN_POINTS_LEAF:
            if node.data:
                try:
                    self.renderer.AddActor(node.data.bstActor)
                    self.renderer.AddActor(node.data.bounding_box_actor)
                except Exception:
                    pass
            return node

        xmin, xmax, ymin, ymax, zmin, zmax = bounds_in
        dx, dy, dz = xmax - xmin, ymax - ymin, zmax - zmin

        if dx >= dy and dx >= dz:
            axis = 0
            median = (xmin + xmax) / 2.0
            left_bounds = [xmin, median, ymin, ymax, zmin, zmax]
            right_bounds = [median, xmax, ymin, ymax, zmin, zmax]
        elif dy >= dx and dy >= dz:
            axis = 1
            median = (ymin + ymax) / 2.0
            left_bounds = [xmin, xmax, ymin, median, zmin, zmax]
            right_bounds = [xmin, xmax, median, ymax, zmin, zmax]
        else:
            axis = 2
            median = (zmin + zmax) / 2.0
            left_bounds = [xmin, xmax, ymin, ymax, zmin, median]
            right_bounds = [xmin, xmax, ymin, ymax, median, zmax]

        # mask points for children: **each point goes to only one child**
        mask_left = mask_points_in_bounds(P[indices,0:3], left_bounds)
        mask_right = mask_points_in_bounds(P[indices,0:3], right_bounds)
        left_indices = indices[mask_left]
        right_indices = indices[mask_right]

        node.left = subdivide(left_bounds, left_indices, depth + 1)
        node.right = subdivide(right_bounds, right_indices, depth + 1)

        if node.data:
            try:
                self.renderer.AddActor(node.data.bstActor)
                self.renderer.AddActor(node.data.bounding_box_actor)
            except Exception:
                pass

        return node

    # root: pass all indices
    all_indices = np.arange(P.shape[0])
    self.root = subdivide(bounds, all_indices, depth=0)
    return self.root


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
    self.parallelProjection = False
    self.shaders = VTKShaders()

    self.selected_mapper = vtk.vtkDataSetMapper()
    self.selected_actor = vtk.vtkActor()

#----------------------------------------------------------------------#
  def modify_shader_global(self):
    global CURRENT_VTK_SHADER
    CURRENT_VTK_SHADER += 1
    self.cloudInfo.current_vtk_shader = CURRENT_VTK_SHADER

#----------------------------------------------------------------------#
  def reset_shader_global(self):
    global CURRENT_VTK_SHADER
    CURRENT_VTK_SHADER = 0
    self.cloudInfo.current_vtk_shader = CURRENT_VTK_SHADER

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
    pg_mapper = vtk.vtkOpenGLPointGaussianMapper()
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
  def getMapper(self, rootNode, vtkShaderPreset):
    pg_mapper = vtk.vtkOpenGLPointGaussianMapper()
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

    if obj.GetKeySym() == 'b': 
      if self.cloudInfo.bounding_boxes:
        self.cloudInfo.bounding_boxes = False
      else:
        self.cloudInfo.bounding_boxes = True

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

      self.cloudInfo.root.data.bstActor.SetMapper(self.setMapperEmissive(self.cloudInfo.root, CURRENT_VTK_SHADER, self.emissive))
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
      self.cloudInfo.root.data.bstActor.SetMapper(self.getMapper(self.cloudInfo.root, CURRENT_VTK_SHADER))
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
  renderer.SetUseDepthPeeling(0)
  renderer.SetMaximumNumberOfPeels(3)
  renderer.SetNearClippingPlaneTolerance(0.001)
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
  screenWidth = 2000
  screenHeight = int(screenWidth * 0.515151)

  # create a render window and add the renderer
  renderWindow = vtk.vtkRenderWindow()
  renderWindow.AddRenderer(renderer)
  renderWindow.SetSize(screenWidth, screenHeight)
  renderWindow.SetWindowName("VTK Point Cloud Visualizer")

  # create the interactor
  areaPicker = vtk.vtkAreaPicker()
  renderWindowInteractor = vtk.vtkRenderWindowInteractor()
  renderWindowInteractor.SetRenderWindow(renderWindow)
  renderWindowInteractor.SetPicker(areaPicker)
  renderWindowInteractor.SetTimerDuration(1)
  renderWindowInteractor.Initialize()

  FpsDisplay = FpsObserver(renderer)

  # read in the data and create the BST to handle the point data and
  # set callbacks to the classes to handle the point cloud manipulation
  handleCloud = HandleVTKCloud(renderer=renderer,
    filename="data/output_pointcloud.vtp",)
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
  renderWindowInteractor.AddObserver('TimerEvent', distApproxVoxelGrid.execute)
  renderWindowInteractor.AddObserver('KeyPressEvent', style.execute)
  # renderWindowInteractor.AddObserver('TimerEvent', occlFilter.execute)
  timerId = renderWindowInteractor.CreateRepeatingTimer(1)

  # render and start the program
  renderWindow.Render()
  renderWindowInteractor.Start()

if __name__ == "__main__":
    main()
