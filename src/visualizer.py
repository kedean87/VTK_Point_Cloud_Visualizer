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
CURRENT_VTK_SHADER = 2

global POINTS
POINTS = None

global COLORS
COLORS = None

import heapq
import numpy as np

class LODPriorityQueue:
    def __init__(self):
        self.queue = []        # (-priority, node)
        self.node_map = {}     # node -> (priority, node)

    def update_node(self, node, cam_pos, sigma=50.0, depth_scale=0.5):
        # compute bounding box center
        xmin, xmax, ymin, ymax, zmin, zmax = node.bounds
        center = np.array([(xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2])
        d = np.linalg.norm(center - np.array(cam_pos))

        # gaussian weighting + asymptotic depth factor
        gaussian = np.exp(- (d**2) / (2 * sigma**2))
        depth_factor = 1.0 - np.exp(-depth_scale * getattr(node, 'depth', 0))
        priority = gaussian * depth_factor

        entry = (-priority, node)
        if node in self.node_map:
            self.queue.remove(self.node_map[node])
            heapq.heapify(self.queue)
        self.node_map[node] = entry
        heapq.heappush(self.queue, entry)

    def pop_highest(self):
        if not self.queue:
            return None, 0.0
        pr, node = heapq.heappop(self.queue)
        self.node_map.pop(node, None)
        return node, -pr
    
    def clear(self):
        self.queue = []

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

        # Ensure actors for visible nodes exist and are visible
        for node_entry in visible_nodes:
            node = node_entry[1]
            nid = id(node)
            if nid not in self.actors:
                actor = self.createBoundingBoxActor(node.data.bstPolyData.GetBounds(),
                    color=(node.level*2, node.level*3, node.level*4))
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

    # create Binary Search Tree
    self.root = self.InitializeTree()

#----------------------------------------------------------------------#
  def isVisible(self, treeNode, fsBounds, vtkPlanes):
    # check if the current tree node is within the bounds of the frustum
    if treeNode.visible:
      treeNode.data.bstActor.SetVisibility(True)
    else:
      treeNode.data.bstActor.SetVisibility(False)

#----------------------------------------------------------------------#
  def checkIntersection(self, bounds1, bounds2):
    # check whether or not the bounds are overlapping one anotherr
    isOverlapping = (bounds1[0] <= bounds2[1] or bounds2[0] <= bounds2[1] \
                 and bounds1[2] <= bounds2[3] or bounds2[2] <= bounds2[3] \
                 and bounds1[4] <= bounds2[5] or bounds2[5] <= bounds2[4])
    return isOverlapping

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
  def subExecute(self, index, cN):
    # select the current node and check if it exists and contains data
    # cN = search(self.root, nodeIndex)
    if not cN or cN.empty:
      return
    
    if cN.loaded:
      # cN.data.bounding_box_actor.SetVisibility(False)
      return False
    
    # when the points get to a certain increment, render the polydatas
    # renders a little faster
    if self.pointCount % 15000 == 0:
        self.obj.GetRenderWindow().Render()

    # mark node as loaded
    cN.loaded = True

    # make it visible (if inside frustum)
    self.isVisible(cN, self.frustum.GetBounds(), self.planes)
    
    # safely remove it from queue after processing
    if 0 <= index < len(self.priorityQueue):
        self.priorityQueue.pop(index)

    nodePoints = cN.data.bstPolyData.GetNumberOfPoints()

    # if adding this node would exceed budget, stop for now
    if self.pointCount + nodePoints > 1000000: #self.PolyData.GetNumberOfPoints():
        return True  # reached threshold, stop early
    
    self.pointCount += nodePoints

    return False  # continue processing

#----------------------------------------------------------------------#
  def execute(self, obj, event):
    # print(f"Actors in renderer: {self.renderer.GetActors().GetNumberOfItems()}")

    if not self.frustum or not self.priorityQueue:
        return

    self.obj = obj
    self.sortPriorityQueue()

    # process until empty or threshold hit
    i = 0
    while i < len(self.priorityQueue):
        _, node = self.priorityQueue[i]

        # subExecute returns True if threshold reached
        stop = self.subExecute(i, node)
        if stop:
            break  # stop early when point budget reached

        # only increment if we didn't pop
        else:
            i += 1
    
    if self.bounding_boxes:
        self.boundingBoxManager.toggleVisibility(True)
        self.boundingBoxManager.updateBoxes(self.priorityQueue)
    else:
        self.boundingBoxManager.toggleVisibility(False)

    self.obj.GetRenderWindow().Render()
    
    # Update true current visible points count
    self.pointCount = self.get_current_visible_points_count()
    # print(f"Current visible point count: {self.pointCount}")
    
    self.priorityQueue = []

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
    self.priorityQueue = list(q[np.argsort(q[:, 0])])

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
          inScene = self.is_point_in_frustum(child.data.center, vtkPlanes)

          if inScene:
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
  '''
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
  '''

  def updateBoundingBoxes(self):
    print("Updating Bounding Boxes")
    # Call this after updating visible nodes
    print(self.priorityQueue)
    self.boundingBoxManager.updateBoxes(self.priorityQueue)

#----------------------------------------------------------------------#
  '''
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
    self.subVolumes = CreateSubVolumes(volumeBounds=bounds, numSubV=100)

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
  '''
  def InitializeTree(self):
      """
      Build a spatial LOD binary tree:
       - recursively median-split along the longest axis
       - assign sequential integer Ids to nodes so existing search()/insert() usage still works
       - create BSTPointCloud at each node containing either a downsampled set (for higher-level nodes)
         or the full (or near-full) point set for leaf nodes
       - add actors to renderer for nodes that have data
      """

      # --- prepare numpy points and colors (same approach you used before) ---
      points = ns.vtk_to_numpy(self.PolyData.GetPoints().GetData())            # Nx3
      Ids = np.arange(start=0, stop=len(points), step=1).reshape(len(points), 1)  # Nx1 (original indices)
      colors = ns.vtk_to_numpy(self.PolyData.GetPointData().GetScalars())     # Nx3 or Nx4 depending on file

      # keep global access for other code
      self.modify_points_global(np.concatenate([points, Ids], axis=1))  # POINTS: Nx4 (x,y,z,origIndex)
      self.modify_colors_global(colors)

      # clear local references
      del points
      del Ids
      del colors

      # convenience references
      P = POINTS        # Nx4: x,y,z,origIndex
      C = COLORS        # (original colors array indexed by origIndex)

      # compute overall bounds (min/max)
      bounds = [
          float(P[:,0].min()), float(P[:,0].max()),
          float(P[:,1].min()), float(P[:,1].max()),
          float(P[:,2].min()), float(P[:,2].max())
      ]

      # parameters for LOD tree - tune as needed
      MAX_DEPTH = 9            # max recursion depth
      MIN_POINTS_LEAF = 30000   # if <= this, stop subdividing and make leaf
      BASE_PARENT_SAMPLE = 0.15  # fraction of points to keep at top-level parent nodes (coarse LOD)
      MIN_SAMPLE_POINTS = 8192   # ensure parent LOD has at least this many samples

      # node id generator (sequential ints so search(root, id) works)
      next_id = [0]
      def gen_id():
          i = next_id[0]
          next_id[0] += 1
          return i

      # helper: points mask within bounds
      def mask_points_in_bounds(ptarr, b):
          xmin, xmax, ymin, ymax, zmin, zmax = b
          mask = (
              (ptarr[:,0] >= xmin) & (ptarr[:,0] <= xmax) &
              (ptarr[:,1] >= ymin) & (ptarr[:,1] <= ymax) &
              (ptarr[:,2] >= zmin) & (ptarr[:,2] <= zmax)
          )
          return mask

      # helper: build BSTPointCloud for subset of indices; apply sampling for LOD
      def build_bst_pointcloud(indices, depth):
          if indices.size == 0:
              return None

          # extract coordinates
          coords = P[indices, 0:3]                       # Nx3 coordinates
          orig_indices = P[indices, 3].astype(np.int32) # original point indices -> use for color lookup

          # choose sampling fraction based on depth (shallower => more aggressive downsampling)
          # sample_fraction = BASE_PARENT_SAMPLE scaled by depth-to-max-depth
          frac = BASE_PARENT_SAMPLE * (0.5 ** depth)     # deeper nodes keep more (frac increases with depth)
          # but ensure fraction is not > 1 and not so tiny that no points remain
          frac = min(1.0, max(frac, MIN_SAMPLE_POINTS / max(len(indices), 1)))
          sample_n = int(round(frac * len(indices)))
          sample_n = max(1, min(sample_n, len(indices)))

          # for leaves (small number of points) keep them all (or near-all)
          if len(indices) <= MIN_POINTS_LEAF:
              sample_idx = np.arange(len(indices))
          else:
              # random uniform sampling but deterministic seed for reproducibility
              rng = np.random.default_rng(seed=42 + int(depth))
              sample_idx = rng.choice(len(indices), size=sample_n, replace=False)

          sampled_coords = coords[sample_idx]
          sampled_orig = orig_indices[sample_idx]

          # fetch colors using original indices
          try:
              sampled_colors = C[sampled_orig]
          except Exception:
              # fallback if color shape mismatch
              sampled_colors = np.zeros((sampled_coords.shape[0], 3), dtype=np.uint8)

          # create BSTPointCloud (keep CURRENT_VTK_SHADER behaviour)
          bstpc = BSTPointCloud(sampled_coords, sampled_colors, self.current_vtk_shader)
          return bstpc

      # recursive subdivide function: returns TreeNode
      def subdivide(bounds_in, depth=0):
          xmin, xmax, ymin, ymax, zmin, zmax = bounds_in
          # get mask of points that lie in this bounds
          mask = mask_points_in_bounds(P[:,0:3], bounds_in)
          indices = np.nonzero(mask)[0]  # indices into P

          node_id = gen_id()

          # if no points, create empty node (no BSTPointCloud)
          if indices.size == 0:
              node = TreeNode(node_id, bounds_in, None, level=depth)
              node.empty = True
              return node

          # create pointcloud data for this node (LOD sample depends on depth)
          bstpc = build_bst_pointcloud(indices, depth)

          node = TreeNode(node_id, bounds_in, bstpc, level=depth)
          node.empty = (bstpc is None)

          # stop criteria: depth or small number of points
          if depth >= MAX_DEPTH or len(indices) <= MIN_POINTS_LEAF:
              # leaf node: do not subdivide further
              # add actor now (if data exists)
              if node.data:
                  try:
                      self.renderer.AddActor(node.data.bstActor)
                      self.renderer.AddActor(node.data.bounding_box_actor)
                  except Exception:
                      pass
              return node

          # decide split axis = longest axis
          dx = xmax - xmin
          dy = ymax - ymin
          dz = zmax - zmin
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

          # recursively subdivide children
          node.left = subdivide(left_bounds, depth + 1)
          node.right = subdivide(right_bounds, depth + 1)

          # add actor for this node (we add actors for internal nodes too — they are coarse LOD)
          if node.data:
              try:
                  self.renderer.AddActor(node.data.bstActor)
                  self.renderer.AddActor(node.data.bounding_box_actor)
              except Exception:
                  pass

          return node

      # build the root node and tree
      root = subdivide(bounds, depth=0)

      # store and return
      self.root = root
      return root

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

    if obj.GetKeySym() == 'b':  # Press 'b' to toggle bounding boxes
      #if hasattr(self.cloudInfo, "boundingBoxManager"):
          #bbm = self.cloudInfo.boundingBoxManager
      if self.cloudInfo.bounding_boxes:
        self.cloudInfo.bounding_boxes = False
      else:
        self.cloudInfo.bounding_boxes = True

          # Step 2: toggle visibility
          #bbm.toggleVisibility()

      # obj.GetRenderWindow().Render()

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
  screenWidth = 3200
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
  #renderWindowInteractor.AddObserver('TimerEvent', distApproxVoxelGrid.execute)
  renderWindowInteractor.AddObserver('KeyPressEvent', style.execute)
  # renderWindowInteractor.AddObserver('TimerEvent', occlFilter.execute)
  timerId = renderWindowInteractor.CreateRepeatingTimer(1)

  # render and start the program
  renderWindow.Render()
  renderWindowInteractor.Start()

if __name__ == "__main__":
    main()
