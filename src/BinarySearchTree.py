'''
Binary Search Tree Class:

Variables of each node within the Tree:
  (1) left / right child Node -> TreeNode() Class
  (2) Id                      -> node identification
  (3) bounds                  -> bounding box region node is responsible
                                 for
  (4) data                    -> BSTPointCloud() Class to store the vtk
                                 polydata points / colors, vtkActors,
                                 etc.
  (5) empty                   -> Boolean flag letting program know
                                 whether or not it should use the region
                                 (depends upon whether there are points
                                 within the region or not).
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

########################################################################
class TreeNode:
    def __init__(self, Id, bounds, data, level=0):
        self.left = None
        self.right = None
        
        self.Id = Id
        self.bounds = bounds
        self.data = data
        self.level = level  # new attribute
        
        self.empty = True
        if data:
            self.empty = False
        
        self.loaded = False

    
########################################################################
# FUNCTIONS
#----------------------------------------------------------------------#
def insert(root, Id, bounds, data):
  if root is None:
    return TreeNode(Id, bounds, data)
  else:
    if root.Id == Id:
      return root
    elif root.Id < Id:
      root.right = insert(root.right, Id, bounds, data)
    else:
      root.left = insert(root.left, Id, bounds, data)
  return root

#----------------------------------------------------------------------#
def inorder(root):
  if root:
    inorder(root.left)
    print(root.Id)
    inorder(root.right)

#----------------------------------------------------------------------#
def search(root, Id):
  if root is None or root.Id == Id:
    return root

  if root.Id < Id:
    return search(root.right, Id)
 
  return search(root.left, Id)

#----------------------------------------------------------------------#
def traverse(rootNode, level=0, debug=True):
  children = [rootNode.left, rootNode.right]
  for child in children:
    if child:
      traverse(child, level + 1)
      
