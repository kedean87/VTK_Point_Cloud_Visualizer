from BinarySearchTree import *
from VTKShaderPresets import *
from BSTPointCloud import *
from FilterOccludedPoints import *
from Frustum import *
import numpy as np
import vtk

def boxIntersectsFrustum(bounds, vtk_planes):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    corners = np.array([
        [xmin, ymin, zmin], [xmax, ymin, zmin],
        [xmin, ymax, zmin], [xmax, ymax, zmin],
        [xmin, ymin, zmax], [xmax, ymin, zmax],
        [xmin, ymax, zmax], [xmax, ymax, zmax],
    ])

    # vtkPlanes gives us normals and points (origins)
    normals = vtk_planes.GetNormals()
    points = vtk_planes.GetPoints()

    for i in range(normals.GetNumberOfTuples()):
        n = np.array(normals.GetTuple3(i))
        p0 = np.array(points.GetPoint(i))
        outside = 0
        for corner in corners:
            # signed distance from plane
            if np.dot(n, corner - p0) < 0:
                outside += 1
        if outside == 8:
            # all corners are behind this plane
            return False
    return True


class UpdateNodeVisibility:
    def __init__(self, handleVTKCloud):
        self.handleVTKCloud = handleVTKCloud

    def execute(self, obj, event):
        if not self.handleVTKCloud:
            return
        
        vtk_planes = getattr(self.handleVTKCloud, "planes", None)
        if vtk_planes is None:
            return
            
        camera = self.handleVTKCloud.renderer.GetActiveCamera()
        
        camera_center = (
            camera.GetPosition()
            if hasattr(self.handleVTKCloud.frustum, "GetCenter")
            else (0, 0, 0)
        )

        # keep existing entries and only add *new visible nodes*
        self.updateVisibleNodes(self.handleVTKCloud.root, camera_center)

    def updateVisibleNodes(self, node, camera_center):
        if node is None or node.empty:
            return

        # ✅ Correct frustum–AABB intersection for vtkPlanes
        #if not boxIntersectsFrustum(node.bounds, vtk_planes):
        #    node.visible = False
        #    return

        node_center = node.data.center
        dist = np.linalg.norm(np.array(camera_center) - np.array(node_center))
        lod_threshold = self.lodThreshold(node.level)
        
        if dist < lod_threshold and (node.left or node.right):
            self.updateVisibleNodes(node.left, camera_center)
            self.updateVisibleNodes(node.right, camera_center)
        else:
            node.visible = True
            self.handleVTKCloud.priorityQueue.append((dist, node))
            node.data.bstActor.SetVisibility(True)
            return
        
        node.data.bstActor.SetVisibility(False)
    
    def lodThreshold(self, level):
        max_dist = 10.0   # threshold at root level
        min_dist = 0.1    # deepest level threshold limit
        k = 0.3             # rate of decay (lower = slower)
        p = 2.0             # curve sharpness (higher = more asymptotic)

        return min_dist + (max_dist - min_dist) / (1 + k * (level ** p))

