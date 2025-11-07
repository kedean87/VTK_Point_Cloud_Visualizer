from BinarySearchTree import *
from VTKShaderPresets import *
from BSTPointCloud import *
from FilterOccludedPoints import *
from Frustum import *
import numpy as np
import vtk


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
        camera_pos = np.array(camera.GetPosition())
        focal_point = np.array(camera.GetFocalPoint())
        camera_forward = focal_point - camera_pos
        camera_forward /= np.linalg.norm(camera_forward)

        self.updateVisibleNodes(camera_pos, camera_forward)

    def updateVisibleNodes(self, camera_pos, camera_forward):
        if not self.handleVTKCloud.root or self.handleVTKCloud.root.empty:
            return

        nodes_to_check = [self.handleVTKCloud.root]

        while nodes_to_check:
            batch = nodes_to_check
            nodes_to_check = []

            # Filter nodes with valid data
            batch = [n for n in batch if n and n.data and hasattr(n.data, "center")]
            if not batch:
                continue

            centers = np.array([n.data.center for n in batch])
            vecs_to_nodes = centers - camera_pos
            dists = np.linalg.norm(vecs_to_nodes, axis=1)

            # Distance to center line (perpendicular)
            norms = np.linalg.norm(vecs_to_nodes, axis=1, keepdims=True)
            vecs_normed = vecs_to_nodes / np.clip(norms, 1e-8, None)
            cos_thetas = np.dot(vecs_normed, camera_forward)
            cross_prods = np.cross(vecs_to_nodes, camera_forward)
            dists_to_center_line = np.linalg.norm(cross_prods, axis=1)

            # Compute LOD thresholds for all nodes
            levels = np.array([n.level for n in batch])
            lods = np.array([self.lodThreshold(l) for l in levels])
            adjusted_lods = lods * (0.25 + 0.25 * cos_thetas)

            # Visibility: dual distance
            visible_mask = (dists < adjusted_lods) & (dists_to_center_line < lods)

            for i, node in enumerate(batch):
                node.visible = visible_mask[i] or (node.level == 0)
                if node.visible:
                    self.handleVTKCloud.priorityQueue.append((dists[i], node))
                    if node.left: nodes_to_check.append(node.left)
                    if node.right: nodes_to_check.append(node.right)

    def point_in_frustum(self, point, vtk_planes):
        num_planes = vtk_planes.GetNumberOfPlanes()
        for i in range(num_planes):
            plane = vtk_planes.GetPlane(i)  # vtkPlane
            normal = np.array(plane.GetNormal())
            origin = np.array(plane.GetOrigin())
            # if point is "behind" the plane, it's outside
            if np.dot(point - origin, normal) < 0:
                return False
        return True

    def lodThreshold(self, level):
        max_dist = 100.0
        min_dist = 1.0
        decay_rate = 0.6
        curve_shape = 2.0
        return min_dist + (max_dist - min_dist) / (1 + decay_rate * (level ** curve_shape))
