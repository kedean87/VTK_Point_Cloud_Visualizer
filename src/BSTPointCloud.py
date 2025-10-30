from BinarySearchTree import *
from VTKShaderPresets import *
from FilterOccludedPoints import *
from Frustum import *
from NodeVisbilitiy import *

########################################################################
class BSTPointCloud(VTKShaders):
########################################################################
# FUNCTIONS
#----------------------------------------------------------------------#
  def __init__(self, points, colors, CURRENT_VTK_SHADER):
    # make sure the class has access to the shader preset codes / scales
    self.vtkShaders = VTKShaders()
    
    self.center = [
		(points.T[0].max() - points.T[0].min()) / 2,
		(points.T[1].max() - points.T[1].min()) / 2,
		(points.T[2].max() - points.T[2].min()) / 2
		]
    
    # convert data from numpy array to vtkPoints
    vtkPoints = vtk.vtkPoints()
    vtkPoints.SetData(ns.numpy_to_vtk(points, deep=True))
    
    # convert color data from numpy array to vtkUnsignedCharArray
    vtkColorArray = ns.numpy_to_vtk(colors, array_type=3)
    vtkColorArray.SetName("RGB")
    
    # create a new polydata to store points, vertices, and color
    newPd = vtk.vtkPolyData()
    newPd.SetPoints(vtkPoints)
    newPd.GetPointData().SetScalars(vtkColorArray)
    
    # this vertex filter creates cell data for the polygonal output
    vgf = vtk.vtkVertexGlyphFilter()
    vgf.SetInputData(newPd)
    vgf.Update()
    
    # update polydata for the gaussian mapper
    self.bstPolyData = vgf.GetOutput()
    self.bstEdgeVisibility = False
    self.visiblePointIds = None
    
    self.locator = vtk.vtkCellLocator()
    self.locator.SetDataSet(self.bstPolyData)
    self.locator.BuildLocator()
    
    # create a point gaussian mapper to acquire screen splat projections
    self.pg_mapper = vtk.vtkPointGaussianMapper()
    self.pg_mapper.SetInputData(self.bstPolyData)
    self.pg_mapper.SetScaleArray('radius')
    self.pg_mapper.SetScalarVisibility(True)
    self.pg_mapper.SetScaleFactor(self.vtkShaders.Shaders[CURRENT_VTK_SHADER][1])
    self.pg_mapper.SetEmissive(False)
    self.pg_mapper.SetSplatShaderCode(self.vtkShaders.Shaders[CURRENT_VTK_SHADER][0])
    
    # add mapper to the actor representing the region of points
    self.bstActor = vtk.vtkActor()
    self.bstActor.SetMapper(self.pg_mapper)
    self.bstActor.GetProperty().BackfaceCullingOn()
    self.bstActor.GetProperty().SetEmissiveFactor(2.0, 2.0, 2.0)
