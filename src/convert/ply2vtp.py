import vtk
from vtk.util import numpy_support
import numpy as np

# --- Input/Output files ---
ply_file = "/Users/kedean/workspace/python/data/rgbd-scenes-v2/pc/11.ply"   # Source .ply file
vtp_file = "example.vtp"  # Output file (renamed for clarity)

# --- Load the PLY file ---
reader = vtk.vtkPLYReader()
reader.SetFileName(ply_file)
reader.Update()
polydata = reader.GetOutput()

# --- Extract points ---
points_vtk = polydata.GetPoints()
points_np = numpy_support.vtk_to_numpy(points_vtk.GetData())

# --- Extract RGB if it exists ---
colors_vtk = polydata.GetPointData().GetScalars()
if colors_vtk is not None:
    colors_np = numpy_support.vtk_to_numpy(colors_vtk)
else:
    # Fallback: check for any 3-component array
    colors_np = None
    n_arrays = polydata.GetPointData().GetNumberOfArrays()
    for i in range(n_arrays):
        arr = polydata.GetPointData().GetArray(i)
        if arr and arr.GetNumberOfComponents() == 3:
            colors_np = numpy_support.vtk_to_numpy(arr)
            break

# --- Handle missing or mismatched color arrays ---
if colors_np is None or len(colors_np) != len(points_np):
    print("No valid RGB found — generating synthetic colors.")
    n = len(points_np)
    colors_np = (np.random.rand(n, 3) * 255).astype(np.uint8)

# --- Downsample to 300,000 points ---
target_points = 300_000
if len(points_np) > target_points:
    indices = np.random.choice(len(points_np), target_points, replace=False)
    points_np = points_np[indices]
    colors_np = colors_np[indices]
    print(f"Downsampled to {target_points:,} points.")
else:
    print(f"Original point count ({len(points_np):,}) ≤ 300,000 — keeping all points.")

# --- Create new VTK PolyData ---
new_poly = vtk.vtkPolyData()
vtk_points = vtk.vtkPoints()
vtk_points.SetData(numpy_support.numpy_to_vtk(points_np))
new_poly.SetPoints(vtk_points)

# --- Attach RGB ---
rgb_array = numpy_support.numpy_to_vtk(colors_np, array_type=vtk.VTK_UNSIGNED_CHAR)
rgb_array.SetNumberOfComponents(3)
rgb_array.SetName("RGB")
new_poly.GetPointData().SetScalars(rgb_array)

# --- Convert to vertices for rendering ---
vg = vtk.vtkVertexGlyphFilter()
vg.SetInputData(new_poly)
vg.Update()

# --- Write out the VTP file ---
writer = vtk.vtkXMLPolyDataWriter()
writer.SetFileName(vtp_file)
writer.SetInputData(vg.GetOutput())
writer.Write()

print(f"🎉 Wrote {vtp_file} successfully with {len(points_np):,} points.")
