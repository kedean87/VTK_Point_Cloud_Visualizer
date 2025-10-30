########################################################################
class VTKShaders():
########################################################################
# FUNCTIONS
#----------------------------------------------------------------------#
  def __init__(self):
    self.SPHERE = \
      "//VTK::Color::Impl\n" \
      "float dist = dot(offsetVCVSOutput.xy,offsetVCVSOutput.xy);\n" \
      "if (dist > 1.0) {\n" \
      "  discard;\n" \
      "} else {\n" \
      "  float scale = (1.0 - dist);\n" \
      "  ambientColor *= scale;\n" \
      "  diffuseColor *= scale;\n" \
      "}\n"
      
    self.BLACK_EDGED_CIRCLE = \
      "//VTK::Color::Impl\n" \
      "float dist = dot(offsetVCVSOutput.xy,offsetVCVSOutput.xy);\n" \
      "if (dist > 1.0) {\n" \
      "  discard;\n" \
      "} else if (dist > 0.8) {\n" \
      "  ambientColor = vec3(0.0, 0.0, 0.0);\n" \
      "  diffuseColor = vec3(0.0, 0.0, 0.0);\n" \
      "}\n"
      
    self.PLAIN_CIRCLE = \
      "//VTK::Color::Impl\n" \
      "float dist = dot(offsetVCVSOutput.xy,offsetVCVSOutput.xy);\n" \
      "if (dist > 1.0) {\n" \
      "  discard;\n" \
      "};\n"
    
    self.TRIANGLE = "//VTK::Color::Impl\n"
    
    self.TRIANGLE_OUTLINE = \
      "//VTK::Color::Impl\n" \
      "if (abs(offsetVCVSOutput.x) < 1.5 && abs(offsetVCVSOutput.y) < 1.5) {\n" \
      "  discard;\n" \
      "}\n"
    
    self.DIAMOND = \
      "//VTK::Color::Impl\n" \
      "if (abs(offsetVCVSOutput.x) > 2.2 || abs(offsetVCVSOutput.y) > 2.2) {\n" \
      "  discard;\n" \
      "}\n" \
      "if ((abs(offsetVCVSOutput.y) > (-abs(offsetVCVSOutput.x) + 0.7334)))\n" \
      "{\n" \
      "  discard;\n" \
      "}\n"

    self.OCTAGON = \
      "//VTK::Color::Impl\n" \
      "if (abs(offsetVCVSOutput.x) > 2.2 || abs(offsetVCVSOutput.y) > 2.2) {\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.y) > abs(offsetVCVSOutput.x + 3.3)) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.y) > abs(offsetVCVSOutput.x - 3.3)) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n"
    
    self.OBLONG_OCTAGON = \
      "//VTK::Color::Impl\n" \
      "if (abs(offsetVCVSOutput.x) > 2.2 || abs(offsetVCVSOutput.y) > 2.2) {\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.y) > abs(offsetVCVSOutput.x + 2.6)) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.y) > abs(offsetVCVSOutput.x - 2.6)) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.x) > 1.4445) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n" 
    
    self.SQUARE = \
      "//VTK::Color::Impl\n" \
      "if (abs(offsetVCVSOutput.x) > 2.2 || abs(offsetVCVSOutput.y) > 2.2) {\n" \
      "  discard;\n" \
      "}\n"
    
    self.SQUARE_OUTLINE = \
      "//VTK::Color::Impl\n" \
      "if (abs(offsetVCVSOutput.x) > 2.2 || abs(offsetVCVSOutput.y) > 2.2) {\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.x) < 1.5 && abs(offsetVCVSOutput.y) < 1.5) {\n" \
      "  discard;\n" \
      "}\n"
    
    self.CUSTOM = \
      "//VTK::Color::Impl\n" \
      "float dist = dot(offsetVCVSOutput.xy,offsetVCVSOutput.xy);\n" \
      "if (dist > 1.2) {\n" \
      "  discard;\n" \
      "};\n" \
      "if (abs(offsetVCVSOutput.y) > abs(offsetVCVSOutput.x + 1.4)) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n" \
      "if (abs(offsetVCVSOutput.y) > abs(offsetVCVSOutput.x - 1.4)) \n" \
      "{\n" \
      "  discard;\n" \
      "}\n"
    
    # set up a list an Nx2 list which contains vtk shader preset shapes
    # as well as preset splat scale.
    self.Shaders = [
      [self.SPHERE, 0.00165],
      [self.BLACK_EDGED_CIRCLE, 0.001165],
      [self.PLAIN_CIRCLE, 0.001165],
      [self.TRIANGLE, 0.001],
      [self.TRIANGLE_OUTLINE, 0.001],
      [self.DIAMOND, 0.006265],
      [self.OCTAGON, 0.001],
      [self.OBLONG_OCTAGON, 0.0031],
      [self.SQUARE, 0.0031],
      [self.SQUARE_OUTLINE, 0.0031],
      [self.CUSTOM, 0.003155]
      ]
