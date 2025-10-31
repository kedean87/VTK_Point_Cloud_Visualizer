import vtk
from timeit import default_timer as timer

class FpsObserver:
  def __init__(self, renderer, x=0, y=0):
    self.mRenderer = renderer
    self.mRenderer.AddObserver(vtk.vtkCommand.EndEvent, self)
    
    self.ActorPosX = x
    self.ActorPosY = y
    
    self.mFrameCount    = 0         # Number of frames collected since last FPS was calculated.
    self.mStartTime     = timer()   # The last time FPS was calculated.
    self.mFpsUpdateRate = 1         # How often to update FPS in seconds.
    
    self._createFpsTextActor()
  
  def setPosition(self, x, y):
    self.ActorPosX = x
    self.ActorPosY = y
    self.mFpsActor.SetPosition(self.ActorPosX, self.ActorPosY)
  
  def __call__(self, caller, event):
    if event == "EndEvent":
      self.mFrameCount = self.mFrameCount + 1
      
      if timer() - self.mStartTime > self.mFpsUpdateRate:
        _currentTime    = timer()
        _duration        = _currentTime - self.mStartTime
        
        _fps = self.mFrameCount/_duration
        # ~ print("fps={:.3f}".format(_fps))
        self.mFpsActor.SetInput("FPS: {:.3f}".format(_fps))
        
        self.mStartTime  = _currentTime
        self.mFrameCount = 0
              
  def _createFpsTextActor(self):
    self.mFpsActor = vtk.vtkTextActor()
    self.mFpsActor.GetTextProperty().SetFontFamilyAsString("Georgia")
    self.mFpsActor.GetTextProperty().SetFontSize(125)
    self.mFpsActor.GetTextProperty().SetColor([1, 2, 3])
    self.mFpsActor.SetPosition(self.ActorPosX, self.ActorPosY)
    self.mRenderer.AddActor(self.mFpsActor)
