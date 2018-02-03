/*********************************************************
*                                                        *
*   Adapted from:                                        *
*   PrimeSense NiTE 2.0 - User Viewer Sample             *
*                                                        *
*********************************************************/

#include "kinect_v2/skviewer-gl.hpp"


int main(int argc, char** argv)
{
  openni::Status rc = openni::STATUS_OK;

  SkViewerGl SkViewerGl("Viewer printing data");

  rc = SkViewerGl.Init(argc, argv);
  if (rc != openni::STATUS_OK)
  {
    return 1;
  }
  SkViewerGl.Run();
}
