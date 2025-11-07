#include "NvVideoEncoder.h"
#include <cstdio>
int main() {
  NvVideoEncoder* e = NvVideoEncoder::createVideoEncoder("enc0");
  if(!e){ std::puts("createVideoEncoder failed"); return 1; }
  e->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, 640, 480);
  e->setCapturePlaneFormat(V4L2_PIX_FMT_HEVC, 640, 480, 2*1024*1024);
  e->setBitrate(4000000);
  e->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
  e->setFrameRate(30,1);
  e->setProfile(V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
  e->setIDRInterval(10);
  e->setInsertSpsPpsAtIdrEnabled(true);
  return 0;
}
