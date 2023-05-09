#ifndef DetectLine_H
#define DetectLine_H

/*******************************
 * Include system header files
 ******************************/
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
/*******************************
 * Include ROS header files
 ******************************/
#include "lib/node_handle.hpp"
/*******************************
 * Define
 ******************************/
// #define DEBUG 1

using namespace std;

class DetectLine : public NodeHandle {
 public:
  DetectLine();
  ~DetectLine();


 private:
  int is_detection_calibration_mode = false;

  int hue_black_min = 0;
  int hue_black_max = 255;
  int saturation_black_min = 0;
  int saturation_black_max = 255;
  int lightness_black_min = 0;
  int lightness_black_max = 255;


};

#endif
