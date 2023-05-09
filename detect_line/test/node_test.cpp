//* Include system header files
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include "detect_line/detect_line.hpp"

using namespace cv;

int main(int argc, char **argv) {
  DetectLine nh;
  nh.init(argc, argv, "test", 1);

  Mat imag, result;
  imag = imread("");

//   int count = 0;
//   while (ros::ok()) {
//     printf("Hello, world! %d\n", count);
//     count++;
//     nh.getRosRate()->sleep();
//   }

  nh.spin();
  nh.shutdown();
  return 0;
}
