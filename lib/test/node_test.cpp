//* Include system header files
#include <stdio.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include "lib/node_handle.hpp"

int main(int argc, char **argv) {
  NodeHandle nh;
  nh.init(argc, argv, "test", 1);

  int count = 0;

  while (ros::ok()) {
    printf("Hello, world! %d\n", count);
    count++;
    nh.getRosRate()->sleep();
  }
  nh.spin();
  nh.shutdown();
  return 0;
}
