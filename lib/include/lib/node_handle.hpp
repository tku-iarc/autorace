#ifndef NodeHandle_H
#define NodeHandle_H
/*******************************
 * Include system header files
 ******************************/
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
/*******************************
 * Include ROS header fi3les
 ******************************/
#include "ros/ros.h"
/*******************************
 * Define
 ******************************/
// #define DEBUG 1

using namespace std;
/**
 * @brief NodeHandle class
 * @details This class is used to create a node handle for ROS
 * @param nh ROS node handle
 * @param rate ROS rate
 */
class NodeHandle {
 public:
  NodeHandle();
  ~NodeHandle();
  void init(int argc, char **argv, string node_name);
  void init(int argc, char **argv, string node_name, int rate);
  void init(int argc, char **argv, string node_name, double rate);
  void init(int argc, char **argv, string node_name, ros::Rate *rate);
  void spin();
  void shutdown();

  // set functions
  void setNodeName(string node_name);
  void setRosRate(int rate);
  void setRosRate(double rate);
  void setRosRate(ros::Rate *rate);
  void setRosNodeHandle(ros::NodeHandle *nh);
  void setParam(string param_name, int param_value);

  // get functions
  string getNodeName();
  ros::NodeHandle *getRosNodeHandle();
  ros::Rate *getRosRate();
  void getParam(string param_name, int &param_value);

 private:
  ros::NodeHandle *nh;
  ros::Rate *rate;
  string node_name;
};

#endif
