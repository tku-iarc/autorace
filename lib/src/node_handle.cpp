#include "lib/node_handle.hpp"

NodeHandle::NodeHandle() {
  this->node_name = "node";
  this->nh = nullptr;
  this->rate = nullptr;
}

NodeHandle::~NodeHandle() {
  delete this->nh;
  delete this->rate;
}

void NodeHandle::init(int argc, char **argv, string node_name) {
  ros::init(argc, argv, node_name);
  this->node_name = node_name;
  this->nh = new ros::NodeHandle();
}

void NodeHandle::spin() { ros::spin(); }

void NodeHandle::shutdown() { ros::shutdown(); }

void NodeHandle::setNodeName(string node_name) { this->node_name = node_name; }

string NodeHandle::getNodeName() { return this->node_name; }

ros::NodeHandle *NodeHandle::getRosNodeHandle() { return this->nh; }

ros::Rate *NodeHandle::getRosRate() { return this->rate; }

void NodeHandle::setRosRate(int rate) { this->rate = new ros::Rate(rate); }

void NodeHandle::setRosRate(double rate) { this->rate = new ros::Rate(rate); }

void NodeHandle::setRosRate(ros::Rate *rate) { this->rate = rate; }

void NodeHandle::setRosNodeHandle(ros::NodeHandle *nh) { this->nh = nh; }
