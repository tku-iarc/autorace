#include "detect_line/detect_line.hpp"

DetectLine::DetectLine() {
  setNodeName("DetectLine");

  getParam("is_detection_calibration_mode", this->is_detection_calibration_mode);

  getParam("background/black/hue_black_min", this->hue_black_min);
  getParam("background/black/hue_black_max", this->hue_black_max);
  getParam("background/black/saturation_black_min", this->saturation_black_min);
  getParam("background/black/saturation_black_max", this->saturation_black_max);
  getParam("background/black/lightness_black_min", this->lightness_black_min);
  getParam("background/black/lightness_black_max", this->lightness_black_max);

}

DetectLine::~DetectLine() {
}

