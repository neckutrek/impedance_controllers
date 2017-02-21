#include "dummy_position_controller.h"

#include <pluginlib/class_list_macros.h>

namespace irb4400 {

  void DummyPositionController::initialize() {

  }

  void DummyPositionController::computeControls(Eigen::VectorXd& u) {
  	u << 0, 0, 0, 0, 0, 0;
  }

}

PLUGINLIB_EXPORT_CLASS(irb4400::DummyPositionController, controller_interface::ControllerBase)