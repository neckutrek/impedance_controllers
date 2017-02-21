#ifndef DUMMY_POSITION_CONTROLLER
#define DUMMY_POSITION_CONTROLLER

#include "base_controller.h"

namespace irb4400 {

  typedef 
  controller_interface::Controller<hardware_interface::PositionJointInterface>
  JointPositionController;

  typedef 
  hardware_interface::PositionJointInterface 
  JointPositionInterface;

  class DummyPositionController : public BaseController<JointPositionController, JointPositionInterface> {
  public:
    DummyPositionController() = default;

    ~DummyPositionController() noexcept = default;

    void initialize();

    void computeControls(Eigen::VectorXd& u);

  private:

  };

}

#endif