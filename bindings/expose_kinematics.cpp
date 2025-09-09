#include "doxystub.h"
#include "kinematics.h"
#include <eigenpy/eigen-to-python.hpp>

using namespace reachy_mini_kinematics;
using namespace boost::python;

void expose_kinematics() {
  class__<Kinematics>("Kinematics", init<double, double>())
      .def_readwrite("motor_arm_length", &Kinematics::motor_arm_length)
      .def_readwrite("rod_length", &Kinematics::rod_length)
      .def_readwrite("line_search_maximum_iterations",
                     &Kinematics::line_search_maximum_iterations)
      .def("add_branch", &Kinematics::add_branch)
      .def("inverse_kinematics", &Kinematics::inverse_kinematics)
      .def("reset_forward_kinematics", &Kinematics::reset_forward_kinematics)
      .def("forward_kinematics", &Kinematics::forward_kinematics)
      .def("wrap_angle", &Kinematics::wrap_angle)
      .staticmethod("wrap_angle");
}
