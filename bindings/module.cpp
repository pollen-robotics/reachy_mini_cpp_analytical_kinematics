#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <ostream>

#include "module.h"

BOOST_PYTHON_MODULE(reachy_mini_kinematics) {
  using namespace boost::python;

  expose_eigen();
  expose_kinematics();
}