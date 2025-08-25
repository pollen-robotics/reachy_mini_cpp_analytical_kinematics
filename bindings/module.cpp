#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <ostream>

#include "module.h"

BOOST_PYTHON_MODULE(perloc) {
  using namespace boost::python;

  expose_eigen();
}