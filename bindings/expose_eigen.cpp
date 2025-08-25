#include "module.h"
#include <iostream>

#include "doxystub.h"
#include <boost/python.hpp>

#include <eigenpy/eigen-to-python.hpp>
#include <eigenpy/eigenpy.hpp>

using namespace boost::python;

// Wrapper to convert Affine3d to a numpy 4x4 matrix
struct Affine3d_to_np {
  static PyObject *convert(Eigen::Affine3d const &T) {
    Py_intptr_t shape[2] = {4, 4};
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> M = T.matrix();

    PyObject *array = PyArray_SimpleNew(2, shape, (int)NPY_DOUBLE);

    std::memcpy(PyArray_DATA((PyArrayObject *)array), M.data(),
                4 * 4 * sizeof(double));

    return array;
  }
};

void expose_eigen() {
  eigenpy::enableEigenPy();

  // Thanks to this, Affine3d will be seamlessly converted from/to numpy 4x4
  // matrices
  implicitly_convertible<Eigen::Matrix4d, Eigen::Affine3d>();
  to_python_converter<Eigen::Affine3d, Affine3d_to_np, false>();
}