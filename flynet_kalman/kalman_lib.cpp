#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/call.hpp>
#include "kalman_filter.h"

namespace p = boost::python;
namespace np = boost::python::numpy;

BOOST_PYTHON_MODULE(_kalman_lib)
{
	Py_Initialize();
	np::initialize();
	p::class_<Kalman>("Kalman")
		.def("initialize",&Kalman::initialize)
		.def("set_Y",&Kalman::set_Y)
		.def("set_RQ",&Kalman::set_RQ)
		.def("filter_data",&Kalman::filter_data)
		.def("results",&Kalman::results);
}
