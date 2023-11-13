#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <string>
#include <stdint.h>
#include <vector>
#include <tuple>
#include <armadillo>
#include <stdlib.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <dirent.h>
#include <chrono>
#include <execution>
#include <math.h>

#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/call.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/thread.hpp>
#include <boost/container/vector.hpp>
#include <cstdio>
#include <ctime>

namespace p = boost::python;
namespace np = boost::python::numpy;

#define PI 3.141592653589793

using namespace std;

class Kalman {

	public:

		int N_state;
		int N_deriv;
		int N_samples;
		int N_meas;

		double dt;
		double dt1;
		double dt2;

		double q_norm;
		double q0;
		double q1;
		double q2;
		double q3;
		double wx;
		double wy;
		double wz;
		double w2;
		double wx_dot;
		double wy_dot;
		double wz_dot;
		double tx;
		double ty;
		double tz;
		double xi;

		bool xi_on;

		arma::vec q_normalized;
		arma::vec state;
		arma::Mat<double> PHI;
		arma::Mat<double> I_mat4;
		arma::Mat<double> I_mat3;
		arma::Mat<double> Q_mat;
		arma::Mat<double> W_mat;
		arma::Mat<double> WQ_mat;
		arma::Mat<double> W_2_mat;
		arma::Mat<double> W_dot_mat;

		arma::Mat<double> I_state;
		arma::Mat<double> P;
		arma::Mat<double> K;
		arma::Mat<double> A;
		arma::Mat<double> H;
		arma::Mat<double> Q;
		arma::Mat<double> R;

		arma::Mat<double> Y;
		arma::Mat<double> x_min;
		arma::Cube<double> P_min;
		arma::Mat<double> x_plus;
		arma::Cube<double> P_plus;
		arma::Mat<double> x_filt;
		arma::Cube<double> P_filt;

		Kalman();

		arma::vec Update(arma::vec &state_in);
		void PHI_Matrix(arma::vec &state_in);
		arma::vec quat_norm(arma::vec &state_in);
		void initialize(int N_deriv_in, double dt_in, bool xi_on_in);
		void set_Y(np::ndarray Y_array, int N_samples_in);
		void set_RQ(double quat_var, double trans_var, double xi_var);
		void forward_step(int ind);
		void backward_step(int ind);
		void filter_data();
		np::ndarray results();
};
#endif