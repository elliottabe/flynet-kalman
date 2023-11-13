#include "kalman_filter.h"

Kalman::Kalman() {
}

arma::vec Kalman::Update(arma::vec &state_in) {
	Kalman::PHI_Matrix(state_in);
	arma::vec state_out = PHI*state_in;
	q_normalized = state_out.rows(0,3);
	state_out.rows(0,3) = Kalman::quat_norm(q_normalized);
	/*
	arma::vec state_out = state_in;
	if (N_deriv == 1) {
		q0 = state_in(0);
		q1 = state_in(1);
		q2 = state_in(2);
		q3 = state_in(3);
		wx = state_in(4);
		wy = state_in(5);
		wz = state_in(6);
		w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
		dt1 = dt/2.0;
		dt2 = pow(dt,2.0)/4.0;
		Q_mat = {
			{-q1,-q2,-q3},
			{q0,-q3,q2},
			{q3,q0,-q1},
			{-q2,q1,q0}};
		W_mat = {
			{0.0,-wx,-wy,-wz},
			{wx,0.0,wz,-wy},
			{wy,-wz,0.0,wx},
			{wz,wy,-wx,0.0}};
		W_2_mat = {{-0.5*dt2*w2,0.0,0.0,0.0},
			{0.0,-0.5*dt2*w2,0.0,0.0},
			{0.0,0.0,-0.5*dt2*w2,0.0},
			{0.0,0.0,0.0,-0.5*dt2*w2}};
		if (xi_on) {
			q_normalized =(I_mat4+dt1*W_mat)*state_out.rows(0,3);
			state_out.rows(0,3) = Kalman::quat_norm(q_normalized);
			state_out.rows(4,7) = I_mat4*state_out.rows(4,7)+dt*I_mat4*state_out.rows(11,14);
		}
		else {
			q_normalized =(I_mat4+dt1*W_mat)*state_out.rows(0,3);
			state_out.rows(0,3) = Kalman::quat_norm(q_normalized);
			state_out.rows(4,6) = I_mat3*state_out.rows(4,6)+dt*I_mat3*state_out.rows(10,12);
		}
	}
	else if (N_deriv==2) {
		q0 = state_in(0);
		q1 = state_in(1);
		q2 = state_in(2);
		q3 = state_in(3);
		wx = state_in(4);
		wy = state_in(5);
		wz = state_in(6);
		w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
		wx_dot = state_in(7);
		wy_dot = state_in(8);
		wz_dot = state_in(9);
		dt1 = dt/2.0;
		dt2 = pow(dt,2.0)/4.0;
		Q_mat = {
			{-q1,-q2,-q3},
			{q0,-q3,q2},
			{q3,q0,-q1},
			{-q2,q1,q0}};
		W_mat = {
			{0.0,-wx,-wy,-wz},
			{wx,0.0,wz,-wy},
			{wy,-wz,0.0,wx},
			{wz,wy,-wx,0.0}};
		W_dot_mat = {
			{0.0,-wx_dot,-wy_dot,-wz_dot},
			{wx_dot,0.0,wz_dot,-wy_dot},
			{wy_dot,-wz_dot,0.0,wx_dot},
			{wz_dot,wy_dot,-wx_dot,0.0}};
		W_2_mat = {{-0.5*dt2*w2,0.0,0.0,0.0},
			{0.0,-0.5*dt2*w2,0.0,0.0},
			{0.0,0.0,-0.5*dt2*w2,0.0},
			{0.0,0.0,0.0,-0.5*dt2*w2}};
		if (xi_on) {
			//q_normalized =(I_mat4+dt1*W_mat+0.5*dt2*W_2_mat+dt2*W_dot_mat)*state_out.rows(0,3);
			q_normalized =(I_mat4+dt1*W_mat+0.5*dt2*W_2_mat+dt2*W_dot_mat)*state_out.rows(0,3);
			state_out.rows(0,3) = Kalman::quat_norm(q_normalized);
			state_out.rows(4,7) = I_mat4*state_out.rows(4,7)+dt*I_mat4*state_out.rows(11,14)+2.0*dt2*I_mat4*state_out.rows(18,21);
		}
		else {
			q_normalized = (I_mat4+dt1*W_mat+0.5*dt2*W_2_mat+dt2*W_dot_mat)*state_out.rows(0,3);
			state_out.rows(0,3) = Kalman::quat_norm(q_normalized);
			state_out.rows(4,6) = I_mat3*state_out.rows(4,6)+dt*I_mat3*state_out.rows(10,12)+2.0*dt2*I_mat3*state_out.rows(16,18);
		}
	}
	*/
	return state_out;
}

/*
void Kalman::PHI_Matrix(arma::vec &state_in) {
	if (N_deriv == 1) {
		if (xi_on) {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
			dt1 = dt/2.0;
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,-q3,q2},
				{q3,q0,-q1},
				{-q2,q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,wz,-wy},
				{wy,-wz,0.0,wx},
				{wz,wy,-wx,0.0}};
			W_dot_mat = {
				{0.0,-wx_dot,-wy_dot,-wz_dot},
				{wx_dot,0.0,wz_dot,-wy_dot},
				{wy_dot,-wz_dot,0.0,wx_dot},
				{wz_dot,wy_dot,-wx_dot,0.0}};
			W_2_mat = {{-0.5*dt2*w2,0.0,0.0,0.0},
				{0.0,-0.5*dt2*w2,0.0,0.0},
				{0.0,0.0,-0.5*dt2*w2,0.0},
				{0.0,0.0,0.0,-0.5*dt2*w2}};
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,-q3,q2},
				{q3,q0,-q1},
				{-q2,q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,wz,-wy},
				{wy,-wz,0.0,wx},
				{wz,wy,-wx,0.0}};
			PHI.resize(N_state,N_state);
			PHI.eye();
			PHI.submat(0,0,3,3) += dt1*W_mat; // q0,q1,q2,q3
			PHI.submat(0,8,3,10) += dt1*Q_mat; // q0,q1,q2,q3
			PHI.submat(4,11,7,14) += dt*I_mat4; // tx,ty,tz,xi

			//PHI.submat(8,15,10,17) += dt*I_mat3;
			//PHI.submat(11,18,14,21) += dt*I_mat4;
		}
		else {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			dt1 = dt/2.0;
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,-q3,q2},
				{q3,q0,-q1},
				{-q2,q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,wz,-wy},
				{wy,-wz,0.0,wx},
				{wz,wy,-wx,0.0}};
			PHI.resize(N_state,N_state);
			PHI.eye();
			PHI.submat(0,0,3,3) += dt1*W_mat; // q0,q1,q2,q3
			PHI.submat(0,7,3,9) += dt1*Q_mat; // q0,q1,q2,q3
			PHI.submat(4,10,6,12) += dt*I_mat3; // tx,ty,tz

			//PHI.submat(7,13,9,15) += dt*I_mat3;
			//PHI.submat(10,16,12,18) += dt*I_mat3;
		}
	}
	else if (N_deriv == 2) {
		if (xi_on) {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
			wx_dot = state_in(7);
			wy_dot = state_in(8);
			wz_dot = state_in(9);
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,-q3,q2},
				{q3,q0,-q1},
				{-q2,q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,wz,-wy},
				{wy,-wz,0.0,wx},
				{wz,wy,-wx,0.0}};
			WQ_mat = {
				{-wx*q0,-wy*q0,-wz*q0},
				{-wx*q1,-wy*q1,-wz*q1},
				{-wx*q2,-wy*q2,-wz*q2},
				{-wx*q3,-wy*q3,-wz*q3}};
			W_dot_mat = {
				{0.0,-wx_dot,-wy_dot,-wz_dot},
				{wx_dot,0.0,wz_dot,-wy_dot},
				{wy_dot,-wz_dot,0.0,wx_dot},
				{wz_dot,wy_dot,-wx_dot,0.0}};
			W_2_mat = {{-w2,0.0,0.0,0.0},
				{0.0,-w2,0.0,0.0},
				{0.0,0.0,-w2,0.0},
				{0.0,0.0,0.0,-w2}};
			dt1 = dt/2.0;
			dt2 = pow(dt,2.0)/4.0;
			PHI.resize(N_state,N_state);
			PHI.eye();
			// quaternion:
			PHI.submat(0,0,3,3) += dt1*W_mat;
			PHI.submat(0,0,3,3) += 0.5*dt2*W_2_mat;
			PHI.submat(0,0,3,3) += dt2*W_dot_mat;
			PHI.submat(0,8,3,10) += dt1*Q_mat;
			PHI.submat(0,8,3,10) += dt2*WQ_mat;
			PHI.submat(0,15,3,17) += dt2*Q_mat;
			// tx,ty,tz,xi:
			PHI.submat(4,11,7,14) += dt*I_mat4;
			PHI.submat(4,18,7,21) += 2.0*dt2*I_mat4;
			// wx,wy,wz:
			PHI.submat(8,15,10,17) += dt*I_mat3;
			// vx,vy,vz,xi_dot:
			PHI.submat(11,18,14,21) += dt*I_mat4;

			//PHI.submat(15,22,17,24) += dt*I_mat3;
			//PHI.submat(18,25,21,28) += dt*I_mat4;
		}
		else {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
			wx_dot = state_in(7);
			wy_dot = state_in(8);
			wz_dot = state_in(9);
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,-q3,q2},
				{q3,q0,-q1},
				{-q2,q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,wz,-wy},
				{wy,-wz,0.0,wx},
				{wz,wy,-wx,0.0}};
			WQ_mat = {
				{-wx*q0,-wy*q0,-wz*q0},
				{-wx*q1,-wy*q1,-wz*q1},
				{-wx*q2,-wy*q2,-wz*q2},
				{-wx*q3,-wy*q3,-wz*q3}};
			W_dot_mat = {
				{0.0,-wx_dot,-wy_dot,-wz_dot},
				{wx_dot,0.0,wz_dot,-wy_dot},
				{wy_dot,-wz_dot,0.0,wx_dot},
				{wz_dot,wy_dot,-wx_dot,0.0}};
			W_2_mat = {{-w2,0.0,0.0,0.0},
				{0.0,-w2,0.0,0.0},
				{0.0,0.0,-w2,0.0},
				{0.0,0.0,0.0,-w2}};
			dt1 = dt/2.0;
			dt2 = pow(dt,2.0)/4.0;
			PHI.resize(N_state,N_state);
			PHI.eye();
			// quaternion:
			PHI.submat(0,0,3,3) += dt1*W_mat;
			PHI.submat(0,0,3,3) += 0.5*dt2*W_2_mat;
			PHI.submat(0,0,3,3) += dt2*W_dot_mat;
			PHI.submat(0,7,3,9) += dt1*Q_mat;
			PHI.submat(0,7,3,9) += dt2*WQ_mat;
			PHI.submat(0,13,3,15) += dt2*Q_mat;
			// tx,ty,tz:
			PHI.submat(4,10,6,12) += dt*I_mat3;
			PHI.submat(4,16,6,18) += 2.0*dt2*I_mat3;
			// wx,wy,wz:
			PHI.submat(7,13,9,15) += dt*I_mat3;
			// vx,vy,vz:
			PHI.submat(10,16,12,18) += dt*I_mat3;

			//PHI.submat(13,19,15,21) += dt*I_mat3;
			//PHI.submat(16,22,18,24) += dt*I_mat3;
		}
	}
}
*/

void Kalman::PHI_Matrix(arma::vec &state_in) {
	if (N_deriv == 1) {
		if (xi_on) {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
			dt1 = dt/2.0;
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,q3,-q2},
				{-q3,q0,q1},
				{q2,-q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,-wz,wy},
				{wy,wz,0.0,-wx},
				{wz,-wy,wx,0.0}};
			PHI.resize(N_state,N_state);
			PHI.eye();
			PHI.submat(0,0,3,3) += dt1*W_mat; // q0,q1,q2,q3
			PHI.submat(0,8,3,10) += dt1*Q_mat; // q0,q1,q2,q3
			PHI.submat(4,11,7,14) += dt*I_mat4; // tx,ty,tz,xi
		}
		else {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			dt1 = dt/2.0;
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,q3,-q2},
				{-q3,q0,q1},
				{q2,-q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,-wz,wy},
				{wy,wz,0.0,-wx},
				{wz,-wy,wx,0.0}};
			PHI.resize(N_state,N_state);
			PHI.eye();
			PHI.submat(0,0,3,3) += dt1*W_mat; // q0,q1,q2,q3
			PHI.submat(0,7,3,9) += dt1*Q_mat; // q0,q1,q2,q3
			PHI.submat(4,10,6,12) += dt*I_mat3; // tx,ty,tz
		}
	}
	else if (N_deriv == 2) {
		if (xi_on) {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
			wx_dot = state_in(7);
			wy_dot = state_in(8);
			wz_dot = state_in(9);
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,q3,-q2},
				{-q3,q0,q1},
				{q2,-q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,-wz,wy},
				{wy,wz,0.0,-wx},
				{wz,-wy,wx,0.0}};
			dt1 = dt/2.0;
			dt2 = pow(dt,2.0)/4.0;
			PHI.resize(N_state,N_state);
			PHI.eye();
			// quaternion:
			PHI.submat(0,0,3,3) += dt1*W_mat;
			PHI.submat(0,8,3,10) += dt1*Q_mat;
			PHI.submat(0,15,3,17) += dt2*Q_mat;
			// tx,ty,tz,xi:
			PHI.submat(4,11,7,14) += dt*I_mat4;
			PHI.submat(4,18,7,21) += 2.0*dt2*I_mat4;
			// wx,wy,wz:
			PHI.submat(8,15,10,17) += dt*I_mat3;
			// vx,vy,vz,xi_dot:
			PHI.submat(11,18,14,21) += dt*I_mat4;
		}
		else {
			q0 = state_in(0);
			q1 = state_in(1);
			q2 = state_in(2);
			q3 = state_in(3);
			wx = state_in(4);
			wy = state_in(5);
			wz = state_in(6);
			w2 = pow(wx,2.0)+pow(wy,2.0)+pow(wz,2.0);
			wx_dot = state_in(7);
			wy_dot = state_in(8);
			wz_dot = state_in(9);
			Q_mat = {
				{-q1,-q2,-q3},
				{q0,q3,-q2},
				{-q3,q0,q1},
				{q2,-q1,q0}};
			W_mat = {
				{0.0,-wx,-wy,-wz},
				{wx,0.0,-wz,wy},
				{wy,wz,0.0,-wx},
				{wz,-wy,wx,0.0}};
			dt1 = dt/2.0;
			dt2 = pow(dt,2.0)/4.0;
			PHI.resize(N_state,N_state);
			PHI.eye();
			// quaternion:
			PHI.submat(0,0,3,3) += dt1*W_mat;
			PHI.submat(0,7,3,9) += dt1*Q_mat;
			PHI.submat(0,13,3,15) += dt2*Q_mat;
			// tx,ty,tz:
			PHI.submat(4,10,6,12) += dt*I_mat3;
			PHI.submat(4,16,6,18) += 2.0*dt2*I_mat3;
			// wx,wy,wz:
			PHI.submat(7,13,9,15) += dt*I_mat3;
			// vx,vy,vz:
			PHI.submat(10,16,12,18) += dt*I_mat3;
		}
	}
}


arma::vec Kalman::quat_norm(arma::vec &state_in) {
	arma::vec state_out = state_in;
	q_norm = arma::norm(state_in.rows(0,3));
	if (q_norm>0.01) {
		state_out(0) = state_in(0)/q_norm;
		state_out(1) = state_in(1)/q_norm;
		state_out(2) = state_in(2)/q_norm;
		state_out(3) = state_in(3)/q_norm;
	}
	return state_out;
}

void Kalman::initialize(int N_deriv_in, double dt_in, bool xi_on_in) {
	dt = dt_in;
	xi_on = xi_on_in;
	N_deriv = N_deriv_in;
	Q_mat.resize(4,3);
	W_mat.resize(4,4);
	WQ_mat.resize(4,3);
	W_dot_mat.resize(4,4);
	W_2_mat.resize(4,4);
	I_mat4.resize(4,4);
	I_mat4.eye();
	I_mat3.resize(3,3);
	I_mat4.eye();
	if (N_deriv == 1) {
		if (xi_on) {
			N_state = 15;
			//N_state = 22;
		}
		else {
			N_state = 13;
			//N_state = 19;
		}
	}
	else if (N_deriv == 2) {
		if (xi_on) {
			N_state = 22;
			//N_state = 29;
		}
		else {
			N_state = 19;
			//N_state = 25;
		}
	}
	state.resize(N_state);
	P.resize(N_state,N_state);
	P.eye();
	PHI.resize(N_state,N_state);
	I_state.resize(N_state,N_state);
	A.resize(N_state,N_state);
}

void Kalman::set_Y(np::ndarray Y_array, int N_samples_in) {
	N_samples = N_samples_in;
	if (xi_on) {
		N_meas = 8;
		Y.resize(N_meas,N_samples);
		Y.zeros();
		H.resize(N_meas,N_state);
		H.zeros();
		H.submat(0,0,7,7).eye();
		for (int i=0; i<N_samples; i++) {
			Y(0,i) = p::extract<double>(Y_array[0][i]);
			Y(1,i) = p::extract<double>(Y_array[1][i]);
			Y(2,i) = p::extract<double>(Y_array[2][i]);
			Y(3,i) = p::extract<double>(Y_array[3][i]);
			Y(4,i) = p::extract<double>(Y_array[4][i]);
			Y(5,i) = p::extract<double>(Y_array[5][i]);
			Y(6,i) = p::extract<double>(Y_array[6][i]);
			Y(7,i) = p::extract<double>(Y_array[7][i]);
		}
	}
	else {
		N_meas = 7;
		Y.resize(N_meas,N_samples);
		Y.zeros();
		H.resize(N_meas,N_state);
		H.zeros();
		H.submat(0,0,6,6).eye();
		for (int i=0; i<N_samples; i++) {
			Y(0,i) = p::extract<double>(Y_array[0][i]);
			Y(1,i) = p::extract<double>(Y_array[1][i]);
			Y(2,i) = p::extract<double>(Y_array[2][i]);
			Y(3,i) = p::extract<double>(Y_array[3][i]);
			Y(4,i) = p::extract<double>(Y_array[4][i]);
			Y(5,i) = p::extract<double>(Y_array[5][i]);
			Y(6,i) = p::extract<double>(Y_array[6][i]);
		}
	}
	K.resize(N_state,N_meas);
	x_min.resize(N_state,N_samples);
	x_min.zeros();
	P_min.resize(N_state,N_state,N_samples);
	P_min.zeros();
	x_plus.resize(N_state,N_samples);
	x_plus.zeros();
	P_plus.resize(N_state,N_state,N_samples);
	P_plus.zeros();
	x_filt.resize(N_state,N_samples);
	x_filt.zeros();
	P_filt.resize(N_state,N_state,N_samples);
	P_filt.zeros();
}

void Kalman::set_RQ(double quat_var, double trans_var, double xi_var) {
	if (N_deriv == 1) {
		if (xi_on) {
			Q.resize(N_state,N_state);
			Q.eye();
			R.resize(N_meas,N_meas);
			R.eye();
			Q(0,0) = quat_var;
			Q(1,1) = quat_var;
			Q(2,2) = quat_var;
			Q(3,3) = quat_var;
			Q(4,4) = trans_var;
			Q(5,5) = trans_var;
			Q(6,6) = trans_var;
			Q(7,7) = xi_var;
			Q(8,8) = quat_var*pow(dt,2.0);
			Q(9,9) = quat_var*pow(dt,2.0);
			Q(10,10) = quat_var*pow(dt,2.0);
			Q(11,11) = trans_var*pow(dt,2.0);
			Q(12,12) = trans_var*pow(dt,2.0);
			Q(13,13) = trans_var*pow(dt,2.0);
			Q(14,14) = xi_var*pow(dt,2.0);
		}
		else {
			Q.resize(N_state,N_state);
			Q.eye();
			R.resize(N_meas,N_meas);
			R.eye();
			Q(0,0) = quat_var;
			Q(1,1) = quat_var;
			Q(2,2) = quat_var;
			Q(3,3) = quat_var;
			Q(4,4) = trans_var;
			Q(5,5) = trans_var;
			Q(6,6) = trans_var;
			Q(7,7) = quat_var;
			Q(8,8) = quat_var;
			Q(9,9) = quat_var;
			Q(10,10) = trans_var;
			Q(11,11) = trans_var;
			Q(12,12) = trans_var;
		}
	}
	else if (N_deriv == 2) {
		if (xi_on) {
			Q.resize(N_state,N_state);
			Q.eye();
			R.resize(N_meas,N_meas);
			R.eye();
			Q(0,0) = quat_var;
			Q(1,1) = quat_var;
			Q(2,2) = quat_var;
			Q(3,3) = quat_var;
			Q(4,4) = trans_var;
			Q(5,5) = trans_var;
			Q(6,6) = trans_var;
			Q(7,7) = xi_var;
			Q(8,8) = quat_var;
			Q(9,9) = quat_var;
			Q(10,10) = quat_var;
			Q(11,11) = trans_var;
			Q(12,12) = trans_var;
			Q(13,13) = trans_var;
			Q(14,14) = xi_var;
			Q(15,15) = quat_var;
			Q(16,16) = quat_var;
			Q(17,17) = quat_var;
			Q(18,18) = trans_var;
			Q(19,19) = trans_var;
			Q(20,20) = trans_var;
			Q(21,21) = xi_var;
		}
		else {
			Q.resize(N_state,N_state);
			Q.eye();
			R.resize(N_meas,N_meas);
			R.eye();
			Q(0,0) = quat_var;
			Q(1,1) = quat_var;
			Q(2,2) = quat_var;
			Q(3,3) = quat_var;
			Q(4,4) = trans_var;
			Q(5,5) = trans_var;
			Q(6,6) = trans_var;
			Q(7,7) = quat_var;
			Q(8,8) = quat_var;
			Q(9,9) = quat_var;
			Q(10,10) = trans_var;
			Q(11,11) = trans_var;
			Q(12,12) = trans_var;
			Q(13,13) = quat_var;
			Q(14,14) = quat_var;
			Q(15,15) = quat_var;
			Q(16,16) = trans_var;
			Q(17,17) = trans_var;
			Q(18,18) = trans_var;
		}
	}
}

void Kalman::forward_step(int ind) {
	// Prediction:
	Kalman::PHI_Matrix(state);
	state = Kalman::Update(state);
	P = PHI*P*PHI.t()+Q;
	x_min.col(ind) = state;
	P_min.slice(ind) = P;
	// Correction:
	//K = arma::trans(arma::solve(arma::trans(H*P*H.t()+R),arma::trans(P*H.t())));
	K = P*H.t()*arma::inv(H*P*H.t()+R);
	state = state+K*(Y.col(ind)-H*state);
	q_normalized = state.rows(0,3);
	state.rows(0,3) = Kalman::quat_norm(q_normalized);
	P = (I_state-K*H)*P;
	x_plus.col(ind) = state;
	P_plus.slice(ind) = P;
}

void Kalman::backward_step(int ind) {
	if (ind==(N_samples-1)) {
		x_filt.col(ind) = x_min.col(ind);
		P_filt.slice(ind) = P_plus.slice(ind);
	}
	else {
		state = x_plus.col(ind);
		Kalman::PHI_Matrix(state);
		A = P_plus.slice(ind)*PHI.t()*arma::inv(P_min.slice(ind+1));
		state = x_plus.col(ind)+A*(x_filt.col(ind+1)-x_min.col(ind+1));
		//state = x_plus.col(ind)+P_plus.slice(ind)*PHI.t()*arma::solve(P_min.slice(ind+1),x_filt.col(ind+1)-x_min.col(ind+1));
		q_normalized = state.rows(0,3);
		state.rows(0,3) = Kalman::quat_norm(q_normalized);
		x_filt.col(ind) = state;
	}
}

void Kalman::filter_data() {
	state.resize(N_state);
	if (xi_on) {
		state.rows(0,7) = Y.col(0);
	}
	else {
		state.rows(0,6) = Y.col(0);
	}
	P.resize(N_state,N_state);
	P.eye();
	// forward sweep:
	for (int i=0; i<N_samples; i++) {
		Kalman::forward_step(i);
	}
	// backward sweep:
	for (int j=N_samples-1; j>=0; j--) {
		Kalman::backward_step(j);
	}
}

np::ndarray Kalman::results() {
	// Return results:
	p::tuple shape = p::make_tuple(N_state,N_samples);
	np::dtype dtype = np::dtype::get_builtin<double>();
	np::ndarray results_out = np::zeros(shape,dtype);

	for (int i=0; i<N_samples; i++) {
		for (int j=0; j<N_state; j++) {
			results_out[j][i] = x_filt(j,i);
		}
	}

	return results_out;
}