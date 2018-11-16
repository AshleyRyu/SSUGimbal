#include "KalmanBalance.h"


void InitStates(float *state){
    int i;
	for(  i = 0; i <33; i++) state[i] = 0.;
	float *xu = state;      // state update (3x1 vector)
	float *xp = xu +3;  // state prediction (3x1 vector)
	float *kg = xp +3;  // Kalman Gain (3x3 matrix)
	float *pu = kg +9;  // Covariance matrix update (3x3 matrix)
	float *pp = pu +9;  // Covariance matrix prediction (3x3 matrix)
	// Identity matrix as initial conariance
	pu[0] = 1.; 
	pu[4] = 1.; 
	pu[8] = 1.; 
	
	pp[0] = 1.; 
	pp[4] = 1.; 
	pp[8] = 1.; 	
}


void GetLKF(float *x, float *zx, float *s, float *mx) {
// variable map x[33]
	float *xu = x;      // state update (3x1 vector)
	float *xp = xu +3;  // state prediction (3x1 vector)
	float *kg = xp +3;  // Kalman Gain (3x3 matrix)
	float *pu = kg +9;  // Covariance matrix update (3x3 matrix)
	float *pp = pu +9;  // Covariance matrix prediction (3x3 matrix)
//	Parameters
//
// xu/xp[0] : state  of angle
// xu/xp[1] : state  of angular velocity
// xu/xp[2] : state  of angular acceleration
//
// zx[0] : observation of angle
// zx[1] : observation  of angular velocity
// zx[2] : observation  of angular acceleration
//
// s[0] : variance of angle
// s[1] : variance of angular velocity
// s[2] : variance of angular acceleration
//
// mx[0] : model mismatch of angle
// mx[1] : model mismatch of angular velocity
// mx[2] : model mismatch of angular acceleration
//
//3x3 to 9 array map:  u[i][j] = U[3*j+i]
//-------------------------------------------------------------
//   Update
//-------------------------------------------------------------
// Kalman Gain Common Denominator
	float KD = pp[2]*(-(pp[3]*pp[7]) + pp[6]*(pp[4] + s[1])) + pp[1]*(-(pp[5]*pp[6]) + pp[3]*(pp[8] + s[2])) +
	   (pp[0] + s[0])*(pp[5]*pp[7] - (pp[4] + s[1])*(pp[8] + s[2]));		
	// Kalman Gain
	kg[0] = (pp[2]*(-(pp[3]*pp[7]) + pp[6]*(pp[4] + s[1])) + pp[1]*(-(pp[5]*pp[6]) + pp[3]*(pp[8] + s[2])) + 
	   		  pp[0]*(pp[5]*pp[7] - (pp[4] + s[1])*(pp[8] + s[2])))/KD;
	kg[1] = (s[0]*(pp[2]*pp[7] - pp[1]*(pp[8] + s[2])))/KD;
	kg[2] = (-(s[0]*(-(pp[1]*pp[5]) + pp[2]*(pp[4] + s[1]))) )/KD;
	kg[3] = ( s[1]*(pp[5]*pp[6] - pp[3]*(pp[8] + s[2])))/KD;
	kg[4] = (pp[2]*(pp[4]*pp[6] - pp[3]*pp[7]) + pp[1]*(-(pp[5]*pp[6]) + pp[3]*(pp[8] + s[2])) + 
	        (pp[0] + s[0])*(pp[5]*pp[7] - pp[4]*(pp[8] + s[2])) )/KD;
	kg[5] = ((pp[2]*pp[3] - pp[5]*(pp[0] + s[0]))*s[1])/KD;
	kg[6] = (-((-(pp[3]*pp[7]) + pp[6]*(pp[4] + s[1]))*s[2]) )/KD;
	kg[7] = ((pp[1]*pp[6] - pp[7]*(pp[0] + s[0]))*s[2])/KD;
	kg[8] = (pp[1]*(-(pp[5]*pp[6]) + pp[3]*pp[8]) + pp[2]*(-(pp[3]*pp[7]) + pp[6]*(pp[4] + s[1])) + 
	        (pp[0] + s[0])*(pp[5]*pp[7] - pp[8]*(pp[4] + s[1])) )/KD;
	// State vector
	xu[0] = xp[0] - kg[0]*xp[0] - kg[2]*xp[2] + kg[0]*zx[0] + kg[1]*(-xp[1] + zx[1]) + kg[2]*zx[2];
	xu[1] = xp[1] - kg[4]*xp[1] - kg[5]*xp[2] + kg[3]*(-xp[0] + zx[0]) + kg[4]*zx[1] + kg[5]*zx[2];
	xu[2] = xp[2] - kg[8]*xp[2] + kg[6]*(-xp[0] + zx[0]) + kg[7]*(-xp[1] + zx[1]) + kg[8]*zx[2];
	// Covariance Matrix
	pu[0] = pp[0] - kg[0]*pp[0] - kg[1]*pp[3] - kg[2]*pp[6];
	pu[1] = pp[1] - kg[0]*pp[1] - kg[1]*pp[4] - kg[2]*pp[7];
	pu[2] = pp[2] - kg[0]*pp[2] - kg[1]*pp[5] - kg[2]*pp[8];
	pu[3] = -(kg[3]*pp[0]) + pp[3] - kg[4]*pp[3] - kg[5]*pp[6];
	pu[4] = -(kg[3]*pp[1]) + pp[4] - kg[4]*pp[4] - kg[5]*pp[7];
	pu[5] = -(kg[3]*pp[2]) + pp[5] - kg[4]*pp[5] - kg[5]*pp[8];
	pu[6] = -(kg[6]*pp[0]) - kg[7]*pp[3] + pp[6] - kg[8]*pp[6];
	pu[7] = -(kg[6]*pp[1]) - kg[7]*pp[4] + pp[7] - kg[8]*pp[7];
	pu[8] = -(kg[6]*pp[2]) - kg[7]*pp[5] + pp[8] - kg[8]*pp[8];
	//-------------------------------------------------------------
	//   Prediction
	//-------------------------------------------------------------
	// State vector
	xp[0] = xu[0] + xu[1];
	xp[1] = xu[1] + xu[2];
	xp[2] = xu[2];
	// Covariance Matrix
	pp[0] = pu[0] + pu[1] + pu[3] + pu[4] + mx[0]; 
	pp[1] = pu[1] + pu[2] + pu[4] + pu[5]; 
	pp[2] = pu[2] + pu[5]; 
	pp[3] = pu[3] + pu[4] + pu[6] + pu[7]; 
	pp[4] = pu[4] + pu[5] + pu[7] + pu[8] + mx[1]; 
	pp[5] = pu[5] + pu[8]; 
	pp[6] = pu[6] + pu[7]; 
	pp[7] = pu[7] + pu[8]; 
	pp[8] = pu[8] + mx[2];
//-------------------------------------------------------------
}
