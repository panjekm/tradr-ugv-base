#include "definitions.h"

using namespace cv;

double rad2deg(double angleInRadians);//! radians to degrees conversion
Vec3fCol rad2deg(Vec3fCol angleInRadians);//! radians to degrees conversion
double deg2rad(double angleInDegrees);//! degrees to radians conversion
Vec3fCol deg2rad(Vec3fCol angleInDegrees);//! degrees to radians conversion
Vec3fCol alignment_coarse( double mACCx, double mACCy, double mACCz, double mGYRx, double mGYRy, double mGYRz, double lat, double g, double er);//! Finding initial euler angles from idle-state acceleration and angular rate values
Vec3fCol dcm2angle(Mat33f dcm);//! Getting angles from DCM
Vec3fCol threeaxisrot(double r11, double r12, double r21, double r31, double r32, double r11a, double r12a);//! Finding angles for rotations about X, Y, and Z axes
Vec3fCol comp_gravity(Vec3fCol LLA);//! Computation of gravitational acceleration in Earth's frame of reference based on WGS84 Gravity model
quat rotv2quat(Vec3fCol rotv);//! Conversion of a rotation vector to quaternion
quat rotv2negquat(Vec3fCol rotv);//! Conversion of a rotation vector to quaternion
quat euler2quat(Vec3fCol angles);//! Conversion of Euler angles to quaternion.
Mat33f quat2dcm(quat quat);//! Quaternion to DCM matrix conversion.
Vec3fCol quat2euler(quat q);//! Conversion of quaternion to Euler angles.
quat quatnormalize(quat quat);//! Normalizes the quaternion to suppress numerical inaccuracies
quat quatmultiply(quat q, quat r);//! Quaternion multiplication
double myMean(double *data, int length);//! Computation of mean value of the input data
Vec3fCol myCross(Vec3fCol vec1, Vec3fCol vec2);//! Computation of 2-vector cross-product
Mat33f dcmecef2ned(double lat_deg, double lon_deg);//! Computation of 2-vector cross-product
quat quatned2ecef(Vec3fCol LLA);//! Computation of quaternion (NED -> ECEF)
//inso::mechanization_output outputMessage(Vec3fCol eul, quat quaternion, Mat33f dcm);//! Constructs native inso output message (obsolete, just for debugging) 
//inso::inso_output inso_outputMessage(Vec3fCol eul, Vec9fCol MM);//! Constructs native inso output message (obsolete, just for debugging) 
Vec3fCol dcm2latlon(Mat33f dcm);//! DCM to latitude/longitude 
Mat33f comp_skew(Vec3fCol vector);//! Compute skew form for vector multiplication
Vec3fCol quat2rotv(quat q);//! Quaternion to rotation vector
quat quatinv(quat q);//! Quaternion inversion
quat quatconj(quat qin);//! Quaternion conjugate 
double lalodist(double la1,double lo1,double la2,double lo2);//! Geodetic distance
double calculateR_N(double LAT);//! Some rotation or whatever
double calculateR_M(double LAT);//! Some other rotation
double myNorm(Vec3fCol input);//! Norm of a vector
double myNorm(quat input);//! Norm of a quaternion
void evaluate_harmonics(double* harmonics, double phi, double th, double psi);//! Evals sin and cos of the three given angles
void fill_fun(Vec9fCol & fun,double sf_x,double sf_y,double sf_z,double ar_x,double ar_y,double ar_z,double vBx,double vBy,double vBz,double * harmonics,Mat33f C_b2n, Vec3fCol gN);//! Nonlinear system equations
void fill_dfun(Mat99f & dfundx, Mat96f & dfundu ,double ar_x,double ar_y,double ar_z,double vBx,double vBy,double vBz,double * harmonics, Vec3fCol gN);//! Linearized system equations
void ekf_time_update(Vec9fCol & MM, Mat99f & PP , Vec9fCol funk , Mat99f Fk , Mat66f Qk);//! Evaluates matrices for time propagation
void ekf_measurement_update(Vec9fCol & MM, Mat99f & PP , Vec6fCol ZZ, Vec6fCol hk, Mat69f Hk, Mat66f Rk);//! Measurement update
