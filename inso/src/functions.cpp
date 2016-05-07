#include "functions.h"

using namespace cv;

//! radians to degrees conversion
double rad2deg(double angleInRadians)
{
	return (180/PI)*angleInRadians;
}

//! radians to degrees conversion
Vec3fCol rad2deg(Vec3fCol angleInRadians)
{
	return (180/PI)*angleInRadians;
}

//! degrees to radians conversion
double deg2rad(double angleInDegrees)
{
	return (PI/180)*angleInDegrees;
}

//! degrees to radians conversion
Vec3fCol deg2rad(Vec3fCol angleInDegrees)
{
	return (PI/180)*angleInDegrees;
}

//! Finding initial euler angles from idle-state acceleration and angular rate values
Vec3fCol alignment_coarse( double mACCx, double mACCy, double mACCz, double mGYRx, double mGYRy, double mGYRz, double lat, double g, double er)
{
	Vec3fCol euler;

	// Approximation of combined acc & gyr measurement vector in NED
	Mat33f align_N = Mat33f(-tan(deg2rad(lat))/g, 1/(er*cos(deg2rad(lat))), 0, 0, 0, -1/(g*er*cos(deg2rad(lat))), -1/g, 0, 0);

	Vec3fCol crProd;
	Vec3fCol mACC = Vec3fCol(mACCx, mACCy, mACCz);
	Vec3fCol mGYR = Vec3fCol(mGYRx, mGYRy, mGYRz);

	crProd = myCross(mACC, mGYR);

	Mat33f align_B = Mat33f(mACCx, mACCy, mACCz, mGYRx, mGYRy, mGYRz, crProd(0), crProd(1), crProd(2));

	/* 
	 * Estimated C_b2n = align_N*align_B does not usually satisfy the
	 * orthogonality and normality conditions, hence, euler angles are
	 * extracted:
	 */
		Mat33f mult = align_N*align_B;
		Vec3fCol ang = dcm2angle(mult.t());
	
	euler(0) = rad2deg(ang(2));
	euler(1) = rad2deg(ang(1));
	euler(2) = rad2deg(ang(0));
	
	return euler;
}

//! Getting angles from DCM
Vec3fCol dcm2angle(Mat33f dcm)
{
	/* DCM FORM:
	 *		[      cy*cz          cy*sz      -sy ]
	 *		[ sy*sx*cz-sz*cx sy*sx*sz+cz*cx cy*sx]
	 *		[ sy*cx*cz+sz*sx sy*cx*sz-cz*sx cy*cx]
	 */
	Vec3fCol r;
	r = threeaxisrot( dcm(0,1), dcm(0,0), -dcm(0,2), dcm(1,2), dcm(2,2), -dcm(1,0), dcm(1,1));
	
	return r;
}

//! Finding angles for rotations about X, Y, and Z axes
Vec3fCol threeaxisrot(double r11, double r12, double r21, double r31, double r32, double r11a, double r12a) /* --- OK ---*/
{

	// VK: SINGULARITY AVOIDANCE
	if(r21 > 1) r21 = 1;	
	if(r21 < -1) r21 = -1;	
	
	
	Vec3fCol r = Vec3fCol(atan2(r11, r12),asin(r21),atan2(r31, r32));
	
	return r;
}

//! Computation of gravitational acceleration in Earth's frame of reference based on WGS84 Gravity model
Vec3fCol comp_gravity(Vec3fCol LLA)
{
	double lat = deg2rad(LLA(0));
	double alt = LLA(2);
	
	// WGS84 Gravity model constants:
	const double a1 = 9.7803267715;
	const double a2 = 0.0052790414;
	const double a3 = 0.0000232718;
	const double a4 = -0.0000030876910891;
	const double a5 = 0.0000000043977311;
	const double a6 = 0.0000000000007211;

	// Gravity computation
	double gn = a1*(1+a2*pow(sin(lat),2)+a3*pow(sin(lat),4))+(a4+a5*pow(sin(lat),2))*alt+a6*alt;
	Vec3fCol gravity = Vec3fCol(0, 0, gn);

	return gravity;
}

//! Conversion of a rotation vector to quaternion
quat rotv2quat(Vec3fCol rotv)
{
	quat q;
	
	// Euclid norm of the 0.5 of the rotation vector:
	rotv = 0.5*rotv;
	double euclid = sqrt(pow(rotv(0),2) + pow(rotv(1),2) + pow(rotv(2),2));

	// No approximation by Taylor series
	q(0) = cos(euclid);
	Vec3fCol temp = (sin(euclid)/euclid)*rotv;
	q(1) = temp(0);
	q(2) = temp(1);
	q(3) = temp(2);

	return q;
}

//! Conversion of a rotation vector to quaternion
quat rotv2negquat(Vec3fCol rotv)
{
	quat q;
	
	// Euclid norm of the 0.5 of the rotation vector:
	rotv = 0.5*rotv;
	double euclid = sqrt(pow(rotv(0),2) + pow(rotv(1),2) + pow(rotv(2),2));

	// No approximation by Taylor series
	q(0) = cos(euclid);
	Vec3fCol temp = (sin(euclid)/euclid)*rotv;
	q(1) = -temp(0);
	q(2) = -temp(1);
	q(3) = -temp(2);

	return q;
}

//! Conversion of Euler angles to quaternion.
quat euler2quat(Vec3fCol angles)
{
	Vec3fCol cang = Vec3fCol(cos(angles(0)/2),cos(angles(1)/2),cos(angles(2)/2));
	Vec3fCol sang = Vec3fCol(sin(angles(0)/2),sin(angles(1)/2),sin(angles(2)/2));
	quat q = quat(cang(0)*cang(1)*cang(2) + sang(0)*sang(1)*sang(2),
					sang(0)*cang(1)*cang(2) - cang(0)*sang(1)*sang(2),
					cang(0)*sang(1)*cang(2) + sang(0)*cang(1)*sang(2),
					cang(0)*cang(1)*sang(2) - sang(0)*sang(1)*cang(2));
	return q;
}


//! Quaternion to DCM matrix conversion.
Mat33f quat2dcm(quat q)
{
	quat qin = quatnormalize(q);

	Mat33f dcm;

	dcm(0,0) = pow(qin(0),2) + pow(qin(1),2) - pow(qin(2),2) - pow(qin(3),2);
	dcm(0,1) = 2*(qin(1)*qin(2) + qin(0)*qin(3));
	dcm(0,2) = 2*(qin(1)*qin(3) - qin(0)*qin(2));
	dcm(1,0) = 2*(qin(1)*qin(2) - qin(0)*qin(3));
	dcm(1,1) = pow(qin(0),2) - pow(qin(1),2) + pow(qin(2),2) - pow(qin(3),2);
	dcm(1,2) = 2*(qin(2)*qin(3) + qin(0)*qin(1));
	dcm(2,0) = 2*(qin(1)*qin(3) + qin(0)*qin(2));
	dcm(2,1) = 2*(qin(2)*qin(3) - qin(0)*qin(1));
	dcm(2,2) = pow(qin(0),2) - pow(qin(1),2) - pow(qin(2),2) + pow(qin(3),2);

	return dcm;
}

//! Conversion of quaternion to Euler angles.
Vec3fCol quat2euler(quat q)
{
	quat qin = quatnormalize(q);
	//ROS_INFO("quatnormalize: %f, %f, %f, %f", qin(0), qin(1), qin(2), qin(3));
	double roll = atan2(2*(qin(2)*qin(3)+qin(0)*qin(1)),pow(qin(0),2)-pow(qin(1),2)-pow(qin(2),2)+pow(qin(3),2));
	double pitch = asin(-2*(qin(1)*qin(3)-qin(0)*qin(2)));
	double yaw = atan2(2*(qin(1)*qin(2)+qin(0)*qin(3)),pow(qin(0),2)+pow(qin(1),2)-pow(qin(2),2)-pow(qin(3),2));
	//ROS_INFO("quat2euler: %f, %f, %f", roll, pitch, yaw);
	return Vec3fCol(roll, pitch, yaw);
}

//! Normalizes the quaternion to suppress numerical inaccuracies
quat quatnormalize(quat q)
{
	double norma = sqrt(pow(q(0),2)+pow(q(1),2)+pow(q(2),2)+pow(q(3),2));
	return (1/norma)*q;
}

//! Quaternion multiplication
quat quatmultiply(quat q, quat r)
{
	// Calculate vector portion of quaternion product
	// vec = s1*v2 + s2*v1 + cross(v1,v2)
	Vec3fCol vec = Vec3fCol(q(0)*r(1),q(0)*r(2),q(0)*r(3))+
			Vec3fCol(r(0)*q(1),r(0)*q(2),r(0)*q(3))+
			Vec3fCol(q(2)*r(3)-q(3)*r(2), q(3)*r(1)-q(1)*r(3), q(1)*r(2)-q(2)*r(1));

	// Calculate scalar portion of quaternion product
	double scalar = q(0)*r(0)-q(1)*r(1)-q(2)*r(2)-q(3)*r(3);

	return quat(scalar,vec(0),vec(1),vec(2));
}

//! Computation of mean value of the input data
double myMean(double *data, int length)
{
	int i;
	double mean=0;
	
	// Sum all the values
	for (i=0;i<length;i++) mean += data[i];

	// Divide them by their count
	mean = mean/length;
	
	return mean;
}

//! Computation of 2-vector cross-product
Vec3fCol myCross(Vec3fCol vec1, Vec3fCol vec2)
{
	Vec3fCol cross;
	cross(0)=vec1(1)*vec2(2)-vec1(2)*vec2(1);
	cross(1)=vec1(2)*vec2(0)-vec1(0)*vec2(2);
	cross(2)=vec1(0)*vec2(1)-vec1(1)*vec2(0);

	return cross;
}

//! Computation of 2-vector cross-product
Mat33f dcmecef2ned(double lat_deg, double lon_deg)
{
	Mat33f dcm;

	double cang[2];
	double sang[2];

	cang[0] = cos(deg2rad(lat_deg));
	cang[1] = cos(deg2rad(lon_deg));

	sang[0] = sin(deg2rad(lat_deg));
	sang[1] = sin(deg2rad(lon_deg));

	dcm(0,0) = -cang[1]*sang[0];
	dcm(0,1) = -sang[1]*sang[0];
	dcm(0,2) = cang[0];
	dcm(1,0) = -sang[1];
	dcm(1,1) = cang[1];
	dcm(1,2) = 0.0;
	dcm(2,0) = -cang[1]*cang[0];
	dcm(2,1) = -sang[1]*cang[0];
	dcm(2,2) = -sang[0];

	return dcm;
}

//! Computation of quaternion (NED -> ECEF)
quat quatned2ecef(Vec3fCol LLA)
{
	quat Q_n2e;

	Q_n2e(0) = cos(-PI/4 - deg2rad(LLA(0))/2)*cos(deg2rad(LLA(1))/2);
	Q_n2e(1) = -sin(-PI/4 - deg2rad(LLA(0))/2)*sin(deg2rad(LLA(1))/2);
	Q_n2e(2) = sin(-PI/4 - deg2rad(LLA(0))/2)*cos(deg2rad(LLA(1))/2);
	Q_n2e(3) = cos(-PI/4 - deg2rad(LLA(0))/2)*sin(deg2rad(LLA(1))/2);

	return Q_n2e;
}

//! Constructs native inso output message (obsolete, just for debugging) 
/*inso::mechanization_output outputMessage(Vec3fCol eul, quat quaternion, Mat33f dcm)
{
	inso::mechanization_output output_msg;

	output_msg.euler.x = eul(0);
	output_msg.euler.y = eul(1);
	output_msg.euler.z = eul(2);

	output_msg.quaternion.w = quaternion(0);
	output_msg.quaternion.x = quaternion(1);
	output_msg.quaternion.y = quaternion(2);
	output_msg.quaternion.z = quaternion(3);

	output_msg.dcm.row_1.col_1 = dcm(0,0); 
	output_msg.dcm.row_1.col_2 = dcm(0,1); 
	output_msg.dcm.row_1.col_3 = dcm(0,2);
	output_msg.dcm.row_2.col_1 = dcm(1,0);
	output_msg.dcm.row_2.col_2 = dcm(1,1); 
	output_msg.dcm.row_2.col_3 = dcm(1,2);
	output_msg.dcm.row_3.col_1 = dcm(2,0); 
	output_msg.dcm.row_3.col_2 = dcm(2,1); 
	output_msg.dcm.row_3.col_3 = dcm(2,2);

	return output_msg;
}
*/
//! Constructs native inso output message (obsolete, just for debugging) 
/*inso::inso_output inso_outputMessage(Vec3fCol eul, Vec9fCol MM){
      
	inso::inso_output message;
	
	message.position.x = MM(0);
	message.position.y = MM(1);
	message.position.z = MM(2);
	
	message.velocity.x = MM(3);
	message.velocity.y = MM(4);
	message.velocity.z = MM(5);
	
	message.euler.x = eul(0);
	message.euler.y = eul(1);
	message.euler.z = eul(2);
	
	return message;
}
*/
//! DCM to latitude/longitude 
Vec3fCol dcm2latlon(Mat33f dcm)
{
	Vec3fCol latlon;
	latlon(0) = rad2deg(asin(-dcm(2,2)));
	latlon(1) = rad2deg(atan2(-dcm(1,0), dcm(1,1)));

	return latlon;
}

//! Compute skew form for vector multiplication
Mat33f comp_skew(Vec3fCol vector)
{
	return Mat33f(0,-vector(2),vector(1),vector(2),0,-vector(0),-vector(1),vector(0),0);
}

//! Quaternion to rotation vector
Vec3fCol quat2rotv(quat q)
{
	Vec3fCol rotv;

	double N = sqrt(pow(q(1),2) + pow(q(2),2) + pow(q(3),2))/q(0);
	double f = 0.5*(1 - pow(N,2)/6 + pow(N,4)/120 - pow(N,6)/5040 + pow(N,8)/362880 - pow(N,10)/39916800);

	if (q(0) == 0) f = 1/PI;
	rotv = (1/f)*Vec3fCol(q(1),q(2),q(3));

	return rotv;
}

//! Quaternion inversion
quat quatinv(quat q)
{
	double suma = pow(q(0),2)+pow(q(1),2)+pow(q(2),2)+pow(q(3),2);

	return quat(q(0)/suma,-q(1)/suma,-q(2)/suma,-q(3)/suma);
}

//! Quaternion conjugate 
quat quatconj(quat qin)
{
	return quat(qin(0),-qin(1),-qin(2),-qin(3));
}

//! Geodetic distance
double lalodist(double la1,double lo1,double la2,double lo2)
{
	double re = 6369628.75; 			// Earth's radius (m)

	//double dla = deg2rad(la2 - la1); 	// distance in latitude
	//double dlo = deg2rad(lo2 - lo1); 	// distance in longitude
	//double al = sin(dla/2)*sin(dla/2) + cos(deg2rad(la1))*cos(deg2rad(la2))*sin(dlo/2)*sin(dlo/2);
	double bl = 2*atan2(sqrt(geo_a),sqrt(1-geo_a));

	return re*bl;
}

//! Some rotation or whatever
double calculateR_N(double LAT)
{
	return geo_a*(1 - pow(geo_e,2))/pow(1 - pow(geo_e,2)*pow(sin(deg2rad(LAT)),2),(double)(3/2));
}

//! Some other rotation
double calculateR_M(double LAT)
{
	return geo_a/sqrt(1 - pow(geo_e,2)*pow(sin(deg2rad(LAT)),2));    
}

//! Norm of a vector
double myNorm(Vec3fCol input)
{
	return sqrt(pow(input(0),2)+pow(input(1),2)+pow(input(2),2));
}

//! Norm of a quaternion
double myNorm(quat input)
{
	return sqrt(pow(input(0),2)+pow(input(1),2)+pow(input(2),2)+pow(input(3),2));
}

//! Evals sin and cos of the three given angles
void evaluate_harmonics(double* harmonics, double phi, double th, double psi)
{
  harmonics[SIN_PHI] = sin(phi);
  harmonics[SIN_TH] = sin(th);
  harmonics[SIN_PSI] = sin(psi);
  harmonics[COS_PHI] = cos(phi);
  harmonics[COS_TH] = cos(th);
  harmonics[COS_PSI] = cos(psi);
}

//! Nonlinear system equations
void fill_fun(Vec9fCol & fun,double sf_x,double sf_y,double sf_z,double ar_x,double ar_y,double ar_z,double vBx,double vBy,double vBz,double * harmonics,Mat33f C_b2n, Vec3fCol gN)
{
  Vec3fCol velocities = Vec3fCol(vBx,vBy,vBz);
  Vec3fCol ang_rates = Vec3fCol(ar_x,ar_y,ar_z);
  Vec3fCol spec_forces = Vec3fCol(sf_x,sf_y,sf_z);
  Mat33f rotate_ang_rates = Mat33f::eye();

  Vec3fCol first = C_b2n * velocities;
  Vec3fCol second = (comp_skew(velocities) * ang_rates) + (C_b2n.t() * gN) + spec_forces;
  //Vec3fCol second = (C_b2n.t() * gN) + spec_forces;
  
  
  //DEBUG  
  
  //std::cout << "s\t" << second(0) << std::endl;  
  //std::cout << " \t" << second(1) << std::endl;
  //std::cout << " \t" << second(2) << std::endl;
  //std::cout << "__________________________________________________________________" << std::endl;
  
  Vec3fCol third = rotate_ang_rates * ang_rates;
  
  for(int i = 0; i < 3; i++) fun(i) = first(i);
  for(int i = 0; i < 3; i++) fun(i+3) = second(i);
  for(int i = 0; i < 3; i++) fun(i+6) = third(i);
  
  // // Matx has not rowRange method implemented...
  //fun.rowRange(Range(0,2)) = C_b2n * velocities;
  //fun.rowRange(Range(3,5)) = (comp_skew(velocities) * ang_rates) + (C_b2n.t() * gN) + spec_forces;
  //fun.rowRange(Range(6,8)) = rotate_ang_rates * ang_rates;
  
  return;
}

//! Linearized system equations
void fill_dfun(Mat99f & dfundx, Mat96f & dfundu,double ar_x,double ar_y,double ar_z,double vBx,double vBy,double vBz,double * harmonics, Vec3fCol gN)
{
  // this function sets many elements a contant value (0,1,-1 etc.) it might left out, but according to the function name, it's done anyway. these operations dont take so much time
  
  // filling dfun/dx element-by-element
  
  dfundx(0,0) = 0;
  dfundx(0,1) = 0;
  dfundx(0,2) = 0;
  dfundx(0,3) = harmonics[COS_PSI] * harmonics[COS_TH];
  dfundx(0,4) = harmonics[COS_PSI] * harmonics[SIN_PHI] * harmonics[SIN_TH] - harmonics[COS_PHI] * harmonics[SIN_PSI];
  dfundx(0,5) = harmonics[SIN_PHI] * harmonics[SIN_PSI] + harmonics[COS_PHI] * harmonics[COS_PSI] * harmonics[SIN_TH];
  dfundx(0,6) = 0;
  dfundx(0,7) = 0;
  dfundx(0,8) = 0;
  
  //
  
  dfundx(1,0) = 0;
  dfundx(1,1) = 0; 
  dfundx(1,2) = 0; 
  dfundx(1,3) = harmonics[COS_TH]*harmonics[SIN_PSI];
  dfundx(1,4) = harmonics[COS_PHI]*harmonics[COS_PSI] + harmonics[SIN_PHI]*harmonics[SIN_PSI]*harmonics[SIN_TH];
  dfundx(1,5) = harmonics[COS_PHI]*harmonics[SIN_PSI]*harmonics[SIN_TH] - harmonics[COS_PSI]*harmonics[SIN_PHI]; 
  dfundx(1,6) = 0;
  dfundx(1,7) = 0;
  dfundx(1,8) = 0;

  //
  
  dfundx(2,0) = 0; 
  dfundx(2,1) = 0; 
  dfundx(2,2) = 0;         
  dfundx(2,3) = -harmonics[SIN_TH];                              
  dfundx(2,4) = harmonics[COS_TH]*harmonics[SIN_PHI];                              
  dfundx(2,5) = harmonics[COS_PHI]*harmonics[COS_TH];                                                                
  dfundx(2,6) = 0;
  dfundx(2,7) = 0;
  dfundx(2,8) = 0;

  //
  
  dfundx(3,0) = 0; 
  dfundx(3,1) = 0; 
  dfundx(3,2) = 0;                
  dfundx(3,3) = 0;                                          
  dfundx(3,4) = ar_z;                                         
  dfundx(3,5) = -ar_y;                                                                                                       
  dfundx(3,6) = 0;                                                                 
  dfundx(3,7) = 0;
  dfundx(3,8) = 0;
  
  //
  
  dfundx(4,0) = 0; 
  dfundx(4,1) = 0; 
  dfundx(4,2) = 0;            
  dfundx(4,3) = -ar_z;                                             
  dfundx(4,4) = 0;                                          
  dfundx(4,5) = ar_x;                                                                                      
  dfundx(4,6) = 0;
  dfundx(4,7) = 0;
  dfundx(4,8) = 0;
  
  //
  
  dfundx(5,0) = 0; 
  dfundx(5,1) = 0; 
  dfundx(5,2) = 0;             
  dfundx(5,3) = ar_y;                                         
  dfundx(5,4) = -ar_x;                                             
  dfundx(5,5) = 0;                                                                                     
  dfundx(5,6) = 0;
  dfundx(5,7) = 0;
  dfundx(5,8) = 0;
  
  //
  
  dfundx(6,0) = 0; 
  dfundx(6,1) = 0; 
  dfundx(6,2) = 0;                
  dfundx(6,3) = 0;                                             
  dfundx(6,4) = 0;                                             
  dfundx(6,5) = 0;                                                              
  dfundx(6,6) = 0;
  dfundx(6,7) = 0;
  dfundx(6,8) = 0;

  //
  
  dfundx(7,0) = 0; 
  dfundx(7,1) = 0; 
  dfundx(7,2) = 0;                
  dfundx(7,3) = 0;                                             
  dfundx(7,4) = 0;                                             
  dfundx(7,5) = 0;                                                                         
  dfundx(7,6) = 0;
  dfundx(7,7) = 0;                                                                                                                          
  dfundx(7,8) = 0;

  //
  
  dfundx(8,0) = 0; 
  dfundx(8,1) = 0; 
  dfundx(8,2) = 0;                
  dfundx(8,3) = 0;                                             
  dfundx(8,4) = 0;                                             
  dfundx(8,5) = 0;                                                       
  dfundx(8,6) = 0;
  dfundx(8,7) = 0;
  dfundx(8,8) = 0;

  /* Supposedly an error; L should be an identity matrix
  // filling dfun/du element-by-element
  
  dfundu(0,0) = 0; 
  dfundu(0,1) = 0; 
  dfundu(0,2) = 0;  
  dfundu(0,3) = 0;                
  dfundu(0,4) = 0;                
  dfundu(0,5) = 0;
  
  //
  
  dfundu(1,0) = 0; 
  dfundu(1,1) = 0; 
  dfundu(1,2) = 0;  
  dfundu(1,3) = 0;                
  dfundu(1,4) = 0;                
  dfundu(1,5) = 0;
  
  //
  
  dfundu(2,0) = 0, 
  dfundu(2,1) = 0, 
  dfundu(2,2) = 0,  
  dfundu(2,3) = 0,                
  dfundu(2,4) = 0,                
  dfundu(2,5) = 0;
  
  //
  
  dfundu(3,0) = 1; 
  dfundu(3,1) = 0; 
  dfundu(3,2) = 0;  
  dfundu(3,3) = 0;               
  dfundu(3,4) = -vBz;                
  dfundu(3,5) = vBy;

  //
  
  dfundu(4,0) = 0; 
  dfundu(4,1) = 1; 
  dfundu(4,2) = 0;  
  dfundu(4,3) = vBz;                
  dfundu(4,4) = 0;             
  dfundu(4,5) = -vBx;
  
  //
  
  dfundu(5,0) = 0; 
  dfundu(5,1) = 0; 
  dfundu(5,2) = 1; 
  dfundu(5,3) = -vBy;                
  dfundu(5,4) = vBx;               
  dfundu(5,5) = 0;
  
  //
  
  dfundu(6,0) = 0; 
  dfundu(6,1) = 0; 
  dfundu(6,2) = 0;  
  dfundu(6,3) = 1; 
  dfundu(6,4) = harmonics[SIN_PHI]*(harmonics[SIN_TH]/harmonics[COS_TH]); 
  dfundu(6,5) = harmonics[COS_PHI]*(harmonics[SIN_TH]/harmonics[COS_TH]);
  
  //
  
  dfundu(7,0) = 0; 
  dfundu(7,1) = 0; 
  dfundu(7,2) = 0;  
  dfundu(7,3) = 0;         
  dfundu(7,4) = harmonics[COS_PHI];        
  dfundu(7,5) = -harmonics[SIN_PHI];

  //
  
  dfundu(8,0) = 0; 
  dfundu(8,1) = 0; 
  dfundu(8,2) = 0;  
  dfundu(8,3) = 0; 
  dfundu(8,4) = harmonics[SIN_PHI]/harmonics[COS_TH]; 
  dfundu(8,5) = harmonics[COS_PHI]/harmonics[COS_TH];
  */
  
  
}

//! Evaluates matrices for time propagation
void ekf_time_update(Vec9fCol & MM, Mat99f & PP , Vec9fCol funk , Mat99f Fk , Mat66f Qk)
{
    Mat96f eye = Mat96f::eye(); 
    MM = funk;
    PP = Fk * PP * Fk.t() +  eye * Qk * eye.t(); 
}

//! Measurement update
void ekf_measurement_update(Vec9fCol & MM, Mat99f & PP , Vec6fCol ZZ, Vec6fCol hk, Mat69f Hk, Mat66f Rk)
{
  // we need to use Mat instead of Matx because of inverse method implemented only for Mat - there is division in K evaluation
  Mat temp_inversion;
  Mat temp;
  
  Mat66f temp_inversion_x;
  Mat96f K;
  
  temp =  Mat(Hk * PP * Hk.t() + Rk);   // compute K evaluation fraction denominator
  invert(temp, temp_inversion);		// find inversion
  temp_inversion_x = Mat66f(temp_inversion);	// return back to Matx format
  
  // measurement update
  
  K = PP * Hk.t() * temp_inversion_x;
  MM = MM + K * (ZZ - hk);
  PP = (Mat99f::eye() - K*Hk) * PP;
  PP = 0.5 * (PP + PP.t());
  
}
