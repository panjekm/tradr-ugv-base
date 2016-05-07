#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv.h"
#include "highgui.h"
#include "functions.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatStatus.h>
#include <tf/transform_broadcaster.h>
#include "string_to_matrix.h"
#include <nifti_robot_driver_msgs/TracksStamped.h>

#include <fstream>
#include <cstring>
#include <iostream>
#include <sstream>


/**
* measurement and system noise covariances- for tuning and testing purpose,
* Final version is to be found in the code (constructor, look for Qt, Rk) and change it to the commented version
*/
#define QT_COVARIANCE	0.001
#define RK_COVARIANCE	0.1


using namespace cv;
using namespace std;


void printMatrix33 (Mat33f matrix);
void printVector3 (Vec3fCol vector);



class SharedObjects
{
	public:
		// VARIABLES
			ros::NodeHandle 	nh;
			
			// topic publishers
			//ros::Publisher 		data_pub;
			//ros::Publisher		imu_pub;
			//ros::Publisher		imu_aiding_debug;

			
			//! FP: topic for odometry output
			ros::Publisher odom_pub;
			//! FP: string containing frame_id names obtained by parameter
			string odom_frame;
			string robot_frame;		
			
			//! FP: add counter for bad msg
			int zeroImuMsgCount;
			
			// topic subscribers
				ros::Subscriber 	sub_imu;
				ros::Subscriber 	sub_gps;
				ros::Subscriber		sub_odom;
				ros::Subscriber		sub_tracks;
				ros::Subscriber		sub_magnetic;	
				
				tf::TransformBroadcaster br;
			
			// output message, transformation and imu message
				
			tf::Transform transform;
			//inso::inso_output output;
			//inso::inso_output output_aiding;		
			sensor_msgs::Imu imu_msg;
			quat q_imu_msg_att;         //! stores attitude quaternion to be published
			nav_msgs::Odometry odom_msg;
			
			

			Vec3fCol meanACC, meanGYR, EUL; 

			
			// node configuration variables

				int stat_len; 				//! number of samples of static data used for averaging during initial alignment
				bool switch_attaiding; 		//! set to 1 for ACC&GYR fusion
				double w_gyr;       		//! gyr weight value for averaging euler angles from gyr and acc
				double w_acc;        		//! acc weight value for averaging euler angles from gyr and acc
				int avgspan;         		//! span for acceleration moving average filtering
				double Ts;		 			//! sampling period
				double iniLAT;     			//! (deg) set to actual approx. position (e.g. from GPS)
				double iniLON;     			//! (deg) set to actual approx. position (e.g. from GPS)
				double iniALT;         		//! set to actual approx. altitude (e.g. from GPS)
				double yaw;
				bool yawSET;			
				bool internalLogged;	//! if imu-streamed orientation data are logged or not 
				int fbtype;

			
			// IMU calibration values
				// Accelerometers:
				Mat33f 		aSF; 		//! accelerometer Scale Factor matrix
				Mat33f 		aMA; 		//! accelerometer MisAlignment matrix
				Vec3fCol 	aB; 		//! accelerometer Biases
		
				// Gyroscopes:
				Mat33f 		gSF;		//! gyroscope Scale Factor matrix
				Mat33f 		gMA;		//! gyroscope MisAlignment matrix
				Vec3fCol 	gB;			//! gyroscope Biases

			Vec10fCol 	IMUdata; 	 	//! vector containing measured values
			Vec3fCol 	gN;				//! gravity vector expressed in NED
		
			quat 		Q_b2b;          //! initialization of t(k-1) -> t(k) transform quaternion
			quat 		prev_Q_b2n; 	//! previous period body -> NED quaternion
			quat 		Q_b2n;     		//! quaternion representation of orientation
			Mat33f 		C_b2n;  		//! BODY -> NED transform matrix: (corresponds to C_b2n = RX*RY*RZ rotation)
			Vec3fCol 	dAB;          	//! initialization of the current value of integrated angular rate
			Vec3fCol 	prev_dAB;     	//! of the previous value of integrated angular 
	  
			Vec3fCol LLA; 			//! LLA (deg) vector initialization
	  
			Vec3fCol prev_EUL;		//! previous period EULer angles computed by the attitude mechanization
			Vec3fCol prev_EULacc;	//! previous period EULer angles computed by alignment
			
			Vec3fCol prev_accs;   	//! previous period acceleration data
			Vec3fCol prev_gyrs;   	//! previous period angular rates
	  
			Vec3fCol coning;
			Vec3fCol rotvB;

			Vec3fCol accs;	//!  vector for acceleration values
			Vec3fCol gyrs;	//!  vector for angular rate values

			
			// Odometry and Absolute model variables
			
			bool 		switch_useconstraints;
			double 		stride_scale;
			double		stride_lenght;
			
			Mat99f 		I_99;
			Vec9fCol 	Mplus;
			Mat99f		Pplus;
			
			double initial_attitude_variance;
			double sigma_acc_x; double sigma_acc_y; double sigma_acc_z;
			double sigma_ang_x; double sigma_ang_y; double sigma_ang_z;
			double aid_sigma_pos; double aid_sigma_vel;
			
			double		Qtc;
			double		Rkc;
			Mat66f		Qt;
			Mat66f		Rk;
			
			Vec9fCol	MM;
			Vec9fCol	previous_MM;
			Mat99f		PP;
			Mat99f		previous_PP;
			Vec6fCol	ZZ;
			Vec3fCol	VN_aid;
			Vec3fCol	PN_aid;
			Vec3fCol	PN_odo;
			Vec3fCol	LLA_aid;
			
			Vec3fCol	Position_in_NWU;
			Vec3fCol	Velocity_in_NWU;
			Mat33f		C_NED_2_NWU;
			
			Vec3fCol	previous_PN_aid;
			Vec3fCol	previous_PN_odo;
			
			// absolute model mechanization variables
			
			double sf_x, sf_y, sf_z, ar_x, ar_y, ar_z, vBx, vBy, vBz, phi, th, psi;			
			Vec9fCol fun;
			Vec9fCol funk;
			Mat99f	 dfundx;
			Mat96f	 dfundu;
			Mat99f	 Fk;
			Mat66f	 Qk;
			
			// Kalman filter update variables
			Vec6fCol hk;  //! measurement equation
			Mat69f   Hk;  //! partial derivates
			
			//! array containing values of sin() and cos() of phi,th and psi angles
			double harmonics[6];
			
			//! boolean signaling odometry data ready for the imu callback functions
			bool odometry_ready;
			
			//! movement direction determined from track speeds: 1=forward, -1=backwards
			int direction_from_tracks;
			//! does the robot move at all?
			bool tracks_dont_move;
			//! track velocity difference for yaw constraint
			double tracks_vel_diff;
			//! maximum tracks vel difference for constraints to apply
			double tracks_vel_diff_tresh;
			
			//tracks mechanization
			ros::Time odometry_previous_time;
			double odometry_previous_v;
			double odometry_previous_w;
			bool odometry_first_run;
			double steering_efficiency; 
			double robot_width;
	
			
			// Geodetic constants:
			float 	earth_rate;	//! earth rate in rad/s ... w_ie_ez
			float 	b;  		//! Length of Earth's semi-minor axis (m)
			float 	f; 		//! Earth's elipticity, also reffered as "flattening"b
			float 	C2;
			
			//! radius of curvature in the prime verctical
			double	R_N;
			//! Meridian radius of curvature
			double  R_M;
			
			//! scaling constant (g -> m.s^-2) - given by MicroStrain, currently not used
			float 	gsc; 

			// initialization of moving average buffers for ACC and ARR values
			double 	*buf_accx; 
			double 	*buf_accy;
			double 	*buf_accz;
			double 	*buf_gyrx;
			double 	*buf_gyry;
			double 	*buf_gyrz;
		
			int 	pointer;			/** variable telling which value in the moving average 
									 	 * buffers is the oldest, that one is about to be replaced */
		  
			int 	iSampleCounter;		//! counter of the samples from the beginning of the measurement
			char 	iState;				//! current measurement phase
		
			// variables used for feedback filtering
			Vec3fCol feedback_Temp;
			Vec3fCol feedback_toDelay;
			Vec3fCol feedback_out;
			Vec3fCol feedback_in;
			double feedback_v;
			double feedback_u;
			double feedback_low_pass_a;
			double feedback_K;
			double feedback_complementary_filter_omega;

			//FILE *cfgFile;
			bool bMechanizationEnabled;
			bool bVpaEnabled;
			
			FILE *fpGps;
			bool bGpsEnabled;
			
			FILE *fpOdometry;
			bool bOdometryEnabled;
			
			FILE *fpTracks;
			bool bTracksEnabled;
			
			FILE *fpMagnetic;
			bool bMagneticEnabled;		

			std::ofstream outputFile;		//! output streaming class declaration
			std::ofstream outputVpaFile;		//! additional stream for velocity, position etc.
		
		// METHODS
			SharedObjects();
			~SharedObjects();
			
			void 	ImuCallback(const sensor_msgs::Imu msg);
			void 	OdometryCallback(const nav_msgs::Odometry msg);
			void 	GpsCallback(const sensor_msgs::NavSatFix msg);
			void 	TracksCallback(const nifti_robot_driver_msgs::TracksStamped msg);
			void 	MagneticCallback(const geometry_msgs::Vector3Stamped msg);
			

			void  	getROSParameters(); //! sweeps through ROS param server and sets ins parameters 
			void	printInitialParameters(); //! prints all initial parameters into std output
};

SharedObjects::SharedObjects():
  zeroImuMsgCount(0)
{
	// initialize configuration and load parameters from the rosparam server
	getROSParameters();
	
	
	// Init variabiles
	meanACC = Vec3fCol::zeros();  	// vector of initial averaging results
	meanGYR = Vec3fCol::zeros(); 	// vector of initial averaging results

	accs = Vec3fCol::zeros();		
	gyrs = Vec3fCol::zeros();		


	
	// Geodetic constants:
	earth_rate = 7.292115147*pow((double)10,-5);	// earth rate in rad/s
	b = 6356752.3142;  								// Length of Earth's semi-minor axis (m)
	f = ((geo_a-b)/geo_a); 								// Earth's elipticity, also reffered as "flattening"
	C2 = -0.00048416685;
	
	R_N = geo_a * (1 - geo_e*geo_e) / pow(1 - pow(geo_e*sin(deg2rad(iniLAT)),2),3/2);
	R_M = geo_a / sqrt(1 - pow(geo_e*sin(deg2rad(iniLAT)),2));

	// scaling constant (g -> m.s^-2) - given by MicroStrain, currently not used
	gsc = 9.80655; 

	// local gravity vector computation
	Vec3fCol iniLLA(iniLAT, iniLON, iniALT);	 
	gN = comp_gravity(iniLLA);	 
	
	
	// Odometry and Absolute model initializations
	
	
	I_99 = Mat99f::eye();			// an identity matrix
	Mplus = Vec9fCol::zeros();		// initial values of the MM state vector
	Pplus = Mat99f::eye();			// initial values of the PP covariance matrix
	    Pplus(6,6) = deg2rad(initial_attitude_variance);
	    Pplus(7,7) = deg2rad(initial_attitude_variance);
	    Pplus(8,8) = deg2rad(initial_attitude_variance);	// ROS - only one number
	
	
	Qt  = Mat66f::eye();
		Qt(0,0) = (sigma_acc_x*sigma_acc_x);
		Qt(1,1) = (sigma_acc_y*sigma_acc_y);
		Qt(2,2) = (sigma_acc_z*sigma_acc_z);
		Qt(3,3) = (sigma_ang_x*sigma_ang_x);
		Qt(4,4) = (sigma_ang_y*sigma_ang_y);
		Qt(5,5) = (sigma_ang_z*sigma_ang_z);
		

	Rk  = Mat66f::eye();
		Rk(0,0) = (aid_sigma_pos*aid_sigma_pos);
		Rk(1,1) = (aid_sigma_pos*aid_sigma_pos);
		Rk(2,2) = (aid_sigma_pos*aid_sigma_pos);
		Rk(3,3) = (aid_sigma_vel*aid_sigma_vel);
		Rk(4,4) = (aid_sigma_vel*aid_sigma_vel);
		Rk(5,5) = (aid_sigma_vel*aid_sigma_vel);	
	

	
	// initialize state and covariance vector according to the initial paremeters set in Mplus and Pplus
	MM = Mplus;
	previous_MM = MM;
	
	PP = Pplus;
	previous_PP = PP;
	
	// initialize odometry aiding vectors
	PN_aid		= Vec3fCol::zeros();
	previous_PN_aid = Vec3fCol::zeros();
	PN_odo		= Vec3fCol::zeros();
	previous_PN_odo = Vec3fCol::zeros();
	VN_aid		= Vec3fCol::zeros();
	LLA_aid		= Vec3fCol::zeros();
	
	// ROS convention position and velocity
	Position_in_NWU = Vec3fCol::zeros();
	Velocity_in_NWU = Vec3fCol::zeros();
	C_NED_2_NWU = Mat33f::eye();
		C_NED_2_NWU(1,1) = -1;
		C_NED_2_NWU(2,2) = -1;
		
	
	sf_x = 0; sf_y = 0; sf_z = 0; ar_x = 0; ar_y = 0; ar_z = 0; vBx = 0; vBz = 0; phi = 0; th = 0; psi = 0;
	
	fun 	= Vec9fCol::zeros();
	funk 	= Vec9fCol::zeros();
	dfundx 	= Mat99f::zeros();
	dfundu 	= Mat96f::zeros();
	Fk	= Mat99f::zeros();
	Qk	= Mat66f::zeros();
	memset(harmonics,0,sizeof(double)*6);
	
	odometry_ready = false;
	direction_from_tracks = 1;
	stride_lenght = 0;
	tracks_dont_move = false;
	tracks_vel_diff = 0;
	tracks_vel_diff_tresh = 0.0001;   //TODO: import as a rosparam
	
	// tracks mechanization
	odometry_previous_v = 0;
	odometry_previous_w = 0;
	odometry_first_run = true;
	steering_efficiency = 0.41;
	robot_width = 0.397;
	
	
	
	hk = Vec6fCol::zeros();  // measurement equation
	Hk = Mat69f::zeros();  // partial derivates
	
	
	
	ROS_WARN("Calibration in progress, do not move the IMU, wait for: Calibration finished.");	
	  
	// moving average buffers initialization
	buf_accx = new double[avgspan]; 
	buf_accy = new double[avgspan];  
	buf_accz = new double[avgspan]; 
	buf_gyrx = new double[avgspan]; 
	buf_gyry = new double[avgspan]; 
	buf_gyrz = new double[avgspan]; 

	// moving average buffers resetting	  
	memset(buf_accx,0,sizeof(double)*avgspan);
	memset(buf_accy,0,sizeof(double)*avgspan);
	memset(buf_accz,0,sizeof(double)*avgspan);
	memset(buf_gyrx,0,sizeof(double)*avgspan);
	memset(buf_gyry,0,sizeof(double)*avgspan);
	memset(buf_gyrz,0,sizeof(double)*avgspan);

	// inform user about the currently used attitude feedback method
	switch (fbtype)
	{
		case ATT_NO_FEEDBACK:
			ROS_INFO("Attitude feedback disabled");
			break;
		case ATT_GYR_ACC_WEIGHTED_AVG:
			ROS_INFO("Attitude weighted averaging feedback chosen");
			break;
		case ATT_FILTERED:
			ROS_INFO("Attitude filtering feedback chosen");
			break;
	}

	feedback_complementary_filter_omega = 1; 					// border frequency of the complementar filter
	feedback_low_pass_a	= feedback_complementary_filter_omega * 2;     
	feedback_K 	= feedback_complementary_filter_omega/2;     

	// reset sample counter and measurement phase
	iSampleCounter 	= 0;
	iState 		= 0;

	// initialize feedback filtering variables
	feedback_Temp 		= Vec3fCol(1,0,0);
	feedback_toDelay 	= Vec3fCol(0,0,0);
	feedback_out 		= Vec3fCol(0,0,0);
	feedback_in 		= Vec3fCol(0,0,0);
	feedback_v		 	= pow(EULER,(-(feedback_low_pass_a)*(Ts)));
	feedback_u 			= 1-feedback_v;

	// define publisher and subscribers
	//data_pub 	= nh.advertise<inso::inso_output>("/mechanization_output_inso", 10);
	//imu_pub 	= nh.advertise<sensor_msgs::Imu>("/inso_imu", 10);
	//imu_aiding_debug = nh.advertise<inso::inso_output>("/mechanization_output_inso_aiddebug",10);
	sub_imu 	= nh.subscribe("/imu/data", 100, &SharedObjects::ImuCallback, this);
	sub_gps 	= nh.subscribe("/fix", 100, &SharedObjects::GpsCallback, this);
	sub_odom 	= nh.subscribe("/odom", 100, &SharedObjects::OdometryCallback, this);
	sub_tracks 	= nh.subscribe("/tracks_vel", 100, &SharedObjects::TracksCallback, this);
	sub_magnetic	= nh.subscribe("/magnetic",100, &SharedObjects::MagneticCallback, this);
	
	// FP: fetch parameter to get more flexibility on published topic name
	string odom_topic_name;
	nh.param<std::string>("odom_topic", odom_topic_name, "/imu_odom");
	odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic_name, 10);
	// FP: fetch frame_ids
	// FP: warning those frame_ids are not used everywhere (but they should)
	nh.param<std::string>("odom_frame", odom_frame, "/odom");
	nh.param<std::string>("robot_frame", robot_frame, "/base_link");
	
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	const int buffLenght = 25;
	char timeCut[buffLenght];
	strftime(timeCut, buffLenght, "_%d_%b_%Y_%Hh%Mm%Ss", timeinfo);
	
	//cout << timeCut << endl;
	ROS_INFO("%s",timeCut);

	string timeName(timeCut);

	string out = "";
	bool bLoggingEnabled = true;
	
	// see if global logging paremeter has been set
	// (remember not to overlog!)
	if (ros::param::has("inso_logging")){
	
		// if yes, check its type and store it in bLoggingEnabled
		if(!(ros::param::get("inso_logging", bLoggingEnabled)))
		{
			ROS_ERROR("Parameter \"inso_logging\" is not a boolean, setting to \"true\"");
			bLoggingEnabled = true;			
		}
	}
	
	// if logging globally enabled, check for each logging file parameter and set its flags
	if(bLoggingEnabled)
	{
		ROS_INFO("Logging globally enabled.");		
		
		// if there is a mechanization logging file defined...
		if (ros::param::has("inso_mech"))
		{
			string sFile;
			ros::param::get("inso_mech", sFile);
			out = sFile + timeName + ".csv";
			outputFile.open(out.c_str());
			
			if (!outputFile.fail())
			{
				bMechanizationEnabled = true;
			}
			// ...if not, disable mechanization logging
			else
			{
				ROS_ERROR("Unable to open mechanization log file");
				bMechanizationEnabled = false;
			} 
		}
		else bMechanizationEnabled = false;
		
		
		// if htere is VPA logging enabled
		if (ros::param::has("inso_vpa"))
		{
			string sFile;
			ros::param::get("inso_vpa", sFile);
			out = sFile + timeName + ".csv";
			outputVpaFile.open(out.c_str());
			
			if (!outputVpaFile.fail())
			{
				bVpaEnabled = true;
			}
			// ...if not, disable mechanization logging
			else
			{
				ROS_ERROR("Unable to open VPA log file");
				bVpaEnabled = false;
			} 
		}
		else bVpaEnabled = false;
	
		// if there is a GPS logging file defined...
		if (ros::param::has("inso_gps"))
		{
			string sFile;
			ros::param::get("inso_gps", sFile);
			out = sFile + timeName + ".csv";
			if ((fpGps = fopen(out.c_str(), "a+")) != NULL)
			{
				bGpsEnabled = true;
			}
			// ...if not, disable GPS logging
			else
			{
				ROS_ERROR("Unable to open GPS log file");
				bGpsEnabled = false;
			} 
		}
		// if there is no GPS logging file defined, disable GPS logging
		else bGpsEnabled = false;

		// if there is an odometry logging file defined...
		if (ros::param::has("inso_odometry"))
		{
			string sFile;
			ros::param::get("inso_odometry", sFile);
			out = sFile + timeName + ".csv";
			
			if ((fpOdometry = fopen(out.c_str(), "a+")) != NULL)
			{
				bOdometryEnabled = true;
			}
			// ...if not, disable odometry logging
			else
			{
				ROS_ERROR("Unable to open odometry log file");
				bOdometryEnabled = false;
			}
		}
		// if there is no odometry logging file defined, disable odometry logging
		else bOdometryEnabled = false;
		
		
		// if there is a tracks logging file defined...
		if (ros::param::has("inso_tracks"))
		{
			string sFile;
			ros::param::get("inso_tracks", sFile);
			out = sFile + timeName + ".csv";
			
			if ((fpTracks = fopen(out.c_str(), "a+")) != NULL)
			{
				bTracksEnabled = true;
			}
			// ...if not, disable tracks logging
			else
			{
				ROS_ERROR("Unable to open tracks log file");
				bTracksEnabled = false;
			}
		}
		// if there is no tracks logging file defined, disable it
		else bTracksEnabled = false;
	

		// if there is a magnetometer logging file defined...
		if (ros::param::has("inso_magnetic"))
		{
			string sFile;
			ros::param::get("inso_magnetic", sFile);
			out = sFile + timeName + ".csv";
			
			if ((fpMagnetic = fopen(out.c_str(), "a+")) != NULL)
			{
				bMagneticEnabled = true;
			}
			// ...if not, disable odometry logging
			else
			{
				ROS_ERROR("Unable to open magnetometer log file");
				bMagneticEnabled = false;
			}
		}
		// if there is no magnetometer logging file defined, disable it
		else bMagneticEnabled = false;

	}
	
	// if logging globally disabled, no files are created and flags are set to false
	else
	{
		bMechanizationEnabled = false;
		bGpsEnabled = false;
		bOdometryEnabled = false;
		bTracksEnabled = false;
		bVpaEnabled = false;
		bMagneticEnabled = false;
		
		ROS_INFO("Logging globally disabled.");
	}
	
	//printInitialParameters();	

	
}

//-----------------------------------------------------------------------------
SharedObjects::~SharedObjects()
{
	fclose(fpGps);
	fclose(fpOdometry);
	fclose(fpTracks);
	fclose(fpMagnetic);
	outputFile.close();
	outputVpaFile.close();
}

//---------------------------------------------------------------------------
void SharedObjects::GpsCallback(const sensor_msgs::NavSatFix msg)
{

	if (bGpsEnabled)
	{
		if((msg.latitude==0) && (msg.longitude==0) && (msg.altitude ==0))
			{
				ROS_ERROR("GPS callback recived only zeros. Ignoring the message.");
				return;
			}	
	
	
		fprintf(fpGps, "%d.%09d\n", msg.header.stamp.sec, msg.header.stamp.nsec);
		fprintf(fpGps, "%f %f %f\n", msg.latitude
			, msg.longitude
			, msg.altitude);
	}
}
//---------------------------------------------------------------------------
void SharedObjects::MagneticCallback(const geometry_msgs::Vector3Stamped msg)
{

	if (bMagneticEnabled)
	{
	
	
		fprintf(fpMagnetic, "%d.%09d ", msg.header.stamp.sec, msg.header.stamp.nsec);
		fprintf(fpMagnetic, "%f %f %f\n", msg.vector.x
			, msg.vector.y
			, msg.vector.z);
	}
}
//---------------------------------------------------------------------------
void SharedObjects::TracksCallback(const nifti_robot_driver_msgs::TracksStamped msg)
{
		
  
	double vl, vr, v, w;
	double dt, d, dtheta, dx, dy;
	ros::Time odometry_now_time;
	Vec3fCol PN_increment;
	Vec3fCol stride;
	
	if(msg.left== 0 && msg.right == 0) { tracks_dont_move = true;}
	else				   { tracks_dont_move = false;}

	tracks_vel_diff = abs(msg.left-msg.right);			
	
	
	
	if(!odometry_first_run)				// this callback has run at least once
	{	
		
		odometry_now_time = ros::Time::now();
			
		// determine whether robots moves forward or backwards
		if((msg.left+msg.right)/2 >= 0) direction_from_tracks = 1;
		else				direction_from_tracks = -1;
		
		// extract track speeds
		vl = msg.left;
		vr = msg.right;
		
		// determine linear and angular velocities
		v = (vl+vr)/2.0;
		w = (vr-vl)*(steering_efficiency/robot_width);
		
		// evaluate increments
		dt = (odometry_now_time - odometry_previous_time).toSec();
		d = dt * (v + odometry_previous_v)/2.0;	// constant acceleration 
		dtheta = dt * (w + odometry_previous_w)/2.0;
		
		// straight line or part of a circle
		if (abs(dtheta)<0.0001) { // straight path
			dx = d;
			dy = 0.0;
		} else {	// circular approximation
			dx = d/dtheta * sin(dtheta);
			dy = d/dtheta * (1 - cos(dtheta));
		}
		
		// save now-values to previous
		odometry_previous_time = odometry_now_time;
		odometry_previous_v = v;
		odometry_previous_w = w;
		
	
		if(iState == MEAS_STATE_MECHANIZATION)	// C_b2n is avaible only at this stage, not before
		{
	
			PN_increment = Vec3fCol::zeros();
			stride       = Vec3fCol::zeros();
	
	  
			
		
			//compute the stride lenght since last update
			stride_lenght = sqrt( dx*dx + dy*dy );
		
			stride(PN) = stride_lenght * stride_scale * direction_from_tracks;
		
			
		
			//rotate stride according to the robot orientation in NED
			PN_increment = C_b2n * stride;  
		
			
			/*	
			if(switch_useconstraints && tracks_dont_move)
			{
			  PN_aid(0) = previous_MM(0); PN_aid(1) = previous_MM(1); PN_aid(2) = previous_MM(2);
			  VN_aid = Vec3fCol::zeros();
			}
			else */
			{		
			  // update aiding vectors
			  PN_aid = previous_PN_aid + PN_increment;
			  VN_aid = (PN_aid - previous_PN_aid) * (1.0/dt);	
			}
		
			previous_PN_aid = PN_aid;
			previous_PN_odo = PN_odo;

			odometry_ready = true;
	
		}
	
	
		
	
	}
	else						// if this is the first run, just evaluate necessary (k-1) stuff
	{
		odometry_now_time = ros::Time::now();
		
		// extract track speeds
		vl = msg.left;
		vr = msg.right;
		
		// determine linear and angular velocities
		v = (vl+vr)/2.0;
		w = (vr-vl)*(steering_efficiency/robot_width);
		
		
		// save now values to previous
		odometry_previous_time = odometry_now_time;
		
		
		odometry_previous_v = v;
		odometry_previous_w = w;	
		
		odometry_first_run = false;
	}
	
		
	
	if (bTracksEnabled)
	{
	     
		ros::Time now = ros::Time::now();
	
		fprintf(fpTracks, "%d.%09d %f %f\n", now.sec, now.nsec, msg.left, msg.right);
		
	}
	
}

//---------------------------------------------------------------------------
void SharedObjects::OdometryCallback(const nav_msgs::Odometry msg)
{
	
	if (bOdometryEnabled)
	{
		fprintf(fpOdometry, "%d.%09d\n", msg.header.stamp.sec, msg.header.stamp.nsec);
		fprintf(fpOdometry, "%f %f %f\n", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
		fprintf(fpOdometry, "%f %f %f %f\n", msg.pose.pose.orientation.x
					 , msg.pose.pose.orientation.y
					 , msg.pose.pose.orientation.z
					 , msg.pose.pose.orientation.w);

		fprintf(fpOdometry, "%f %f %f\n", msg.twist.twist.linear.x
					  ,	msg.twist.twist.linear.y
					  , msg.twist.twist.linear.z);
	 
		fprintf(fpOdometry, "%f %f %f\n", msg.twist.twist.angular.x
					  ,	msg.twist.twist.angular.y
					  , msg.twist.twist.angular.z);
	}
	
}


void SharedObjects::ImuCallback(const sensor_msgs::Imu msg)
{
	// read data from the topic and their rotation to match robot axes
		// raw accelerations
			IMUdata(0) = msg.linear_acceleration.y;
			IMUdata(1) = msg.linear_acceleration.x;
			IMUdata(2) = -msg.linear_acceleration.z;

		// raw angular velocity
			IMUdata(3) = msg.angular_velocity.y;
			IMUdata(4) = msg.angular_velocity.x;
			IMUdata(5) = -msg.angular_velocity.z;			

		// internal IMU orientation
			IMUdata(6) = msg.orientation.y; 	//! roll
			IMUdata(7) = msg.orientation.x; 	//! pitch	
			IMUdata(8) = -msg.orientation.z; 	//! yaw
			
			
	if(IMUdata(0)==0 && IMUdata(1)==0 && IMUdata(2)==0 && IMUdata(3)==0 && IMUdata(4)==0 && IMUdata(5)==0 )
	{
		zeroImuMsgCount++;
		if(zeroImuMsgCount % 100 == 0)
		{
			ROS_ERROR_STREAM("ImuCallback recieved " << zeroImuMsgCount << " messages full of zeros. Ignoring those ones.");
		}
		return;
	}
	
	// set to XSENS mti-g maximum scale, anything higher must be an error
	if(
		IMUdata(0)>60 || IMUdata(0)<-60 || IMUdata(1)>60 || IMUdata(1)<-60 || IMUdata(2)>60 || IMUdata(2)<-60 ||
		IMUdata(3)>6 || IMUdata(3)<-6 || IMUdata(4)>6 || IMUdata(4)<-6 || IMUdata(5)>6 || IMUdata(5)<-6  )
	{
		ROS_ERROR("ImuCallback recieved a message with values higher than maximum sensor scale. Ignoring that one.");
		//ros::shutdown();
		return;
	}
	

	if (iSampleCounter > 65000) iSampleCounter = stat_len+2;

	IMUdata(9) = (double)iSampleCounter / 120;


	accs(0,0) = IMUdata(0);		// fill vector for acceleration values
	accs(1,0) = IMUdata(1);
	accs(2,0) = IMUdata(2);

	gyrs(0,0) = IMUdata(3);		// fill vector for angular rate values
	gyrs(1,0) = IMUdata(4);
	gyrs(2,0) = IMUdata(5);


	char text[512];		// initialize char buffer for output data

	switch (iState)
	{
		// first measurement phase - calibration
		case MEAS_STATE_CALIB:
			iSampleCounter++;
			
			// if the desired number of samples for calibration is collected, 
			// proceed to processing of the calibration data
			if (iSampleCounter >= stat_len) iState++;

			accs = accs - aB; 			// accelerometer Bias correction	
			accs = aSF*(aMA*accs); 		// accelerometer Scale Factor and MisAlignment correction
			
			meanACC = meanACC + accs;	// integrate ACC values
			meanGYR = meanGYR + gyrs;	// integrate ARR values

			if (bMechanizationEnabled)
			{
				// create the line of the output file
				if (internalLogged)
				{
					sprintf(text,"%d.%09d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",
							msg.header.stamp.sec, msg.header.stamp.nsec,IMUdata(0),
							IMUdata(1),IMUdata(2),IMUdata(3),IMUdata(4),IMUdata(5),
							EUL(0),EUL(1),EUL(2),IMUdata(6),IMUdata(7),IMUdata(8));
				}
				else
				{
				
					sprintf(text,"%d.%09d, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",
							msg.header.stamp.sec, msg.header.stamp.nsec,IMUdata(0),IMUdata(1),IMUdata(2),IMUdata(3),
							IMUdata(4),IMUdata(5),EUL(0),EUL(1),EUL(2));
				}
			}
			// save the line into the file
			outputFile << text;
			

			if (bVpaEnabled)
			{
					//                     vx  vy  vz |vel. var.  | x   y   z| pos.var    |attitude   | att. var  | VN_aid     | PN_aid 
					sprintf(text,"%d.%09d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
							msg.header.stamp.sec, msg.header.stamp.nsec,
							MM(3),MM(4),MM(5), PP(3,3), PP(4,4), PP(5,5), // velocity and its variance
							MM(0),MM(1),MM(2), PP(0,0), PP(1,1), PP(2,2), // position and its variance
							MM(6),MM(7),MM(8), PP(6,6), PP(7,7), PP(8,8), // attitude in euler and its variance (note that these are evaluated separately from Kalman
							VN_aid(0), VN_aid(1), VN_aid(2),	      // velocity aiding vector
							PN_aid(0), PN_aid(1), PN_aid(2)              // position aiding vector
						); 
				
			}
			// save the line into the file
			outputVpaFile << text;
			
			break;

		// processing of the measured calibration data
		case MEAS_STATE_CALIB_RESULT:

			// divide all the summed values with their counts to get averages
			for (int i = 0; i < 3; i++)
			{
				meanACC(i) = meanACC(i)/stat_len;
				meanGYR(i) = meanGYR(i)/stat_len;
			}

			// use angular rate value averages as biases 
			gB(0,0) = meanGYR(0);
			gB(1,0) = meanGYR(1);
			gB(2,0) = meanGYR(2);
			

			// perform initial alignment
			EUL = alignment_coarse(meanACC(0), meanACC(1), meanACC(2), 
									meanGYR(0), meanGYR(1), meanGYR(2), 
									iniLAT, gN(2), earth_rate);   
			// get the yaw value from the IMU
			EUL(2) = IMUdata(8);

			// if there is the yaw value specified, use this one
			if (yawSET) EUL(2) = yaw; 	// set desired initial yaw value
			// after the processing, continue to collecting the first mechanization samples
			
			// VK: INSO additional
			//////////////////////
			
			// set euler angles in the state vector MM
			MM(6,0) = EUL(0);
			MM(7,0) = EUL(1);
			MM(8,0) = EUL(2);
			
			// update the previous_MM vector as well
			
			previous_MM = MM;
			
			iState++;
			break;
 
 		// collecting the first mechanization samples and additional initializations
		case MEAS_STATE_FIRST: 
			LLA(0) = iniLAT;
			LLA(1) = iniLON;
			LLA(2) = iniALT; // LLA (deg) vector initialization
		

			// Computation of quaternions using recomputed rotation vectors
			Q_b2n = euler2quat(deg2rad(EUL));	// quaternion representation of orientation

			// BODY to NED transform matrix: (corresponds to C_b2n = RX*RY*RZ rotation)
			C_b2n = (quat2dcm(Q_b2n)).t();

			// First measurements used for mechanization:
				// acceleration data
					prev_accs(0) = IMUdata(0);
					prev_accs(1) = IMUdata(1);
					prev_accs(2) = IMUdata(2);
				// angular rates data
					prev_gyrs(0) = IMUdata(3);
					prev_gyrs(1) = IMUdata(4);
					prev_gyrs(2) = IMUdata(5);
			 
			pointer = 0;
			  
			// set the current computed values as previus for the next pariod
			prev_EUL = EUL;
			prev_EULacc = EUL;
			
			prev_dAB = Vec3fCol::zeros();	// of the previous value of integrated angular 
			prev_Q_b2n = Q_b2n;				// initialization of t(k-2) -> t(k-1) transform quaternion

			// proceed to mechanization
			ROS_INFO("Calibration finished.");
			iState++;  
			break;

		// main mechanization (potentially infinite loop)
		case MEAS_STATE_MECHANIZATION:  
			iSampleCounter++;
			
			
			/* buffer the current values of acceleration and angular rate
			 * into moving average buffers and increment the pointer
			 */
				buf_accx[pointer] = IMUdata(0); 
				buf_accy[pointer] = IMUdata(1); 
				buf_accz[pointer] = IMUdata(2);
				buf_gyrx[pointer] = IMUdata(3); 
				buf_gyry[pointer] = IMUdata(4); 
				buf_gyrz[pointer] = IMUdata(5);
			  
				pointer++;

			// when pointer overflows the bounds of buffers, it is reset to 0
			if (pointer == avgspan) pointer = 0;

			/* -----------------------------------------------------------------------------
			 * ATTITUDE MECHANIZATION - GYR
			 * ------------------------------------------------------------------------------
			 */	
			// bias correction
			gyrs = gyrs - gB;
			// scale factor and misalignment correction
			gyrs = gSF*(gMA*gyrs);
			
			if (fbtype == ATT_FILTERED)
			{
				/* --------------------
				 * Feedback filtering
				 * --------------------
				 */
				 				 	
				 	feedback_in = prev_EULacc - prev_EUL;
					feedback_Temp = feedback_toDelay;
					feedback_toDelay = feedback_v*feedback_out + feedback_u*feedback_in;
					feedback_out = feedback_K*Vec3fCol(feedback_Temp(0),feedback_Temp(1),0);
			}
			
			
			 
			// trapezoidal integration of angular rate values
			dAB = (0.5*(prev_gyrs + gyrs)+feedback_out)*Ts;
			
			coning = myCross((1/12)*prev_dAB,dAB);			// coning calculation
			rotvB = dAB + coning;					// last step in calculating rotation vector (t(k-1) -> t(k))
			
			
			
			// calculate BODY to BODY (t(k-1) -> t(k)) quaternion
			if (rotvB == Vec3fCol(0,0,0)) Q_b2b = quat(1, 0, 0, 0);		// Correction to avoid division by 0
			else Q_b2b = rotv2quat(rotvB);
			
			if(isnan(Q_b2b(0)) || isnan(Q_b2b(1)) || isnan(Q_b2b(2)) || isnan(Q_b2b(3))) 
			{
				cout << "rotv2quat nan" << endl; 
				ros::shutdown(); 
			}

			// Computation of quaternion from BODY to NED
			Q_b2n = quatmultiply(prev_Q_b2n, Q_b2b);


			// Extraction and saving of attitude angles Q_b2n for time tk:
			EUL = rad2deg(quat2euler(Q_b2n));	// Yaw range 0 - 360 or +/- 180
			
			
			// check for nans, in that case, fatal
			if(isnan(EUL(0)) || isnan(EUL(1)) || isnan(EUL(2))) 
			{ 
				cout << "quat2euler nan" << endl;  ros::shutdown(); 
			}

			
			// constraint on yaw
			if(switch_useconstraints && tracks_vel_diff < tracks_vel_diff_tresh)
			{
				EUL(2) = prev_EUL(2);
				Q_b2n = euler2quat(deg2rad(EUL));

			}


			C_b2n = (quat2dcm(Q_b2n)).t(); 		// Computation of body->NED DCM
			
			// check for nans in C matrix
			for(int i = 0; i <3; i++){
				for(int u = 0; u <3; u++){
					if(isnan(C_b2n(i,u)))
					{ 
						cout << "quat2dcm nan:" << i <<","<< u << endl;  
						ros::shutdown(); 
					}	
				}
			}
			

			/* -------------------------------------------------------------------------
			 * ATTITUDE MECHANIZATION - ACC
			 * -------------------------------------------------------------------------
			 */
			
			
			accs = accs - aB; 									// bias correction
			accs = aSF*(aMA*accs);								// scale factor and misalignment correction

			// calculate moving averages for ACCs and ARRs
			Vec3fCol mACC(myMean(buf_accx,avgspan),
							myMean(buf_accy,avgspan),
							myMean(buf_accz,avgspan));

			Vec3fCol mGYR(myMean(buf_gyrx,avgspan),
							myMean(buf_gyry,avgspan),
							myMean(buf_gyrz,avgspan));

			// perform coarse alignment
			Vec3fCol EULacc = alignment_coarse(mACC(0), mACC(1), mACC(2), 
												mGYR(0), mGYR(1), mGYR(2), 
												iniLAT, gN(2), earth_rate);
			// check for nans in the coarse alignment
			if(isnan(EULacc(0)) || isnan(EULacc(1)) || isnan(EULacc(2))) 
			{ 
				cout << "coarse nan";  
				ros::shutdown(); 
			}


			/* in the case of weighted averaging, euler angles computed by attitude mechanization
			 * and by coarse alignment are weighted by their weights (w_gyr, w_acc) */
			//if (fbtype == ATT_GYR_ACC_WEIGHTED_AVG)
			//{
			//	EUL(0) = w_gyr*EUL(0) + w_acc*EULacc(0);
			//	EUL(1) = w_gyr*EUL(1) + w_acc*EULacc(1);
			//}
			
			// set the current computed values as previus for the next period
			prev_EUL = EUL;
			prev_EULacc = EULacc;

			// calculate quaternion describin transformation body -> NED and corresponding DCM
			//Q_b2n = euler2quat(deg2rad(EUL));
			//C_b2n = (quat2dcm(Q_b2n)).t();

			// set the current values to previous for the next period
			prev_Q_b2n = Q_b2n;
			prev_dAB = dAB;
			prev_gyrs = gyrs;

			
			/*
			 *---------------------------------------------------------------------------------------------------
			 * ABSOLUTE MODEL UPDATE
			 *---------------------------------------------------------------------------------------------------
			 */
			
			// specific forces
			sf_x = accs(0);
			sf_y = accs(1);
			sf_z = accs(2);
			
			// angular rates
			ar_x = gyrs(0);
			ar_y = gyrs(1);
			ar_z = gyrs(2);
			
			// speeds in BODY
			vBx = previous_MM(3);
			vBy = previous_MM(4);
			vBz = previous_MM(5);
			
			// angles taken from attitute mechanization, saved into MM
			phi = deg2rad(EUL(0));
			th  = deg2rad(EUL(1));
			psi = deg2rad(EUL(2));
			previous_MM(6) = phi;
			previous_MM(7) = th;
			previous_MM(8) = psi;
			
			// prepare cosinus and sinus values for the nonlinear time update
			evaluate_harmonics(harmonics,phi,th,psi);
			
			
			// evaluate fun, dfundx and dfundu
			
			fill_fun(fun,sf_x,sf_y,sf_z,ar_x,ar_y,ar_z,vBx,vBy,vBz,harmonics,C_b2n,gN);
			fill_dfun(dfundx,dfundu ,ar_x,ar_y,ar_z,vBx,vBy,vBz,harmonics,gN);
			
			// discretized system equation
			funk = previous_MM + fun * Ts;			
			
			// ZOH discretization of dfundx
			Fk = I_99 + dfundx * Ts;
			
			// covariance matrix Q discretization (L = identity)
			Qk = Qt * Ts;
			
			
			
			
			/*
			 *---------------------------------------------------------------------------------------------------
			 * EXTENDED KALMAN STEP
			 *---------------------------------------------------------------------------------------------------
			 */
			
			Vec3fCol velocity_in_NED;
			
			// check odometry for a new measurement
			if(odometry_ready)
			{
			  
			  odometry_ready = false;
			  
			  //aiding measurementz
			  ZZ(0) = PN_aid(0); ZZ(1) = PN_aid(1); ZZ(2) = PN_aid(2);
			  ZZ(3) = VN_aid(0); ZZ(4) = VN_aid(1); ZZ(5) = VN_aid(2);
			  
			  // PREDICTION STEP
			  ekf_time_update(MM, PP , funk , Fk , Qk);
			  
			  // MEASUREMENT UPDATE
			  // measurement equation (hk) update
			  
			  hk(0) = MM(0);
			  hk(1) = MM(1);
			  hk(2) = MM(2);
			  
			  velocity_in_NED = C_b2n * Vec3fCol(MM(3),MM(4),MM(5));
			  
			  hk(3) = velocity_in_NED(0);
			  hk(4) = velocity_in_NED(1);
			  hk(5) = velocity_in_NED(2);
			  
			  // measurement matrix
			  
			  Hk = Mat69f::zeros();
			    // eye 3x3
			    Hk(0,0) = 1; Hk(1,1) = 1; Hk(2,2) = 1;
			    // C_b2n
			    Hk(3,3) = C_b2n(0,0);	Hk(3,4) = C_b2n(0,1);		Hk(3,5) = C_b2n(0,2); 
			    Hk(4,3) = C_b2n(1,0);	Hk(4,4) = C_b2n(1,1);		Hk(4,5) = C_b2n(1,2);
			    Hk(5,3) = C_b2n(2,0);	Hk(5,4) = C_b2n(2,1);		Hk(5,5) = C_b2n(2,2);
			  
			  ekf_measurement_update(MM, PP , ZZ , hk , Hk , Rk);
			    
			  
			}
			else // no odometry measurement avaible, we leave the prediction step vectors untouched
 			{
			  // PREDICTION STEP
			  ekf_time_update(MM, PP , funk , Fk , Qk);
			  
			}
			
			
			
			// save current MM and PP for next run
			previous_MM = MM;
			previous_PP = PP;
			
			
			
			/*
			 *---------------------------------------------------------------------------------------------------
			 * OUTPUT LOGGING
			 *---------------------------------------------------------------------------------------------------
			 */
			
			
			if (bMechanizationEnabled)
			{
				// create the line of the output file
				if (internalLogged)
				{
					sprintf(text,"%d.%09d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",
							msg.header.stamp.sec, msg.header.stamp.nsec ,IMUdata(0),IMUdata(1),
							IMUdata(2),IMUdata(3), IMUdata(4),IMUdata(5),EUL(0),EUL(1),EUL(2),
							IMUdata(6),IMUdata(7),IMUdata(8));
				}
				else
				{
					sprintf(text,"%d.%09d, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",
							msg.header.stamp.sec, msg.header.stamp.nsec,IMUdata(0),IMUdata(1),IMUdata(2),IMUdata(3),
							IMUdata(4),IMUdata(5),EUL(0),EUL(1),EUL(2));
				}			
			} 
			// save the line into the file 
			outputFile << text;
				
			if (bVpaEnabled)
			{
					//                     vx  vy  vz |vel. var.  | x   y   z| pos.var    |euler     | euler var | VN_aid     | PN_aid 
					sprintf(text,"%d.%09d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
							msg.header.stamp.sec, msg.header.stamp.nsec,
							MM(3),MM(4),MM(5), PP(3,3), PP(4,4), PP(5,5), // velocity and its variance
							MM(0),MM(1),MM(2), PP(0,0), PP(1,1), PP(2,2), // position and its variance
							EUL(0),EUL(1),EUL(2),-1.0, -1.0, -1.0, // attitude in euler and its variance (note that these are evaluated separately from Kalman
							VN_aid(0), VN_aid(1), VN_aid(2),	      // velocity aiding vector
							PN_aid(0), PN_aid(1), PN_aid(2)	              // position aiding vector
						); 
				
			}	
			// save the line into the file
			outputVpaFile << text;
			
			break;
	}

	// build output message and publish it
	//output = inso_outputMessage(EUL, MM);
	
	// VK:DEBUG AIDING
	//output_aiding.position.x = PN_aid(0);
	//output_aiding.position.y = PN_aid(1);
	//output_aiding.position.z = PN_aid(2);
	
	//output_aiding.velocity.x = VN_aid(0);
	//output_aiding.velocity.y = VN_aid(1);
	//output_aiding.velocity.z = VN_aid(2);
	
	//output_aiding.euler.x = prev_EULacc(0);
	//output_aiding.euler.y = prev_EULacc(1);
	//output_aiding.euler.z = prev_EULacc(2);
	
	
	// quaternion to be published
	q_imu_msg_att = euler2quat(Vec3fCol(PI*EUL(0)/180,-PI*EUL(1)/180,-PI*EUL(2)/180));
    	
	
	// FP: build the odometry structure, all uninitialized value will be zeros
			
			odom_msg.header.stamp = ros::Time::now(); //FP: is that the good time?
			odom_msg.header.frame_id = odom_frame;
			odom_msg.child_frame_id = robot_frame;
			//FP: set the orientation
			odom_msg.pose.pose.orientation.w = q_imu_msg_att(0);
			odom_msg.pose.pose.orientation.x = q_imu_msg_att(1);
			odom_msg.pose.pose.orientation.y = q_imu_msg_att(2);
			odom_msg.pose.pose.orientation.z = q_imu_msg_att(3);
			
			//VK: set the position
			
			Position_in_NWU = C_NED_2_NWU * Vec3fCol(MM(0),MM(1),MM(2));
			
			odom_msg.pose.pose.position.x =  Position_in_NWU(0);
			odom_msg.pose.pose.position.y =  Position_in_NWU(1);
			odom_msg.pose.pose.position.z =  Position_in_NWU(2);
			
			// set the position covariance
			for(int u = 0; u < 36; u++) odom_msg.pose.covariance[u] = 0; // initialize to zero
			
			for(int u = 0; u < 3; u++)	// go through first three rows
			{
			  for(int i = 0; i < 3; i++)	// go through first three columns
			  {
			    odom_msg.pose.covariance[u*6+i] = PP(u,i); // copy the whole position covariance subbatrix from PP
			  }
			}
			
			// leaving angle variances as they are, this is actually quite confusing, 4x4 covar. matrix expected for quaternion,
			// while odom message contains 6x6 covar. matrix  (x,y,z, and some three angles ??)
			const double angCov = 0.2;
						
			odom_msg.pose.covariance[3*6+3] = angCov;
			odom_msg.pose.covariance[4*6+4] = angCov;
			odom_msg.pose.covariance[5*6+5] = angCov;
			
			//set velocity			
			
			Velocity_in_NWU = C_NED_2_NWU * Vec3fCol(MM(3),MM(4),MM(5));
			
			odom_msg.twist.twist.linear.x =  Velocity_in_NWU(0);
			odom_msg.twist.twist.linear.y =  Velocity_in_NWU(1);
			odom_msg.twist.twist.linear.z =  Velocity_in_NWU(2);
			
			odom_msg.twist.twist.angular.x = IMUdata(3);
			odom_msg.twist.twist.angular.y = -IMUdata(4);
			odom_msg.twist.twist.angular.z = -IMUdata(5);
			
			// set linear velocity covariance matrix
			for(int u = 0; u < 36; u++) odom_msg.twist.covariance[u] = 0; // initialize to zero
			
			for(int u = 0; u < 3; u++)	// go through first three rows
			{
			  for(int i = 0; i < 3; i++)	// go through first three columns
			  {
			    odom_msg.twist.covariance[u*6+i] = PP(u+3,i+3); // copy the whole velocity covariance subbatrix from PP
			  }
			}  
			
			// set angular rates variances
			odom_msg.twist.covariance[3*6+3] = sigma_ang_x*sigma_ang_x;
			odom_msg.twist.covariance[4*6+4] = sigma_ang_y*sigma_ang_y;
			odom_msg.twist.covariance[5*6+5] = sigma_ang_z*sigma_ang_z;
			
			
			  
			// FP: publish odometry 
			if(iState==MEAS_STATE_MECHANIZATION)
			{
				odom_pub.publish(odom_msg);
			}
	
	
	if(iState==MEAS_STATE_MECHANIZATION)
	{
		// FP: this cause conflic in the tf tree
		transform.setRotation(tf::createQuaternionFromRPY(PI*EUL(0)/180,-PI*EUL(1)/180,-PI*EUL(2)/180));
		transform.setOrigin( tf::Vector3(Position_in_NWU(0), Position_in_NWU(1), Position_in_NWU(2)) );
		br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, odom_frame, robot_frame));
	
	
		//data_pub.publish(output);
		//imu_aiding_debug.publish(output_aiding);
	}
	
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	SharedObjects ImuObject;
	while(ros::ok()) ros::spin();
	
	ImuObject.outputFile.close();

	return 0;
}



void   SharedObjects::getROSParameters()
{

	string input_matrix = "";
	Mat33f eye3 = Mat33f(1,0,0,
				  0,1,0,
				  0,0,1);
	Vec3fCol zeros = Vec3fCol(0,0,0);
	Vec3fCol temp_vector = zeros;

		
	// Load number of samples of static data used for averaging during initial alignment:

	stat_len = 500;
	if (ros::param::has("inso_InitialAlignmentSamples")){

		if(!(ros::param::get("inso_InitialAlignmentSamples", stat_len)))
		{
			ROS_ERROR("Parameter \"inso_InitialAlignmentSamples\" is not an integer. Setting to 500");
			stat_len = 500;			
		}else
		{
			if(stat_len <= 0)
			{
				ROS_ERROR("Parameter \"inso_InitialAlignmentSamples\" was set to zero or less. Setting to 500");
				stat_len = 500;
			}
		}
	}else{
		ROS_INFO("Parameter inso_InitialAlignmentSamples not found, setting to 500");
	}
	
	
	// Load ACC & GYR fusion: 0=ATT_NO_FEEDBACK=disable, 1=ATT_GYR_ACC_WEIGHTED_AVG=weighted average,  2=ATT_FILTERED=filtering, default: filtering 

	fbtype = ATT_FILTERED;

	if (ros::param::has("inso_AccGyrFusion")){

		if(!(ros::param::get("inso_AccGyrFusion", fbtype)))
		{
			ROS_ERROR("Parameter \"inso_AccGyrFusion\" is not an integer. Setting to ATT_FILTERED (2)");
			fbtype = ATT_FILTERED;		
		}else
		{
			if( (fbtype != ATT_NO_FEEDBACK) && (fbtype != ATT_GYR_ACC_WEIGHTED_AVG) && (fbtype != ATT_FILTERED) )
			{
				ROS_ERROR("Parameter \"inso_AccGyrFusion\" has an invalid value. Setting to ATT_FILTERED (2)");
				fbtype = ATT_FILTERED;
			}
		}
	}else{
		ROS_INFO("Parameter inso_AccGyrFusion not found, setting to 2 (Filtered feedback)");
	}
	


	// Load Gyroscope based quaternion weight (0-1)
	
	w_gyr = 0.999;

	if (ros::param::has("inso_WeightedFusionGyroscopeWeight")){

		if(!(ros::param::get("inso_WeightedFusionGyroscopeWeight", w_gyr)))
		{
			ROS_ERROR("Parameter \"inso_WeightedFusionGyroscopeWeight\" is not float. Setting to 0.999");
			w_gyr = 0.999;			
		}else
		{
			if((w_gyr < 0) || (w_gyr > 1))
			{
				ROS_ERROR("Parameter \"inso_WeightedFusionGyroscopeWeight\" was not in <0,1>. Setting to 0.999");
				w_gyr = 0.999;
			}
		}
	}else{
		ROS_INFO("Parameter inso_WeightedFusionGyroscopeWeight not found, setting to 0.999");
	}


	// Determine Accelerometer based quaternion 
	
	w_acc = 1 - w_gyr;
	

	
	// Load moving average filter span

	avgspan = 25;

	if (ros::param::has("inso_MovingAverageFilterSpan")){

		if(!(ros::param::get("inso_MovingAverageFilterSpan", avgspan)))
		{
			ROS_ERROR("Parameter \"inso_MovingAverageFilterSpan\" is not integer. Setting to 25");
			avgspan = 25;			
		}else
		{
			if(avgspan <= 0)
			{
				ROS_ERROR("Parameter \"inso_MovingAverageFilterSpan\" was set equal or less than zero. Setting to 25");
				avgspan = 25;
			}
		}
	}else{
		ROS_INFO("Parameter inso_MovingAverageFilterSpan not found, setting to 25");
	}



	// Load sampling period

	Ts = 0.01;

	if (ros::param::has("inso_SamplingPeriod")){

		if(!(ros::param::get("inso_SamplingPeriod", Ts)))
		{
			ROS_ERROR("Parameter \"inso_SamplingPeriod\" is not a float. Setting to 0.01");
			Ts = 0.01;			
		}else
		{
			if(Ts <= 0)
			{
				ROS_ERROR("Parameter \"inso_SamplingPeriod\" was set equal or less than zero. Setting to 0.01");
				Ts = 0.01;
			}
		}
	}else{
		ROS_INFO("Parameter inso_SamplingPeriod not found, setting to 0.01");
	}


	// Load Gps Coordinates

	iniLAT = 0;
	iniLON = 0;

	if (ros::param::has("inso_InitialLatitude") && ros::param::has("inso_InitialLongitude")){

		if(!(ros::param::get("inso_InitialLatitude", iniLAT)))
		{
			ROS_ERROR("Parameter \"inso_InitialLatitude\" is not a float. Setting to 0");
			iniLAT = 0;		
		}else
		{
			if((iniLAT < -90) || (iniLAT > 90))
			{
				ROS_ERROR("Parameter \"inso_InitialLatitude\" was a nonsense. Setting to 0");
				iniLAT = 0;
			}
		}
	
	
		if(!(ros::param::get("inso_InitialLongitude", iniLON)))
		{
			ROS_ERROR("Parameter \"inso_InitialLongitude\" is not a float. Setting to 0");
			iniLON = 0;
		}else
		{
			if((iniLON < -180) || (iniLON > 180))
			{
				ROS_ERROR("Parameter \"inso_InitialLongitude\" was a nonsense. Setting to 0");
				iniLON = 0;
			}
		}
	}else{
		ROS_INFO("Parameters inso_InitialLatitude or inso_InitialLongitude not found, setting to 0 and 0");
	}


	// Load Altitude	
	
	iniALT = 0;

	if (ros::param::has("inso_InitialAltitude")){

		if(!(ros::param::get("inso_InitialAltitude", iniALT)))
		{
			ROS_ERROR("Parameter \"inso_InitialAltitude\" is not a float. Setting to 0");
			iniALT = 0;			
		}else
		{
			if(iniALT < -6378000)
			{
				ROS_ERROR("Parameter \"inso_InitialAltitude\" was lower than Earth's diameter. Setting to 0");
				iniALT = 0;
			}
		}
	}else{
		ROS_INFO("Parameter inso_InitialAltitude not found, setting to 0");
	}



	
	// Accelerometer calibration matrices import from parameters	
	
	aSF = eye3;	

	if (ros::param::has("inso_AccelerometerSF")){

		if(!(ros::param::get("inso_AccelerometerSF", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_AccelerometerSF\" is not a string. Setting to a unit matrix");
			aSF = eye3;	
		}else
		{
			if(parseMatrixString(aSF,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_AccelerometerSF\" could not be parsed (check format). Setting to a unit matrix");
				aSF = eye3;	
			}
			else{
			//	std::cout << aSF(0) << ',' << aSF(1) << ',' << aSF(2) << ',' << aSF(3) << ',' << aSF(4) << ',' << aSF(5) << ','
			//	<< aSF(6) << ',' << aSF(7) << ',' << aSF(8) << endl;
			}
		}
	}else{
		ROS_INFO("Parameter inso_AccelerometerSF not found, setting to a unit matrix");
	}
	
	
	
	aMA = eye3;	
	

	if (ros::param::has("inso_AccelerometerMA")){

		if(!(ros::param::get("inso_AccelerometerMA", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_AccelerometerMA\" is not a string. Setting to a unit matrix");
			aMA = eye3;	
				
		}else
		{
			if(parseMatrixString(aMA,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_AccelerometerMA\" could not be parsed (check format). Setting to a unit matrix");
				aMA = eye3;	
			}
			else{
				//std::cout << aMA(0) << ',' << aMA(1) << ',' << aMA(2) << ',' << aMA(3) << ',' << aMA(4) << ',' << aMA(5) << ','
				//<< aMA(6) << ',' << aMA(7) << ',' << aMA(8) << endl;
			}
		}
	}else{
		ROS_INFO("Parameter inso_AccelerometerMA not found, setting to a unit matrix");
	}
	
	
	aB =  zeros;	
	

	if (ros::param::has("inso_AccelerometerB")){

		if(!(ros::param::get("inso_AccelerometerB", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_AccelerometerB\" is not a string. Setting to a zero vector.");
			aB = zeros;
		}else
		{
			if(parseVectorString(aB,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_AccelerometerB\" could not be parsed (check format). Setting to a zero vector");
				aB = zeros;	
			}
			else{
				//std::cout << aB(0) << ',' << aB(1) << ',' << aB(2) <<  endl;
			}
		}
	}else{
		ROS_INFO("Parameter inso_AccelerometerB not found, setting to a zero vector");
	}
	
	
	// Gyroscope calibration matrices import from parameters
	
	
	gSF = eye3;	

	if (ros::param::has("inso_GyroscopeSF")){

		if(!(ros::param::get("inso_GyroscopeSF", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_GyroscopeSF\" is not a string. Setting to a unit matrix");
			gSF = eye3;	
		}else
		{
			if(parseMatrixString(gSF,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_GyroscopeSF\" could not be parsed (check format). Setting to a unit matrix");
				gSF = eye3;	
			}
			else{
				//std::cout << gSF(0) << ',' << gSF(1) << ',' << gSF(2) << ',' << gSF(3) << ',' << gSF(4) << ',' << gSF(5) << ','
				//<< gSF(6) << ',' << gSF(7) << ',' << gSF(8) << endl;
			}
		}
	}else{
		ROS_INFO("Parameter inso_GyroscopeSF not found, setting to a unit matrix");
	}
	
	
	gMA = eye3;	

	if (ros::param::has("inso_GyroscopeMA")){

		if(!(ros::param::get("inso_GyroscopeMA", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_GyroscopeMA\" is not a string. Setting to a unit matrix");
			gMA = eye3;	
		}else
		{
			if(parseMatrixString(gMA,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_GyroscopeMA\" could not be parsed (check format). Setting to a unit matrix");
				gMA = eye3;	
			}
			else{
				//std::cout << gMA(0) << ',' << gMA(1) << ',' << gMA(2) << ',' << gMA(3) << ',' << gMA(4) << ',' << gMA(5) << ','
				//<< gMA(6) << ',' << gMA(7) << ',' << gMA(8) << endl;
			}
		}
	}else{
		ROS_INFO("Parameter inso_GyroscopeMA not found, setting to a unit matrix");
	}
	
	
	gB =  zeros;	
	

	if (ros::param::has("inso_GyroscopeB")){

		if(!(ros::param::get("inso_GyroscopeB", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_GyroscopeB\" is not a string. Setting to a zero vector.");
			gB = zeros;
		}else
		{
			if(parseVectorString(gB,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_GyroscopeB\" could not be parsed (check format). Setting to a zero vector");
				gB = zeros;	
			}
			else{
				//std::cout << gB(0) << ',' << gB(1) << ',' << gB(2) <<  endl;
			}
		}
	}else{
		ROS_INFO("Parameter inso_GyroscopeB not found, setting to a zero vector");
	}
	
	
	
	// Load initial YAW
				
	yaw = 0;
	yawSET = false;

	if (ros::param::has("inso_InitialYaw")){

		if(!(ros::param::get("inso_InitialYaw", yaw)))
		{
			ROS_ERROR("Parameter \"inso_InitialYaw\" is not a float. The value will not be used");
			yaw = 0;			
		}else
		{
			if(yaw != 666)
			{
				yawSET = true;
			}else{
				ROS_INFO("Parameter inso_InitialYaw set to 666, internal IMU value will be used");
			}
		}
	}else{
		ROS_INFO("Parameter inso_InitialYaw not found, internal IMU value will be used");
	}
				
	// Load parameter enabling logging of XSENS evaluated X,Y,Z,attitude
	
	internalLogged = false;

	if (ros::param::has("inso_InternalLogged")){

		if(!(ros::param::get("inso_InternalLogged", internalLogged)))
		{
			ROS_ERROR("Parameter \"inso_InternalLogged\" is not a boolean. Setting to false");
			internalLogged = false;			
		}
		
	}else{
		ROS_INFO("Parameter inso_InternalLogged not found, setting to false");
	}
	
	// Load parameter enabling usage of odometry constraints
	
	switch_useconstraints = false;		// set to enable odometry constraints
	
	if (ros::param::has("inso_UseConstraints")){

		if(!(ros::param::get("inso_UseConstraints", switch_useconstraints)))
		{
			ROS_ERROR("Parameter \"inso_UseConstraints\" is not a boolean. Setting to false");
			switch_useconstraints = false;			
		}
		
	}else{
		ROS_INFO("Parameter inso_UseConstraints not found, setting to false");
	}
	
	// Load stride scale
	
	stride_scale = 0.88;			// odometry scale factor
	
	if (ros::param::has("inso_StrideScale")){

		if(!(ros::param::get("inso_StrideScale", stride_scale)))
		{
			ROS_ERROR("Parameter \"inso_StrideScale\" is not a number. Setting to 0.88");
			stride_scale = 0.88;			
		}
		
	}else{
		ROS_INFO("Parameter inso_StrideScale not found, setting to 0.88");
	}
	
	// Load initial attitude angle variance (one used for all three angles
	
	initial_attitude_variance = 5;
	
	if (ros::param::has("inso_InitialAttitudeVariance")){

		if(!(ros::param::get("inso_InitialAttitudeVariance", initial_attitude_variance)))
		{
			ROS_ERROR("Parameter \"inso_InitialAttitudeVariance\" is not a number. Setting to 5");
			initial_attitude_variance = 5;			
		}
		
	}else{
		ROS_INFO("Parameter inso_InitialAttitudeVariance not found, setting to 5");
	}
	
	// Load accelerometer standart deviations
	
	sigma_acc_x = 0.0146; sigma_acc_y = 0.0241; sigma_acc_z = 0.0151;	// IMU standart deviations, accelerometers
	
	if (ros::param::has("inso_SigmaAcc")){

		if(!(ros::param::get("inso_SigmaAcc", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_SigmaAcc\" is not a string. Setting to default values.");
		}else
		{
			if(parseVectorString(temp_vector,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_SigmaAcc\" could not be parsed (check format). Setting to default values.");	
			}
			else{
				sigma_acc_x = temp_vector(0); sigma_acc_y = temp_vector(1); sigma_acc_z = temp_vector(2);
			}
		}
	}else{
		ROS_INFO("Parameter inso_SigmaAcc not found, setting to default values");
	}
	
	// Load gyroscopes standart deviations
	
	sigma_ang_x = 0.0062; sigma_ang_y = 0.0055; sigma_ang_z = 0.0056;	// IMU standart deviations, gyros 
	
	if (ros::param::has("inso_SigmaAng")){

		if(!(ros::param::get("inso_SigmaAng", input_matrix)))
		{
			ROS_ERROR("Parameter \"inso_SigmaAng\" is not a string. Setting to default values.");
		}else
		{
			if(parseVectorString(temp_vector,input_matrix))
			{
				ROS_ERROR("Parameter \"inso_SigmaAng\" could not be parsed (check format). Setting to default values.");	
			}
			else{
				sigma_ang_x = temp_vector(0); sigma_ang_y = temp_vector(1); sigma_ang_z = temp_vector(2);
			}
		}
	}else{
		ROS_INFO("Parameter inso_SigmaGyr not found, setting to default values");
	}
	
	// Load odometry position standart deviation
	
	aid_sigma_pos = 1;
	
	if (ros::param::has("inso_AidSigmaPos")){

		if(!(ros::param::get("inso_AidSigmaPos", aid_sigma_pos)))
		{
			ROS_ERROR("Parameter \"inso_AidSigmaPos\" is not a number. Setting to 1");
			aid_sigma_pos = 5;			
		}
		
	}else{
		ROS_INFO("Parameter inso_AidSigmaPos not found, setting to 1");
	}
	
	// Load odometry velocity standart deviation
	
	aid_sigma_vel = 1; 
	
	if (ros::param::has("inso_AidSigmaVel")){

		if(!(ros::param::get("inso_AidSigmaVel", aid_sigma_vel)))
		{
			ROS_ERROR("Parameter \"inso_AidSigmaVel\" is not a number. Setting to 1");
			aid_sigma_vel = 5;			
		}
		
	}else{
		ROS_INFO("Parameter inso_AidSigmaVel not found, setting to 1");
	}
	
}

void SharedObjects::printInitialParameters(){
	
	std::cout << "Initial parameters:" << std::endl;
	std::cout << "\t" << "number of samples of static data used for averaging during initial alignment: " << stat_len << std::endl;
	std::cout << "\t" << "gyr weight value for averaging euler angles from gyr and acc: " << w_gyr << std::endl;
	std::cout << "\t" << "acc weight value for averaging euler angles from gyr and acc: " << w_acc << std::endl;
	std::cout << "\t" << "span for acceleration moving average filtering: " << avgspan << std::endl;
	std::cout << "\t" << "sampling period: " << Ts << std::endl;
	std::cout << "\t" << "init latitude: " << iniLAT << std::endl;
	std::cout << "\t" << "init longitude: " << iniLON << std::endl;
	std::cout << "\t" << "init altitude: " << iniALT << std::endl;
	std::cout << "\t" << "init yaw: " << yaw << std::endl;
	std::cout << "\t" << "is yaw used: " << yawSET << std::endl;
	std::cout << "\t" << "logging of mtig xyzAtt: " << internalLogged << std::endl;
	std::cout << "\t" << "feedback type: " << fbtype << std::endl;
	std::cout << "\t" << "accelerometer Scale Factor matrix: ";
	printMatrix33 (aSF);
	
	std::cout << "\t" << "accelerometer MisAlignment matrix: ";
	printMatrix33 (aMA);
	
	std::cout << "\t" << "accelerometer Biases: ";
	printVector3 (aB);
	
	std::cout << "\t" << "gyroscope Scale Factor matrix: ";
	printMatrix33 (gSF);
	
	std::cout << "\t" << "gyroscope MisAlignment matrix: ";
	printMatrix33 (gMA);
	
	std::cout << "\t" << "gyroscope Biases: ";
	printVector3 (gB);
	
	std::cout << "\t" << "use constraints: " << switch_useconstraints << endl;
	std::cout << "\t" << "stride scale: " << stride_scale << endl;
	std::cout << "\t" << "initial attitude variance: " << initial_attitude_variance << endl;
	
	std::cout << "\t" << "accelerometer standart deviations: ";
	printVector3 (Vec3fCol(sigma_acc_x,sigma_acc_y,sigma_acc_z));
	
	std::cout << "\t" << "gyroscope standart deviations: ";
	printVector3 (Vec3fCol(sigma_ang_x,sigma_ang_y,sigma_ang_z));
	
	std::cout << "\t" << "aiding position standart deviation: " << aid_sigma_pos << endl;
	std::cout << "\t" << "aiding velocity standart deviation: " << aid_sigma_vel << endl;
	
	
	
	
	

}


void printMatrix33 (Mat33f matrix){
	std::cout << std::endl;
	for(int i = 0; i < 3; i ++)
	{		
		std::cout << "\t" << "|" << matrix(i*3) << "\t" << matrix(i*3+1) << "\t" << matrix(i*3+2) << "|" << std::endl;		
	}
	std::cout << std::endl;
}

void printVector3 (Vec3fCol vector){
	std::cout << std::endl;	
	std::cout << "\t" << "|" << vector(0) << "\t" << vector(1) << "\t" << vector(2) << "|'" << std::endl;		
	std::cout << std::endl;
}


