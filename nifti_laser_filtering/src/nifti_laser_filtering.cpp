#include <math.h>
#include <vector>
#include <list>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}


class NiftiLaserFiltering {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	NiftiLaserFiltering();

	virtual ~NiftiLaserFiltering();

protected:

	//! /tf listener
	tf::TransformListener tf_listener;

	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	//! Name of the laser frame (default: "/laser")
	std::string laser_frame;

	//! Name of the robot frame (default: "/base_link")
	std::string robot_frame;

	//! Name of the smooth reference world frame (default: "/odom")
	std::string world_frame;

	//! Subscriber to laser scans (default topic: "/scan")
	ros::Subscriber laser_scan_sub;

	//! Publisher for the filtered scans (default topic: "/scan_filtered")
	ros::Publisher scan_filtered_pub;

	//! Scan to filter and modify
	sensor_msgs::LaserScan tmp_scan;

	//! Min distance to keep point (default: 0)
	double min_distance;

	//! eps inflation parameter (default: 0.015): The shape of the robot is inflated by this value to decide whether a point belongs to robot's body (part) or not.
	double eps;

	//! Time offset (default: 0)
	ros::Duration time_offset;

	//! tan of min_angle to do shadow filtering (default: tan(0.1))
	const double tan_shadow_filter_min_angle;

	//! flag indicating if there is an arm or not
	bool has_arm;

	//! number of failures to get any arm tf
	int arm_failures;

	//! Time correction plus setting ranges
	void time_correct(sensor_msgs::LaserScan& scan);

	//! Distance distortion correction
	void undistort(sensor_msgs::LaserScan& scan);

	//! Scan filtering function
	void shadow_filter(sensor_msgs::LaserScan& scan);

	//! remove points on the robot
	void robot_filter(sensor_msgs::LaserScan& scan);

	//! Scan callback function
	void scan_cb(const sensor_msgs::LaserScan& scan);

};


/*
 * Constructor
 */
NiftiLaserFiltering::NiftiLaserFiltering():
	tf_listener(ros::Duration(60.)),
	n_("~"),
	tan_shadow_filter_min_angle(tan(M_PI/2 -
				getParam<double>(n_, "shadow_filter_min_angle", 0.1))),
	has_arm(getParam<bool>(n, "has_arm", true)),
	arm_failures(0)
{
	// eps
	eps = getParam<double>(n_, "eps", 0.015);//0.015 is default value used in NIFTi project

	// frame names
	laser_frame = getParam<std::string>(n_, "laser_frame", "/laser");
	robot_frame = getParam<std::string>(n_, "robot_frame", "/base_link");
	world_frame = getParam<std::string>(n_, "world_frame", "/odom");

	// 2d scans
	scan_filtered_pub = n.advertise<sensor_msgs::LaserScan>("/scan_filtered", 50);

	// filtering
	double offset;
	offset = getParam<double>(n_, "time_offset", 0.0);
	time_offset = ros::Duration(offset);
	min_distance = getParam<double>(n_, "min_distance", 0.0);
	// initialized so that the first test always fails

	if (!tf_listener.waitForTransform(laser_frame, world_frame, ros::Time(0),
			ros::Duration(30.)))
		ROS_WARN_STREAM("Timeout (30s) while waiting between "<<laser_frame<<
				" and "<<world_frame<<" at startup.");

	// laser scan subscriber
	laser_scan_sub = n.subscribe("/scan", 250,
			&NiftiLaserFiltering::scan_cb, this);
}


/*
 * Destructor
 */
NiftiLaserFiltering::~NiftiLaserFiltering()
{
	// Nothing to do?
}

/*
 * Time and range correction
 */
void NiftiLaserFiltering::time_correct(sensor_msgs::LaserScan& scan){
	scan.header.stamp = scan.header.stamp + time_offset;
	scan.range_min = std::max(scan.range_min, (float)min_distance);
}

/*
 * Scan filtering
 */
void NiftiLaserFiltering::shadow_filter(sensor_msgs::LaserScan& scan)
{
	//const double invalid = scan.range_max+1;
	const float sin_gamma = sin(scan.angle_increment);
	const float cos_gamma = cos(scan.angle_increment);
	float x,y,r0,r1,smallest;
	const float r_min = scan.range_min;
	const float r_max = scan.range_max;

	const sensor_msgs::LaserScan scan_ref(scan);

	r0 = scan_ref.ranges[0];
	for (unsigned int i=1; i<scan_ref.ranges.size(); i++) {
		r1 = scan_ref.ranges[i];
		smallest = r1<r0?r1:r0;

		if ((r1>=r_min)&&(r1<=r_max)) {
			x = r0*sin_gamma;
			y = fabs(r1 - r0*cos_gamma);
			
			if (y > x*tan_shadow_filter_min_angle){
				scan.ranges[i-1] = -smallest;
				scan.ranges[i] = -smallest;
			}
			r0 = r1;
		}
	}
}

/* 
 * Undistort
 */
void NiftiLaserFiltering::undistort(sensor_msgs::LaserScan& scan)
{
	const double c1 = 0.03;
	const double d1 = 0.13;
	const double c2 = 0.008;
	const double d2 = 0.25;
	const double beta = (c1*d1*d1-c2*d2*d2)/(c2-c1);
	const double alpha = c1*(beta+d1*d1);
	//const double alpha = 0.000;
	//const double beta = 0.005;

	for (unsigned int i=0; i<scan.ranges.size(); i++) {
		if (scan.ranges[i]>scan.range_min) {
			scan.ranges[i] += alpha/(beta+scan.ranges[i]*scan.ranges[i]);
		}
	}
}

/*
 * Filter robot out of the current scan
 */
void NiftiLaserFiltering::robot_filter(sensor_msgs::LaserScan& scan)
{
	const double invalid = scan.range_max+1;
	std::list<double> blacklisted;
	sensor_msgs::PointCloud close_ptcld;
	sensor_msgs::PointCloud transformed_ptcld;
	close_ptcld.header = scan.header;
	sensor_msgs::ChannelFloat32 index;
	index.name = "index";
	geometry_msgs::Point32 point;
	point.z = 0;
	double angle;
	const ros::Time time = scan.header.stamp;

	double max_dist = 0.7;
	if (has_arm) { // when extended the end of the arm is further away
		max_dist = 1.4;
	}

	// create a small point cloud with only the close points
	for (unsigned int i=0; i<scan.ranges.size(); i++) {
		if (scan.ranges[i]<max_dist) {
			angle = scan.angle_min + i*scan.angle_increment;
			point.x = scan.ranges[i]*cos(angle);
			point.y = scan.ranges[i]*sin(angle);
			close_ptcld.points.push_back(point);
			index.values.push_back(i);
		}
	}
	close_ptcld.channels.push_back(index);
	//std::cout << close_ptcld.points.size() << std::endl;

	double x, y, z;
	// remove body
	transformed_ptcld = close_ptcld;
	for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
		x = transformed_ptcld.points[i].x;
		y = transformed_ptcld.points[i].y;
		z = transformed_ptcld.points[i].z;
		if ((fabs(y)<eps+0.15)&&(fabs(z)<eps+0.10)&&(x<eps+0.05))
			blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
	}
	// remove left track
	if (tf_listener.waitForTransform(laser_frame, "/left_track", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/left_track", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if
			((fabs(y)<eps+(0.097/2))&&(fabs(x)<eps+0.09+(0.4997/2))&&(z>-eps-0.0705)&&(z<eps+0.1095))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /left_track for robot_filter (ignoring this part).");
	}
	// remove left track
	if (tf_listener.waitForTransform(laser_frame, "/right_track", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/right_track", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if
			((fabs(y)<eps+(0.097/2))&&(fabs(x)<eps+0.09+(0.4997/2))&&(z>-eps-0.0705)&&(z<eps+0.1095))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /right_track for robot_filter (ignoring this part).");
	}

	// remove front left flipper
	if (tf_listener.waitForTransform(laser_frame, "/front_left_flipper", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/front_left_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<3*eps+0.090))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /front_left_flipper for robot_filter (ignoring this part).");
	}
	// remove front right flipper
	if (tf_listener.waitForTransform(laser_frame, "/front_right_flipper", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/front_right_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<3*eps+0.090))
					// Beware 3*eps !
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /front_right_flipper for robot_filter (ignoring this part).");
	}
	// remove rear left flipper
	if (tf_listener.waitForTransform(laser_frame, "/rear_left_flipper", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/rear_left_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<eps+0.090))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /rear_left_flipper for robot_filter (ignoring this part).");
	}
	// remove rear right flipper
	if (tf_listener.waitForTransform(laser_frame, "/rear_right_flipper", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/rear_right_flipper", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((fabs(y)<eps+(0.050/2))&&(x>-eps-0.090)&&(x<eps+0.3476)&&
					((x*sin(11.1*M_PI/180)+fabs(z)*cos(11.1*M_PI/180))<eps+0.090))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /rear_right_flipper for robot_filter (ignoring this part).");
	}
	
	// remove omnicam
	if (tf_listener.waitForTransform(laser_frame, "/omnicam", time,
			ros::Duration(0.01))) {
		tf_listener.transformPointCloud("/omnicam", close_ptcld, transformed_ptcld);
		for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
			x = transformed_ptcld.points[i].x;
			y = transformed_ptcld.points[i].y;
			z = transformed_ptcld.points[i].z;
			if ((x*x+y*y<(eps+0.070)*(eps+0.070))&&(fabs(z)<eps+(0.15/2)))
				blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
		}
	} else {
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and /omnicam for robot_filter (ignoring this part).");
	}

	// remove arm
	if (has_arm) {
		bool no_arm = true;
		// upper arm
		if (tf_listener.waitForTransform(laser_frame, "/shoulder", time,
				ros::Duration(0.01))) {
			no_arm = false;
			tf_listener.transformPointCloud("/shoulder", close_ptcld, transformed_ptcld);
			for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
				x = transformed_ptcld.points[i].x;
				y = transformed_ptcld.points[i].y;
				z = transformed_ptcld.points[i].z;
				if ((y*y+z*z<(eps+0.050/2)*(eps+0.050/2))&&(fabs(x-0.500/2)<eps+(0.500/2)))
					blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
			}
		} else {
			ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
					" and /shoulder for arm of robot_filter (ignoring this part).");
		}
		// elbow + forearm
		if (tf_listener.waitForTransform(laser_frame, "/elbow", time,
				ros::Duration(0.01))) {
			no_arm = false;
			tf_listener.transformPointCloud("/elbow", close_ptcld, transformed_ptcld);
			for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
				x = transformed_ptcld.points[i].x;
				y = transformed_ptcld.points[i].y;
				z = transformed_ptcld.points[i].z;
				// elbow
				if ((x*x+z*z<(eps+0.052/2)*(eps+0.052/2))&&(fabs(y+(0.122/2-0.029))<eps+(0.122/2)))
					blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
				// forearm
				if (((y+0.064)*(y+0.064)+z*z<(eps+0.050/2)*(eps+0.050/2))&&(fabs(x-0.450/2)<eps+(0.450/2)))
					blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
			}
		} else {
			ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
					" and /elbow for arm of robot_filter (ignoring this part).");
		}
		// pan tilt
		if (tf_listener.waitForTransform(laser_frame, "/ptu_joint", time,
				ros::Duration(0.01))) {
			no_arm = false;
			tf_listener.transformPointCloud("/ptu_joint", close_ptcld, transformed_ptcld);
			for (unsigned int i=0; i<transformed_ptcld.points.size(); i++) {
				x = transformed_ptcld.points[i].x;
				y = transformed_ptcld.points[i].y;
				z = transformed_ptcld.points[i].z;
				if ((x*x+y*y+z*z<(eps+0.120)*(eps+0.120)))
					blacklisted.push_back(transformed_ptcld.channels[0].values[i]);
			}
		} else {
			ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
					" and /ptu_joint for arm of robot_filter (ignoring this part).");
		}
		if (no_arm) {
			arm_failures += 1;
			if (arm_failures>50) {
				ROS_WARN_STREAM("Too many failures to find the arm /tf: disabling arm filtering.");
				has_arm = false;
			}
		} else {
			arm_failures = 0;
		}
	}
	

	// remove blacklisted points
	//std::cout << blacklisted.size() << " -> ";
	//blacklisted.sort();	// don't know if that's necessary
	//blacklisted.unique();
	//std::cout << blacklisted.size()<< std::endl;

	for (std::list<double>::const_iterator it=blacklisted.begin();it!=blacklisted.end();++it) {
		scan.ranges[int(*it)] = invalid;
	}

}


/*
 * laser scan callback
 */
void NiftiLaserFiltering::scan_cb(const sensor_msgs::LaserScan& scan)
{
	//ROS_INFO_STREAM("Got scan");
	tmp_scan = scan;
	time_correct(tmp_scan);
	undistort(tmp_scan);
	robot_filter(tmp_scan);
	shadow_filter(tmp_scan);

	scan_filtered_pub.publish(tmp_scan);
}



/*
 * Main function
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nifti_laser_filtering");

	NiftiLaserFiltering nlf;

	ros::spin();

	return 0;
}

