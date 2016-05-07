/** \brief Mock-up interface for the NIFTi robot.
*/
//#include "bluebotics_nifti_interface.h"
#include <librover/librover.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>

#ifdef VERBOSE
#define GETPRINT(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define GETPRINT(fmt, ...) {}
#endif
#ifdef SILENT
#define SETPRINT(fmt, ...) {}
#else
#define SETPRINT(fmt, ...) printf(fmt, ##__VA_ARGS__);
#endif

#define RETURN_ERROR {return (rand()>(int)(1.00*RAND_MAX))*(1+(int)(5.0*rand()/RAND_MAX));}

double robot_vl, robot_vr;
double frontLeft, frontRight, rearLeft, rearRight;
int brake_on;
double scanning_speed;
double laser_pos;
double last_laser_time;
RoverParams roverParams;
int flippersInitState;

/* fill in a structure with factory parameters (geometry of rover, min/max
values and position of dofs, xyzrpy du laser scanner, etc.). This
function can be called at any time, even before library initialization
*/
void nrGetDefaultParams(RoverParams* params)
{
	GETPRINT("nrGetDefaultParams(*params=%p)\n", params);
	params->trackDistance = 0.397;
	params->trackWheelRadius = 0.089;
	params->trackLength = 0.5;
	params->trackWidth = 0.097;
	params->flipperLength = 0.34;
	params->flipperWidth = 0.05;
	params->flipperOffset = 0.193731547;
	params->laserX = 0.2502;
	params->laserY = 0.0;
	params->laserZ = 0.1427;
	params->imuX = 0;
	params->imuY = 0;
	params->imuZ = 0.1694;
	params->omniX = 0.070;
	params->omniY = 0.068;
	params->omniZ = 0.2707;
	params->omniAngleOffset = 0.942477796;
	params->referentialX = 0.0;
	params->referentialY = 0.0;
	params->referentialZ = 0.0705;
	params->vMax = 0.6;
	params->wMax = 2.0 * params->vMax / params->trackDistance;
}

/* initialize the library with CAN_device path and rover parameters
*/
void nrInit(const char* CAN_device, RoverParams* params, int bestInit)
{
	SETPRINT("nrInit(CAN_device=\"%s\", *params=%p, bestInit=%d)\n", CAN_device,
			params, bestInit);
	if(params == NULL) nrGetDefaultParams(&roverParams);
	else memcpy(&roverParams, params, sizeof(RoverParams));
	robot_vl = 0.0;
	robot_vr = 0.0;
	frontLeft = 0.0;
	frontRight = 0.0;
	rearLeft = 0.0;
	rearRight = 0.0;
	brake_on = 1;
	scanning_speed = 0.0;
	laser_pos = 0.0;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	last_laser_time = (tv.tv_sec + 0.000001*tv.tv_usec);
	nrInitFlippers(bestInit);
}

/*  Initialise flippers. Return value: 1 = SUCCESS; 0 = ESTOP is pressed
*/
int nrInitFlippers(int bestInit)
{
	SETPRINT("nrInitFlippers(bestInit=%d)\n", bestInit);
	int estop;
	nrGetEstop(&estop);
	// mimicking structure from the library
	if (bestInit)
	{
		if (!estop)
		{
			frontLeft = -3*M_PI/4;
			frontRight = -3*M_PI/4;
			rearLeft = 3*M_PI/4;
			rearRight = 3*M_PI/4;
			flippersInitState = 1;
		} else 
		{
			flippersInitState = 0;
		}
	} else {
		flippersInitState = 1;
	}
	return flippersInitState;
}

/* Restart 3D laser scanner
*/
int nrRestart3D()
{
	SETPRINT("nrRestart3D()\n");
	scanning_speed = 0.0;
	laser_pos = 0.0;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	last_laser_time = (tv.tv_sec + 0.000001*tv.tv_usec);
	RETURN_ERROR;
}

/* Retrieve the current rover parameters.
*/
void nrGetRoverParams(RoverParams *params){
	memcpy(params, &roverParams, sizeof(RoverParams));
}

/* cleanup library
*/
void nrDestroy()
{
	SETPRINT("nrDestroy()\n");
}

/* Enable/disable rover controllers */
int nrEnable(int enable)
{
	SETPRINT("nrEnable(enable=%s)\n", enable?"true":"false");
	RETURN_ERROR;
}

/* set rover translational and rotational speeds
*/
int nrSetSpeed(double v, double w)
{
	SETPRINT("nrSetSpeed(v=%f, w=%f)\n", v, w);
	robot_vl = v - 0.2*w;
	robot_vr = v + 0.2*w;
	RETURN_ERROR;
}

/* get rover translational and rotational speeds
*/
int nrGetSpeed(double *v, double *w)
{
	*v = (robot_vl + robot_vr)/2.0;
	*w = (robot_vr - robot_vl)/0.4;
	GETPRINT("nrGetSpeed(*v=%f, *w=%f)\n", *v, *w);
	RETURN_ERROR;
}

/* set left and right tracks speeds
*/
int nrSetSpeedLR(double vl, double vr)
{
	SETPRINT("nrSetSpeedLR(vl=%f, vr=%f)\n", vl, vr);
	robot_vl = vl;
	robot_vr = vr;
	RETURN_ERROR;
}

/* return actual tracks speeds
*/
int nrGetSpeedLR(double* vl, double* vr)
{
	*vl = robot_vl;
	*vr = robot_vr;
	GETPRINT("nrGetSpeedLR(*vl=%f, *vr=%f)\n", *vl, *vr);
	RETURN_ERROR;
}

/* return angles of the last gear stage of the track. Useful to do odometry.
*/
int nrGetEncodersLR(int* left, int* right)
{
	*left = 0;
	*right = 0;
	GETPRINT("nrGetEncodersLR(*el=%d, *er=%d)\n", *left, *right);
	RETURN_ERROR;
}

// flipper stops a bit before, with some noise 
double update_flipper(double old_value, double new_value)
{
	double v = new_value;
	double bias = 1.5;
	// undershoot
	if (new_value<old_value)
		bias = 0.5;
	if (new_value>old_value)
		bias = 2.5;
	v += (3.0*rand()/RAND_MAX - bias)/180.0*M_PI;
	return v;
}

/* set flippers angles
*/
int nrSetFlippers(double fL, double fR, double rL,
		double rR)
{
	SETPRINT("nrSetFlippers(frontLeft=%f, frontRight=%f, rearLeft=%f, rearRight=%f)\n",
			fL, fR, rL, rR);
	frontLeft = update_flipper(frontLeft, fL);
	frontRight = update_flipper(frontRight, fR);
	rearLeft = update_flipper(rearLeft, rL);
	rearRight = update_flipper(rearRight, rR);
	RETURN_ERROR;
}

/* Set flipper torques [\a A] while conserving the positions of the flippers.
*/
int nrSetFlippersTorque(double frontTorque, double rearTorque)
{
	SETPRINT("nrSetFlippersTorque(frontTorque=%f, rearTorque=%f)\n",
			frontTorque, rearTorque);
	RETURN_ERROR;
}


/* Set flipper angles [\a rad]. All flippers has the same referential coordinate. They all rotate around the \a Y-axis.
 *  That means negative values for front flippers makes the flipper to move up.
 *  \param[in] angle Angle to be set [\a rad].
 *  \param[in] object Object id. Possible object id are: ID_FLIPPER_FRONT_LEFT, ID_FLIPPER_FRONT_RIGHT,
 *  ID_FLIPPER_REAR_LEFT, ID_FLIPPER_REAR_RIGHT.*/
int nrSetFlipper(double angle, int object)
{
	SETPRINT("nrSetFlipper(angle=%f, object=%s)\n", angle, CTRL_NAMES[object]);
	switch (object){
	case ID_FLIPPER_FRONT_LEFT:
		frontLeft = update_flipper(frontLeft, angle);
		break;
	case ID_FLIPPER_FRONT_RIGHT:
		frontRight = update_flipper(frontRight, angle);
		break;
	case ID_FLIPPER_REAR_RIGHT:
		rearRight = update_flipper(rearRight, angle);
		break;
	case ID_FLIPPER_REAR_LEFT:
		rearLeft = update_flipper(rearLeft, angle);
		break;
	default:
		printf("ERROR: nrSetFlipper called with wrong object ID: %d.\n", object);
	}
	RETURN_ERROR;


}



/* get flippers angles
*/
int nrGetFlippers(double *fL, double *fR, double *rL,
		double *rR)
{
	GETPRINT("nrGetFlippers(*frontLeft=%f, *frontRight=%f, *rearLeft=%f, *rearRight=%f)\n",
			frontLeft, frontRight, rearLeft, rearRight);
	*fL = frontLeft;
	*fR = frontRight;
	*rL = rearLeft;
	*rR = rearRight;
	RETURN_ERROR;
}
/* Say if the flippers has reached their target angle.
*/
int nrFlippersAngleReached(int *reached)
{
	*reached = 1;
	GETPRINT("nrFlippersAngleReached(*reached=1)\n");
	RETURN_ERROR;
}


/* enable/disable differential break
*/
int nrSetBrake(int on)
{
	SETPRINT("nrSetBrake(on=%s)\n", on?"true":"false");
	brake_on = on;
	RETURN_ERROR;
}

/* get the current brake state
*/
int nrGetBrake(int *val)
{
	GETPRINT("nrGetBrake(*val=%s)\n", brake_on?"true":"false");
	*val = brake_on;
	RETURN_ERROR;
}


/* Get the motor failure register for all controllers (see controller.h for details) */
int nrGetMotorsFailure(int status[])
{
	int i;
	GETPRINT("nrGetMotorFailure()\n");
	//for(i=0; i<ID_CTRL_MAX; status[i++]=0);
	for(i=0; i<ID_CTRL_MAX; status[i++]=rand());
	RETURN_ERROR;
}

/* Get the status register for all controllers (see controller.h for details) */
int nrGetControllersStatus(int status[])
{
	int i;
	GETPRINT("nrGetControllerStatus()\n");
	//for(i=0; i<ID_CTRL_MAX; status[i++]=0);
	for(i=0; i<ID_CTRL_MAX; status[i++]=rand());
	RETURN_ERROR;
}

/* Get the error code register for all controllers (see controller.h for details) */
int nrGetControllersError(int error[])
{
	int i;
	GETPRINT("nrGetControllerError()\n");
	//for(i=0; i<ID_CTRL_MAX; error[i++]=0);
	for(i=0; i<ID_CTRL_MAX; error[i++]=(int)(200.0*rand()/RAND_MAX));
	RETURN_ERROR;
}

/* Get the current differential angles */
int nrGetDifferentialAngles(double *left, double *right)
{
	*left = -0.1 + 0.03*rand()/RAND_MAX;
	*right = -*left;
	GETPRINT("nrGetDifferentialAngles(*left=%f, *right=%f)\n", *left, *right);
	RETURN_ERROR;
}

// update laser position
void updateLaserPos()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double now = (tv.tv_sec + 0.000001*tv.tv_usec);
	laser_pos += scanning_speed*(now - last_laser_time);
	//printf("laser_pos = %f\n", laser_pos);
	last_laser_time = now;
}

/* set the scanning speed of the 3D laser (in rad/s)
*/
int nrSetScanningSpeed(double val)
{
	SETPRINT("nrSetScanningSpeed(val=%f)\n", val);
	updateLaserPos();
	scanning_speed = val;
	RETURN_ERROR;
}

/* Get the current scanning rate */
int nrGetScanningSpeed(double *val)
{
	GETPRINT("nrGetScanningSpeed(*val=%f)\n", scanning_speed);
	*val = scanning_speed;
	RETURN_ERROR;
}

/* Get the current scanner angle in [-PI/2, PI/2] */
int nrGetScannerAngle(double *val)
{
	updateLaserPos();
	double out_angle = atan2(sin(laser_pos), fabs(cos(laser_pos)));
	//printf("laser: %f -> %f\n", laser_pos, out_angle);
	GETPRINT("nrGetScannerAngle(*val=%f)\n", out_angle);
	*val = out_angle;
	RETURN_ERROR;
}

/* Get the active current of the motors [\a A]. Use the object ID_xxx defined in types.h.
 *
 * ID_CORE; ID_TRACK_LEFT; ID_TRACK_RIGHT; ID_FLIPPER_FRONT_LEFT;
 * ID_FLIPPER_FRONT_RIGHT; ID_FLIPPER_REAR_LEFT; ID_FLIPPER_REAR_RIGHT; ID_CTRL_MAX
 *
 *  \param[out] *iq Actual active current of the motors in [\a A].
 *  \param[in] object The objects id are defined in the types.h only. */
int nrReadDOFCurrent(double *iq, int object)
{
	GETPRINT("nrReadDOFCurrent(*iq=0A, object=%s)\n", CTRL_NAMES[object]);
	*iq = 0.0;
	RETURN_ERROR;
}


/* Get the current battery voltage and state {BAT_OK, BAT_WARNING, BAT_CRITICAL} */
int nrGetBatteryLevel(double *level, int *state)
{
	GETPRINT("nrGetBatteryLevel(*level=95%%, *state=BAT_OK)\n");
	*level = 95.0;
	*state = BAT_OK;
	RETURN_ERROR;
}

/* Get the actual E-Stop state.
*/
int nrGetEstop(int *eStop)
{
	*eStop = (rand()>(int)(0.95*RAND_MAX));
	GETPRINT("nrGetEstop(*eStop=%s)\n", *eStop?"true":"false");
	RETURN_ERROR;
}

/* Actual initialisation state: 1 = initialised; 0 = NOT initialised
*/
int nrGetFlippersInitState()
{
	GETPRINT("nrGetFlippersInitState()=%s\n", flippersInitState?"true":"false");
	return flippersInitState;
}

/* Tell the 3D laser scanner to go to the predefined middle position with an user offset.
*/
int nrGoMiddlePos(double angleOffset){
	SETPRINT("nrGoMiddlePos(angleOffset=%f)\n", angleOffset);
	scanning_speed = 0.0;
	laser_pos = 0.0;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	last_laser_time = (tv.tv_sec + 0.000001*tv.tv_usec);
	RETURN_ERROR;
}


