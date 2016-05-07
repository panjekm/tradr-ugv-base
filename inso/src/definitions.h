#include "cv.h"

// CONSTANTS
#define		PI		3.141592653589793238462643
#define		geo_a	6378137.0                			
#define		geo_e		0.081819191
#define 	EULER	2.7182818284590452353602874713527

// ATTITUDE FEEDBACK TYPES
#define 	ATT_NO_FEEDBACK				0
#define 	ATT_GYR_ACC_WEIGHTED_AVG	1
#define 	ATT_FILTERED				2

// MEASUREMENT STATES
#define		MEAS_STATE_CALIB			0
#define		MEAS_STATE_CALIB_RESULT		1
#define		MEAS_STATE_FIRST			2
#define		MEAS_STATE_MECHANIZATION	3

#define 	YAW_NOT_SET					1000

// VELOCITY and POSITION indices

#define		VN				0
#define		VE				1
#define		VD				2
#define		PN				0
#define		PE				1
#define		PD				2

// harmonics array indices

#define SIN_PHI 0
#define SIN_TH 1
#define SIN_PSI 2
#define COS_PHI 3
#define COS_TH 4
#define COS_PSI 5

using namespace cv;

/*-------------------------------------------------------------------------
 * TYPE DEFINITIONS
 *-------------------------------------------------------------------------
 */
	typedef Matx<double, 3, 1> Vec3fCol;
	typedef Matx<double, 4, 1> quat;
	typedef Matx<double, 1, 3> Vec3fRow;
	typedef Matx<double, 1, 4> Vec4fRow;
	typedef Matx<double, 3, 3> Mat33f;
	typedef Matx<double, 9, 9> Mat99f;
	typedef Matx<double, 6, 6> Mat66f;
	typedef Matx<double, 9, 6> Mat96f;
	typedef Matx<double, 6, 9> Mat69f;

	typedef Matx<double, 10, 1> Vec10fCol;
	typedef Matx<double, 7, 1> Vec7fCol;
	typedef Matx<double, 6, 1> Vec6fCol;
	typedef Matx<double, 9, 1> Vec9fCol;
	
