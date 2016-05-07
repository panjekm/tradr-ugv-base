#include "cv.h"
#include "highgui.h"
#include "functions.h"




/**
* expects an output matrix as a first parameter 
*  matlab-format matrix string containing a 3x3 matrix : [X X X; X X X; X X X]
*  returns true if something gone wrong
*  false if worked and saved output into the first parameter
*/
bool parseMatrixString(Mat33f &output_matrix, string input);



/**
* expects an output column vector as a first parameter 
*  matlab-format matrix string containing a 1x3 vector : [X; X; X]. In case of a failure
*  returns true if something gone wrong
*  false if worked and saved output into the first parameter
*/
bool parseVectorString(Vec3fCol &output_matrix, string input);
