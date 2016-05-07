#include "std_msgs/String.h"
#include "cv.h"
#include "highgui.h"
#include "functions.h"
#include <cstring>
#include <iostream>
#include <sstream>

using namespace std;
/**
* expects an output matrix as a first parametr 
*  matlab-format matrix string containing a 3x3 matrix : [X X X; X X X; X X X].
*  returns true if something gone wrong
*  false if worked and saved output into the first parameter
*/
bool parseMatrixString(Mat33f &output_matrix, string input)
{
	stringstream ss (input, stringstream::in);
	stringstream sst (stringstream::in | stringstream::out);
	string element = "";
	char ch = 0;
	double values[] = {0,0,0,0,0,0,0,0,0};
	bool output = true;
	
    // find the beginning
	while(ch != '[')
	{
		ss.get(ch);
		if(ss.eof()){return output;}
	}
	
	
	// read the first three numbers
	//
	
	for(int i = 0; i < 3; i++){
		// number beginning:
		do
		{
			ss.get(ch);
			if(ss.eof()){return output;}
	
		}
		while(ch == ' ' || ch == ',');
		
		// store chars into the sst stream
		do
		{
			sst << ch;
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while( ch != ' ' && ch != ',' && ch != ';' );
	
		// parse contents of sst into double
		sst >> values[i];
		if(sst.fail()) {return output;}
		
		// empty sst
		while(!sst.eof()) sst.get();
		sst.clear();
			
	}
	
	// go to the beginning of the second row	
	while(ch != ';') 
	{
		if(ch != ' ') {return output;}
		ss.get(ch);
	}	
	
		
	// read the second row	
	for(int i = 3; i < 6; i++){
	
		// number beginning:
		do
		{
			ss.get(ch);
			if(ss.eof()){return output;}
	
		}
		while(ch == ' ' || ch == ',');
		
		// store chars into the sst stream
		do
		{
			sst << ch;
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while( ch != ' ' && ch != ',' && ch != ';' );
	
		// parse contents of sst into double
		sst >> values[i];
		if(sst.fail()) {return output;}
		
		// empty sst
		while(!sst.eof()) sst.get();
		sst.clear();
			
	}
	
	// go to the beginning of the third row
	while(ch != ';') 
	{
		if(ch != ' ') {return output;}
		ss.get(ch);
	}	

	// read the last row
	for(int i = 6; i < 9; i++){
	
		// number beginning:
		do
		{
			ss.get(ch);
			if(ss.eof()){return output;}
	
		}
		while(ch == ' ' || ch == ',');
		
		// store chars into the sst stream
		do
		{
			sst << ch;
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while( ch != ' ' && ch != ',' && ch != ';' && ch != ']' );
	
		// parse contents of sst into double
		sst >> values[i];
		if(sst.fail()) {return output;}
				
		// empty sst
		while(!sst.eof()) sst.get();
		sst.clear();
	}
	
	// chech if the matrix is properly terminated by ]
	while(ch != ']')
	{
		ss.get(ch);
		if(ss.eof()){return output;}
	}
	
	// if we got this far, values should contain valid values...
	output_matrix = Mat33f(values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8]);			 		   
		
	return false;
}



/**
* expects an output column vector as a first parameter 
*  matlab-format matrix string containing a 1x3 vector : [X; X; X]. In case of a failure
*  returns true if something gone wrong
*  false if worked and saved output into the first parameter
*/
bool parseVectorString(Vec3fCol &output_matrix, string input)
{
	stringstream ss (input, stringstream::in);
	stringstream sst (stringstream::in | stringstream::out);
	string element = "";
	char ch = 0;
	double values[] = {0,0,0};
	bool output = true;
	
    // find the beginning
	while(ch != '[')
	{
		ss.get(ch);
		if(ss.eof()){return output;}
	}
	
	
	// read the first number
	//
	{
		// number beginning:
		do
		{
			ss.get(ch);
			if(ss.eof()){return output;}
	
		}
		while(ch == ' ' || ch == ',');
		
		// store chars into the sst stream
		do
		{
			sst << ch;
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while( ch != ' ' && ch != ',' && ch != ';' );
	
		// parse contents of sst into double
		sst >> values[0];
		if(sst.fail()) {return output;}
		
		// empty sst
		while(!sst.eof()) sst.get();
		sst.clear();
	}		
	
	// go to the beginning of the second row	
	while(ch != ';') 
	{
		if(ch != ' ') {return output;}
		ss.get(ch);
	}	
	
	// read the second number		
	{
		// number beginning:
		do
		{
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while(ch == ' ' || ch == ',');
		
		// store chars into the sst stream
		do
		{
			sst << ch;
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while( ch != ' ' && ch != ',' && ch != ';' );
	
		// parse contents of sst into double
		sst >> values[1];
		if(sst.fail()) {return output;}
		
		// empty sst
		while(!sst.eof()) sst.get();
		sst.clear();
			
	}
	
	// go to the beginning of the third row
	while(ch != ';') 
	{
		if(ch != ' ') {return output;}
		ss.get(ch);
	}	

	// read the last number
	{
		// number beginning:
		do
		{
			ss.get(ch);
			if(ss.eof()){return output;}
	
		}
		while(ch == ' ' || ch == ',');
		
		// store chars into the sst stream
		do
		{
			sst << ch;
			ss.get(ch);
			if(ss.eof()){return output;}
		}
		while( ch != ' ' && ch != ',' && ch != ';' && ch != ']' );
	
		// parse contents of sst into double
		sst >> values[2];
		if(sst.fail()) {return output;}
				
		// empty sst
		while(!sst.eof()) sst.get();
		sst.clear();
	}
	
	// chech if the matrix is properly terminated by ]
	while(ch != ']')
	{
		ss.get(ch);
		if(ss.eof()){return output;}
	}
	
	// if we got this far, values should contain valid values...
	output_matrix = Vec3fCol(values[0],values[1],values[2]);			 		   
		
	return false;
}

