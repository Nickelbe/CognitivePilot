#pragma once

#include <cmath>

#define DATA_FILENAME "mxm_data.txt"

double const M_WPI        = (2.0*M_PI);
double const M_HALF_PI    = (0.5*M_PI);
double const M_QUARTER_PI = (0.25*M_PI);

double const  E2WGS84 = 6.6943799901413e-3; // WGS84 Earth ellipsoide compress coefficients quadrate 
double const  AWGS84 = 6378137.0;          // WGS84 Earth ellipsoide big semimajor axis in meter 

// =================================
//  Radians <--> Degrees Conversion
// ---------------------------------
#define RadianToDegree  (180./M_PI)

#define D2R(Degrees)    ((Degrees)/RadianToDegree)          // Degree  --> Radians
#define M2R(Minutes)    ((Minutes/60.)/RadianToDegree)      // Minutes --> Radians
#define S2R(Seconds)    ((Seconds/3600.)/RadianToDegree)    // Seconds --> Radians
#define R2D(Radians)    ((Radians)*RadianToDegree)          // Radians --> Degree
#define R2M(Radians)    ((Radians)*RadianToDegree*60.)      // Radians --> Minutes
#define R2S(Radians)    ((Radians)*RadianToDegree*3600.)    // Radians --> Seconds
#define DH2RS(x)        (D2R(x)/3600.)
#define RS2DH(x)        (R2D(x)*3600.)
#define DMS2R(DD,MM,SS) (((DD)+(MM)/60.+(SS)/3600.)/RadianToDegree)
#define HMS2S(HH,MM,SS) ((SS)+(MM)*60+(HH)*3600.)
