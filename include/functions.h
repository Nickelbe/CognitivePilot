#pragma once 

#include "structures.h"
#include <Eigen/Dense>

int DataDim(char* filename);
void DataReading(DataBlock *Data, int Dimension);
void Make_RE_and_RN(double Lat, double Height, double* Re, double* Rn);
void East2NorthAngle(double *Angle);
void North2EastAngle(double *Angle);
