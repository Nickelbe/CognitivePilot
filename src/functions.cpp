#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "structures.h"
#include "constants.h"
#include <Eigen/Dense>

using namespace Eigen;

void North2EastAngle(double* Angle)
{
    *Angle += M_HALF_PI;
    if (*Angle > M_PI) {
        *Angle -= M_WPI;
    }
    else if (*Angle < -M_PI) {
        *Angle += M_WPI;
    }
    return;
}

void East2NorthAngle(double* Angle)
{
    *Angle -= M_HALF_PI;
    if (*Angle > M_PI) {
        *Angle -= M_WPI;
    }
    else if (*Angle < -M_PI) {
        *Angle += M_WPI;
    }
    return;
}

int DataDim(char* filename)
{
    FILE* pf = fopen(filename, "r");
    unsigned int Dimension = 0;
    int ch;
    while (EOF != (ch = getc(pf))) {
        if ('\n' == ch) {
            ++Dimension;
        }
    }
    return Dimension-1;
}

void DataReading(DataBlock *Data, int Dimension)
{
    FILE *pf = NULL;
    char Buffer[512];
	memset(Buffer, 0, sizeof Buffer);

    pf = fopen(DATA_FILENAME, "r");

    fgets(Buffer, sizeof Buffer, pf);

    for (long ll = 0; ll < Dimension; ll++)
	{
        fgets(Buffer, sizeof Buffer, pf);
        sscanf(Buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        &Data[ll].Time,
        &Data[ll].FlagGPS,
        &Data[ll].Lon,
        &Data[ll].Lat,
        &Data[ll].Hei,
        &Data[ll].VelGPS,
        &Data[ll].Yaw,
        &Data[ll].FlagOdo,
        &Data[ll].VelOdo,
        &Data[ll].Steering
        );
        Data[ll].Steering = D2R(Data[ll].Steering);
        Data[ll].Lon = D2R(Data[ll].Lon);
        Data[ll].Lat = D2R(Data[ll].Lat);
        Data[ll].Yaw = D2R(Data[ll].Yaw);
        North2EastAngle(&Data[ll].Yaw);
        if (ll % 100) {
            printf("\rDataReading: %.2lf", Data[ll].Time);
        }
    }
    printf("\rDataReading:                                     \n");
	fclose(pf);
}

void Make_RE_and_RN(double Lat, double Height, double* Re, double* Rn)
{
    double SinPhi, C, Cs;
    SinPhi = sin(Lat);
    C = 1.0 - E2WGS84 * SinPhi * SinPhi;
    Cs = sqrt(C);
    *Re = AWGS84 / Cs + Height;
    *Rn = AWGS84 * (1.0 - E2WGS84) / (C * Cs) + Height;
    return;
}
