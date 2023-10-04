#pragma once 

typedef struct DATA
{
   double Time; // Time [sec]
   double FlagGPS;
   double Lon; // Lon [radian]
   double Lat; // Lat[radian]
   double Hei; // Hei [m]
   double VelGPS; // [m/sec]
   double Yaw; // [d]
   double FlagOdo;
   double VelOdo; // [m/sec]
   double Steering; // [d]
  
} DataBlock;
