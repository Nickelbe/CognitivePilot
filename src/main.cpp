#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "functions.h"
#include "constants.h"
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(void) 
{
    FILE* out = NULL;
    fopen_s(&out, "output.txt", "w");
    fprintf(out, "%-17s %-25s%-25s%-25s%-25s%-25s%-25s%-25s%-25s\n", "Time[s]", "x[m]", "y[m]", 
        "x_gps[m]", "y_gps[m]", "yaw[d]", "k_v", "c_0", "c_1");

    FILE* cov = NULL;
    fopen_s(&cov, "covariance.txt", "w");
    fprintf(cov, "%-17s %-25s%-25s%-25s%-25s%-25s%-25s\n", "Time[s]", "SigmaDx[m]", "SigmaDy[m]",
        "SigmaDpsi[d]", "Sigmak_v", "Sigmak_0", "Sigmak_1");

    double kv = 0.0, k0 = 0.0, k1 = 0.0;
    double Re = 0.0, Rn = 0.0;
    double L = 2.85;
    double yaw_k_1 = 0.0, yaw_k = 0.0, x_k_1 = 0.0, x_k = 0.0, y_k_1 = 0.0, y_k = 0.0;
    double x_gps = 0.0, y_gps = 0.0;
    double dt = 0.0;
    double Vel = 0.0, delta = 0.0;
    double c0 = -k0 / (1.0 + k1);
    double c1 = 1.0 / (RadianToDegree * (1.0 + k1));

    int Dimension = DataDim(DATA_FILENAME);
    DataBlock* Data = new DataBlock[Dimension];
    DataReading(Data, Dimension);

    Matrix<float, 6, 2> B;
    Matrix<float, 3, 6> H;
    Matrix<float, 6, 3> K;
    Matrix<float, 3, 3> Temp;
    Matrix<float, 6, 6> F, A, P_ap, P_apr, E;
    VectorXf X_ap(6), X_apr(6), z(3);

    B.setZero();
    F.setZero();
    A.setZero();
    H.setZero();
    X_ap.setZero();
    X_apr.setZero();
    E.setIdentity();
    P_ap.setIdentity();
    P_apr.setIdentity();

    Matrix<float, 2, 2> Q;
    Q.setZero();
    Q(0, 0) = 1.0e-4;
    Q(1, 1) = 1.0e-6;

    Matrix<float, 3, 3> R;
    R.setZero();
    R(0, 0) = R(1, 1) = 0.0225;
    R(2, 2) = 0.0025;

    H(0, 0) = H(1, 1) = H(2, 2) = 1.0;

    // Sigma^2_Delta x/y
    P_ap(0, 0) = P_ap(1, 1) = 2.25;
    // Sigma^2_DeltaTheta
    P_ap(2, 2) = 1.0e-4;
    // Sigma^2_k_v
    P_ap(3, 3) = 1.0e-8;
    // Sigma^2_k_0
    P_ap(4, 4) = 4.0e-7;
    // Sigma^2_k_1
    P_ap(5, 5) = 4.0e-3;

    // Theta_0 = Theta_0 (GPS), x0 = y0 = x_GPS = y_GPS = 0 
    yaw_k_1 = Data[0].Yaw;

    for (unsigned int l = 1; l <= Dimension-1; l++) {
        dt = Data[l].Time - Data[l - 1].Time;
        Vel = Data[l-1].VelOdo / (1.0 + kv);
        delta = (Data[l-1].Steering - k0) / (1.0 + k1);

        Make_RE_and_RN(Data[l].Lat, Data[l].Hei, &Re, &Rn);
        x_gps = (Data[l].Lon - Data[0].Lon) * Re * cos(Data[l].Lat);
        y_gps = (Data[l].Lat - Data[0].Lat) * Rn;

        // Prediction
        x_k = x_k_1 + Vel * cos(yaw_k_1) * dt;
        y_k = y_k_1 + Vel * sin(yaw_k_1) * dt;
        yaw_k = yaw_k_1 + Vel * tan(delta) / L * dt;

        // Transition Matrix
        A(0, 2) = -Vel * sin(yaw_k_1);
        A(0, 3) = Vel * cos(yaw_k_1);
        A(1, 2) = Vel * cos(yaw_k_1);
        A(1, 3) = Vel * sin(yaw_k_1);
        A(2, 3) = Vel * tan(delta) / L;
        A(2, 4) = Vel / (L * pow(cos(delta), 2));
        A(2, 5) = Vel * delta / (L * pow(cos(delta), 2));

        B(0, 0) = cos(yaw_k_1);
        B(1, 0) = sin(yaw_k_1);
        B(2, 0) = tan(delta) / L;
        B(2, 1) = Vel / (L * pow(cos(delta), 2));

        F = E + A * dt + A * A * pow(dt, 2) / 2.0 + A * A * A * pow(dt, 3) / 6.0;

        X_apr = F * X_ap;
        P_apr = F * P_ap * F.transpose() + B * Q * B.transpose() * dt;

        // Update
        if (Data[l].FlagGPS == 1.0) {
            z(0) = x_k - x_gps;
            z(1) = y_k - y_gps;
            z(2) = yaw_k - Data[l].Yaw;

            K = P_apr * H.transpose();
            Temp = H * P_apr * H.transpose() + R;
            K *= Temp.inverse();

            X_ap = X_apr + K * (z - H * X_apr);
            P_ap = (E - K * H) * P_apr;

            x_k -= X_ap(0);
            y_k -= X_ap(1);
            yaw_k -= X_ap(2);

            kv += X_ap(3);
            k0 += X_ap(4);
            k1 += X_ap(5);

            X_ap.setZero();

            //new coef-s' recomputation
            c0 = -k0 / (1.0 + k1);
            c1 = 1.0 / (RadianToDegree * (1.0 + k1));
        }
        else {
            X_ap = X_apr;
            P_ap = P_apr;
        }

        if (yaw_k > M_PI) {
            yaw_k -= M_WPI;
        }
        else if (yaw_k < -M_PI) {
            yaw_k += M_WPI;
        }

        x_k_1 = x_k;
        y_k_1 = y_k;
        yaw_k_1 = yaw_k;

        fprintf(cov, "%-17.3lf%- 25.8lf%- 25.8lf%- 25.8lf%- 25.8lf%- 25.8lf%- 25.8lf\n",
            Data[l].Time, P_ap(0, 0), P_ap(1, 1), P_ap(2, 2), P_ap(3, 3), P_ap(4, 4), P_apr(5, 5));

        fprintf(out, "%-17.3lf%- 25.6lf%- 25.6lf%- 25.6lf%- 25.6lf%- 25.6lf%- 25.6lf%- 25.6lf%- 25.6lf\n",
            Data[l].Time, x_k, y_k, x_gps, y_gps, R2D(yaw_k), kv, c0, c1);
    }

    delete[] Data;

    fclose(out);
    fclose(cov);
    cout << "Press 'Enter' to quit";
    cin.get();
    return 0;
}