#include <iostream>
#include <math.h>

class Imu
{
    // Have the following functions:
    // constructor
public:
    double quat[4];
    double dT;
    Imu(double *quat_init, double dT) : dT(dT)
    {
        quat[0] = quat_init[0];
        quat[1] = quat_init[1];
        quat[2] = quat_init[2];
        quat[3] = quat_init[3];
    }
    void update(double *data)
    {
        double b[4];
        b[0] = 1;
        b[1] = 0.5 * data[0] * dT;
        b[2] = 0.5 * data[1] * dT;
        b[3] = 0.5 * data[2] * dT;
        quatMultiply(quat, b);
        quatNormalize(quat);
    }
    // quatMultiply
private:
    void quatMultiply(double *quat, double *b)
    {
        double a_ssm[4][4];

        a_ssm[0][0] = quat[0];
        a_ssm[0][1] = -quat[1];
        a_ssm[0][2] = -quat[2];
        a_ssm[0][3] = -quat[3];
        a_ssm[1][0] = quat[1];
        a_ssm[1][1] = quat[0];
        a_ssm[1][2] = -quat[3];
        a_ssm[1][3] = quat[2];
        a_ssm[2][0] = quat[2];
        a_ssm[2][1] = quat[3];
        a_ssm[2][2] = quat[0];
        a_ssm[2][3] = -quat[1];
        a_ssm[3][0] = quat[3];
        a_ssm[3][1] = -quat[2];
        a_ssm[3][2] = quat[1];
        a_ssm[3][3] = quat[0];

        quat[0] = a_ssm[0][0] * b[0] + a_ssm[0][1] * b[1] + a_ssm[0][2] * b[2] + a_ssm[0][3] * b[3];
        quat[1] = a_ssm[1][0] * b[0] + a_ssm[1][1] * b[1] + a_ssm[1][2] * b[2] + a_ssm[1][3] * b[3];
        quat[2] = a_ssm[2][0] * b[0] + a_ssm[2][1] * b[1] + a_ssm[2][2] * b[2] + a_ssm[2][3] * b[3];
        quat[3] = a_ssm[3][0] * b[0] + a_ssm[3][1] * b[1] + a_ssm[3][2] * b[2] + a_ssm[3][3] * b[3];
    }
    // quatNormalize
    void quatNormalize(double *quat)
    {
        double quatmag = sqrt(pow(quat[0], 2) + pow(quat[1], 2) + pow(quat[2], 2) + pow(quat[3], 2));

        quat[0] = quat[0] / quatmag;
        quat[1] = quat[1] / quatmag;
        quat[2] = quat[2] / quatmag;
        quat[3] = quat[3] / quatmag;
    }
    // update
    // calculate deltaTheta
    // quatMultiply
    // quatNormalize
};
int main()
{
    double data[10][3] = {
        -1.07761367200000, -0.0316569890000000, 1.42161709700000,
        -0.854200598000000, 0.251783428000000, 1.67855073700000,
        -0.632536703000000, 0.248501854000000, 1.71443714800000,
        -0.449975489000000, -0.0473337250000000, 1.44042730900000,
        -0.0847380910000000, -0.774319697000000, 0.962126567000000,
        0.182800025000000, -1.56475014100000, 0.461543371000000,
        0.456295310000000, -2.05308475600000, -0.326379857000000,
        0.825959496000000, -2.34036753200000, -0.821455668000000,
        1.12616863200000, -1.79507662300000, -0.967248971000000,
        1.74725191900000, -1.55064817400000, -1.06787789100000};

    int n = 10;
    double dT = 1.0 / 128.0;
    double quat_init[4] = {1.0, 0, 0, 0};
    Imu imu(quat_init, dT);
    double gyr[3];
    for (int i = 0; i < n; i++)
    {
        gyr[0] = data[i][0];
        gyr[1] = data[i][1];
        gyr[2] = data[i][2];
        imu.update(gyr);
    }

    // Hopefully your output matches the following:
    // 0.999121, 0.00618785, -0.0372197, 0.0182653
    std::cout << imu.quat[0] << ", " << imu.quat[1] << ", " << imu.quat[2] << ", " << imu.quat[3] << std::endl;
}