#include "IKSolver.h"
#include <time.h>
#include <math.h>

// Precision of the solver in mm
const float g_IK_tolerance = 0.1;

// Maximum number of milliseconds in operation
const float g_IK_maxmillis = 5;

// Offset for the 4th joint angle in the leg
const float g_IK_4thOffset = 34.6666;

void IK_getServoValues(short legNum, float sx, float sy, float sz, float tx, float ty, float tz,
                                float* theta1, float* theta2, float* theta3, float* theta4){
    float t1,t2,t3,t4;
    t1 = *theta1;
    t2 = *theta2;
    t3 = *theta3;
    t4 = *theta4 + g_IK_4thOffset;
    long dt = clock();
    while((
           !(tx - g_IK_tolerance < sx && sx < tx + g_IK_tolerance) ||
           !(ty - g_IK_tolerance < sy && sy < ty + g_IK_tolerance) ||
           !(tz - g_IK_tolerance < sz && sz < tz + g_IK_tolerance)
          ) && clock() - dt < g_IK_maxmillis*CLOCKS_PER_SEC/1000.f){
        float sxt1, sxt2, sxt3, sxt4,
              syt1, syt2, syt3, syt4,
              szt1, szt2, szt3, szt4;

        float cos1, sin1, cos2, sin2,
              cos3, sin3, cos4, sin4,
              L2, L3, L4;

        // Lengths on the leg (mm)
        L2 = 130;
        L3 = 79;
        L4 = 174;

        // Calculate trigs for efficiency

        cos1 = cos(t1); sin1 = sin(t1);
        cos2 = cos(t2); sin2 = sin(t2);
        cos3 = cos(t3); sin3 = sin(t3);
        cos4 = cos(t4 - t3); sin4 = sin(t4 - t3);

        sxt1 = -sin1*(L2 + L3*sin3 + L4*sin4) + cos1*cos2*(L3*cos3 - L4*cos4);
        sxt2 = -sin1*sin2*(L3*cos3 - L4*cos4);
        sxt3 = cos1*(L3*cos3 - L4*cos4) - sin1*cos2*(L3*sin3 + L4*cos4);
        sxt4 = cos1*L4*cos4 + sin1*cos2*L4*sin4;

        syt1 = cos1*(L2 + L3*sin3 + L4*sin4) + sin1*cos2*(L3*cos3 - L4*cos4);
        syt2 = cos1*sin2*(L3*cos3 - L4*cos4);
        syt3 = sin1*(L3*cos3 + L4*cos4) + cos1*cos2*(L3*cos3 + L4*cos4);
        syt4 = sin1*L4*cos4 + cos1*cos2*L4*sin4;

        szt1 = 0;
        szt2 = cos2*(L3*cos3 - L4*cos4);
        szt3 = sin2*(-L3*sin3 + L4*sin4);
        szt4 = sin2*L4*sin4;

        float e[3] = {tx - sx, ty - sy, tz - sz};

        float JJTe[3] = { (sxt1*sxt1 + sxt2*sxt2 + sxt3*sxt3 + sxt4*sxt4)*e[0]
                             + (sxt1*syt1 + sxt2*syt2 + sxt3*syt3 + sxt4*syt4)*e[1]
                             + (sxt1*szt1 + sxt2*szt2 + sxt3*szt3 + sxt4*szt4)*e[2],

                               (syt1*sxt1 + syt2*sxt2 + syt3*sxt3 + syt4*sxt4)*e[0]
                             + (syt1*syt1 + syt2*syt2 + syt3*syt3 + syt4*syt4)*e[1]
                             + (syt1*szt1 + syt2*szt2 + syt3*szt3 + syt4*szt4)*e[2],

                               (szt1*sxt1 + szt2*sxt2 + szt3*sxt3 + szt4*sxt4)*e[0]
                             + (szt1*syt1 + szt2*syt2 + szt3*syt3 + szt4*syt4)*e[1]
                             + (szt1*szt1 + szt2*szt2 + szt3*szt3 + szt4*szt4)*e[2]};

        float alpha = (e[0]*JJTe[0] + e[1]*JJTe[1] + e[2]*JJTe[2])
            /(JJTe[0]*JJTe[0] + JJTe[1]*JJTe[1] + JJTe[2]*JJTe[2]);

        t1 += alpha*sxt1*e[0] + alpha*syt1*e[1] + alpha*szt1*e[2];
        t2 += alpha*sxt2*e[0] + alpha*syt2*e[1] + alpha*szt2*e[2];
        t3 += alpha*sxt3*e[0] + alpha*syt3*e[1] + alpha*szt3*e[2];
        t4 += alpha*sxt4*e[0] + alpha*syt4*e[1] + alpha*szt4*e[2];

        sx = cos(t1)*(L2 + L3*sin(t3) + L4*sin(t4 - t3))
            + sin(t1)*cos(t2)*(L3*cos(t3) - L4*cos(t4 - t3));
        sy = sin(t1)*(L2 + L3*sin(t3) + L4*cos(t4-t3))
            - cos(t1)*cos(t2)*(L3*cos(t3) - L4*cos(t4-t3));
        sz = sin(t2)*(L3*cos(t3) - L4*cos(t4 - t3));

    }

    *theta1 = t1;
    *theta2 = t2;
    *theta3 = t3;
    *theta4 = t4 - g_IK_4thOffset;

}
