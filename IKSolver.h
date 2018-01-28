/**********************************************
 ** 28/01/2018 - Nick Hafner                 **
 ** ICRS Hexapod Course                      **
 **********************************************/
#ifndef _IKSOLVER_H
#define _IKSOLVER_H

/**********************************************
 ** HEXAPOD COURSE INVERSE KINEMATICS SOLVER **
 ** @param legNum                            **
 ** Which leg to solve for, top left is 0,   **
 ** travelling around the robot anti         **
 ** clockwise in increments of 1             **
 ** @params sx, sy, sz                       **
 ** The current Cartesian coordinates of the **
 ** foot of the leg, relative to (0,0,0) of  **
 ** the leg                                  **
 ** @params tx, ty, tz                       **
 ** The target Cartesian coordinates of the  **
 ** foot of the leg, relative to (0,0,0) of  **
 ** the leg                                  **
 ** @params theta1, theta2, theta3, theta4   **
 ** The current values of the servo motors   **
 ** that this leg consists of                **
 ** @return                                  **
 ** Returns a vector containing the 4 servo  **
 ** positions required to position the foot  **
 ** at the specified location                **
 **********************************************/
void IK_getServoValues(short legNum, float sx, float sy, float sz, float tx, float ty, float tz,
                                float* theta1, float* theta2, float* theta3, float* theta4);

// FORWARD KINEMATIC EQUATION
// x = cos(theta1)*(L2 + L3*sin(theta3) + L4*sin(theta4 - theta3 - 34.6666) + sin(theta1)*cos(theta2)*(L3*sin(theta3) - L4*sin(theta4-theta3+34.6666))
// y = sin(theta1)*(L2 + L3*sin(theta3) + L4*sin(theta4 - theta3 - 34.6666) - cos(theta1)*cos(theta2)*(L3*sin(theta3) - L4*sin(theta4-theta3+34.6666))
// z = sin(theta2)*L3*cos(theta3 - L4*cos(theta4-theta3+34.6666))

#endif // _IKSOLVER_H

