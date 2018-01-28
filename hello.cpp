#include <python2.7/Python.h>

#include "IKSolver.h"

#ifdef __CPLUSPLUS
extern "C" {
#endif

static PyObject *
call_motors(PyObject *self, PyObject *args)
{
    long leg;
    float sx;
    float sy;
    float sz;
    float tx;
    float ty;
    float tz;
    float theta1;
    float theta2;
    float theta3;
    float theta4;

    int status = PyArg_ParseTuple(
        args, "lffffff", &leg, &sx, &sy, &sz, &tx, &ty, &tz);

    if (!status)
        return NULL;

    IK_getServoValues(
        leg, sx, sy, sz, tx, ty, tz, &theta1, &theta2, &theta3, &theta4
    );

    PyObject *ret = PyTuple_New(4);
    PyTuple_SET_ITEM(ret, 0, PyFloat_FromDouble(theta1));
    PyTuple_SET_ITEM(ret, 1, PyFloat_FromDouble(theta2));
    PyTuple_SET_ITEM(ret, 2, PyFloat_FromDouble(theta3));
    PyTuple_SET_ITEM(ret, 3, PyFloat_FromDouble(theta4));
    return ret;
}

static PyMethodDef HelloMethods[] = {
    {"call_motors",  call_motors, METH_VARARGS,
     "Run the motors."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};


PyMODINIT_FUNC
inithello(void)
{
    (void) Py_InitModule("hello", HelloMethods);
}

#ifdef __CPLUSPLUS
}  // extern "C"
#endif
