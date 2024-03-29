#ifndef CONTROL_H
#define CONTROL_H

#define USE_QUATERNION 0//For quaternion sampling
#define USE_EULER 1

#define SAMPLING USE_QUATERNION //Kind of sampling

//#define NOISE 0.02

#define QRS //Comment to disable QRS
#define PART_MMSE

//Variances
#define VARQUATERNION 0.2 //variance for quaternions (0.2 for MMSE)
#define VAREULER 0.2 //variance for euler angles

#define VAROFFSET 0.01 //variance for offset
#define VAROFFSETFREE 0.1 //variance for offset with Free dof
//End variance definitions

#define EVOLVEOFFSET //Enable offset filtering

#define NBMODELS 200 //Nb particles

#define SAVE_MATLAB //To save curves

#endif
