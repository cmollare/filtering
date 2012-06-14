#ifndef CONTROL_H
#define CONTROL_H

#define USE_QUATERNION //For quaternion sampling
//#define USE_EULER

#define QRS //Comment to disable QRS

#define PART_MMSE

//Variances
#define VARQUATERNION 0.2 //variance for quaternions

#define VAROFFSET 0.01 //variance for offset
#define VAROFFSETFREE 0.1 //variance for offset with Free dof

#define EVOLVEOFFSET //Enable offset filtering

#define NBMODELS 100 //Nb particles

#define SAVE_MATLAB //To save curves

#endif
