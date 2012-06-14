#ifndef CONTROL_H
#define CONTROL_H

#define USE_QUATERNION //For quaternion sampling
//#define USE_EULER

#define QRS //Comment to disable QRS

#define PART_MMSE

//variances (a modifier)
#define TEMP3 0.2 //partQRS
#define TEMPO 0.2 //prior + PF
#define TEMP 0.2 //update partionned

#define VAROFFSET 0.01
#define VAROFFSETFREE 0.1

#define EVOLVEOFFSET

#define NBMODELS 100 //Nb particles

#define SAVE_MATLAB //To save curves

#endif
