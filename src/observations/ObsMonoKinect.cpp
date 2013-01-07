#include "ObsMonoKinect.h"

ObsMonoKinect::ObsMonoKinect(std::vector<std::vector<double> >& obs) : mObs(obs)
{
}

void ObsMonoKinect::update(std::vector<std::vector<double> >& newObs)
{
	mObs=newObs;
}

int ObsMonoKinect::ObsPerJoint()
{
	return 1;
}
