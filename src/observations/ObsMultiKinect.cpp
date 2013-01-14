#include "ObsMultiKinect.h"

ObsMultiKinect::ObsMultiKinect()
{
}

ObsMultiKinect::ObsMultiKinect(std::vector<std::vector<std::vector<double> > >& obs) : mObs(obs)
{
}

ObsMultiKinect::~ObsMultiKinect()
{
}

int ObsMultiKinect::obsPerJoint()
{
	return 3;
}

std::vector<std::vector<std::vector<double> > > ObsMultiKinect::getMultiFrame()
{
	return mObs;
}

std::vector<std::vector<double> > ObsMultiKinect::getFrame()
{
	return mObs[0];
}
