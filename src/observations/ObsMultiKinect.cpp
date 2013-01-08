#include "ObsMultiKinect.h"

ObsMultiKinect::ObsMultiKinect()
{
}

ObsMultiKinect::ObsMultiKinect(std::vector<std::vector<double> >& kinect1, std::vector<std::vector<double> >& kinect2, std::vector<std::vector<double> >& kinect3)
{
}

ObsMultiKinect::~ObsMultiKinect()
{
}

int ObsMultiKinect::obsPerJoint()
{
	return 3;
}
