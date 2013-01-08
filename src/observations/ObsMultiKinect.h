#ifndef OBSMULTIKINECT_H
#define OBSMULTIKINECT_H

#include <vector>

class ObsMultiKinect
{
	public:
		ObsMultiKinect();
		ObsMultiKinect(std::vector<std::vector<double> >& kinect1, std::vector<std::vector<double> >& kinect2, std::vector<std::vector<double> >& kinect3);
		~ObsMultiKinect();
		
		virtual int obsPerJoint();
};

#endif
