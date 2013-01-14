#ifndef OBSMULTIKINECT_H
#define OBSMULTIKINECT_H

#include <vector>

class ObsMultiKinect
{
	public:
		ObsMultiKinect();
		ObsMultiKinect(std::vector<std::vector<std::vector<double> > >& obs);
		~ObsMultiKinect();
		
		std::vector<std::vector<std::vector<double> > > getMultiFrame();
		std::vector<std::vector<double> > getFrame();
		
		virtual int obsPerJoint();
		
	protected:
		std::vector<std::vector<std::vector<double> > > mObs;
};

#endif
