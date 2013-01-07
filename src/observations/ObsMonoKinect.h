#ifndef OBSMONOKINECT_H
#define OBSMONOKINECT_H

#include <vector>

class ObsMonoKinect
{
	public:
		ObsMonoKinect(std::vector<std::vector<double> >& obs);
		void update(std::vector<std::vector<double> >& newObs);
		virtual int ObsPerJoint();
		
	protected:
		std::vector<std::vector<double> > mObs;
};

#endif
