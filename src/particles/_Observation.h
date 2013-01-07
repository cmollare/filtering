#ifndef _OBSERVATION_H
#define _OBSERVATION_H

#include <vector>

class _Observation
{
	public:
		_Observation();
		_Observation(std::vector<std::vector<double> > obs);
		~_Observation();
		std::vector<std::vector<double> >& getObservation();
		
	protected:
		std::vector<std::vector<double> > mObs;
};

#endif

