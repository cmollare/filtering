#include "_Observation.cpp"

_Observation::_Observation()
{
}

_Observation::_Observation(std::vector<std::vector<double> > obs) : mObs(obs)
{
}

_Observation::~_Observation()
{
}

std::vector<std::vector<double> >& _Observation::getObservation()
{
	return mObs;
}

