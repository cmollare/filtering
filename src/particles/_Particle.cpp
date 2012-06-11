#include "_Particle.h"

template<class Observations>
_Particle<Observations>::_Particle()
{
	this->mCurrentLikelihood=0;
}

template<class Observations>
_Particle<Observations>::_Particle(_Particle& particle)
{
	this->mCurrentLikelihood = particle.mCurrentLikelihood;
}

template<class Observations>
double _Particle<Observations>::getLikelihood()
{
	return this->mCurrentLikelihood;
}
