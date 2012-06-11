#include "_Filter.h"

template<class Particles, class Observations>
_Filter<Particles, Observations>::_Filter(int nbParticles, Particles& model)
{
	mNbParticles = nbParticles;
	if (model != NULL)
	{
		for (int i=0 ; i<mNbParticles ; i++)
		{
			mParticles.push_back(new Particles(model));
		}
	}
	else
	{
		for (int i=0 ; i<mNbParticles ; i++)
		{
			mParticles.push_back(new Particles());
		}
	}
}

template<class Particles, class Observations>
_Filter<Particles, Observations>::~_Filter()
{
	for (int i=0 ; i<mNbParticles ; i++)
	{
		delete mParticles[i];
	}
}

template<class Particles, class Observations>
std::vector<Particles*> _Filter<Particles, Observations>::getParticleVector()
{
	return this->mParticles;
}
