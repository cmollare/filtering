#include "_Filter.h"

template<class Particles>
_Filter<Particles>::_Filter(int nbParticles, Particles& model)
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

template<class Particles>
_Filter<Particles>::~_Filter()
{
	for (int i=0 ; i<mNbParticles ; i++)
	{
		delete mParticles[i];
	}
}
