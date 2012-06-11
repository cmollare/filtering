#include "SIR.h"

template<class Particles, class Observations>
SIR<Particles, Observations>::SIR(int nbParticles, Particles& model) : _Filter<Particles, Observations>(nbParticles, model)
{
}

template<class Particles, class Observations>
SIR<Particles, Observations>::~SIR()
{
}

template<class Particles, class Observations>
void SIR<Particles, Observations>::init()
{
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}
