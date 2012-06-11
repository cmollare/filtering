#ifndef SIR_H
#define SIR_H

#include "_Filter.h"

template<class Particles>
class SIR : public _Filter<Particles>
{
	public:
		SIR(int nbParticles, Particles& model=NULL);
		virtual ~SIR();
};

#endif
