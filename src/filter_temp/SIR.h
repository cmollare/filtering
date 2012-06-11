#ifndef SIR_H
#define SIR_H

#include "_Filter.h"

template<class Particles, class Observations>
class SIR : public _Filter<Particles, Observations>
{
	public:
		SIR(int nbParticles, Particles& model=NULL);
		virtual ~SIR();
		
		virtual void init();
};

#endif
