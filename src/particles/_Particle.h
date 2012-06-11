#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <Eigen/Dense>

template<class Observations>
class _Particle
{
	public:
		_Particle();
		_Particle(_Particle& particle); //copy constructor
		
		virtual void sampleFromPrior() =0;
		virtual void update() =0;
		virtual void esitmateLikelihood(Observations obs) =0;
		virtual double getLikelihood();
		
		//_Particle operator +(const _Particle& part) const;
	
	protected:
		double mCurrentLikelihood;
};

#endif
