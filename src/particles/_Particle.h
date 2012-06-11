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
		virtual void update(Observations obs) =0;
		
		//_Particle operator +(const _Particle& part) const;
	
	protected:
		
};

#endif
