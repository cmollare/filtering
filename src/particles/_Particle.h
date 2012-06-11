#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <Eigen/Dense>

class _Particle
{
	public:
		_Particle();
		virtual void sampleFromPrior() =0;
		
		//_Particle operator +(const _Particle& part) const;
	
	protected:
		
};

#endif
