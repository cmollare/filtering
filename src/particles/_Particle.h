#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <Eigen/Dense>

template<class Observations>
class _Particle
{
	public:
		_Particle()
		{
			this->mCurrentLikelihood=0;
		}
		_Particle(_Particle& particle) //copy constructor
		{
			this->mCurrentLikelihood = particle.mCurrentLikelihood;
		}
		
		virtual void sampleFromPrior() =0;
		virtual void update() =0;
		virtual void esitmateLikelihood(Observations obs) =0;
		virtual double getLikelihood()
		{
			return this->mCurrentLikelihood;
		}
		
		virtual void setId(int id)
		{
			mId = id;
		}
		
		virtual int getId()
		{
			return mId;
		}
		
		virtual _Particle<Observations>& operator =(const _Particle<Observations>& part)
		{
		}
		
		//virtual _Particle<Observations> operator +(const _Particle<Observations>& part) const;
		//virtual _Particle<Observations>& operator +=(const _Particle<Observations>& part);
	
	protected:
		double mCurrentLikelihood;
		int mId;
};

#endif
