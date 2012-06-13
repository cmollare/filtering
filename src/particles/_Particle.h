#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <Eigen/Dense>
#include <vector>

template<class Observations>
class _Particle
{
	public:
		_Particle()
		{
			mColor = std::vector<float>(4,1);
			this->mCurrentLikelihood=0;
			mIsVisible = true;
			mNumberOfPartitions=0;
		}
		_Particle(const _Particle& particle) //copy constructor
		{
			this->mColor = particle.mColor;
			this->mCurrentLikelihood = particle.mCurrentLikelihood;
			this->mIsVisible = particle.mIsVisible;
			this->mNumberOfPartitions = particle.mNumberOfPartitions;
		}
		
		virtual void update(int partition=-1) =0;
		virtual void estimateLikelihood(Observations& obs, int partition=-1) =0;
		
		virtual void estimateMMSE(Eigen::VectorXd& weights, _Particle** particles, int nbParticles)
		{
		}
		
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
		
		virtual void setColor(float R=1, float G=0, float B=1, float alpha=0.1)
		{
			mColor[0] = R;
			mColor[1] = G;
			mColor[2] = B;
			mColor[3] = alpha;
		}
		
		virtual void setVisible(bool visible)
		{
			mIsVisible = visible;
		}
		
		virtual bool isVisible()
		{
			return mIsVisible;
		}
		
		virtual _Particle<Observations>& operator =(const _Particle<Observations>& part)
		{
			this->mCurrentLikelihood = part.mCurrentLikelihood;
		}
		
		virtual int getNumberOfPartitions()
		{
			return mNumberOfPartitions;
		}
	
	protected:
		double mCurrentLikelihood;
		int mId;
		std::vector<float> mColor;
		bool mIsVisible;
		
		int mNumberOfPartitions;
};

#endif
