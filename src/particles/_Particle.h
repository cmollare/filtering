#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <Eigen/Dense>
#include <vector>
#include "_Observation.h"
#include "../FileParsers/ResultParser.h"

//template<class Observations>
class _Particle
{
	public:
		_Particle();
		_Particle(const _Particle& particle); //copy constructor
		
		virtual void update(int partition=-1) =0;
		virtual void estimateLikelihood(_Observation& obs, int partition=-1) =0;
		
		virtual void saveResults(ResultParser* resParser);
		virtual void estimateMMSE(Eigen::VectorXd& weights, _Particle** particles, int nbParticles);
		
		virtual double getLikelihood();
		virtual void setLikelihood(double likelihood);
		virtual void setId(int id);
		virtual int getId();
		virtual void setColor(float R=1, float G=0, float B=1, float alpha=0.1);
		virtual void setVisible(bool visible);
		virtual bool isVisible();
		virtual _Particle& operator =(const _Particle& part);
		virtual int getNumberOfPartitions();
		virtual _Observation& getCurrentObservations();
	
	protected:
		double mCurrentLikelihood;
		int mId;
		std::vector<float> mColor;
		bool mIsVisible;
		
		int mNumberOfPartitions;
		_Observation mObservations;
};

#endif
