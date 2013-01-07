#ifndef SIR_H
#define SIR_H

#include "_Filter.h"

class SIR : public _Filter
{
	public:
		SIR(int& nbParticles);
		SIR(int& nbParticles, _Particle& model);
		virtual ~SIR();
		
		virtual void init(_Observation obs);
		virtual void step(_Observation obs);
		virtual void resample();
		virtual void updateWeights();
};

/* ********************************************************* */
/* *******************Implementation************************ */
/* ********************************************************* */

SIR::SIR(int& nbParticles) : _Filter(nbParticles)
{
}

SIR::SIR(int& nbParticles, _Particle& model) : _Filter(nbParticles, model)
{
}

SIR::~SIR()
{
}

void SIR::init(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

void SIR::step(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->update();
		this->mParticles[i]->estimateLikelihood(this->mCurrentObservations);
	}
	this->updateWeights();
	this->estimateMMSE();
	this->resample();

	this->saveResults();

}

void SIR::resample()
{
	double invNbSamp = 1./this->mNbParticles;
	Eigen::VectorXf cdf(this->mNbParticles);
	cdf[0]=this->mCurrentWeights[0];
	for (int i=1 ; i<this->mNbParticles ; i++)
	{
		cdf[i]=cdf[i-1]+this->mCurrentWeights[i];
	}
	
	int i=0;
	double u = this->randUnif(invNbSamp);
	for (int j=0 ; j<this->mNbParticles ; j++)
	{
		while(u>cdf[i])
		{
			i++;
			if (i>=this->mNbParticles)
			{
				i=this->mNbParticles-1;
				break;
			}
		}
		
		*this->mParticles[j] = *this->mParticles[i]; //Careful ! do not equalize particle adresses !
		this->mCurrentWeights[j]=invNbSamp;
		
		u=u+invNbSamp;
	}
}

void SIR::updateWeights()
{
	double sum=0;
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mCurrentWeights[i] = this->mCurrentWeights[i]*this->mParticles[i]->getLikelihood();
		sum += this->mCurrentWeights[i];
	}
	
	this->mCurrentWeights=this->mCurrentWeights/sum;
}

#endif
