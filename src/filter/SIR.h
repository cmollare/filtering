#ifndef SIR_H
#define SIR_H

#include "_Filter.h"

template<class Particles>
class SIR : public _Filter<Particles>
{
	public:
		SIR(int& nbParticles);
		SIR(int& nbParticles, Particles& model);
		virtual ~SIR();
		
		virtual void init(_Observation obs);
		virtual void step(_Observation obs);
		virtual void resample();
		virtual void updateWeights();
};

/* ********************************************************* */
/* *******************Implementation************************ */
/* ********************************************************* */

template<class Particles>
SIR<Particles>::SIR(int& nbParticles) : _Filter<Particles>(nbParticles)
{
}

template<class Particles>
SIR<Particles>::SIR(int& nbParticles, Particles& model) : _Filter<Particles>(nbParticles, model)
{
}

template<class Particles>
SIR<Particles>::~SIR()
{
}

template<class Particles>
void SIR<Particles>::init(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

template<class Particles>
void SIR<Particles>::step(_Observation obs)
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

template<class Particles>
void SIR<Particles>::resample()
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

template<class Particles>
void SIR<Particles>::updateWeights()
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
