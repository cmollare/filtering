#ifndef SIR_H
#define SIR_H

#include "_Filter.h"

template<class Particles, class Observations>
class SIR : public _Filter<Particles, Observations>
{
	public:
		SIR(int& nbParticles);
		SIR(int& nbParticles, Particles& model);
		virtual ~SIR();
		
		virtual void init(Observations& obs);
		virtual void step(Observations& obs);
		virtual void resample();
		virtual void updateWeights();
};

/* ********************************************************* */
/* *******************Implementation************************ */
/* ********************************************************* */

template<class Particles, class Observations>
SIR<Particles, Observations>::SIR(int& nbParticles) : _Filter<Particles, Observations>(nbParticles)
{
}

template<class Particles, class Observations>
SIR<Particles, Observations>::SIR(int& nbParticles, Particles& model) : _Filter<Particles, Observations>(nbParticles, model)
{
}

template<class Particles, class Observations>
SIR<Particles, Observations>::~SIR()
{
}

template<class Particles, class Observations>
void SIR<Particles, Observations>::init(Observations& obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

template<class Particles, class Observations>
void SIR<Particles, Observations>::step(Observations& obs)
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

template<class Particles, class Observations>
void SIR<Particles, Observations>::resample()
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

template<class Particles, class Observations>
void SIR<Particles, Observations>::updateWeights()
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
