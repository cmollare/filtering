#include "SIR.h"

template<class Particles, class Observations>
SIR<Particles, Observations>::SIR(int nbParticles, Particles& model) : _Filter<Particles, Observations>(nbParticles, model)
{
}

template<class Particles, class Observations>
SIR<Particles, Observations>::~SIR()
{
}

template<class Particles, class Observations>
void SIR<Particles, Observations>::init(Observations& obs)
{
	this->mCurrentObservation = obs;
	
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
		}
		//Reecrire avec surcharge operateurs
		/*for (int k=0 ; k<mOrientationVec[i].size() ; k++)
		{
			
		}*/
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
	
	this->mCurrentWeights/sum;
}
