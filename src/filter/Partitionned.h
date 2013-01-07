#ifndef PARTITIONNED_H
#define PARTITIONNED_H

#include "_Filter.h"

class Partitionned : public _Filter
{
	public:
		Partitionned(int& nbParticles);
		Partitionned(int& nbParticles, _Particle& model);
		virtual ~Partitionned();
		
		virtual void init(_Observation obs);
		virtual void step(_Observation obs);
		virtual void resample();
		virtual void updateWeights();
		
	protected:
		int mNumberOfPartitions;
};

/* ********************************************************* */
/* *******************Implementation************************ */
/* ********************************************************* */

Partitionned::Partitionned(int& nbParticles) : _Filter(nbParticles)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

Partitionned::Partitionned(int& nbParticles, _Particle& model) : _Filter<Particles>(nbParticles, model)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

Partitionned::~Partitionned()
{
}

void Partitionned::init(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

void Partitionned::step(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	for (int l=1 ; l<=mNumberOfPartitions ; l++)
	{
		for (int i=0 ; i<this->mNbParticles ; i++)
		{
			this->mParticles[i]->update(l);
			this->mParticles[i]->estimateLikelihood(this->mCurrentObservations, l);
		}
		this->updateWeights();
		
		if (l<mNumberOfPartitions)
			this->resample();
	}
	this->estimateMMSE();

	this->saveResults();
}

void Partitionned::resample()
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

void Partitionned::updateWeights()
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
