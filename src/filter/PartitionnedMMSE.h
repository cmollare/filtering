#ifndef PARTITIONNEDMMSE_H
#define PARTITIONNEDMMSE_H

#include "_Filter.h"

template<class Particles>
class PartitionnedMMSE : public _Filter<Particles>
{
	public:
		PartitionnedMMSE(int& nbParticles);
		PartitionnedMMSE(int& nbParticles, Particles& model);
		virtual ~PartitionnedMMSE();
		
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

template<class Particles>
PartitionnedMMSE<Particles>::PartitionnedMMSE(int& nbParticles) : _Filter<Particles>(nbParticles)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

template<class Particles>
PartitionnedMMSE<Particles>::PartitionnedMMSE(int& nbParticles, Particles& model) : _Filter<Particles>(nbParticles, model)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

template<class Particles>
PartitionnedMMSE<Particles>::~PartitionnedMMSE()
{
}

template<class Particles>
void PartitionnedMMSE<Particles>::init(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

template<class Particles>
void PartitionnedMMSE<Particles>::step(_Observation obs)
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
		this->resample();
	}
	
	this->saveResults();
	
}

template<class Particles>
void PartitionnedMMSE<Particles>::resample()
{
	double invNbSamp = 1./this->mNbParticles;
	
	this->estimateMMSE();
	this->mParticleMMSE->setLikelihood(1.);
	for (int j=0 ; j<this->mNbParticles ; j++)
	{
		*this->mParticles[j] = *this->mParticleMMSE; //Careful ! do not equalize particle adresses !
		this->mCurrentWeights[j]=invNbSamp;
	}
}

template<class Particles>
void PartitionnedMMSE<Particles>::updateWeights()
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

