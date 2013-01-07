#ifndef PARTITIONNEDMMSE_H
#define PARTITIONNEDMMSE_H

#include "_Filter.h"

class PartitionnedMMSE : public _Filter
{
	public:
		PartitionnedMMSE(int& nbParticles);
		PartitionnedMMSE(int& nbParticles, _Particle& model);
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

PartitionnedMMSE::PartitionnedMMSE(int& nbParticles) : _Filter(nbParticles)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

PartitionnedMMSE::PartitionnedMMSE(int& nbParticles, _Particle& model) : _Filter(nbParticles, model)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

PartitionnedMMSE::~PartitionnedMMSE()
{
}

void PartitionnedMMSE::init(_Observation obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

void PartitionnedMMSE::step(_Observation obs)
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

void PartitionnedMMSE::resample()
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

void PartitionnedMMSE::updateWeights()
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

