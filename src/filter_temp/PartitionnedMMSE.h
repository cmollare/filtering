#ifndef PARTITIONNEDMMSE_H
#define PARTITIONNEDMMSE_H

#include "_Filter.h"

template<class Particles, class Observations>
class PartitionnedMMSE : public _Filter<Particles, Observations>
{
	public:
		PartitionnedMMSE(int& nbParticles);
		PartitionnedMMSE(int& nbParticles, Particles& model);
		virtual ~PartitionnedMMSE();
		
		virtual void init(Observations& obs);
		virtual void step(Observations& obs);
		virtual void resample();
		virtual void updateWeights();
		virtual void estimateMMSE();
		
	protected:
		int mNumberOfPartitions;
};

/* ********************************************************* */
/* *******************Implementation************************ */
/* ********************************************************* */

template<class Particles, class Observations>
PartitionnedMMSE<Particles, Observations>::PartitionnedMMSE(int& nbParticles) : _Filter<Particles, Observations>(nbParticles)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

template<class Particles, class Observations>
PartitionnedMMSE<Particles, Observations>::PartitionnedMMSE(int& nbParticles, Particles& model) : _Filter<Particles, Observations>(nbParticles, model)
{
	mNumberOfPartitions = this->mParticles[0]->getNumberOfPartitions();
}

template<class Particles, class Observations>
PartitionnedMMSE<Particles, Observations>::~PartitionnedMMSE()
{
}

template<class Particles, class Observations>
void PartitionnedMMSE<Particles, Observations>::init(Observations& obs)
{
	this->mCurrentObservations = obs;
	
	this->mCurrentWeights.setConstant(this->mNbParticles, 1, 1./this->mNbParticles); // Initialisation of weight at 1/N
	
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mParticles[i]->sampleFromPrior();
	}
}

template<class Particles, class Observations>
void PartitionnedMMSE<Particles, Observations>::step(Observations& obs)
{
	this->mCurrentObservations = obs;
	
	for (int l=1 ; l<=mNumberOfPartitions ; l++)
	{
		for (int i=0 ; i<this->mNbParticles ; i++)
		{
			this->mParticles[i]->update(l);
			this->mParticles[i]->esitmateLikelihood(this->mCurrentObservations, l);
			if (l==mNumberOfPartitions)
				cout << this->mParticles[i]->getLikelihood() << endl;
		}
		this->updateWeights();
		this->resample();
	}
}

template<class Particles, class Observations>
void PartitionnedMMSE<Particles, Observations>::resample()
{
	double invNbSamp = 1./this->mNbParticles;
	
	this->estimateMMSE();
	for (int j=0 ; j<this->mNbParticles ; j++)
	{
		*this->mParticles[j] = *this->mParticleMMSE; //Careful ! do not equalize particle adresses !
		this->mCurrentWeights[j]=invNbSamp;
	}
}

template<class Particles, class Observations>
void PartitionnedMMSE<Particles, Observations>::updateWeights()
{
	double sum=0;
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		this->mCurrentWeights[i] = this->mCurrentWeights[i]*this->mParticles[i]->getLikelihood();
		sum += this->mCurrentWeights[i];
	}
	
	this->mCurrentWeights=this->mCurrentWeights/sum;
}

template<class Particles, class Observations>
void PartitionnedMMSE<Particles, Observations>::estimateMMSE()
{
	*this->mParticleMMSE = (this->mCurrentWeights[0]*(*this->mParticles[0]));
	
	for (int i=1 ; i<this->mNbParticles ; i++)
	{
		*this->mParticleMMSE += (this->mCurrentWeights[i]*(*this->mParticles[i]));
	}
	
	this->mParticleMMSE->normalize();
}

#endif

