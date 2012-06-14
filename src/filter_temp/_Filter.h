#ifndef _FILTER_H
#define _FILTER_H

/*!
 * \file _Filter.h
 */
 
#include "../Control.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include "../FileParsers/ResultParser.h"
#include "../particles/_Particle.h"
#include "../tools/_Stats.h"

template<class Particles, class Observations>
class _Filter : public _Stats
{
	public:
		_Filter(int& nbParticles);
		_Filter(int& nbParticles, Particles& model);
		virtual ~_Filter();
		
		virtual void init(Observations& obs) =0;
		virtual void step(Observations& obs) =0;
		virtual void resample() =0;
		virtual void updateWeights() =0;
		virtual void estimateMMSE();
		//virtual computeMAP() =0;
		virtual void saveResults();
		
		std::vector<Particles*> getParticleVector();
		
	protected:
		int mNbParticles;
		Eigen::VectorXd mCurrentWeights;
		Particles** mParticles;
		Particles* mParticleMMSE;
		Particles* mParticleMAP;
		Observations mCurrentObservations;
		
		#ifdef SAVE_MATLAB
		ResultParser *mResParser; //For Matlab results
		#endif
};

template<class Particles, class Observations>
_Filter<Particles, Observations>::_Filter(int& nbParticles) : _Stats()
{
	mNbParticles = nbParticles;
	mParticles = new Particles*[mNbParticles];
	for (int i=0 ; i<this->mNbParticles ; i++)
	{
		mParticles[i] = new Particles();
		mParticles[i]->setId(i);
		mParticles[i]->setColor(1, 0, 1, 0.1);
	}
	mParticleMMSE = new Particles();
	mParticleMMSE->setId(mNbParticles);
	mParticleMMSE->setColor(0, 1, 0, 1);
	
	mParticleMAP = new Particles();
	mParticleMAP->setId(mNbParticles+1);
	mParticleMAP->setColor(1, 0, 0, 1);
	
	#ifdef SAVE_MATLAB
	mResParser = new ResultParser("../output/");
	#endif
}

template<class Particles, class Observations>
_Filter<Particles, Observations>::_Filter(int& nbParticles, Particles& model) : _Stats()
{
	mNbParticles = nbParticles;
	mParticles = new Particles*[mNbParticles];
	for (int i=0 ; i<mNbParticles ; i++)
	{
		mParticles[i] = new Particles(model);
		mParticles[i]->setId(i);
		mParticles[i]->setColor(1, 0, 1, 0.1);
	}
	
	mParticleMMSE = new Particles(model);
	mParticleMMSE->setId(mNbParticles);
	mParticleMMSE->setColor(0, 1, 0, 1);
	
	mParticleMAP = new Particles(model);
	mParticleMAP->setId(mNbParticles+1);
	mParticleMAP->setColor(1, 0, 0, 1);
	
	#ifdef SAVE_MATLAB
	mResParser = new ResultParser("../output/");
	#endif
}

template<class Particles, class Observations>
_Filter<Particles, Observations>::~_Filter()
{
	for (int i=0 ; i<mNbParticles ; i++)
	{
		delete mParticles[i];
	}
	delete[] mParticles;
	delete mParticleMMSE;
	delete mParticleMAP;
	
	#ifdef SAVE_MATLAB
	delete mResParser;
	#endif
}

template<class Particles, class Observations>
std::vector<Particles*> _Filter<Particles, Observations>::getParticleVector()
{
	std::vector<Particles*> parts;
	for (int i=0 ; i<mNbParticles ; i++)
	{
		 parts.push_back(this->mParticles[i]);
	}
	parts.push_back(mParticleMMSE);
	//parts.push_back(mParticleMAP);
	
	return parts;
}

template<class Particles, class Observations>
void _Filter<Particles, Observations>::estimateMMSE()
{
	this->mParticleMMSE->estimateMMSE(mCurrentWeights, mParticles, mNbParticles);
	
	//#ifdef SAVE_MATLAB
	//	this->mParticleMMSE->saveResults(mResParser);
	//#endif
}

template<class Particles, class Observations>
void _Filter<Particles, Observations>::saveResults()
{
	#ifdef SAVE_MATLAB
		this->mParticleMMSE->saveResults(mResParser);
	#endif
}


#endif
