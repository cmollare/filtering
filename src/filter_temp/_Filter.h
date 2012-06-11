#ifndef _FILTER_H
#define _FILTER_H

/*!
 * \file _Filter.h
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include "../3DModel/S3DModel.h"
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
		virtual void step(Observations& obs=NULL) =0;
		virtual void resample() =0;
		virtual void updateWeights() =0;
		//virtual void computeMMSE() =0;
		//virtual computeMAP() =0;
		
		std::vector<Particles*> getParticleVector();
		
	protected:
		int mNbParticles;
		Eigen::VectorXd mCurrentWeights;
		std::vector<Particles*> mParticles;
		Observations mCurrentObservations;
};

template<class Particles, class Observations>
_Filter<Particles, Observations>::_Filter(int& nbParticles)
{
	mNbParticles = nbParticles;
	for (int i=0 ; i<mNbParticles ; i++)
	{
		mParticles.push_back(new Particles());
	}
}

template<class Particles, class Observations>
_Filter<Particles, Observations>::_Filter(int& nbParticles, Particles& model)
{
	mNbParticles = nbParticles;
	for (int i=0 ; i<mNbParticles ; i++)
	{
		mParticles.push_back(new Particles(model));
	}
}

template<class Particles, class Observations>
_Filter<Particles, Observations>::~_Filter()
{
	for (int i=0 ; i<mNbParticles ; i++)
	{
		delete mParticles[i];
	}
}

template<class Particles, class Observations>
std::vector<Particles*> _Filter<Particles, Observations>::getParticleVector()
{
	return this->mParticles;
}

#endif
