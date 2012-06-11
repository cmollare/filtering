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

template<class Particles, class Observations>
class _Filter
{
	public:
		_Filter(int nbParticles, Particles& model=NULL);
		virtual ~_Filter();
		
		virtual void init() =0;
		virtual void computeWeights() =0;
		virtual void computeMMSE() =0;
		//virtual computeMAP() =0;
		
	protected:
		int mNbParticles;
		std::vector<Particles*> mParticles;	
};

#endif
