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
//#include "sobol.h"
#include "../FileParsers/ResultParser.h"

template<class Particles>
class _Filter
{
	public:
		_Filter(int nbParticles, Particles* model=NULL);
		
		virtual void computeWeights() =0;
		virtual void computeMMSE() =0;
		//virtual computeMAP() =0;
		
};

#endif
