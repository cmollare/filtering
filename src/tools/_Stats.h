#ifndef _STATS_H
#define _STATS_H

#include "sobol.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

class _Stats
{
	protected:
		void initQMC();
		
		
		//Quasi MonteCarlo
		int mSeedQMC; /*!< Seed for QMC */
		int mNbEchantillonsUnite; /*!< Number of samples for QMC */
		int mNbEchantillons; /*!< Number of samples for QMC */
		double *mGaussCDFInv; /*!< Inverse gaussian CDF */
		double *mGaussPDF; /*!< Gaussian PDF */
		double *mGaussCDF; /*!< Gaussian CDF */
		double mBorneMin; /*!< Lower bound of CDFs */
		double mBorneMax; /*!< Upper bound of CDFs */
		double mPasUnite; /*!< Resolution of inverse Guassian CDF */
		double mPas; /*!< Resolution of Guassian CDF */
		
		float *mVectorQMC; /*!< Current Sobol's sequence */
		int mDimQMCVec; /*!< Dimension of Sobol's sequence */
		int mIndexQMC; /*!< Current Sobol's suequence dimension used */
		
};

#endif
