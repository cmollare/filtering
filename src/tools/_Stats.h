#ifndef _STATS_H
#define _STATS_H

#include "sobol.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

class _Stats
{
	public:
		_Stats();
		void initStatsTool(int dim);
		virtual ~_Stats();
		
	protected:
		double randUnif(double sup=1);
		double randn(double sigma=1);
		Eigen::Quaterniond sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1);
		
		//QRS functions
		void initQMC();
		void sampleQRS();
		double quasiRandn(double sigma=1);
		Eigen::Quaterniond sampleQuasiQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1);
		
		//Quasi MonteCarlo
		static int mSeedQMC; /*!< Seed for QMC */
		static int mNbEchantillonsUnite; /*!< Number of samples for QMC */
		static int mNbEchantillons; /*!< Number of samples for QMC */
		static double *mGaussCDFInv; /*!< Inverse gaussian CDF */
		static double *mGaussPDF; /*!< Gaussian PDF */
		static double *mGaussCDF; /*!< Gaussian CDF */
		static double mBorneMin; /*!< Lower bound of CDFs */
		static double mBorneMax; /*!< Upper bound of CDFs */
		static double mPasUnite; /*!< Resolution of inverse Guassian CDF */
		static double mPas; /*!< Resolution of Guassian CDF */
		
		static float *mVectorQMC; /*!< Current Sobol's sequence */
		static int mDimQMCVec; /*!< Dimension of Sobol's sequence */
		static int mIndexQMC; /*!< Current Sobol's suequence dimension used */
		
		static bool mIsInitialized;
		static int mNbInstances;
};

#endif
