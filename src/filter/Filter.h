#ifndef FILTER_H
#define FILTER_H

/*!
 * \file Filter.h
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
#include "../tools/sobol.h"
#include "../FileParsers/ResultParser.h"

/*!
 * \class Filter
 * \brief Abstract class for the posture filter
 */
class Filter
{
	public:
		/*!
		 * \fn Filter(std::vector<S3DModel*> mods)
		 * \brief Constructor of class Filter.
		 * \param mods Vector of pointers on class S3DModel.
		 */
		Filter(std::vector<S3DModel*> mods);
		
		/*!
		 * \fn ~Filter()
		 * \brief Destructor of Filter class.
		 */
		~Filter();
		
		/*!
		 * \fn virtual void initFilter()=0
		 * \brief Abstract method for filter initialization.
		 */
		virtual void initFilter()=0;
		
		/*!
		 * \fn virtual void computeLikelihood()=0
		 * \brief Abstract method for likelihood computation.
		 */
		virtual void computeLikelihood()=0;
		
		/*!
		 * \fn virtual void step(std::vector<std::vector<double> > frame)=0
		 * \brief Abstract method for filtering step.
		 * \param frame New frame;
		 */
		virtual void step(std::vector<std::vector<double> > frame)=0;
		
		/*!
		 * \fn Eigen::Quaterniond sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1)
		 * \brief Sampling following a QuTEM law.
		 * \param mean Mean quaternion.
		 * \param sigma Rotation deviation.
		 * \param sigma1 X component deviation.
		 * \param sigma2 Y component deviation.
		 * \param sigma3 Z component deviation.
		 * \return Sampled quaternion.
		 */
		Eigen::Quaterniond sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1);
		
		/*!
		 * \fn void sampleQRS()
		 * \brief Sample a Sobol sequence
		 * 
		 * Function based on Mathias Fontmarty thesis.
		 * 
		 */
		void sampleQRS();
		
		/*!
		 * \fn Eigen::Quaterniond sampleQuasiQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1)
		 * \brief Sampling following a quasi QuTEM law.
		 * 
		 * Each normal sampling used for QuTEM sampling is quasi gaussian.
		 * Function based on Mathias Fontmarty thesis.
		 * 
		 * \param mean Mean quaternion.
		 * \param sigma Rotation deviation.
		 * \param sigma1 X component deviation.
		 * \param sigma2 Y component deviation.
		 * \param sigma3 Z component deviation.
		 * \return Sampled quaternion.
		 */
		Eigen::Quaterniond sampleQuasiQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1);
		
	protected:
	
		/*!
		 * \fn virtual void computeDistance()=0
		 * \brief Abstract method for distance computation.
		 */
		virtual void computeDistance()=0;
		
		/*!
		 * \fn virtual bool isPositive(double num)
		 * \brief Test if the number is positive.
		 * \param num Number to test.
		 * \return True if positive.
		 */
		virtual bool isPositive(double num);
		
		/*!
		 * \fn double randn(double sigma=1)
		 * \brief Sampling from a normal distribution using the Box-Muller algorithm
		 * \param sigma Deviation of normal law.
		 * \return A float sampled following a normal distribution.
		 */
		double randn(double sigma=1);
		
		/*!
		 * \fn double randn(double sigma=1)
		 * \brief Sampling from a quasi normal distribution.
		 * 
		 * This function is based on Mathias Fontmarty thesis.
		 * 
		 * \param sigma Deviation of normal law.
		 * \return A float sampled following a normal distribution.
		 */
		double quasiRandn(double sigma=1);
		
		/*!
		 * \fn double randUnif(double sup=1.0)
		 * \brief Sampling following uniform distribution between [0-sup]
		 * \param sup Upper bound of the ditribution.
		 * \return Sampled number.
		 */
		double randUnif(double sup=1.0);
		
		/*!
		 * \fn void initQMC()
		 * \brief Sampling of gaussian and inverse gaussian CDF.
		 */
		void initQMC();
		
		/*!
		 * \fn void saveMMSE()
		 * \brief Save MMSE caracteristics.
		 */
		void saveMMSE();
		
		/*!
		 * \fn void saveObservations()
		 * \brief Save observations caracteristics.
		 */
		void saveObservations();
		
		std::map<std::string, std::string> mJointNameToPosName; /*!< Map between Joint Names file and animation file */
		std::map<std::string, int> mJointNameToPos; /*!< Name of the Joint to its position in the observation vector, -1 if there is no observation associated */
		std::map<std::string, int> mJointNameToInt; /*!< Name of the Joint to its position in the orientation vector */
		std::vector<std::string> mPosNames; /*!< Names of observations */
		
		std::vector<S3DModel*> mModels; /*!< Vector of particules */
		S3DModel* mModelMMSE; /*!< MMSE Model (filtering result) */
		std::vector<std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > > mOrientationVec; /*!< Orientation vector for each particule */
		std::vector<std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > > mOffsetVec; /*!< Offset vector for each particule */
		std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > > mDefaultOrientationVec; /*!< Default orientation vector for each particule */
		std::vector<std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > > mDefaultOffsetVec; /*!< Default offset vector for each particule */
		std::vector<std::vector<std::string> > mNameVec; /*!< Vector of names for each particule */
		std::vector<std::vector<std::string> > mConstOffsetVec; /*!< Vector of offset DOF for each particule */
		std::vector<std::vector<std::string> > mConstOrientVec; /*!< Vector of orientation DOF for each particule */
		
		std::vector<std::vector<double> > mCurrentFrame; /*!< Current frame (observations) */
		Eigen::VectorXf mCurrentDistances; /*!< Current distances */
		Eigen::VectorXf mCurrentLikelihood; /*!< Current Likelihood */
		
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
		
		ResultParser *mResParser; /*!< Instance of ResultParser */
};

#endif
