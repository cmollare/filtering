#ifndef IKSOLVER_H
#define IKSOLVER_H

/*!
 * \file IKSolver.h
 */
 
#define PI 0.06

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

using namespace std;

/*!
 * \class IKSolver
 * \brief Abstract class for the inverse kinematic solver
 */
 
template<class Model>
class IKSolver
{
	public:
		IKSolver(std::vector<Model*> mods);
		
		virtual void initFilter()=0;
		//virtual void init()=0;
		virtual void computeLikelihood()=0;
		virtual void step()=0;
		Eigen::Quaterniond sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1);
		
	protected:
		virtual void computeDistance()=0;
		virtual bool isPositive(double num);
		
		/*!
		 * \fn float randn()
		 * \brief Sampling from a normal distribution using the Box-Muller algorithm
		 * \return A float sampled following a normal distribution.
		 */
		double randn(double sigma=1);
		double randUnif(double sup=1.0);
		
		std::map<std::string, std::string> mJointNameToPosName; /*!< Map between Joint Names file and animation file */
		std::map<std::string, int> mJointNameToPos;
		std::map<std::string, int> mJointNameToInt; /*!< Name of the Joint to its position in the orientation vector */
		std::vector<std::string> mPosNames;
		std::string mRootName;
		
		std::vector<Model*> mModels;
		std::vector<std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > > mOrientationVec;
		std::vector<std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > > mOffsetVec;
		std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > > mDefaultOrientationVec;
		std::vector<std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > > mDefaultOffsetVec;
		std::vector<std::vector<std::string> > mNameVec;
		std::vector<std::vector<std::string> > mConstOffsetVec;
		std::vector<std::vector<std::string> > mConstOrientVec;
		
		std::vector<std::vector<double> > mCurrentFrame;
		Eigen::VectorXf mCurrentDistances;
		Eigen::VectorXf mCurrentLikelihood;
};

#include "IKSolver.h"

template<class Model>
IKSolver<Model>::IKSolver(std::vector<Model*> mods)
{
	srand (time(NULL));//Initialisation of randon numbers
	mModels = mods;
	mRootName = mods[0]->getRootJoint()->getName();
}

template<class Model>
Eigen::Quaterniond IKSolver<Model>::sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1, double sigma2, double sigma3)
{
	Eigen::Vector4d axis(0, this->randn(), this->randn(), this->randn());
	
	//N
	axis.normalize();
	axis[1]=axis[1]*sigma1;
	axis[2]=axis[2]*sigma2;
	axis[3]=axis[3]*sigma3;
	
	//theta
	double theta;
	do
	{
		theta = this->randn()*sigma;
		//theta = abs(theta);
	}
	while(sigma < abs(theta));
	
	//exp(N*theta)
	axis=axis*(double)sin(theta);
	axis[0]=(double)cos(theta);
	
	//to quaternion
	Eigen::Quaterniond quat(axis[0], axis[1], axis[2], axis[3]);
	quat = mean*quat;
	
	return quat;
}

template<class Model>
double IKSolver<Model>::randn(double sigma)
{
	/*int U1int = rand()%10001;
	int U2int = rand()%10001;
	float U1 = (float)U1int / 10001.;
	float U2 = (float)U2int / 10001.;*/
	double U1, U2, X, Y;
	do
	{
		U1 = randUnif();
		U2 = randUnif();
		
		X = sqrt(-2*log(U1))*cos(2*3.14*U2);
		Y = sqrt(-2*log(U1))*sin(2*3.14*U2);
		X *= sigma;
		Y *= sigma;
	}
	while(X != X);
	return X;
}

template<class Model>
double IKSolver<Model>::randUnif(double sup)
{
	int Uint = rand()%10001;
	double U = (double)Uint / 10001.;
	U*=sup;
	return U;
}

template<class Model>
bool IKSolver<Model>::isPositive(double num)
{
	return num >= 0;
}



#endif
