#include "IKSolver.h"

IKSolver::IKSolver(std::vector<S3DModel*> mods)
{
	srand (time(NULL));//Initialisation of randon numbers
	mModels = mods;
	mRootName = mods[0]->getRootJoint()->getName();
}

Eigen::Quaterniond IKSolver::sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1, double sigma2, double sigma3)
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

double IKSolver::randn(double sigma)
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

double IKSolver::randUnif(double sup)
{
	int Uint = rand()%10001;
	double U = (double)Uint / 10001.;
	U*=sup;
	return U;
}

bool IKSolver::isPositive(double num)
{
	return num >= 0;
}

