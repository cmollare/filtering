#include "_Stats.h"

int _Stats::mNbInstances = 0;
bool _Stats::mIsInitialized = false;
int _Stats::mSeedQMC = ((int)time(NULL))%2000000;
int _Stats::mNbEchantillonsUnite;
int _Stats::mNbEchantillons;
double *_Stats::mGaussCDFInv;
double *_Stats::mGaussPDF;
double *_Stats::mGaussCDF;
double _Stats::mBorneMin;
double _Stats::mBorneMax;
double _Stats::mPasUnite;
double _Stats::mPas;
		
float *_Stats::mVectorQMC;
int _Stats::mDimQMCVec;
int _Stats::mIndexQMC;

_Stats::_Stats()
{
	mNbInstances++;
}

void _Stats::initStatsTool(int dim)
{
	if (!mIsInitialized && mNbInstances>0)
	{
		
		
		//Init QMC
		mNbEchantillonsUnite = 101;
		mPasUnite = 1.0/(mNbEchantillonsUnite-1);
		mBorneMin = -5;
		mBorneMax = 5;
		mNbEchantillons = 101;
		mPas = (mBorneMax-mBorneMin)/(mNbEchantillons-1);
		mGaussCDFInv = new double [mNbEchantillonsUnite];
		mGaussPDF = new double[mNbEchantillons];
		mGaussCDF = new double[mNbEchantillons];
		mDimQMCVec = dim;//mModels[0]->getNumberJoint()*6;
		mVectorQMC = new float[mDimQMCVec];
		
		initQMC();
		
		mIsInitialized=true;
	}
}

_Stats::~_Stats()
{
	if(mNbInstances == 1 && mIsInitialized)
	{
		delete[] mGaussCDFInv;
		delete[] mGaussPDF;
		delete[] mGaussCDF;
		delete[] mVectorQMC;
	}
	mNbInstances--;
}

double _Stats::randUnif(double sup)
{
	int Uint = rand()%10000 + 1;
	double U = (double)Uint / 10000.;
	U*=sup;
	return U;
}

double _Stats::randn(double sigma)
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

Eigen::Quaterniond _Stats::sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1, double sigma2, double sigma3)
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

void _Stats::initQMC()
{

    std::cout << "Initialization of QMC generator\n";
  
    /* Construction de la fonction de distribution cumulee (CDF) d'une gaussienne */
    //std::cout << "\n\nCDF\n";
    for (int i=0; i<mNbEchantillons; i++)
	{
		double x = mBorneMin + ((double)i*mPas);
		mGaussPDF[i] = 1.0/(sqrt(2.0*M_PI))*exp(-0.5*x*x);
		  
		/* Integration par la methode des trapezes */
		if (i==0) mGaussCDF[i] = 0;//(gaussPDF[i]*pas);
		else if (i==mNbEchantillons-1) mGaussCDF[i] = 1;
		else mGaussCDF[i] = mGaussCDF[i-1]+((mGaussPDF[i-1]+mGaussPDF[i])*0.5*mPas);
		  
		//std::cout << x << "\t: " << gaussPDF[i] << " - " << gaussCDF[i] << "\n";
	}
      
      
      
    /* Construction de l'inverse de la CDF d'une gaussienne */
    //std::cout << "\n\nCDFInv\n";
    int   j=0;
	for (int i=0; i<mNbEchantillonsUnite; i++)
	{
		double y = (double)i*mPasUnite;
		while (mGaussCDF[j]<y && j<mNbEchantillons-1) j++;
	  
		if (j>0 && mGaussCDF[j]>=y)
			mGaussCDFInv[i] = mBorneMin+((double)(j-1)*mPas) 
			+ (mPas * (y-mGaussCDF[j-1])/(mGaussCDF[j]-mGaussCDF[j-1]) );
		else if (j==0)
			mGaussCDFInv[i] = mBorneMin;
		else
			mGaussCDFInv[i] = mBorneMax;
		//std::cout << y << "\t: " << gaussCDFInv[i] << "\n";
	}

}

void _Stats::sampleQRS()
{
	i4_sobol(mDimQMCVec,&mSeedQMC,mVectorQMC);
	mIndexQMC=0;
}

double _Stats::quasiRandn(double sigma)
{
	int n = (int)floor(mVectorQMC[mIndexQMC]/mPasUnite);
	double x = mGaussCDFInv[n] + ( (mGaussCDFInv[n+1] - mGaussCDFInv[n]) * (mVectorQMC[mIndexQMC] - ((double)n*mPasUnite)) / mPasUnite);
	mIndexQMC++;
	return x*sigma;
}

Eigen::Quaterniond _Stats::sampleQuasiQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1, double sigma2, double sigma3)
{
	int n[4];
	double x[4];
	
	//Quasi Gaussian law
	for(int i=0 ; i<4 ; i++)
	{
		n[i] = (int)floor(mVectorQMC[mIndexQMC+i]/mPasUnite);
		
		/*if (n[i]==0 || n[i]==mNbEchantillons-2)
		{
			cout << "Error in function sampleQuasiQuTEM" << endl;
			cout << n[i] << endl;
		}*/
		
		x[i] = mGaussCDFInv[n[i]] + ( (mGaussCDFInv[n[i]+1] - mGaussCDFInv[n[i]]) * (mVectorQMC[mIndexQMC+i] - ((double)n[i]*mPasUnite)) / mPasUnite  );
	}
	mIndexQMC += 4;
	
	Eigen::Vector4d axis(0, x[1], x[2], x[3]);
	
	//N
	axis.normalize();
	axis[1]=axis[1]*sigma1;
	axis[2]=axis[2]*sigma2;
	axis[3]=axis[3]*sigma3;
	
	//theta
	double theta = x[0]*sigma;
	
	//exp(N*theta)
	axis=axis*(double)sin(theta);
	axis[0]=(double)cos(theta);
	
	//to quaternion
	Eigen::Quaterniond quat(axis[0], axis[1], axis[2], axis[3]);
	quat = mean*quat;
	
	return quat;
}
