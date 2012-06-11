#include "_Stats.h"

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
