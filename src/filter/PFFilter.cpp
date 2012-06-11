#include "PFFilter.h"

PFFilter::PFFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions) : Filter(mods)
{
	mCurrentFrame = jointsXYZPositions;
	mPosNames = posNames;
	
	mMaxWeightIndex=0;
}

void PFFilter::initFilter()
{
	mModelMMSE->setColor(0, 1, 0, 1); //Color of the MMSE model
	mCurrentWeights.setConstant(mModels.size(), 1, 1./mModels.size()); // Initialisation of weight at 1/N
	mCurrentDistances.resize(mModels.size(), 1); // Initialisation of distance vector
	mCurrentLikelihood.resize(mModels.size(), 1); // Initialisation of likelihood vector
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mOrientationVec.push_back(mModels[i]->getOrientationVec());					// Creation of orientation vector<vector>
		mOffsetVec.push_back(mModels[i]->getOffsetVector());						// Creation of offset vector<vector>
		mDefaultOrientationVec.push_back(mModels[i]->getDefaultOrientationVec());	// Creation of default orientation vector<vector>
		mDefaultOffsetVec.push_back(mModels[i]->getDefaultOffsetVector());			// Creation of default offset vector<vector>
		mNameVec.push_back(mModels[i]->getNameVec());								// Creation of Joint name vector<vector>
		mConstOffsetVec.push_back(mModels[i]->getConstOffsetVec());					// Creation of offset dof vector<vector>
		mConstOrientVec.push_back(mModels[i]->getConstOrientVec());					// Creation of orientation dof vector<vector>
		
	}
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			mDefaultOrientationVec[i][j] = (*mOrientationVec[i][j]); // Base orientation becomes default orientation
			mDefaultOffsetVec[i][j] = (*mOffsetVec[i][j]);  // Base offset becomes default offset
			
			Eigen::Quaterniond quat = mDefaultOrientationVec[i][j]; // Mean of orientations == default orientation
			bool invalide = false;
			Eigen::Vector3d offs = mDefaultOffsetVec[i][j].vector(); // Mean of offset == default offset
			do
			{
				invalide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.001, this->randn()*0.001, this->randn()*0.001) + offs;
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), 0, 0) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), this->randn(0.001), 0) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(0.001), this->randn(0.001)) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), 0, this->randn(0.001)) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[i][j]->x() != mOffsetVec[i][j]->x()) || (mOffsetVec[i][j]->y() != mOffsetVec[i][j]->y()) || (mOffsetVec[i][j]->z() != mOffsetVec[i][j]->z()));
				invalide |= (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}
			while(invalide);
			
		}
	}
}

/*void PFFilter::computeDistance()
{
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		double distance=0;
		for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				// Mahalanobis distance
				Eigen::Vector3d jtPos = mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(mCurrentFrame[(*it).second][1], mCurrentFrame[(*it).second][2], mCurrentFrame[(*it).second][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				Eigen::Matrix3d cov;
				cov.setIdentity();
				distance += diff.transpose()*(cov*diff);
			}
		}
		mCurrentDistances[i] = sqrt(distance);
	}
}*/

void PFFilter::computeDistance()
{
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		double distance=0;
		for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				double distTemp=0;
				// Mahalanobis distance
				Eigen::Vector3d jtPos = mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(mCurrentFrame[(*it).second][1], mCurrentFrame[(*it).second][2], mCurrentFrame[(*it).second][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				//Eigen::Matrix3d cov;
				//cov.setIdentity();
				//distTemp = diff.transpose()*(cov*diff);
				distTemp = diff.norm();
				distance += distTemp;
			}
		}
		mCurrentDistances[i] = sqrt(distance);
	}
}

void PFFilter::computeLikelihood()
{	
	//contraintes a ajouter
	this->computeDistance();
	
	for (int i=0 ; i<mCurrentDistances.size() ; i++)
	{
		mCurrentLikelihood[i] = exp(-abs(mCurrentDistances[i]));
	}
	//cout << mCurrentLikelihood << "//" << endl;
}

void PFFilter::step(std::vector<std::vector<double> > frame)
{
	mCurrentFrame = frame;
	//cout << mCurrentDistances[mMaxWeightIndex] << endl;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mModels[i]->setColor(1,0,1,0.1);
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			double variance; // To delete
			variance=1;

			bool invalide = false;
			
			Eigen::Quaterniond quat = (*mOrientationVec[i][j]); // Mean orientation is the previous orientation
			Eigen::Vector3d offs;
			
			if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
			{
				offs = mOffsetVec[i][j]->vector(); // For Free dof, mean is the previous offset
			}
			else
			{
				offs = mDefaultOffsetVec[i][j].vector(); // Mean offset for bones and planar DOF is the default offset
			}
			do
			{
				invalide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO*variance, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO*variance, 0.5, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO*variance, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO*variance, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO*variance, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMPO*variance, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize();
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.05, this->randn()*0.05, this->randn()*0.05) + offs;
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), 0, 0) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), this->randn(0.01), 0) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(0.01), this->randn(0.01)) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), 0, this->randn(0.01)) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[i][j]->x() != mOffsetVec[i][j]->x()) || (mOffsetVec[i][j]->y() != mOffsetVec[i][j]->y()) || (mOffsetVec[i][j]->z() != mOffsetVec[i][j]->z()));
				invalide |= (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}
			while(invalide);
			
			
		}
	}
	
	this->updateWeights();
	this->computeMMSE();
	double Neff = this->computeNeff();;
	//cout << Neff << "****" << endl;
	//if (Neff < 1.5 || Neff < mModels.size()*0.5)
		this->resample(); // systematic resampling
}

double PFFilter::computeNeff()
{
	return 1./(mCurrentWeights.dot(mCurrentWeights));
}

void PFFilter::updateWeights()
{
	double sum=0;
	this->computeLikelihood();
	mModels[mMaxWeightIndex]->setColor(1,0,1,0.1);
	
	for (int i=0 ; i<mCurrentLikelihood.size() ; i++)
	{
		mCurrentWeights[i] = mCurrentWeights[i]*mCurrentLikelihood[i];
		sum+=mCurrentWeights[i];
		
		/*if(mCurrentWeights[i]>=mCurrentWeights[mMaxWeightIndex])
		{
			mMaxWeightIndex=i;
		}*/
	}
	for (int i=0 ; i<mCurrentLikelihood.size() ; i++)
	{
		if(mCurrentWeights[i]>=mCurrentWeights[mMaxWeightIndex])
		{
			mMaxWeightIndex=i;
		}
	}
	
	mCurrentWeights/=sum;
	
	/*for (int i=0 ; i<mModels.size() ; i++)
	{
		mModels[i]->setColor(1-mCurrentWeights[i]/mCurrentWeights[mMaxWeightIndex], mCurrentWeights[i]/mCurrentWeights[mMaxWeightIndex], 0, 1);
		//mCurrentWeights[i] = mCurrentWeights[i]*mCurrentLikelihood[i];
		//sum+=mCurrentWeights[i];
	}*/
	//mModels[mMaxWeightIndex]->setColor(0,1,1,1);
	//cout << mMaxWeightIndex << endl;
}

void PFFilter::resample()
{
	double invNbSamp = 1./mModels.size();
	Eigen::VectorXf cdf(mModels.size());
	cdf[0]=mCurrentWeights[0];
	for (int i=1 ; i<mModels.size() ; i++)
	{
		cdf[i]=cdf[i-1]+mCurrentWeights[i];
	}
	
	int i=0;
	double u = randUnif(invNbSamp);
	for (int j=0 ; j<mModels.size() ; j++)
	{
		while(u>cdf[i])
		{
			i++;
		}
		for (int k=0 ; k<mOrientationVec[i].size() ; k++)
		{
			(*mOrientationVec[j][k])=(*mOrientationVec[i][k]);
			(*mOffsetVec[j][k])=(*mOffsetVec[i][k]);
		}
		mCurrentWeights[j]=invNbSamp;
		u=u+invNbSamp;
	}
}

void PFFilter::mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
{
	vector<std::string> jtNames = mModels[0]->getNameVec();
	mJointNameToPosName = jointNameToPosName;
	for (int i=0 ; i < jtNames.size() ; i++)
	{
		bool found=false;
		mJointNameToInt[jtNames[i]] = i;

		for (int j=0 ; j<mPosNames.size() ; j++)
		{
			if(mJointNameToPosName[jtNames[i]] == mPosNames[j])
			{
				found = true;
				mJointNameToPos[jtNames[i]]=j;
				break;
			}
			else if (mJointNameToPosName[jtNames[i]] == "NULL")
			{
				found = true;
				mJointNameToPos[jtNames[i]]=-1;
				break;
			}
		}
		if (!found)
			cout << jtNames[i] <<" : No match found" << endl;
		
	}
}

S3DModel* PFFilter::computeMMSE()
{
	std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > orient = mModelMMSE->getOrientationVec();
	std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > offset = mModelMMSE->getOffsetVector();
	for (int i=0 ; i<mOrientationVec[0].size() ; i++)
	{
		std::vector<double> quat(4, 0);
		Eigen::Vector3d tempo(0, 0, 0);
		for (int j=0 ; j<mModels.size() ; j++)
		{
			Eigen::Vector3d offs = mOffsetVec[j][i]->vector();
			tempo += mCurrentWeights[j]*offs;
			quat[0] += mCurrentWeights[j]*mOrientationVec[j][i]->w();
			quat[1] += mCurrentWeights[j]*mOrientationVec[j][i]->x();
			quat[2] += mCurrentWeights[j]*mOrientationVec[j][i]->y();
			quat[3] += mCurrentWeights[j]*mOrientationVec[j][i]->z();
		}
		(*orient[i]) = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
		orient[i]->normalize();
		(*offset[i]) = Eigen::Translation3d(tempo);
	}
	
	return mModelMMSE;
}


