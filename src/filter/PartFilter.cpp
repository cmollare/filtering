#include "PartFilter.h"

PartFilter::PartFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions) : Filter(mods)
{
	mCurrentFrame = jointsXYZPositions;
	mPosNames = posNames;
	
	mMaxWeightIndex=0;
}

void PartFilter::initFilter()
{
	mModelMMSE->setColor(0, 1, 0, 1); //Color of the MMSE model
	mCurrentWeights.setConstant(mModels.size(), 1, 1./mModels.size()); // Initialisation of weight at 1/N
	mCurrentDistances.resize(mModels.size(), 1); // Initialisation of distance vector
	mCurrentLikelihood.resize(mModels.size(), 1); // Initialisation of likelihood vector
	
	mPartitionNumber = mModels[0]->getPartitionNumber();
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mModels[i]->setColor(1, 0, 1, 0.1);
		mModels[i]->setVisible(false);
		mOrientationVec.push_back(mModels[i]->getOrientationVec());					// Creation of orientation vector<vector>
		mOffsetVec.push_back(mModels[i]->getOffsetVector());						// Creation of offset vector<vector>
		mDefaultOrientationVec.push_back(mModels[i]->getDefaultOrientationVec());	// Creation of default orientation vector<vector>
		mDefaultOffsetVec.push_back(mModels[i]->getDefaultOffsetVector());			// Creation of default offset vector<vector>
		mNameVec.push_back(mModels[i]->getNameVec());								// Creation of Joint name vector<vector>
		mConstOffsetVec.push_back(mModels[i]->getConstOffsetVec());					// Creation of offset dof vector<vector>
		mConstOrientVec.push_back(mModels[i]->getConstOrientVec());					// Creation of orientation dof vector<vector>
		mOffsetPartToName.push_back(mModels[i]->getOffsetPartitionMultimap());		// Creation of offset partition vector<multimap>
		mOrientPartToName.push_back(mModels[i]->getOrientPartitionMultimap());		// Creation of orientation partition vector<multimap>
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
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMP, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMP, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMP, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMP, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMP, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, TEMP, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn()*0.001, this->randn()*0.001, this->randn()*0.001) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
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

void PartFilter::computeDistance()
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
}

void PartFilter::computeDistance(int partition)
{
	std::multimap<int, std::string>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		double distance=0;
		it = mOffsetPartToName[i].find(partition);
		for (it = mOffsetPartToName[i].equal_range(partition).first ; it != mOffsetPartToName[i].equal_range(partition).second ; ++it)
		{
			if (mJointNameToPos[(*it).second] != -1)
			{
				int pos = mJointNameToPos[(*it).second];
				double distTemp=0;
				// Mahalanobis distance
				//cout << (*it).second << "=>" << mPosNames[pos] << endl;
				Eigen::Vector3d jtPos = mModels[i]->getJoint((*it).second)->getXYZVect();
				Eigen::Vector3d jtObs(mCurrentFrame[pos][1], mCurrentFrame[pos][2], mCurrentFrame[pos][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				Eigen::Matrix3d cov;
				cov.setIdentity();
				distTemp = diff.transpose()*(cov*diff);
				distance += distTemp;
			}
		}
		mCurrentDistances[i] = distance;
	}
}

void PartFilter::computeLikelihood()
{	
	//contraintes a ajouter
	this->computeDistance();
	
	for (int i=0 ; i<mCurrentDistances.size() ; i++)
	{
		mCurrentLikelihood[i] = exp(-abs(mCurrentDistances[i]));
	}
	//cout << mCurrentLikelihood << "//" << endl;
}

void PartFilter::computeLikelihood(int partition)
{
	for (int i=0 ; i<mCurrentDistances.size() ; i++)
	{
			mCurrentLikelihood[i] = 1;
	}
	
	for (int i=1 ; i<=partition ; i++)
	{
		this->computeDistance(i);
		for (int i=0 ; i<mCurrentDistances.size() ; i++)
		{
			mCurrentLikelihood[i] *= exp(-abs(mCurrentDistances[i])*20);
		}
	}
}

void PartFilter::step(std::vector<std::vector<double> > frame)
{
	mCurrentFrame = frame;// Observation update

	for (int j=1 ; j<=mPartitionNumber ; j++)//Partition loop
	{
		for (int i=0 ; i<mModels.size() ; i++)//Particle loop
		{
			std::multimap<int, std::string>::iterator itOff = mOffsetPartToName[i].find(j);
			std::multimap<int, std::string>::iterator itOrient = mOrientPartToName[i].find(j);
			
			//Quaternion sampling
			for (itOrient = mOrientPartToName[i].equal_range(j).first ; itOrient != mOrientPartToName[i].equal_range(j).second ; ++itOrient)
			{
				double variance; // To delete
				variance=1;
				int pos = mJointNameToInt[(*itOrient).second];// Retrieve position of the Joint in orientation vectors
				

				bool invalide = false;
				//Eigen::Quaterniond quat = mDefaultOrientationVec[i][pos];
				Eigen::Quaterniond quat = (*mOrientationVec[i][pos]); // Mean orientation is the previous orientation
				
				do
				{
					invalide = false;
					if (mConstOrientVec[i][pos] == ORIENT_CONST_FREE)
					{
						(*mOrientationVec[i][pos])=this->sampleQuTEM(quat, TEMP*2, 1, 1, 1);//A modifier suivant les contraintes
					}
					else if(mConstOrientVec[i][pos] == ORIENT_CONST_TWIST)
					{
						(*mOrientationVec[i][pos])=this->sampleQuTEM(quat, TEMP*variance, 0.5, 0.1, 0.05);
					}
					else if(mConstOrientVec[i][pos] == ORIENT_CONST_FLEX)
					{
						(*mOrientationVec[i][pos])=this->sampleQuTEM(quat, TEMP, 0.1, 1, 0.05);
					}
					else if(mConstOrientVec[i][pos] == ORIENT_CONST_TFLEX)
					{
						(*mOrientationVec[i][pos])=this->sampleQuTEM(quat, TEMP, 1, 1, 0.1);
					}
					else if(mConstOrientVec[i][pos] == ORIENT_CONST_BIFLEX)
					{
						(*mOrientationVec[i][pos])=this->sampleQuTEM(quat, TEMP, 0.1, 1, 1);
					}
					else if(mConstOrientVec[i][pos] == ORIENT_CONST_FIXED)
					{
						(*mOrientationVec[i][pos]) = (*mOrientationVec[i][pos]);
					}
					else
					{
						(*mOrientationVec[i][pos])=this->sampleQuTEM(quat, TEMP*variance, 1, 1, 1);
					}
					mOrientationVec[i][pos]->normalize();
					
					invalide |= (mOrientationVec[i][pos]->w() != mOrientationVec[i][pos]->w());
				}
				while(invalide);
			}// End of quaternion sampling
			
			
			// Offset sampling
			for (itOff = mOffsetPartToName[i].equal_range(j).first ; itOff != mOffsetPartToName[i].equal_range(j).second ; ++itOff)
			{
				
				int pos = mJointNameToInt[(*itOff).second];// Retrieve position of the Joint in offset vectors
				
				Eigen::Vector3d offs;
				if (mConstOffsetVec[i][pos] == OFFSET_CONST_FREE)
				{
					offs = mOffsetVec[i][pos]->vector(); // For Free dof, mean is the previous offset
				}
				else
				{
					offs = mDefaultOffsetVec[i][pos].vector(); // Mean offset for bones and planar DOF is the default offset
				}
				
				bool invalide = false;
				
				do
				{
					invalide = false;
					
					Eigen::Vector3d tempo;
					if (mConstOffsetVec[i][pos] == OFFSET_CONST_FREE)
					{
						tempo = Eigen::Vector3d(this->randn()*0.05, this->randn()*0.05, this->randn()*0.05) + offs;
					}
					else if (mConstOffsetVec[i][pos] == OFFSET_CONST_BONE)
					{

						do
						{
							tempo = Eigen::Vector3d(this->randn(0.001), 0, 0) + offs;
						}
						while(!mModels[i]->getJoint(mNameVec[i][pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[i][pos] == OFFSET_CONST_PLANARXY)
					{
						do
						{
							tempo = Eigen::Vector3d(this->randn(0.01), this->randn(0.01), 0) + offs;
						}
						while(!mModels[i]->getJoint(mNameVec[i][pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[i][pos] == OFFSET_CONST_PLANARYZ)
					{
						do
						{
							tempo = Eigen::Vector3d(0, this->randn(0.01), this->randn(0.01)) + offs;
						}
						while(!mModels[i]->getJoint(mNameVec[i][pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[i][pos] == OFFSET_CONST_PLANARXZ)
					{
						do
						{
							tempo = Eigen::Vector3d(this->randn(0.01), 0, this->randn(0.01)) + offs;
						}
						while(!mModels[i]->getJoint(mNameVec[i][pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[i][pos] == OFFSET_CONST_FIXED)
					{
						tempo = Eigen::Vector3d(0, 0, 0) + offs;
					}
					(*mOffsetVec[i][pos])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

					//To avoid infinite and NaN cases
					invalide |= ((mOffsetVec[i][pos]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][pos]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][pos]->z() == std::numeric_limits<double>::infinity()));
					invalide |= ((mOffsetVec[i][pos]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][pos]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][pos]->z() == -std::numeric_limits<double>::infinity()));
					invalide |= ((mOffsetVec[i][pos]->x() != mOffsetVec[i][pos]->x()) || (mOffsetVec[i][pos]->y() != mOffsetVec[i][pos]->y()) || (mOffsetVec[i][pos]->z() != mOffsetVec[i][pos]->z()));
				}
				while(invalide);
			}// End of offset sampling
		}// End of particle loop
		
		this->updateWeights(j);
		this->resample();
		
	}// End of partition loop
	

	this->computeMMSE();

}

double PartFilter::computeNeff()
{
	return 1./(mCurrentWeights.dot(mCurrentWeights));
}

void PartFilter::updateWeights(int partition)
{
	double sum=0;
	this->computeLikelihood(partition);
	
	for (int i=0 ; i<mCurrentLikelihood.size() ; i++)
	{
		mCurrentWeights[i] = mCurrentWeights[i]*mCurrentLikelihood[i];
		sum+=mCurrentWeights[i];
	}
	
	mCurrentWeights/=sum;

}

void PartFilter::resample()
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

void PartFilter::mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
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

S3DModel* PartFilter::computeMMSE()
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
	
	this->saveMMSE();
	this->saveObservations();
	
	return mModelMMSE;
}
