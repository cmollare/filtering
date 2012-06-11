#include "IKSolverPFOrient.h"

IKSolverPFOrient::IKSolverPFOrient(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions) : IKSolver(mods)
{
	mCurrentFrame = jointsXYZPositions;
	mPosNames = posNames;
	
	mMaxWeightIndex=0;
}

void IKSolverPFOrient::initFilter()
{
	mCurrentWeights.setConstant(mModels.size(), 1, 1./mModels.size());
	mCurrentDistances.resize(mModels.size(), 1);
	mCurrentLikelihood.resize(mModels.size(), 1);
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mOrientationVec.push_back(mModels[i]->getOrientationVec());
		mOffsetVec.push_back(mModels[i]->getOffsetVector());
		mDefaultOrientationVec.push_back(mModels[i]->getDefaultOrientationVec());
		mDefaultOffsetVec.push_back(mModels[i]->getDefaultOffsetVector());
		mNameVec.push_back(mModels[i]->getNameVec());
		mConstOffsetVec.push_back(mModels[i]->getConstOffsetVec());
		mConstOrientVec.push_back(mModels[i]->getConstOrientVec());
		
	}
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			
			Eigen::Quaterniond quat = mDefaultOrientationVec[i][j];
			//quat.setIdentity();
			bool invalide = false;
			Eigen::Vector3d offs = mDefaultOffsetVec[i][j].vector();
			do
			{
				invalide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize();
				
				
				Eigen::Vector3d tempo;
				if (mNameVec[i][j] == mRootName)
				{
					vector<double> vec = mCurrentFrame[mJointNameToPos[mNameVec[i][j]]];
					tempo=Eigen::Vector3d(vec[1], vec[2], vec[3]);
				}
				else if(mJointNameToPos[mNameVec[i][j]] != -1 && mJointNameToPos[mModels[i]->getJoint(mNameVec[i][j])->getParent()->getName()] != -1)
				{
					std::string parentName = mModels[i]->getJoint(mNameVec[i][j])->getParent()->getName();
					vector<double> vec = mCurrentFrame[mJointNameToPos[mNameVec[i][j]]];
					vector<double> vecParent = mCurrentFrame[mJointNameToPos[parentName]];
					if(mConstOffsetVec[i][j] == OFFSET_CONST_FREE || mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY || mConstOffsetVec[i][j] == OFFSET_CONST_PLANARYZ || mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXZ || mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
					{
						tempo=Eigen::Vector3d(vec[1]-vecParent[1], vec[2]-vecParent[2], vec[3]-vecParent[3]);
					}
					else if(mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
					{
						double dist = abs(vec[1]-vecParent[1]) + abs(vec[2]-vecParent[2]) + abs(vec[3]-vecParent[3]);
						tempo=Eigen::Vector3d(dist, 0, 0);
					}
					else
					{
						tempo=Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01);
					}
				}
				else
				{
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01) + offs;
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.1), 0, 0) + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
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
				}
				(*mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[i][j]->x() != mOffsetVec[i][j]->x()) || (mOffsetVec[i][j]->y() != mOffsetVec[i][j]->y()) || (mOffsetVec[i][j]->z() != mOffsetVec[i][j]->z()));
				invalide |= (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}//*/
			while(invalide);
			
		}
	}//*/
}

void IKSolverPFOrient::computeDistance()
{	
	//ajouter plus de poids au root ???
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		double distance=0;
		for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				Eigen::Vector3d jtPos = mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(mCurrentFrame[(*it).second][1], mCurrentFrame[(*it).second][2], mCurrentFrame[(*it).second][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				Eigen::Matrix3d cov;
				cov.setIdentity();
				distance += diff.transpose()*(cov*diff);
				//cout << (*it).first << endl;
				/*if((*it).first == "Head")
				{
					cout << "********" << endl;
					/*cout << jtPos << endl;
					cout << "*" << endl;
					cout << jtObs << endl;//
					cout << diff.transpose()*(cov*diff) << endl;
					cout << "********" << endl;
				}*/
			}
		}
		mCurrentDistances[i] = sqrt(distance);
	}
}

void IKSolverPFOrient::computeDistanceHiera()
{	
	//ajouter plus de poids au root ???
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		double distance=0;
		for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				Eigen::Vector3d jtPos = mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(mCurrentFrame[(*it).second][1], mCurrentFrame[(*it).second][2], mCurrentFrame[(*it).second][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				Eigen::Matrix3d cov;
				cov.setIdentity();
				double tempo = diff.transpose()*(cov*diff);
				//distance += tempo*(5./(double)mModels[i]->getJoint((*it).first)->getHieraLevel());
				distance += tempo/log((double)mModels[i]->getJoint((*it).first)->getHieraLevel()*6);
				//distance += tempo*(double)mModels[i]->getJoint((*it).first)->getHieraLevel()*0.1;
				//cout << (*it).first << endl;
				/*if((*it).first == "Head")
				{
					cout << "********" << endl;
					/*cout << jtPos << endl;
					cout << "*" << endl;
					cout << jtObs << endl;//
					cout << diff.transpose()*(cov*diff) << endl;
					cout << "********" << endl;
				}*/
			}
		}
		mCurrentDistances[i] = sqrt(distance);
	}
}

void IKSolverPFOrient::computeLikelihood()
{	
	//contraintes a ajouter
	this->computeDistance();
	
	for (int i=0 ; i<mCurrentDistances.size() ; i++)
	{
		mCurrentLikelihood[i] = exp(-abs(mCurrentDistances[i]));
	}
	//cout << mCurrentLikelihood << "//" << endl;
}

void IKSolverPFOrient::step()
{
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mModels[i]->setColor(1,0,1,0.1);
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			bool invalide = false;
			Eigen::Quaterniond quat = mDefaultOrientationVec[i][j];
			Eigen::Vector3d offs;
			if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
			{
				offs = mOffsetVec[i][j]->vector();
			}
			else
			{
				offs = mDefaultOffsetVec[i][j].vector();
			}
			
			do
			{
				invalide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.5, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize();

				Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01) + offs;
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.1), 0, 0) + offs;
					}
					while (tempo[0] <= 0);
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
	double Neff = this->computeNeff();;
	//cout << Neff << "****" << endl;
	if (Neff < 1.5 || Neff < mModels.size()*0.1)
		this->resample();
}

double IKSolverPFOrient::stepAlt()
{
	cout << mCurrentDistances[mMaxWeightIndex] << endl;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mModels[i]->setColor(1,0,1,0.1);
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			double variance;
			//double variance = log(1+mCurrentDistances[i])*5;
			//if (mCurrentDistances[i] < 2)
			//	variance = (exp(mCurrentDistances[i])-1)/10.;
			//else
				variance = mCurrentDistances[i];
			//if (randUnif(1000)<10)
				//variance = 2;
			//variance=10;
			//cout << mCurrentDistances[i] << endl;
			if (variance>100)
			{
				cout << "lol" << endl;
				variance = 1;
			}
			bool invalide = false;
			//Eigen::Quaterniond quat = mDefaultOrientationVec[i][j];
			Eigen::Quaterniond quat = (*mOrientationVec[i][j]);
			Eigen::Vector3d offs;
			if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
			{
				offs = mOffsetVec[i][j]->vector();
			}
			else
			{
				offs = mOffsetVec[i][j]->vector();
				//offs = mDefaultOffsetVec[i][j].vector();
			}
			do
			{
				invalide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 0.5, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize();
				
				/*Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01)*variance + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), 0, 0)*variance + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), this->randn(0.01), 0)*variance + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(0.01), this->randn(0.01))*variance + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), 0, this->randn(0.01))*variance + offs;
					}
					while(!mModels[i]->getJoint(mNameVec[i][j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes*/

				//To avoid infinite and NaN cases
				//invalide |= ((mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				//invalide |= ((mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				//invalide |= ((mOffsetVec[i][j]->x() != mOffsetVec[i][j]->x()) || (mOffsetVec[i][j]->y() != mOffsetVec[i][j]->y()) || (mOffsetVec[i][j]->z() != mOffsetVec[i][j]->z()));
				invalide |= (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}
			while(invalide);
			
			
		}
	}
	this->updateWeights();
	double Neff = this->computeNeff();;
	//cout << Neff << "****" << endl;
	//if (Neff < 1.5 || Neff < mModels.size()*0.1)
		this->resample();
	return mCurrentDistances[mMaxWeightIndex];
}

double IKSolverPFOrient::computeNeff()
{
	return 1./(mCurrentWeights.dot(mCurrentWeights));
}

void IKSolverPFOrient::updateWeights()
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
	mModels[mMaxWeightIndex]->setColor(0,1,1,1);
	//cout << mMaxWeightIndex << endl;
}

void IKSolverPFOrient::resample()
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

void IKSolverPFOrient::save()
{
	for (int j=0 ; j<mModels.size() ; j++)
	{
		mModels[j]->setColor(0,1,0,1);
		for (int k=0 ; k<mOrientationVec[j].size() ; k++)
		{
			if (j!=mMaxWeightIndex)
			{
				(*mOrientationVec[j][k])=(*mOrientationVec[mMaxWeightIndex][k]);
				(*mOffsetVec[j][k])=(*mOffsetVec[mMaxWeightIndex][k]);
			}
		}
	}
}

void IKSolverPFOrient::mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
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

