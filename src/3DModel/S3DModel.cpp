#include "S3DModel.h"

S3DModel::S3DModel(const Joint* jt)
{
	mRootJoint = new Joint(*jt);
	mNbJoints = -1;
	mPartitionNumber = 0;
	mIsVisible = true;
	createMaps();
	createOrientationVec();
	createOffsetVec();
	createNameVec();
	createConstraintVecs();
	createDefaultVecs();
	createPartitionMultimaps();
	//std::cout << "S3DModel : model index successfully created !" << std::endl;
}

S3DModel::S3DModel(const S3DModel& model)
{
	mRootJoint = new Joint(*(model.mRootJoint));
	mId = -2;
	mNbJoints = -1;
	mPartitionNumber = 0;
	mIsVisible = model.mIsVisible;
	createMaps();
	createOrientationVec();
	createOffsetVec();
	createNameVec();
	createConstraintVecs();
	createDefaultVecs();
	createPartitionMultimaps();
	//std::cout << "S3DModel : model index successfully created !" << std::endl;
}

S3DModel::~S3DModel()
{
	delete mRootJoint;
}

Joint* S3DModel::getRootJoint()
{
	return mRootJoint;
}

void S3DModel::createMaps()
{
	mStringToJoint.clear();
	mIntToJoint.clear();
	mStringToInt.clear();
	
	if (mRootJoint != NULL)
	{
		mNbJoints++;
		mStringToJoint[mRootJoint->getName()] = mRootJoint;
		mIntToJoint[mNbJoints] = mRootJoint;
		mStringToInt[mRootJoint->getName()] = mNbJoints;
		
		if (mRootJoint->hasChildren())
		{
			vector<Joint*> children = mRootJoint->getChildren();
			createMaps(children);
		}
	}
}

int S3DModel::getNumberJoint()
{
	return mNbJoints+1;
}

Joint* S3DModel::getJoint(std::string jtName)
{
	return mStringToJoint[jtName];
}

vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > S3DModel::getOrientationVec()
{
	return mOrientationVec;
}

vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > S3DModel::getOffsetVector()
{
	return mOffsetVec;
}

vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > S3DModel::getDefaultOrientationVec()
{
	return mDefaultOrientationVec;
}
		
vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > S3DModel::getDefaultOffsetVector()
{
	return mDefaultOffsetVec;
}

vector<std::string> S3DModel::getNameVec()
{
	return mNameVec;
}

vector<std::string> S3DModel::getConstOffsetVec()
{
	return mConstOffsetVec;
}

vector<std::string> S3DModel::getConstOrientVec()
{
	return mConstOrientVec;
}

std::multimap<int, std::string> S3DModel::getOffsetPartitionMultimap()
{
	return mOffsetPartToName;
}

std::multimap<int, std::string> S3DModel::getOrientPartitionMultimap()
{
	return mOrientPartToName;
}

int S3DModel::getPartitionNumber()
{
	return mPartitionNumber;
}

void S3DModel::setColor(float R, float G, float B, float alpha)
{
	_Particle<std::vector<std::vector<double > > >::setColor(R, G, B, alpha);
	for(int i=0 ; i<=mNbJoints ; i++)
	{
		mIntToJoint[i]->setColor(R, G, B, alpha);
	}
}

void S3DModel::sampleFromPrior()
{
	#ifdef USE_QUATERNION
		for (int j=0 ; j < mOrientationVec.size() ; j++)
		{
			mDefaultOrientationVec[j] = (*mOrientationVec[j]); // Base orientation becomes default orientation
			mDefaultOffsetVec[j] = (*mOffsetVec[j]);  // Base offset becomes default offset
			
			Eigen::Quaterniond quat = mDefaultOrientationVec[j]; // Mean of orientations == default orientation
			bool invalide = false;
			Eigen::Vector3d offs = mDefaultOffsetVec[j].vector(); // Mean of offset == default offset
			do
			{
				invalide = false;
				if (mConstOrientVec[j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, TEMPO, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, TEMPO, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, TEMPO, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, TEMPO, 1, 1, 0.1);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, TEMPO, 0.1, 1, 1);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[j]) = (*mOrientationVec[j]);
				}
				else
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, TEMPO, 1, 1, 1);
				}
				mOrientationVec[j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.001, this->randn()*0.001, this->randn()*0.001) + offs;
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), 0, 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), this->randn(0.001), 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(0.001), this->randn(0.001)) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.001), 0, this->randn(0.001)) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*mOffsetVec[j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((mOffsetVec[j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[j]->x() != mOffsetVec[j]->x()) || (mOffsetVec[j]->y() != mOffsetVec[j]->y()) || (mOffsetVec[j]->z() != mOffsetVec[j]->z()));
				invalide |= (mOrientationVec[j]->w() != mOrientationVec[j]->w());
			}
			while(invalide);
		}
	#endif
}

void S3DModel::update()
{
}

void S3DModel::esitmateLikelihood(std::vector<std::vector<double > > obs)
{
}

void S3DModel::mapJointToObs(std::vector<std::string> posNames, std::map<std::string, std::string> jointNameToPosName)
{
	mPosNames = posNames;
	
	vector<std::string> jtNames = this->getNameVec();
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

S3DModel& S3DModel::operator =(const S3DModel& part)
{
	_Particle<std::vector<std::vector<double > > >::operator =(part);
}

void S3DModel::createMaps(vector<Joint*>& jts)
{
	if (jts.size() > 0)
	{
		for (int i=0 ; i < jts.size() ; i++)
		{
			mNbJoints++;
			mStringToJoint[jts[i]->getName()] = jts[i];
			mIntToJoint[mNbJoints] = jts[i];
			mStringToInt[jts[i]->getName()] = mNbJoints;
			
			if (jts[i]->hasChildren())
			{
				vector<Joint*> children = jts[i]->getChildren();
				createMaps(children);
			}
		}
	}
}

void S3DModel::createOrientationVec()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mOrientationVec.push_back(mIntToJoint[i]->getOrientation());
		}
	}
}

void S3DModel::createOffsetVec()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mOffsetVec.push_back(mIntToJoint[i]->getOffset());
		}
	}
}

void S3DModel::createNameVec()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mNameVec.push_back(mIntToJoint[i]->getName());
		}
	}
}

void S3DModel::createConstraintVecs()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mConstOffsetVec.push_back(mIntToJoint[i]->getOffsetConstraint());
			mConstOrientVec.push_back(mIntToJoint[i]->getOrientationConstraint());
		}
	}
}

void S3DModel::createDefaultVecs()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mDefaultOrientationVec.push_back(mIntToJoint[i]->getDefaultOrientation());
			mDefaultOffsetVec.push_back(mIntToJoint[i]->getDefaultOffset());
		}
	}
}

void S3DModel::createPartitionMultimaps()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			int offsetPart = mIntToJoint[i]->getOffsetPartition();
			int orientPart = mIntToJoint[i]->getOrientationPartition();
			mOffsetPartToName.insert(pair<int, std::string>(offsetPart, mIntToJoint[i]->getName()));
			mOrientPartToName.insert(pair<int, std::string>(orientPart, mIntToJoint[i]->getName()));
			
			if(offsetPart > mPartitionNumber)
				mPartitionNumber = offsetPart;
			if(orientPart > mPartitionNumber)
				mPartitionNumber = orientPart;
		}
	}
}


