#include "S3DModel.h"

S3DModel::S3DModel(int id)
{
	mId = id;
	mRootJoint = NULL;
	mNbJoints = -1;
	mPartitionNumber = 0;
	mIsVisible = true;
}

S3DModel::S3DModel(const Joint* jt, unsigned int id)
{
	mRootJoint = new Joint(*jt);
	mId = id;
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
	for(int i=0 ; i<=mNbJoints ; i++)
	{
		mIntToJoint[i]->setColor(R, G, B, alpha);
	}
}
void S3DModel::setVisible(bool visible)
{
	mIsVisible = visible;
}

bool S3DModel::isVisible()
{
	return mIsVisible;
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


