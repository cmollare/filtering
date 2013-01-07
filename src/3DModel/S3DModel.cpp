#include "S3DModel.h"

S3DModel::S3DModel(const Joint* jt) : _Particle(), _Stats()
{
	mRootJoint = new Joint(*jt);
	mNbJoints = -1;
	mNumberOfPartitions = 0;
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

S3DModel::S3DModel(const S3DModel& model) : _Particle(model)
{
	mRootJoint = new Joint(*(model.mRootJoint));
	mId = -2;
	mNbJoints = -1;
	mNumberOfPartitions = 0;
	mIsVisible = model.mIsVisible;
	mJointNameToPosName = model.mJointNameToPosName;
	mJointNameToPos = model.mJointNameToPos;
	mJointNameToInt = model.mJointNameToInt;
	mPosNames = model.mPosNames;
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

std::map<std::string, int> S3DModel::getJointToIntMap()
{
	return mJointNameToInt;
}

void S3DModel::setColor(float R, float G, float B, float alpha)
{
	_Particle::setColor(R, G, B, alpha);
	for(int i=0 ; i<=mNbJoints ; i++)
	{
		mIntToJoint[i]->setColor(R, G, B, alpha);
	}
}

void S3DModel::sampleFromPrior()
{
	#if SAMPLING == USE_QUATERNION
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
					(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 0.1);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 0.1, 1, 1);
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[j]) = (*mOrientationVec[j]);
				}
				else
				{
					(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 1);
				}
				mOrientationVec[j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE) + offs;
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), this->randn(VAROFFSET), 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(VAROFFSET), this->randn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, this->randn(VAROFFSET)) + offs;
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
	#elif SAMPLING == USE_EULER
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
					(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0));
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0));
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0));
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
				}
				else if(mConstOrientVec[j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[j]) = (*mOrientationVec[j]);
				}
				else
				{
					(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
				}
				mOrientationVec[j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE) + offs;
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), this->randn(VAROFFSET), 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(VAROFFSET), this->randn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, this->randn(VAROFFSET)) + offs;
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

void S3DModel::updateAll()
{
	#if SAMPLING == USE_QUATERNION
	
	for (int j=0 ; j < mOrientationVec.size() ; j++)
	{
		mDefaultOrientationVec[j] = (*mOrientationVec[j]); // Base orientation becomes default orientation
		mDefaultOffsetVec[j] = (*mOffsetVec[j]);  // Base offset becomes default offset
			
		Eigen::Quaterniond quat = mDefaultOrientationVec[j]; // Mean of orientations == default orientation
		bool invalide = false;
		
		#ifndef EVOLVEOFFSET
		Eigen::Vector3d offs = mDefaultOffsetVec[j].vector(); // Mean of offset == default offset
		#else
		Eigen::Vector3d offs = mOffsetVec[j]->vector(); // Mean of offset == default offset
		#endif
		do
		{
			invalide = false;
			if (mConstOrientVec[j] == ORIENT_CONST_FREE)
			{
				(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 1);//A modifier suivant les contraintes
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_TWIST)
			{
				(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 0.1, 0.05);
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_FLEX)
			{
				(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 0.1, 1, 0.05);
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_TFLEX)
			{
				(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 0.1);
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
			{
				(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 0.1, 1, 1);
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_FIXED)
			{
				(*mOrientationVec[j]) = (*mOrientationVec[j]);
			}
			else
			{
				(*mOrientationVec[j])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 1);
			}
			mOrientationVec[j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
			
			Eigen::Vector3d tempo;
			if (mConstOffsetVec[j] == OFFSET_CONST_FREE)
			{
				tempo = Eigen::Vector3d(this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE) + offs;
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_BONE)
			{
					
				do
				{
					tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, 0) + offs;
				}
				while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
			{
				int i=0;
				do
				{
					tempo = Eigen::Vector3d(this->randn(VAROFFSET), this->randn(VAROFFSET), 0) + offs;
				}
				while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
			{
				do
				{
					tempo = Eigen::Vector3d(0, this->randn(VAROFFSET), this->randn(VAROFFSET)) + offs;
				}
				while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
			{
				do
				{
					tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, this->randn(VAROFFSET)) + offs;
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
	
	#elif SAMPLING == USE_EULER
	
	for (int j=0 ; j < mOrientationVec.size() ; j++)
	{
		mDefaultOrientationVec[j] = (*mOrientationVec[j]); // Base orientation becomes default orientation
		mDefaultOffsetVec[j] = (*mOffsetVec[j]);  // Base offset becomes default offset
			
		Eigen::Quaterniond quat = mDefaultOrientationVec[j]; // Mean of orientations == default orientation
		bool invalide = false;
		
		#ifndef EVOLVEOFFSET
		Eigen::Vector3d offs = mDefaultOffsetVec[j].vector(); // Mean of offset == default offset
		#else
		Eigen::Vector3d offs = mOffsetVec[j]->vector(); // Mean of offset == default offset
		#endif
		do
		{
			invalide = false;
			if (mConstOrientVec[j] == ORIENT_CONST_FREE)
			{
				(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_TWIST)
			{
				(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0));
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_FLEX)
			{
				(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0));
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_TFLEX)
			{
				(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0));
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
			{
				(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
			}
			else if(mConstOrientVec[j] == ORIENT_CONST_FIXED)
			{
				(*mOrientationVec[j]) = (*mOrientationVec[j]);
			}
			else
			{
				(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
			}
			mOrientationVec[j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
			
			Eigen::Vector3d tempo;
			if (mConstOffsetVec[j] == OFFSET_CONST_FREE)
			{
				tempo = Eigen::Vector3d(this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE) + offs;
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_BONE)
			{
					
				do
				{
					tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, 0) + offs;
				}
				while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
			{
				int i=0;
				do
				{
					tempo = Eigen::Vector3d(this->randn(VAROFFSET), this->randn(VAROFFSET), 0) + offs;
				}
				while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
			{
				do
				{
					tempo = Eigen::Vector3d(0, this->randn(VAROFFSET), this->randn(VAROFFSET)) + offs;
				}
				while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
			}
			else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
			{
				do
				{
					tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, this->randn(VAROFFSET)) + offs;
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

void S3DModel::updatePart(int partition)
{
	#if SAMPLING == USE_QUATERNION
		//cout << "WHOOT" << endl;
		std::multimap<int, std::string>::iterator itOff = mOffsetPartToName.find(partition);
		std::multimap<int, std::string>::iterator itOrient = mOrientPartToName.find(partition);
		
		//Quaternion sampling
		for (itOrient = mOrientPartToName.equal_range(partition).first ; itOrient != mOrientPartToName.equal_range(partition).second ; ++itOrient)
		{

			int pos = mJointNameToInt[(*itOrient).second];// Retrieve position of the Joint in orientation vectors
				

			bool invalide = false;
			//Eigen::Quaterniond quat = mDefaultOrientationVec[i][pos];
			Eigen::Quaterniond quat = (*mOrientationVec[pos]); // Mean orientation is the previous orientation
			do
			{
				invalide = false;
				if (mConstOrientVec[pos] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[pos])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[pos])=this->sampleQuTEM(quat, VARQUATERNION, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[pos])=this->sampleQuTEM(quat, VARQUATERNION, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[pos])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 0.1);
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[pos])=this->sampleQuTEM(quat, VARQUATERNION, 0.1, 1, 1);
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[pos]) = (*mOrientationVec[pos]);
				}
				else
				{
					(*mOrientationVec[pos])=this->sampleQuTEM(quat, VARQUATERNION, 1, 1, 1);
				}
				mOrientationVec[pos]->normalize();
				
				invalide |= (mOrientationVec[pos]->w() != mOrientationVec[pos]->w());
			}
			while(invalide);
		}// End of quaternion sampling
		
		
		// Offset sampling
		for (itOff = mOffsetPartToName.equal_range(partition).first ; itOff != mOffsetPartToName.equal_range(partition).second ; ++itOff)
		{
			
			int pos = mJointNameToInt[(*itOff).second];// Retrieve position of the Joint in offset vectors
			
			Eigen::Vector3d offs;
			#ifndef EVOLVEOFFSET
			if (mConstOffsetVec[pos] == OFFSET_CONST_FREE)
			{
				offs = mOffsetVec[pos]->vector(); // For Free dof, mean is the previous offset
			}
			else
			{
				offs = mDefaultOffsetVec[pos].vector(); // Mean offset for bones and planar DOF is the default offset
			}
			#else
				offs = mOffsetVec[pos]->vector();
			#endif
			
			bool invalide = false;
			
			do
			{
				invalide = false;
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[pos] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE) + offs;
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, 0) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXY)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), this->randn(VAROFFSET), 0) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(VAROFFSET), this->randn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, this->randn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*mOffsetVec[pos])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((mOffsetVec[pos]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[pos]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[pos]->x() != mOffsetVec[pos]->x()) || (mOffsetVec[pos]->y() != mOffsetVec[pos]->y()) || (mOffsetVec[pos]->z() != mOffsetVec[pos]->z()));
			}
			while(invalide);
		}// End of offset sampling
		
	#elif SAMPLING == USE_EULER
	
		std::multimap<int, std::string>::iterator itOff = mOffsetPartToName.find(partition);
		std::multimap<int, std::string>::iterator itOrient = mOrientPartToName.find(partition);
		
		//Quaternion sampling
		for (itOrient = mOrientPartToName.equal_range(partition).first ; itOrient != mOrientPartToName.equal_range(partition).second ; ++itOrient)
		{

			int pos = mJointNameToInt[(*itOrient).second];// Retrieve position of the Joint in orientation vectors
				

			bool invalide = false;
			//Eigen::Quaterniond quat = mDefaultOrientationVec[i][pos];
			Eigen::Quaterniond quat = (*mOrientationVec[pos]); // Mean orientation is the previous orientation
			do
			{
				invalide = false;
				if (mConstOrientVec[pos] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0));
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0));
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0));
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
				}
				else if(mConstOrientVec[pos] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[pos]) = (*mOrientationVec[pos]);
				}
				else
				{
					(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->randn(VAREULER), Eigen::Vector3d(0, 0, 1));
				}
				mOrientationVec[pos]->normalize();
				
				invalide |= (mOrientationVec[pos]->w() != mOrientationVec[pos]->w());
			}
			while(invalide);
		}// End of quaternion sampling
		
		
		// Offset sampling
		for (itOff = mOffsetPartToName.equal_range(partition).first ; itOff != mOffsetPartToName.equal_range(partition).second ; ++itOff)
		{
			
			int pos = mJointNameToInt[(*itOff).second];// Retrieve position of the Joint in offset vectors
			
			Eigen::Vector3d offs;
			#ifndef EVOLVEOFFSET
			if (mConstOffsetVec[pos] == OFFSET_CONST_FREE)
			{
				offs = mOffsetVec[pos]->vector(); // For Free dof, mean is the previous offset
			}
			else
			{
				offs = mDefaultOffsetVec[pos].vector(); // Mean offset for bones and planar DOF is the default offset
			}
			#else
				offs = mOffsetVec[pos]->vector();
			#endif
			
			bool invalide = false;
			
			do
			{
				invalide = false;
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[pos] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE, this->randn()*VAROFFSETFREE) + offs;
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, 0) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXY)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), this->randn(VAROFFSET), 0) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(VAROFFSET), this->randn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(VAROFFSET), 0, this->randn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[pos] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*mOffsetVec[pos])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((mOffsetVec[pos]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[pos]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[pos]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((mOffsetVec[pos]->x() != mOffsetVec[pos]->x()) || (mOffsetVec[pos]->y() != mOffsetVec[pos]->y()) || (mOffsetVec[pos]->z() != mOffsetVec[pos]->z()));
			}
			while(invalide);
		}// End of offset sampling
		
	#endif
}

void S3DModel::update(int partition)
{
	if (partition==-1)
	{
		this->updateAll();
	}
	else
	{
		this->updatePart(partition);
	}
}

void S3DModel::estimateLikelihoodAll(std::vector<std::vector<double > >& obs)
{
	std::map<std::string, int>::iterator it;
	double distance=0;
	for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
	{
		if ((*it).second != -1)
		{
			double distTemp=0;
			// Mahalanobis distance
			Eigen::Vector3d jtPos = this->getJoint((*it).first)->getXYZVect();
			Eigen::Vector3d jtObs(obs[(*it).second][1], obs[(*it).second][2], obs[(*it).second][3]);
			Eigen::Vector3d diff = jtPos - jtObs;
			Eigen::Matrix3d cov;
			cov.setIdentity();
			distTemp = diff.transpose()*(cov*diff);
			distTemp = sqrt(distTemp);
			distance += distTemp;
		}
	}
	mCurrentLikelihood = exp(-abs(distance));
}

void S3DModel::estimateLikelihoodPart(std::vector<std::vector<double > >& obs, int partition)
{
	std::multimap<int, std::string>::iterator it;
	double distance=0;
	it = mOffsetPartToName.find(partition);
	for (it = mOffsetPartToName.equal_range(partition).first ; it != mOffsetPartToName.equal_range(partition).second ; ++it)
	{
		if (mJointNameToPos[(*it).second] != -1)
		{
			int pos = mJointNameToPos[(*it).second];
			double distTemp=0;
			// Mahalanobis distance
			//cout << (*it).second << "=>" << mPosNames[pos] << endl;
			Eigen::Vector3d jtPos = this->getJoint((*it).second)->getXYZVect();
			Eigen::Vector3d jtObs(obs[pos][1], obs[pos][2], obs[pos][3]);
			Eigen::Vector3d diff = jtPos - jtObs;
			Eigen::Matrix3d cov;
			cov.setIdentity();
			distTemp = diff.transpose()*(cov*diff);
			distance += distTemp;
		}
	}
	if (partition==1)
		mCurrentLikelihood = exp(-abs(distance)*20);
	else
		mCurrentLikelihood *= exp(-abs(distance)*20);
		
}

void S3DModel::estimateLikelihood(std::vector<std::vector<double > >& obs, int partition)
{
	mObservations = obs;
	if (partition==-1)
	{
		this->estimateLikelihoodAll(obs);
	}
	else
	{
		this->estimateLikelihoodPart(obs, partition);
	}
}

void S3DModel::estimateMMSE(Eigen::VectorXd& weights, S3DModel** particles, int nbParticles)
{
	mObservations = particles[0]->getCurrentObservations();
	
	std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > orient = this->getOrientationVec();
	std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > offset = this->getOffsetVector();
	for (int i=0 ; i<orient.size() ; i++)
	{
		std::vector<double> quat(4, 0);
		Eigen::Vector3d tempo(0, 0, 0);
		for (int j=0 ; j<nbParticles ; j++)
		{
			Eigen::Vector3d offs = particles[j]->getOffsetVector()[i]->vector();
			tempo += weights[j]*offs;
			quat[0] += weights[j]*particles[j]->getOrientationVec()[i]->w();
			quat[1] += weights[j]*particles[j]->getOrientationVec()[i]->x();
			quat[2] += weights[j]*particles[j]->getOrientationVec()[i]->y();
			quat[3] += weights[j]*particles[j]->getOrientationVec()[i]->z();
		}
		(*orient[i]) = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
		orient[i]->normalize();
		(*offset[i]) = Eigen::Translation3d(tempo);
	}
}

void S3DModel::saveResults(ResultParser* resParser)
{
	std::map<std::string, int>::iterator it;
	
	std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > orient = this->getOrientationVec();
	std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > offset = this->getOffsetVector();
	
	for (it=mJointNameToInt.begin() ; it!=mJointNameToInt.end() ; it++)
	{
		Eigen::Vector3d pos = this->getJoint((*it).first)->getXYZVect();
		resParser->saveJoint("Joint_" + (*it).first, pos, *offset[(*it).second], *orient[(*it).second]);
	}
	
	for (int i=0 ; i<mPosNames.size() ; i++)
	{
		resParser->saveObs("Obs_" + mPosNames[i], mObservations[i]);
	}
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
	for (int j=0 ; j < this->mOrientationVec.size() ; j++)
	{
		(*this->mOrientationVec[j])=(*(part.mOrientationVec[j]));
		(*this->mOffsetVec[j])=(*(part.mOffsetVec[j]));
	}
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
			
			if(offsetPart > mNumberOfPartitions)
				mNumberOfPartitions = offsetPart;
			if(orientPart > mNumberOfPartitions)
				mNumberOfPartitions = orientPart;
		}
	}
}
