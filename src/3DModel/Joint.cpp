#include "Joint.h"

Joint::Joint(string name, Joint *parent, vector<double> offset, Eigen::Quaterniond quat)
{
	mParentJoint = parent;
	mName = name;
	
	if (parent == NULL)
		mHieraLevel = 1;
	else
		mHieraLevel = parent->getHieraLevel()+1;
	
	mQLocal = quat;

	if (offset.size() == 3)
		mLocalOffset = Eigen::Translation3d(offset[0], offset[1], offset[2]);
	else
		mLocalOffset = Eigen::Translation3d(0, 0, 0);
		
	mDefaultOffset = mLocalOffset;
	mQDefault = mQLocal;
	
	mOffsetConst = OFFSET_CONST_FREE;
	mOrientationConst = ORIENT_CONST_FREE;
	mColors = std::vector<float>(4,1);
	//mColors.push_back(1);
	//mColors.push_back(0);
	//mColors.push_back(1);
	//mColors.push_back(1);
	setColor();
}

Joint::Joint(const Joint& jtCopy)
{
	mName = jtCopy.mName;
	mColors = jtCopy.mColors;

	mQDefault = jtCopy.mQDefault;
	mQLocal = jtCopy.mQLocal;
	//mQCurrent = jtCopy.mQCurrent;
	mDefaultOffset = jtCopy.mDefaultOffset;
	mLocalOffset = jtCopy.mLocalOffset;
	//mCurrentOffset = jtCopy.mCurrentOffset;
	mOffsetConst = jtCopy.mOffsetConst;
	mOrientationConst = jtCopy.mOrientationConst;
	mOffsetSignConst = jtCopy.mOffsetSignConst;
	mHieraLevel = jtCopy.mHieraLevel;
	
	mOffsetPartition = jtCopy.mOffsetPartition;
	mOrientationPartition = jtCopy.mOrientationPartition;
	
	//Dynamics allocations
	
	if (jtCopy.mParentJoint == NULL);
		mParentJoint = NULL;
	
	if (jtCopy.mChildrenJoint.size() > 0)
	{
		for (int i=0 ; i < jtCopy.mChildrenJoint.size() ; i++)
		{
			mChildrenJoint.push_back(new Joint(*(jtCopy.mChildrenJoint[i])));
			mChildrenJoint[i]->setParentIfChild(this);
		}
	}
}

Joint::~Joint()
{
	if (mChildrenJoint.size() > 0)
	{
		for (int i=0 ; i < mChildrenJoint.size() ; i++)
		{
			delete mChildrenJoint[i];
		}
	}
}

Joint* Joint::getRoot()
{
	Joint* parent = this;
	Joint* prev = NULL;

	do
	{
		prev = parent;
		parent = parent->getParent();
	}while(parent != NULL);
	
	return prev;
}

void Joint::setParentIfChild(Joint *jt)
{
	vector<Joint*> parentChildren = jt->getChildren();
	bool chgt = false;
	if (parentChildren.size() > 0)
	{
		for (int i=0 ; i<parentChildren.size() ; i++)
		{
			if (parentChildren[i] == this)
			{
				mParentJoint = jt;
				mHieraLevel = jt->getHieraLevel() + 1;
				chgt = true;
			}
		}
	}
	
	if (!chgt)
	{
		cout << "Error : Child doesn't exist for this parent" << endl;
	}
}

Joint* Joint::getParent()
{
	return mParentJoint;
}

Joint* Joint::getJointFromName(std::string name)
{
	Joint* root = this->getRoot();
	
	if (this->getName() == name)
	{
		return root;
	}
	
	Joint* result = getJointFromName(root, name);
	
	return result;
}

Joint* Joint::getJointFromName(Joint *jt, std::string name)
{
	if (jt->hasChildren())
	{
		std::vector<Joint*> children = jt->getChildren();
		Joint* result = NULL;
		for (int i=0 ; i<children.size() ; i++)
		{
			if (children[i]->getName() == name)
			{
				return children[i];
			}
			else
			{
				result=getJointFromName(children[i], name);
				if (result != NULL)
					return result;
			}
		}
	}
	
	return NULL;
}

std::string Joint::getName()
{
	return mName;
}

int Joint::getHieraLevel()
{
	return mHieraLevel;
}

bool Joint::hasChildren()
{
	if (mChildrenJoint.size()>0)
		return true;
	else
		return false;
}

std::vector<Joint*>& Joint::getChildren()
{
	return mChildrenJoint;
}

Joint* Joint::addChild(std::string name, vector<double> offset, Eigen::Quaterniond quat)
{
	Joint *jt = new Joint(name, this, offset, quat);
	mChildrenJoint.push_back(jt);
	return jt;
}

Joint* Joint::addChild(Joint* jt)
{
	mChildrenJoint.push_back(jt);
	return jt;
}

void Joint::setOrientation(Eigen::Quaterniond quat)
{
	mQLocal = quat;
}

Joint* Joint::setConstraints(const std::string offset, const std::string orientation)
{
	this->mOffsetConst = offset;
	this->mOrientationConst = orientation;
	return this;
}

Joint* Joint::setLimits(const std::vector<std::string>& signConst)
{
	mOffsetSignConst = signConst;
	for (int i=0 ; i<mOffsetSignConst.size() ; i++)
	{
		if ((mOffsetSignConst[i] != CONST_POSITIVE) && (mOffsetSignConst[i] != CONST_NEGATIVE) && (mOffsetSignConst[i] != CONST_NONE))
		{
			cout << "Error in Sign constraint definition, set to NULL" << endl;
			mOffsetSignConst[i] = CONST_NONE;
		}
	}
	return this;
}

Joint* Joint::setPartition(int offset, int orientation)
{
	if ((orientation < 1) || (offset < 1))
	{
		mOffsetPartition = 1;
		mOrientationPartition = 1;
		cout << "Error ! Partition index has to be > 0" << endl;
		cout << "orientation and offset partitions set to 1" << endl;
	}
	else
	{
		mOffsetPartition = offset;
		mOrientationPartition = orientation;
	}
	return this;
}

bool Joint::checkValidity(const Eigen::Vector3d& offset)
{
	for (int i=0 ; i<mOffsetSignConst.size() ; i++)
	{
		std::string signConst = mOffsetSignConst[i];
		
		if (signConst == CONST_POSITIVE)
		{
			if (offset[i] < 0)
				return false;
		}
		else if (signConst == CONST_NEGATIVE)
		{
			if (offset[i] > 0)
				return false;
		}
		else if (signConst == CONST_NONE)
		{
		}
		else
		{
			cout << "Joint class : Error" << endl;
		}
	}
	return true;
}

Eigen::Quaterniond* Joint::getOrientation()
{
	return &mQLocal;
}

const Eigen::Quaterniond Joint::getDefaultOrientation()
{
	return mQDefault;
}

Eigen::Translation3d* Joint::getOffset()
{
	return &mLocalOffset;
}

const Eigen::Translation3d Joint::getDefaultOffset()
{
	return mDefaultOffset;
}

const Eigen::Vector3d Joint::getXYZVect()
{
	return this->getGlobalTransformationMatrix().translation();
}

Eigen::Transform<double, 3, Eigen::Affine> Joint::getTransformationMatrix()
{
	return mLocalOffset*mQLocal;
}

Eigen::Transform<double, 3, Eigen::Affine> Joint::getGlobalTransformationMatrix()
{
	Joint* jt = this;
	Eigen::Transform<double, 3, Eigen::Affine> result = jt->getTransformationMatrix();
	Eigen::Transform<double, 3, Eigen::Affine> resultBis;
	resultBis.setIdentity();
	
	while(jt->getParent() != NULL)
	{
		jt = jt->getParent();
		result = jt->getTransformationMatrix()*result;
	}
	return result;
}

std::string Joint::getOffsetConstraint()
{
	return mOffsetConst;
}

std::string Joint::getOrientationConstraint()
{
	return mOrientationConst;
}

int Joint::getOffsetPartition()
{
	return mOffsetPartition;
}

int Joint::getOrientationPartition()
{
	return mOrientationPartition;
}

void Joint::setColor(float R, float G, float B, float alpha)
{
	mColors[0] = R;
	mColors[1] = G;
	mColors[2] = B;
	mColors[3] = alpha;
}

std::vector<float> Joint::getColor()
{
	return mColors;
}
