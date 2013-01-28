#ifdef IKSOLVERMATHIAS_H

template<class Model>
IKSolverMathias<Model>::IKSolverMathias(std::vector<Model*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions) : mModels(mods), mPosNames(posNames), mCurrentFrame(jointsXYZPositions)
{
	mRootName = mods[0]->getRootJoint()->getName();
}

template<class Model>
void IKSolverMathias<Model>::mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
{
	std::vector<std::string> jtNames = this->mModels[0]->getNameVec();
	this->mJointNameToPosName = jointNameToPosName;
	for (int i=0 ; i < jtNames.size() ; i++)
	{
		bool found=false;
		this->mJointNameToInt[jtNames[i]] = i;

		for (int j=0 ; j<this->mPosNames.size() ; j++)
		{
			if(this->mJointNameToPosName[jtNames[i]] == this->mPosNames[j])
			{
				found = true;
				this->mJointNameToPos[jtNames[i]]=j;
				break;
			}
			else if (this->mJointNameToPosName[jtNames[i]] == "NULL")
			{
				found = true;
				this->mJointNameToPos[jtNames[i]]=-1;
				break;
			}
		}
		if (!found)
			cout << jtNames[i] <<" : No match found" << endl;
		
	}
}

template<class Model>
void IKSolverMathias<Model>::initFilter()
{
	
	for (int i=0 ; i<this->mModels.size() ; i++)
	{
		this->mOrientationVec.push_back(this->mModels[i]->getOrientationVec());
		this->mOffsetVec.push_back(this->mModels[i]->getOffsetVector());
		this->mDefaultOrientationVec.push_back(this->mModels[i]->getDefaultOrientationVec());
		this->mDefaultOffsetVec.push_back(this->mModels[i]->getDefaultOffsetVector());
		this->mNameVec.push_back(this->mModels[i]->getNameVec());
		this->mConstOffsetVec.push_back(this->mModels[i]->getConstOffsetVec());
		this->mConstOrientVec.push_back(this->mModels[i]->getConstOrientVec());
		
	}
	
	for (int i=0 ; i<this->mModels.size() ; i++)
	{
		for (int j=0 ; j < this->mOrientationVec[i].size() ; j++)
		{
			
				Eigen::Vector3d tempo;
				if (this->mNameVec[i][j] == this->mRootName)
				{
					std::vector<double> vec = this->mCurrentFrame[this->mJointNameToPos[this->mNameVec[i][j]]];
					tempo=Eigen::Vector3d(vec[1], vec[2], vec[3]);
				}
				else if(this->mJointNameToPos[this->mNameVec[i][j]] != -1 && this->mJointNameToPos[this->mModels[i]->getJoint(this->mNameVec[i][j])->getParent()->getName()] != -1)
				{
					std::string parentName = this->mModels[i]->getJoint(this->mNameVec[i][j])->getParent()->getName();
					std::vector<double> vec = this->mCurrentFrame[this->mJointNameToPos[this->mNameVec[i][j]]];
					std::vector<double> vecParent = this->mCurrentFrame[this->mJointNameToPos[parentName]];
					if(this->mConstOffsetVec[i][j] == OFFSET_CONST_FREE || this->mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY || this->mConstOffsetVec[i][j] == OFFSET_CONST_PLANARYZ || this->mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXZ || this->mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
					{
						tempo=Eigen::Vector3d(vec[1]-vecParent[1], vec[2]-vecParent[2], vec[3]-vecParent[3]);
					}
					else if(this->mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
					{
						double dist = abs(vec[1]-vecParent[1]) + abs(vec[2]-vecParent[2]) + abs(vec[3]-vecParent[3]);
						tempo=Eigen::Vector3d(dist, 0, 0);
					}
				}
		}
	}
}

#endif
