#ifndef IKSOLVERMATHIAS_H
#define IKSOLVERMATHIAS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include "../3DModel/S3DModel.h"

template<class Model>
class IKSolverMathias
{
	public:
		IKSolverMathias(std::vector<Model*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		virtual void initFilter();
		
	protected:	
		std::vector<Model*> mModels;
		std::vector<std::string> mPosNames;
		std::vector<std::vector<double> > mCurrentFrame;
		std::string mRootName;
		std::map<std::string, std::string> mJointNameToPosName; /*!< Map between Joint Names file and animation file */
		std::map<std::string, int> mJointNameToPos;
		std::map<std::string, int> mJointNameToInt; /*!< Name of the Joint to its position in the orientation vector */
				
		std::vector<std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > > mOrientationVec;
		std::vector<std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > > mOffsetVec;
		std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > > mDefaultOrientationVec;
		std::vector<std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > > mDefaultOffsetVec;
		std::vector<std::vector<std::string> > mNameVec;
		std::vector<std::vector<std::string> > mConstOffsetVec;
		std::vector<std::vector<std::string> > mConstOrientVec;
};

#include "IKSolverMathias.cpp"

#endif
