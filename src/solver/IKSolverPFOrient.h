#ifndef IKSOLVERPFORIENT_H
#define IKSOLVERPFORIENT_H

#include "IKSolver.h"

using namespace std;

class IKSolverPFOrient : public IKSolver
{
	public:
		IKSolverPFOrient(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);

		virtual void initFilter();
		virtual void computeLikelihood();
		virtual void step();
		virtual double stepAlt();
		double computeNeff();
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		void save();
		
	protected:
		virtual void computeDistance();
		virtual void computeDistanceHiera();
		void updateWeights();
		void resample();
		
		
		Eigen::VectorXf mCurrentWeights;
		int mMaxWeightIndex;
};

#endif
