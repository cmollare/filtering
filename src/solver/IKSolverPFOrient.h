#ifndef IKSOLVERPFORIENT_H
#define IKSOLVERPFORIENT_H

#include "IKSolver.h"

using namespace std;

template<class Model>
class IKSolverPFOrient : public IKSolver<Model>
{
	public:
		IKSolverPFOrient(std::vector<Model*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);

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

template<class Model>
IKSolverPFOrient<Model>::IKSolverPFOrient(std::vector<Model*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions) : IKSolver<Model>(mods)
{
	this->mCurrentFrame = jointsXYZPositions;
	this->mPosNames = posNames;
	
	this->mMaxWeightIndex=0;
}

template<class Model>
void IKSolverPFOrient<Model>::initFilter()
{
	this->mCurrentWeights.setConstant(this->mModels.size(), 1, 1./this->mModels.size());
	this->mCurrentDistances.resize(this->mModels.size(), 1);
	this->mCurrentLikelihood.resize(this->mModels.size(), 1);
	
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
			
			Eigen::Quaterniond quat = this->mDefaultOrientationVec[i][j];
			//quat.setIdentity();
			bool invalide = false;
			Eigen::Vector3d offs = this->mDefaultOffsetVec[i][j].vector();
			do
			{
				invalide = false;
				if (this->mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);//A modifier suivant les contraintes
					//(*this->mOrientationVec[i][j])=quat;
				}	
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 0.1, 0.05);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 0.05);
					(*this->mOrientationVec[i][j])=quat;
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 0.1);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 1);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*this->mOrientationVec[i][j]) = (*this->mOrientationVec[i][j]);
				}
				else
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);
				}
				this->mOrientationVec[i][j]->normalize();
				
				
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
					else
					{
						tempo=Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01);
					}
				}
				else
				{
				if (this->mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01) + offs;
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.1), 0, 0) + offs;
					}
					while(!this->mModels[i]->getJoint(this->mNameVec[i][j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), this->randn(0.01), 0) + offs;
					}
					while(!this->mModels[i]->getJoint(this->mNameVec[i][j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->randn(0.01), this->randn(0.01)) + offs;
					}
					while(!this->mModels[i]->getJoint(this->mNameVec[i][j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->randn(0.01), 0, this->randn(0.01)) + offs;
					}
					while(!this->mModels[i]->getJoint(this->mNameVec[i][j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				}
				(*this->mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((this->mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((this->mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((this->mOffsetVec[i][j]->x() != this->mOffsetVec[i][j]->x()) || (this->mOffsetVec[i][j]->y() != this->mOffsetVec[i][j]->y()) || (this->mOffsetVec[i][j]->z() != this->mOffsetVec[i][j]->z()));
				invalide |= (this->mOrientationVec[i][j]->w() != this->mOrientationVec[i][j]->w());
			}//*/
			while(invalide);
			
		}
	}//*/
}

template<class Model>
void IKSolverPFOrient<Model>::computeDistance()
{	
	//ajouter plus de poids au root ???
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<this->mModels.size() ; i++)
	{
		double distance=0;
		for (it = this->mJointNameToPos.begin() ; it != this->mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				Eigen::Vector3d jtPos = this->mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(this->mCurrentFrame[(*it).second][1], this->mCurrentFrame[(*it).second][2], this->mCurrentFrame[(*it).second][3]);
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
		this->mCurrentDistances[i] = sqrt(distance);
	}
}

template<class Model>
void IKSolverPFOrient<Model>::computeDistanceHiera()
{	
	//ajouter plus de poids au root ???
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<this->mModels.size() ; i++)
	{
		double distance=0;
		for (it = this->mJointNameToPos.begin() ; it != this->mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				Eigen::Vector3d jtPos = this->mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(this->mCurrentFrame[(*it).second][1], this->mCurrentFrame[(*it).second][2], this->mCurrentFrame[(*it).second][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				Eigen::Matrix3d cov;
				cov.setIdentity();
				double tempo = diff.transpose()*(cov*diff);
				//distance += tempo*(5./(double)mModels[i]->getJoint((*it).first)->getHieraLevel());
				distance += tempo/log((double)this->mModels[i]->getJoint((*it).first)->getHieraLevel()*6);
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
		this->mCurrentDistances[i] = sqrt(distance);
	}
}

template<class Model>
void IKSolverPFOrient<Model>::computeLikelihood()
{	
	//contraintes a ajouter
	this->computeDistance();
	
	for (int i=0 ; i<this->mCurrentDistances.size() ; i++)
	{
		this->mCurrentLikelihood[i] = exp(-abs(this->mCurrentDistances[i]));
	}
	//cout << mCurrentLikelihood << "//" << endl;
}

template<class Model>
void IKSolverPFOrient<Model>::step()
{
	for (int i=0 ; i<this->mModels.size() ; i++)
	{
		this->mModels[i]->setColor(1,0,1,0.1);
		for (int j=0 ; j < this->mOrientationVec[i].size() ; j++)
		{
			bool invalide = false;
			Eigen::Quaterniond quat = this->mDefaultOrientationVec[i][j];
			Eigen::Vector3d offs;
			if (this->mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
			{
				offs = this->mOffsetVec[i][j]->vector();
			}
			else
			{
				offs = this->mDefaultOffsetVec[i][j].vector();
			}
			
			do
			{
				invalide = false;
				if (this->mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.5, 0.1, 0.05);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 0.05);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 0.1);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 0.1, 1, 1);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*this->mOrientationVec[i][j]) = (*this->mOrientationVec[i][j]);
				}
				else
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI, 1, 1, 1);
				}
				this->mOrientationVec[i][j]->normalize();

				Eigen::Vector3d tempo;
				if (this->mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01) + offs;
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->randn(0.1), 0, 0) + offs;
					}
					while (tempo[0] <= 0);
				}
				else if (this->mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*this->mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((this->mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((this->mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (this->mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((this->mOffsetVec[i][j]->x() != this->mOffsetVec[i][j]->x()) || (this->mOffsetVec[i][j]->y() != this->mOffsetVec[i][j]->y()) || (this->mOffsetVec[i][j]->z() != this->mOffsetVec[i][j]->z()));
				invalide |= (this->mOrientationVec[i][j]->w() != this->mOrientationVec[i][j]->w());
			}
			while(invalide);
			
			
		}
	}
	
	this->updateWeights();
	double Neff = this->computeNeff();;
	//cout << Neff << "****" << endl;
	if (Neff < 1.5 || Neff < this->mModels.size()*0.1)
		this->resample();
}

template<class Model>
double IKSolverPFOrient<Model>::stepAlt()
{
	cout << this->mCurrentDistances[mMaxWeightIndex] << endl;
	for (int i=0 ; i<this->mModels.size() ; i++)
	{
		this->mModels[i]->setColor(1,0,1,0.1);
		for (int j=0 ; j < this->mOrientationVec[i].size() ; j++)
		{
			double variance;
			//double variance = log(1+mCurrentDistances[i])*5;
			//if (mCurrentDistances[i] < 2)
			//	variance = (exp(mCurrentDistances[i])-1)/10.;
			//else
				variance = this->mCurrentDistances[i];
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
			Eigen::Quaterniond quat = (*this->mOrientationVec[i][j]);
			Eigen::Vector3d offs;
			if (this->mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
			{
				offs = this->mOffsetVec[i][j]->vector();
			}
			else
			{
				offs = this->mOffsetVec[i][j]->vector();
				//offs = mDefaultOffsetVec[i][j].vector();
			}
			do
			{
				invalide = false;
				if (this->mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 0.5, 0.1, 0.05);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 0.1, 1, 0.05);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 1, 1, 0.1);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 0.1, 1, 1);
				}
				else if(this->mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*this->mOrientationVec[i][j]) = (*this->mOrientationVec[i][j]);
				}
				else
				{
					(*this->mOrientationVec[i][j])=this->sampleQuTEM(quat, PI*variance, 1, 1, 1);
				}
				this->mOrientationVec[i][j]->normalize();
				
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
				invalide |= (this->mOrientationVec[i][j]->w() != this->mOrientationVec[i][j]->w());
			}
			while(invalide);
			
			
		}
	}
	this->updateWeights();
	double Neff = this->computeNeff();;
	//cout << Neff << "****" << endl;
	//if (Neff < 1.5 || Neff < mModels.size()*0.1)
		this->resample();
	return this->mCurrentDistances[this->mMaxWeightIndex];
}

template<class Model>
double IKSolverPFOrient<Model>::computeNeff()
{
	return 1./(this->mCurrentWeights.dot(this->mCurrentWeights));
}

template<class Model>
void IKSolverPFOrient<Model>::updateWeights()
{
	double sum=0;
	this->computeLikelihood();
	this->mModels[mMaxWeightIndex]->setColor(1,0,1,0.1);
	
	for (int i=0 ; i<this->mCurrentLikelihood.size() ; i++)
	{
		this->mCurrentWeights[i] = this->mCurrentWeights[i]*this->mCurrentLikelihood[i];
		sum+=this->mCurrentWeights[i];
		
		/*if(mCurrentWeights[i]>=mCurrentWeights[mMaxWeightIndex])
		{
			mMaxWeightIndex=i;
		}*/
	}
	for (int i=0 ; i<this->mCurrentLikelihood.size() ; i++)
	{
		if(this->mCurrentWeights[i]>=this->mCurrentWeights[mMaxWeightIndex])
		{
			this->mMaxWeightIndex=i;
		}
	}
	
	this->mCurrentWeights/=sum;
	this->mModels[mMaxWeightIndex]->setColor(0,1,1,1);
	//cout << mMaxWeightIndex << endl;
}

template<class Model>
void IKSolverPFOrient<Model>::resample()
{
	double invNbSamp = 1./this->mModels.size();
	Eigen::VectorXf cdf(this->mModels.size());
	cdf[0]=this->mCurrentWeights[0];
	for (int i=1 ; i<this->mModels.size() ; i++)
	{
		cdf[i]=cdf[i-1]+this->mCurrentWeights[i];
	}
	
	int i=0;
	double u = this->randUnif(invNbSamp);
	for (int j=0 ; j<this->mModels.size() ; j++)
	{
		while(u>cdf[i])
		{
			i++;
		}
		for (int k=0 ; k<this->mOrientationVec[i].size() ; k++)
		{
			(*this->mOrientationVec[j][k])=(*this->mOrientationVec[i][k]);
			(*this->mOffsetVec[j][k])=(*this->mOffsetVec[i][k]);
		}
		this->mCurrentWeights[j]=invNbSamp;
		u=u+invNbSamp;
	}
}

template<class Model>
void IKSolverPFOrient<Model>::save()
{
	for (int j=0 ; j<this->mModels.size() ; j++)
	{
		this->mModels[j]->setColor(0,1,0,1);
		for (int k=0 ; k<this->mOrientationVec[j].size() ; k++)
		{
			if (j!=this->mMaxWeightIndex)
			{
				(*this->mOrientationVec[j][k])=(*this->mOrientationVec[this->mMaxWeightIndex][k]);
				(*this->mOffsetVec[j][k])=(*this->mOffsetVec[this->mMaxWeightIndex][k]);
			}
		}
	}
}

template<class Model>
void IKSolverPFOrient<Model>::mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
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



#endif
