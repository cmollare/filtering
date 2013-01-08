//#include "S3DModelQRS.h"
#ifdef S3DMODELQRS_H

template<class Observations>
S3DModelQRS<Observations>::S3DModelQRS(const Joint* jt) : S3DModel<Observations>(jt)
{
	this->initStatsTool((this->mNbJoints+1)*6);
}

template<class Observations>
S3DModelQRS<Observations>::S3DModelQRS(const S3DModelQRS<Observations>& model) : S3DModel<Observations>(model)
{
}

template<class Observations>
S3DModelQRS<Observations>::~S3DModelQRS()
{
}

template<class Observations>
void S3DModelQRS<Observations>::sampleFromPrior()
{
	#if SAMPLING == USE_QUATERNION
		this->sampleQRS();
		for (int j=0 ; j < this->mOrientationVec.size() ; j++)
		{
			this->mDefaultOrientationVec[j] = (*this->mOrientationVec[j]); // Base orientation becomes default orientation
			this->mDefaultOffsetVec[j] = (*this->mOffsetVec[j]);  // Base offset becomes default offset
			
			Eigen::Quaterniond quat = this->mDefaultOrientationVec[j]; // Mean of orientations == default orientation
			bool invalide = false;
			Eigen::Vector3d offs = this->mDefaultOffsetVec[j].vector(); // Mean of offset == default offset
			do
			{
				invalide = false;
				if (this->mConstOrientVec[j] == ORIENT_CONST_FREE)
				{
					(*this->mOrientationVec[j])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(this->mConstOrientVec[j] == ORIENT_CONST_TWIST)
				{
					(*this->mOrientationVec[j])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 0.1, 0.05);
				}
				else if(this->mConstOrientVec[j] == ORIENT_CONST_FLEX)
				{
					(*this->mOrientationVec[j])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 0.1, 1, 0.05);
				}
				else if(this->mConstOrientVec[j] == ORIENT_CONST_TFLEX)
				{
					(*this->mOrientationVec[j])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 1, 0.1);
				}
				else if(this->mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
				{
					(*this->mOrientationVec[j])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 0.1, 1, 1);
				}
				else if(this->mConstOrientVec[j] == ORIENT_CONST_FIXED)
				{
					(*this->mOrientationVec[j]) = (*this->mOrientationVec[j]);
				}
				else
				{
					(*this->mOrientationVec[j])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 1, 1);
				}
				this->mOrientationVec[j]->normalize(); // NORMALIZATION STEP EXTREMELY IMPORTANT
				
				Eigen::Vector3d tempo;
				if (this->mConstOffsetVec[j] == OFFSET_CONST_FREE)
				{
					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE) + offs;
					}
					while(!this->getJoint(this->mNameVec[j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, 0) + offs;
					}
					while(!this->getJoint(this->mNameVec[j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET), 0) + offs;
					}
					while(!this->getJoint(this->mNameVec[j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(this->mNameVec[j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, this->quasiRandn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(this->mNameVec[j])->checkValidity(tempo));
				}
				else if (this->mConstOffsetVec[j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0) + offs;
				}
				(*this->mOffsetVec[j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				invalide |= ((this->mOffsetVec[j]->x() == std::numeric_limits<double>::infinity()) || (this->mOffsetVec[j]->y() == std::numeric_limits<double>::infinity()) || (this->mOffsetVec[j]->z() == std::numeric_limits<double>::infinity()));
				invalide |= ((this->mOffsetVec[j]->x() == -std::numeric_limits<double>::infinity()) || (this->mOffsetVec[j]->y() == -std::numeric_limits<double>::infinity()) || (this->mOffsetVec[j]->z() == -std::numeric_limits<double>::infinity()));
				invalide |= ((this->mOffsetVec[j]->x() != this->mOffsetVec[j]->x()) || (this->mOffsetVec[j]->y() != this->mOffsetVec[j]->y()) || (this->mOffsetVec[j]->z() != this->mOffsetVec[j]->z()));
				invalide |= (this->mOrientationVec[j]->w() != this->mOrientationVec[j]->w());
			}
			while(invalide);
			
		}
	#elif SAMPLING == USE_EULER
	
		this->sampleQRS();
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
						(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 0, 1));
					}
					else if(mConstOrientVec[j] == ORIENT_CONST_TWIST)
					{
						(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0));
					}
					else if(mConstOrientVec[j] == ORIENT_CONST_FLEX)
					{
						(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0));
					}
					else if(mConstOrientVec[j] == ORIENT_CONST_TFLEX)
					{
						(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0));
					}
					else if(mConstOrientVec[j] == ORIENT_CONST_BIFLEX)
					{
						(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 0, 1));
					}
					else if(mConstOrientVec[j] == ORIENT_CONST_FIXED)
					{
						(*mOrientationVec[j]) = (*mOrientationVec[j]);
					}
					else
					{
						(*mOrientationVec[j])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 0, 1));
					}
					mOrientationVec[j]->normalize();
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[j] == OFFSET_CONST_FREE)
				{
					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_BONE)
				{

					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXY)
				{
					int i=0;
					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET), 0) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARYZ)
				{
					do
					{
						tempo = Eigen::Vector3d(0, this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET)) + offs;
					}
					while(!this->getJoint(mNameVec[j])->checkValidity(tempo));
				}
				else if (mConstOffsetVec[j] == OFFSET_CONST_PLANARXZ)
				{
					do
					{
						tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, this->quasiRandn(VAROFFSET)) + offs;
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

template<class Observations>
void S3DModelQRS<Observations>::updatePart(int partition)
{
	#if SAMPLING == USE_QUATERNION
		this->sampleQRS();
			std::multimap<int, std::string>::iterator itOff = this->mOffsetPartToName.find(partition);
			std::multimap<int, std::string>::iterator itOrient = this->mOrientPartToName.find(partition);
			
			//Quaternion sampling
			for (itOrient = mOrientPartToName.equal_range(partition).first ; itOrient != mOrientPartToName.equal_range(partition).second ; ++itOrient)
			{
				int pos = this->mJointNameToInt[(*itOrient).second];// Retrieve position of the Joint in orientation vectors
				

				bool invalide = false;
				//Eigen::Quaterniond quat = mDefaultOrientationVec[i][pos];
				Eigen::Quaterniond quat = (*this->mOrientationVec[pos]); // Mean orientation is the previous orientation
				
				do
				{
					invalide = false;
					if (mConstOrientVec[pos] == ORIENT_CONST_FREE)
					{
						(*mOrientationVec[pos])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 1, 1);//A modifier suivant les contraintes
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_TWIST)
					{
						(*mOrientationVec[pos])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 0.1, 0.05);
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_FLEX)
					{
						(*mOrientationVec[pos])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 0.1, 1, 0.05);
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_TFLEX)
					{
						(*mOrientationVec[pos])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 1, 0.1);
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_BIFLEX)
					{
						(*mOrientationVec[pos])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 0.1, 1, 1);
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_FIXED)
					{
						(*mOrientationVec[pos]) = (*mOrientationVec[pos]);
					}
					else
					{
						(*mOrientationVec[pos])=this->sampleQuasiQuTEM(quat, VARQUATERNION, 1, 1, 1);
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
						tempo = Eigen::Vector3d(this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE) + offs;
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_BONE)
					{

						do
						{
							tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, 0) + offs;
						}
						while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXY)
					{
						do
						{
							tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET), 0) + offs;
						}
						while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARYZ)
					{
						do
						{
							tempo = Eigen::Vector3d(0, this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET)) + offs;
						}
						while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXZ)
					{
						do
						{
							tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, this->quasiRandn(VAROFFSET)) + offs;
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
		
			this->sampleQRS();
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
						(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 0, 1));
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_TWIST)
					{
						(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0));
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_FLEX)
					{
						(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0));
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_TFLEX)
					{
						(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0));
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_BIFLEX)
					{
						(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 0, 1));
					}
					else if(mConstOrientVec[pos] == ORIENT_CONST_FIXED)
					{
						(*mOrientationVec[pos]) = (*mOrientationVec[pos]);
					}
					else
					{
						(*mOrientationVec[pos])=quat*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(1, 0, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 1, 0))*Eigen::AngleAxisd(this->quasiRandn(VAREULER), Eigen::Vector3d(0, 0, 1));
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
						tempo = Eigen::Vector3d(this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE, this->quasiRandn()*VAROFFSETFREE) + offs;
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_BONE)
					{

						do
						{
							tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, 0) + offs;
						}
						while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXY)
					{
						do
						{
							tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET), 0) + offs;
						}
						while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARYZ)
					{
						do
						{
							tempo = Eigen::Vector3d(0, this->quasiRandn(VAROFFSET), this->quasiRandn(VAROFFSET)) + offs;
						}
						while(!this->getJoint(mNameVec[pos])->checkValidity(tempo));
					}
					else if (mConstOffsetVec[pos] == OFFSET_CONST_PLANARXZ)
					{
						do
						{
							tempo = Eigen::Vector3d(this->quasiRandn(VAROFFSET), 0, this->quasiRandn(VAROFFSET)) + offs;
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

template<class Observations>
void S3DModelQRS<Observations>::estimateMMSE(Eigen::VectorXd& weights, S3DModelQRS<Observations>** particles, int nbParticles)
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

#endif

