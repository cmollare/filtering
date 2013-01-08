#ifndef S3DMODELQRS_H
#define S3DMODELQRS_H

#include "S3DModel.h"

template<class Observations>
class S3DModelQRS : public S3DModel<Observations>
{
	public:
		S3DModelQRS(const Joint* jt);
		S3DModelQRS(const S3DModelQRS& model);
		virtual ~S3DModelQRS();
		
		virtual void sampleFromPrior();
		virtual void updatePart(int partition);
		virtual void estimateMMSE(Eigen::VectorXd& weights, S3DModelQRS** particles, int nbParticles);
};

#include "S3DModelQRS.cpp"

#endif
