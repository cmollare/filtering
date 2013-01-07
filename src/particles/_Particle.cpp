#include "_Particle.h"

_Particle::_Particle()
{
	mColor = std::vector<float>(4,1);
	this->mCurrentLikelihood=0;
	mIsVisible = true;
	mNumberOfPartitions=0;
}

_Particle::_Particle(const _Particle& particle) //copy constructor
{
	this->mColor = particle.mColor;
	this->mCurrentLikelihood = particle.mCurrentLikelihood;
	this->mIsVisible = particle.mIsVisible;
	this->mNumberOfPartitions = particle.mNumberOfPartitions;
	this->mObservations = particle.mObservations;
}

void _Particle::saveResults(ResultParser* resParser)
{
}

void _Particle::estimateMMSE(Eigen::VectorXd& weights, _Particle** particles, int nbParticles)
{
}

double _Particle::getLikelihood()
{
	return this->mCurrentLikelihood;
}

void _Particle::setLikelihood(double likelihood)
{
	this->mCurrentLikelihood = likelihood;
}

void _Particle::setId(int id)
{
	mId = id;
}

int _Particle::getId()
{
	return mId;
}

void _Particle::setColor(float R=1, float G=0, float B=1, float alpha=0.1)
{
	mColor[0] = R;
	mColor[1] = G;
	mColor[2] = B;
	mColor[3] = alpha;
}

void _Particle::setVisible(bool visible)
{
	mIsVisible = visible;
}

bool _Particle::isVisible()
{
	return mIsVisible;
}

_Particle& _Particle::operator =(const _Particle& part)
{
	this->mCurrentLikelihood = part.mCurrentLikelihood;
	this->mObservations = part.mObservations;
}

int _Particle::getNumberOfPartitions()
{
	return mNumberOfPartitions;
}

_Observation& _Particle::getCurrentObservations()
{
	return mObservations;
}
