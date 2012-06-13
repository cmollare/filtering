#ifndef PFFILTER_H
#define PFFILTER_H

/*!
 * \file PFFilter.h
 */

#include "Filter.h"

#define TEMPO 0.05

using namespace std;

/*!
 * \class PFFilter
 * \brief Class for posture filtering
 * 
 * Basic particle filter
 * 
 */
class PFFilter : public Filter
{
	public:
	
		/*!
		 * \fn PFFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions)
		 * \brief Constructor of PFFilter class.
		 * \param mods vector of pointer on S3DModel.
		 * \param posNames Name of observations.
		 * \param jointsXYZPositions First observations.
		 */
		PFFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);
		
		/*!
		 * \fn virtual void initFilter()
		 * \brief Initialization function inherited from Filter class.
		 */
		virtual void initFilter();
		
		/*!
		 * \fn virtual void computeLikelihood()
		 * \brief Likelihood computation function infherited from Filter class.
		 */
		virtual void computeLikelihood();
		
		/*!
		 * \fn virtual void step(std::vector<std::vector<double> > frame)
		 * \brief Filtering step function inherited from Filter class.
		 * \param frame New observations.
		 */
		virtual void step(std::vector<std::vector<double> > frame);
		
		/*!
		 * \fn double computeNeff()
		 * \brief Neff comuptation.
		 * \return Neff result.
		 */
		double computeNeff();
		
		/*!
		 * \fn void mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
		 * \brief Map Joint names to observation names. 
		 * \param jointNameToPosName Map between Joint names and observation names.
		 */
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		
		/*!
		 * \fn S3DModel* computeMMSE()
		 * \brief Compute MMSE particle (result).
		 * \return Pointer on MMSE particle.
		 */
		S3DModel* computeMMSE();
		
	protected:
	
		/*!
		 * \fn virtual void computeDistance()
		 * \brief distance computation, inherited from Filter class.
		 */
		virtual void computeDistance();
		
		/*!
		 * \fn void updateWeights()
		 * \brief update and normalize weights.
		 */
		void updateWeights();
		
		/*!
		 * \fn void resample()
		 * \brief Resampling function.
		 */
		void resample();
		
		Eigen::VectorXf mCurrentWeights; /*!< Current weights */
		int mMaxWeightIndex; /*!< Index of max weight (not useful anymore) */
};

#endif
