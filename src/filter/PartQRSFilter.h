#ifndef PARTQRSFILTER_H
#define PARTQRSFILTER_H

/*!
 * \file PartQRSFilter.h
 */

#include "Filter.h"

#define TEMP3 0.1

/*!
 * \class PartQRSFilter
 * \brief Class inherited from Filter.
 */
class PartQRSFilter : public Filter
{
	public:
		/*!
		 * \fn PartQRSFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions)
		 * \brief PartQRSFilter constructor.
		 * \param mods Vector of pointers on S3DModel.
		 * \param posNames Vector of observation names.
		 * \param jointsXYZPositions First observation.
		 */
		PartQRSFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);
		
		/*!
		 * \fn virtual void initFilter()
		 * \brief initialization function inherited from Filter class.
		 */
		virtual void initFilter();
		
		/*!
		 * \fn virtual void computeLikelihood()
		 * \brief Computation of likelihood inherited from Filter class.
		 * 
		 * (not used in this class)
		 * 
		 */
		virtual void computeLikelihood();
		
		/*!
		 * \fn virtual void computeLikelihood(int partition)
		 * \brief Computation of likelihood for partionned filter.
		 * \param partition Index of the partition.
		 */
		virtual void computeLikelihood(int partition);
		
		/*!
		 * \fn virtual void step(std::vector<std::vector<double> > frame)
		 * \brief Filtering step inherited from Filter class.
		 * \param frame New frame.
		 */
		virtual void step(std::vector<std::vector<double> > frame);
		
		/*!
		 * \fn double computeNeff()
		 * \brief Compute Neff
		 * \return Neff result.
		 */
		double computeNeff();
		
		/*!
		 * \fn void mapJointToObs(std::map<std::string, std::string> jointNameToPosName)
		 * \brief Map Joints to observations.
		 * \param jointNameToPosName Map of Joints to Observations.
		 */
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		
		/*!
		 * \fn S3DModel* computeMMSE()
		 * \brief Compute the MMSE particle (result)
		 * \return Pointer on MMSE particle.
		 */
		S3DModel* computeMMSE();
		
	protected:
	
		/*!
		 * \fn virtual void computeDistance()
		 * \brief Compute distance, inherited from Filter class.
		 * 
		 * (Not used in this class)
		 * 
		 */
		virtual void computeDistance();
		
		/*!
		 * \fn virtual void computeDistance(int partition)
		 * \brief Compute distance for partionned filter.
		 * \param partition Index of the partition.
		 */
		virtual void computeDistance(int partition);
		
		/*!
		 * \fn void updateWeights(int partition)
		 * \brief Update weight for partionned filter.
		 * \param partition Index of the partition.
		 */
		void updateWeights(int partition);
		
		/*!
		 * \fn void resample()
		 * \brief Resampling function.
		 */
		void resample();
		
		Eigen::VectorXf mCurrentWeights; /*!< Current weights */
		int mMaxWeightIndex; /*!< Index of maximum weighted model */
		
		std::vector<std::multimap<int, std::string> > mOffsetPartToName; /*!< Map between offset and Joint names */
		std::vector<std::multimap<int, std::string> > mOrientPartToName; /*!< Map between orientations and Joint names */
		
		int mPartitionNumber; /*!< Number of partitions */
};

#endif
