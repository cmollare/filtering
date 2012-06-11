#ifndef S3DMODEL_H
#define S3DMODEL_H

/*!
 * \file S3DModel.h
 * 
 */

#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "Joint.h"

/*!
 * \class S3DModel
 * \brief class representing the 3D model
 * 
 * The class S3DModel contains the Joint hierarchy and can be used as an interface to work on the model.
 * This class also map some informations for efficiency.
 */

class S3DModel
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
		
		/*!
		 * \fn S3DModel(int id)
		 * \brief S3DModel constructor
		 * 
		 * This constructor create a single empty Joint.
		 * 
		 * \param id Unique id of the model.
		 */
		S3DModel(int id);
		
		/*!
		 * \fn S3DModel(const Joint* jt, unsigned int id=0)
		 * \brief S3DModel constructor
		 * 
		 * This constructor takes a complete hierarchy as paramater, and map the model.
		 * 
		 * \param jt Pointer on the model root Joint.
		 * \param id Unique id of the model.
		 */
		S3DModel(const Joint* jt, unsigned int id=0);
		
		/*!
		 * \fn S3DModel(const S3DModel& model)
		 * \brief S3DModel copy constructor
		 * 
		 * \param model S3DModel to be copied.
		 */
		S3DModel(const S3DModel& model);
		
		/*!
		 * \fn ~S3DModel()
		 * \brief S3DModel destructor
		 * 
		 * Delete all the Joint, you don't have to do it yourself.
		 * 
		 */
		~S3DModel();
		
		/*!
		 * \fn Joint* getRootJoint()
		 * \brief Get the root Joint
		 * \return Return a pointer on the model root Joint.
		 */
		Joint* getRootJoint();
		
		/*!
		 * \fn void createMaps()
		 * \brief Map the model
		 */
		void createMaps();
		
		/*!
		 * \fn int getNumberJoint()
		 * \brief Get the number of Joints
		 * \return Return the number of Joints int the model
		 */
		int getNumberJoint();
		
		/*!
		 * \fn Joint* getJoint(std::string)
		 * \brief get Joint from name
		 * \param str Name of the Joint.
		 * \return Pointer on Joint class.
		 */
		Joint* getJoint(std::string str);
		
		/*!
		 * \fn vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > getOrientationVec()
		 * \brief Get the orientation vector
		 * 
		 * Return the local orientation of each Joints in a vector.
		 * Warning : If you modify the orientation, it will be modified in the corresponding Joint.
		 * 
		 * \return A vector of Eigen::quaterniond*
		 */
		vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > getOrientationVec();
		
		/*!
		 * \fn vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > getOffsetVector()
		 * \brief Get the offset vector
		 * 
		 * Return the local offset of each Joints in a vector.
		 * Warning : If you modify the offset, it will be modified in the corresponding Joint.
		 * 
		 * \return A vector of Eigen::Translation3d*
		 */
		vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > getOffsetVector();
		
		/*!
		 * \fn vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > getDefaultOrientationVec()
		 * \brief Get the default orientation vector
		 * 
		 * Return the default local orientation of each Joints in a vector.
		 * 
		 * \return A vector of Eigen::Quaterniond
		 */
		vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > getDefaultOrientationVec();
		
		/*!
		 * \fn vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > getDefaultOffsetVector()
		 * \brief Get the default offset vector
		 * 
		 * Return the default local offset of each Joints in a vector.
		 * 
		 * \return A vector of Eigen::Translation3d
		 */
		vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > getDefaultOffsetVector();
		
		/*!
		 * \fn vector<std::string> getNameVec()
		 * \brief Return the name of each Joints in a vector
		 * \return vector of names
		 */
		vector<std::string> getNameVec();
		
		/*!
		 * \fn vector<std::string> getConstOffsetVec()
		 * \brief get a Vector of dof offset type on each Joint
		 * 
		 * The different types are defined in the Joint.h file.
		 * 
		 * \return Vector of std::string.
		 */
		vector<std::string> getConstOffsetVec();
		
		/*!
		 * \fn vector<std::string> getConstOffsetVec()
		 * \brief get a Vector of dof orientation type on each Joint
		 * 
		 * The different types are defined in the Joint.h file.
		 * 
		 * \return Vector of std::string.
		 */
		vector<std::string> getConstOrientVec();
		
		/*!
		 * \fn std::multimap<int, std::string> getOffsetPartitionMultimap()
		 * \brief get the partition associated with the offset of each Joint
		 * \return A multimap between partitions and Joint names.
		 */
		std::multimap<int, std::string> getOffsetPartitionMultimap();
		
		/*!
		 * \fn std::multimap<int, std::string> getOrientPartitionMultimap()
		 * \brief get the partition associated with the orientation of each Joint
		 * \return A multimap between partitions and Joint names.
		 */
		std::multimap<int, std::string> getOrientPartitionMultimap();
		
		/*!
		 * \fn int getPartitionNumber()
		 * \brief get the number of partitions (useful for PartFilter only)
		 * \return The number of partitions.
		 */
		int getPartitionNumber();
		
		/*!
		 * \fn void setColor(float R=1, float G=0, float B=1, float alpha=0.1)
		 * \brief Change the color of a model.
		 * \param R Red component.
		 * \param G Green component.
		 * \param B Blue component.
		 * \param alpha Transparency component.
		 */
		void setColor(float R=1, float G=0, float B=1, float alpha=0.1);
		
		/*!
		 * \fn void setVisible(bool visible)
		 * \brief Make a model visible or unsvisible.
		 * \param visible True if the model has to be visible.
		 */
		void setVisible(bool visible);
		
		/*!
		 * \fn bool isVisible()
		 * \brief Is the model visible ?
		 * \return True if the model is visible.
		 */
		bool isVisible();
		
		void debug()//fonction temporaire
		{
			for (int i=0 ; i<mOrientationVec.size() ; i++)
			{
				cout << "****************" << endl;
				cout << Eigen::Matrix3d(*mOrientationVec[i]) << endl;
				cout << "****************" << endl;
			}
		}
		
	private:
	
		/*!
		 * \fn void createMaps(vector<Joint*>& jts)
		 * \brief map recursively
		 * 
		 * For recursivity of createMaps public function.
		 * 
		 * \param jts Vector of pointer on child Joints.
		 */
		void createMaps(vector<Joint*>& jts);
		
		/*!
		 * \fn void createOrientationVec()
		 * \brief create the local orientation vector of the Joints.
		 * 
		 * This function is used in the S3DModel constructor.
		 * 
		 */
		void createOrientationVec();
		
		/*!
		 * \fn void createOffsetVec()
		 * \brief create the local offset vector of the Joints.
		 * 
		 * This function is used in the S3DModel constructor.
		 * 
		 */
		void createOffsetVec();
		
		/*!
		 * \fn void createNameVec()
		 * \brief Create a vector of Joint names
		 */
		void createNameVec();
		
		/*!
		 * \fn void createConstraintVecs()
		 * \brief Create the dof vectors.
		 * 
		 * For mapping only.
		 * 
		 */
		void createConstraintVecs();
		
		/*!
		 * \fn void createDefaultVecs()
		 * \brief Create default offset and orientation vectors.
		 * 
		 * For mapping only.
		 * 
		 */
		void createDefaultVecs();
		
		/*!
		 * \fn void createPartitionMultimaps()
		 * \brief Create multimaps for partitionned filter.
		 * 
		 * For mapping only.
		 * 
		 */
		void createPartitionMultimaps();
		
		unsigned int mId; /*!< id of the model */
		Joint *mRootJoint; /*!< pointer on the root Joint */
		
		std::map<std::string, Joint*> mStringToJoint; /*!< mapping from Joints name to Joint pointers (for speed) */
		std::map<std::string, int> mStringToInt; /*!< mapping from Joints name to Joint indexes (for speed) */
		std::map<int, Joint*> mIntToJoint; /*!< mapping from Joints index to Joint pointers (for speed) */
		int mNbJoints; /*!< total number of Joints */
		
		std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > mOrientationVec; /*!< Orientation vector */
		std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > mOffsetVec; /*!< Offset vector */
		std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > mDefaultOrientationVec; /*!< Default orientation vector */
		std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > mDefaultOffsetVec; /*!< Default offset vector */
		std::vector<std::string> mNameVec; /*!< Vector of Joint names */
		std::vector<std::string> mConstOffsetVec; /*!< vector of offset dofs */
		std::vector<std::string> mConstOrientVec; /*!< vector of orientation dofs */
		
		std::multimap<int, std::string> mOffsetPartToName; /*!< offset MultiMap from partition number to Joint name */
		std::multimap<int, std::string> mOrientPartToName; /*!< orientation MultiMap from partition number to Joint name */
		int mPartitionNumber; /*!< Number of partitions */
		
		bool mIsVisible; /*!< Is the model visible ? True if yes */
};

#endif
