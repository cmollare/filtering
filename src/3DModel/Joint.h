#ifndef JOINT_H
#define JOINT_H

/*!
 * \file Joint.h
 * \brief Contains definitions of DOF and constraints.
 */

#include <cstdlib> 
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;//A supprimer

//Constraints on orientations
#define ORIENT_CONST_FREE "Free"
#define ORIENT_CONST_TWIST "Twist"
#define ORIENT_CONST_FLEX "Flex"
#define ORIENT_CONST_TFLEX "TFlex"
#define ORIENT_CONST_BIFLEX "BiFlex"
#define ORIENT_CONST_FIXED "Fixed"

//Constraints on offsets
#define OFFSET_CONST_FREE "Free"
#define OFFSET_CONST_BONE "Bone"
#define OFFSET_CONST_FIXED "Fixed"
#define OFFSET_CONST_PLANARXY "PlanarXY"
#define OFFSET_CONST_PLANARYZ "PlanarYZ"
#define OFFSET_CONST_PLANARXZ "PlanarXZ"

//Signs constraints
#define CONST_POSITIVE "Pos"
#define CONST_NEGATIVE "Neg"
#define CONST_NONE "NULL"


/*!
 * \class Joint
 * \brief class representing model joints
 * 
 * Joints are created following a hierarchical structure. The root Joint has a NULL parent.
 * Each Joint has a UNIQUE name. The root Joint has to be passed to the S3DModel class constructor.
 */

class Joint
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
		
		/*!
		 * \fn Joint(string name, Joint *parent=NULL, vector<double> offset=vector<double>(3,0), Eigen::Quaterniond quat=Eigen::Quaterniond(1,0,0,0))
		 * \brief Joint constructor
		 * 
		 * Constructor of the Joint class.
		 * 
		 * \param name Name of the joint.
		 * \param parent Pointer to the parent of the joint, the root joint has a NULL parent.
		 * \param offset Position relative to the parent joint.
		 * \param quat Local orientation of the joint.
		 */
		Joint(string name, Joint *parent=NULL, vector<double> offset=vector<double>(3,0), Eigen::Quaterniond quat=Eigen::Quaterniond(1,0,0,0));
		
		/*!
		 * \fn Joint(const Joint& jtCopy)
		 * \brief Joint copy constructor
		 * 
		 * Copy constructor of the Joint class.
		 * 
		 * \param jtCopy Joint to be copied.
		 */
		Joint(const Joint& jtCopy);
		
		/*!
		 * \fn ~Joint()
		 * \brief Joint destructor
		 * 
		 * Destructor of the Joint class. All joints under the root one are deleted.
		 * Be carefull to not delete them elsewhere, or use them after !
		 */
		~Joint();
		
		//void setDof(vector<bool> dof);
		/*!
		 * \fn Joint *getRoot()
		 * \brief Gets the root Joint pointer
		 * 
		 * This function gets the root Joint pointer
		 * 
		 * \return pointer on the root Joint
		 */
		Joint *getRoot();
		
		/*!
		 * \fn void setParentIfChild(Joint *jt)
		 * \brief Check if the param Joint is children, if it is, set parent.
		 * \param jt Children Joint.
		 */
		void setParentIfChild(Joint *jt);
		
		/*!
		 * \fn Joint *getParent()
		 * \brief Gets the parent pointer of the Joint
		 * 
		 * This function return a pointer on the parent Joint of this Joint
		 * 
		 * \return pointer on the parent Joint
		 */
		Joint *getParent();
		
		/*!
		 * \fn Joint *getJointFromName(std::string name)
		 * \brief Gets the pointer on the Joint with given name
		 * 
		 * This function finds the pointer on the Joint with the name taken in parameter.
		 * If the Joint is not found, the function return a NULL pointer.
		 * 
		 * \param name Name of the searched Joint.
		 * \return Pointer on Joint, NULL if not found.
		 */
		Joint *getJointFromName(std::string name);
		
		/*!
		 * \fn Joint *getJointFromName(Joint *jt, std::string name)
		 * \brief Gets the pointer on the Joint with given name
		 * 
		 * This function finds the pointer on the Joint with the name taken in parameter.
		 * If the Joint is not found, the function return a NULL pointer.
		 * The search is a descending search in the hierarchy.
		 * 
		 * \param jt Pointer on the Joint to start from.
		 * \param name Name of the searched Joint.
		 * \return Pointer on Joint, NULL if not found.
		 */
		Joint *getJointFromName(Joint *jt, std::string name);
		
		/*!
		 * \fn std::string getName()
		 * \brief Return the name of the Joint
		 * \return std::string name
		 */
		std::string getName();
		
		/*!
		 * \fn int getHieraLevel()
		 * \brief Get the hierarchy level of the Joint
		 * 
		 * It's 1 if the Joint is Root, 2 for root's children, etc...
		 * 
		 * \return The hierarchy level.
		 */
		int getHieraLevel();
		
		/*!
		 * \fn bool hasChildren()
		 * \brief Test for children
		 * \return True if the Joint has children, false otherwise.
		 */
		bool hasChildren();
		
		/*!
		 * \fn std::vector<Joint*>& getChildren()
		 * \brief Gets Joint children
		 * \return A vector on pointer on children.
		 */
		std::vector<Joint*>& getChildren();
		
		/*!
		 * \fn Joint* addChild(std::string name, vector<double> offset=vector<double>(3,0), Eigen::Quaterniond quat=Eigen::Quaterniond(1,0,0,0))
		 * \brief Add a child to this Joint
		 * 
		 * Allocate a new Joint, calling the constructor.
		 * This Joint will be automatically deleted during the destructor call.
		 * 
		 * \param name Name of the child.
		 * \param offset Relative position to the parent.
		 * \param quat Local orientation of the Joint.
		 * \return Pointer on the new created Joint.
		 */
		Joint* addChild(std::string name, vector<double> offset=vector<double>(3,0), Eigen::Quaterniond quat=Eigen::Quaterniond(1,0,0,0));
		
		/*!
		 * \fn Joint* addChild(Joint* jt)
		 * \brief Add a child to this Joint
		 * 
		 * Add a new child to this Joint.
		 * The new child will be automatically deleted during the destructor call.
		 * 
		 * \param jt Pointer on the Joint to add. It has to be dynamically allocated.
		 * \return Pointer on the new added Joint.
		 */
		Joint* addChild(Joint* jt);
		
		/*!
		 * \fn void setOrientation(const Eigen::Quaterniond quat)
		 * \brief Set the Joint new local orientation
		 * \param quat Joint local orientation.
		 */
		void setOrientation(const Eigen::Quaterniond quat);
		
		/*!
		 * \fn Joint* setConstraints(const std::string offset=OFFSET_CONST_FREE, const std::string orientation=ORIENT_CONST_FREE)
		 * \brief Set dof constraints on a Joint.
		 * 
		 * This function is called in YamlBodyJoint class.
		 * Dof are defined in Joint.h header.
		 * 
		 * \param offset Offset dof.
		 * \param orientation Orientation dof.
		 * \return Pointer to the current Joint.
		 */
		Joint* setConstraints(const std::string offset=OFFSET_CONST_FREE, const std::string orientation=ORIENT_CONST_FREE);
		
		/*!
		 * \fn Joint* setLimits(const std::vector<std::string>& signConst)
		 * \brief set offset sign limits on a Joint.
		 * 
		 * This function is called in YamlBodyJoint class.
		 * Sign limits are defined in Joint.h header.
		 * 
		 * \param signConst Vector of limits.
		 * \return Pointer to the current Joint.
		 */
		Joint* setLimits(const std::vector<std::string>& signConst);
		
		/*!
		 * \fn Joint* setPartition(int offset, int orientation)
		 * \brief set partition number associated with offset and orientation.
		 * 
		 * This function is called in YamlBodyJoint class.
		 * 
		 * \param offset Offset partition number.
		 * \param orientation Orientation partition number.
		 * \return Pointer to the current Joint.
		 */
		Joint* setPartition(int offset, int orientation);
		
		/*!
		 * \fn bool checkValidity(const Eigen::Vector3d& offset)
		 * \brief Check if the offset is contained in previously defined boundaries.
		 * \param offset Offset to check.
		 * \return True if everithing is right.
		 */
		bool checkValidity(const Eigen::Vector3d& offset);
		
		/*!
		 * \fn Eigen::Quaterniond* getOrientation()
		 * \brief Get the Joint local orientation.
		 * \return Quaternion of Joint local orientation.
		 */
		Eigen::Quaterniond* getOrientation();
		
		/*!
		 * \fn const Eigen::Quaterniond getDefaultOrientation()
		 * \brief Get the Joint local default orientation.
		 * \return Quaternion of Joint local default orientation.
		 */
		const Eigen::Quaterniond getDefaultOrientation();
		
		/*!
		 * \fn Eigen::Translation3d* getOffset()
		 * \brief Get relative position from parent
		 * \return Translation vector
		 */
		Eigen::Translation3d* getOffset();
		
		/*!
		 * \fn Eigen::Translation3d getDefaultOffset()
		 * \brief Get default relative position from parent
		 * \return Translation vector
		 */
		const Eigen::Translation3d getDefaultOffset();
		
		/*!
		 * \fn const Eigen::Vector3d getXYZVect()
		 * \brief Get world position
		 * 
		 * Get the Joint world position using forward kinematic.
		 * 
		 * \return A vector of (X,Y,Z) position.
		 */
		const Eigen::Vector3d getXYZVect();
		
		/*!
		 * \fn Eigen::Transform<double, 3, Eigen::Affine> getTransformationMatrix()
		 * \brief Get the transformation Matrix
		 * 
		 * This function retrieve the transformation Matrix of the Joint.
		 * 
		 * \return Return the 4x4 transformation Matrix.
		 */
		Eigen::Transform<double, 3, Eigen::Affine> getTransformationMatrix();
		
		/*!
		 * \fn Eigen::Transform<double, 3, Eigen::Affine> getGlobalTransformationMatrix()
		 * \brief Get the global transformation Matrix
		 * 
		 * This function retrieve the global transformation Matrix of the Joint.
		 * (from Root Joint)
		 * 
		 * \return Return the 4x4 transformation Matrix.
		 */
		Eigen::Transform<double, 3, Eigen::Affine> getGlobalTransformationMatrix();
		
		/*!
		 * \fn std::string getOffsetConstraint()
		 * \brief Return the constraint on the offset
		 * \return Constraint defined in the header
		 */
		std::string getOffsetConstraint();
		
		/*!
		 * \fn std::string getOrientationConstraint()
		 * \brief Return the constraint on the orientation
		 * \return Constraint defined in the header
		 */
		std::string getOrientationConstraint();
		
		/*!
		 * \fn int getOffsetPartition()
		 * \brief Get the partition number of the offset parameter.
		 * \return Partition number.
		 */
		int getOffsetPartition();
		
		/*!
		 * \fn int getOrientationPartition()
		 * \brief Get the partition number of the orientation parameter.
		 * \return Partition number.
		 */
		int getOrientationPartition();
		
		/*!
		 * \fn void setColor(float R=1, float G=1, float B=1, float alpha=0.1)
		 * \brief Set color of the Joint.
		 * \param R Red component.
		 * \param G Green component.
		 * \param B Blue component.
		 * \param alpha Transparency component.
		 */
		void setColor(float R=1, float G=1, float B=1, float alpha=0.1);
		
		/*!
		 * \fn std::vector<float> getColor()
		 * \brief Return the color of the Joint specified by "setColor" function.
		 * \return Color of the Joint.
		 */
		std::vector<float> getColor();
		
	private:
	
		std::string mName; /*!< Joint unique name */
		Joint *mParentJoint; /*!< parent Joint, NULL if root */
		std::vector<Joint*> mChildrenJoint; /*!< vector of child Joints */
		std::vector<float> mColors; /*!< Color of the Joint */
		int mHieraLevel; /*!< index of hierarchy level : 1 for the root */
		
		Eigen::Quaterniond mQDefault; /*!< Joint default local orientation */
		Eigen::Quaterniond mQLocal; /*!< Joint local orientation */
		//Eigen::Matrix2f mQCurrent;
		Eigen::Translation3d mDefaultOffset; /*!< Joint default relative position to parent */
		Eigen::Translation3d mLocalOffset;/*!< Joint relative position to parent */
		//float mCurrentOffset;
		
		std::string mOffsetConst; /*!< Constraint on offset */
		std::vector<std::string> mOffsetSignConst; /*!< Sign constraints */
		std::string mOrientationConst; /*!< Constraint on orientation */
		
		//For partitionned filtering
		int mOffsetPartition; /*!< Partition number of offset parameter */
		int mOrientationPartition; /*!< Partition number of orientation parameter */
};

#endif
