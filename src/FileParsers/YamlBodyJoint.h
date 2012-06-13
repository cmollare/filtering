#ifndef YAMLBODYJOINT_H
#define YAMLBODYJOINT_H

/*!
 * \file YamlBodyJoint.h
 */

#include <yaml-cpp/yaml.h>
#include "../3DModel/Joint.h"
#include <fstream>
#include <string>

/*!
 * \struct SOffset
 * \brief Structure reprensenting the offset of the Joint
 * 
 * This structure contains all information on offset.
 * 
 */
typedef struct SOffset
{
	std::vector<double> Mean; /*!< Position of the Joint relative to its parent */
	std::string Dof; /*!< Constraint on the local offset of the Joint */
	std::vector<std::string> SignConst; /*!< Constraint on offset sign */
	int Partition; /*!< Partition number of offset */
}SOffset;


/*!
 * \struct SOrientation
 * \brief Structure reprensenting the orientation of the Joint
 * 
 * This structure contains all information on orientation.
 * 
 */
typedef struct SOrientation
{
	Eigen::Quaterniond Mean; /*!< Local orientation of the Joint */
	std::string Dof; /*!< Constraint on the local orientation of the Joint */
	int Partition; /*!< Partition number of orientation */
}SOrientation;

/*!
 * \struct SBJoints
 * \brief Structure reprensenting a Joint
 * 
 * This structure contains all information for a Joint extracted from the .ymd file.
 * 
 */
typedef struct SBJoints
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::string Joint; /*!< Name of the Joint */
	std::string Parent; /*!< Name of the parent = "_Root" if this Joint is the root */ 
	SOffset Offset; /*!< Associated offset structure */
	SOrientation Orientation; /*!< Associated orientation structure */
}SBJoints;
			
/*!
 * \struct SYmdFile
 * \brief Structure representing the ymd file
 */
typedef struct SYmdFile
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::string NbPoses; /*!< Nb of poses in the ymd file (for mixed format) */
	std::string NbJoints; /*!< Nb of Joints in the ymd file */
	std::string FirstIndex; /*!< First index (not implemented yet) */
	std::vector<SBJoints, Eigen::aligned_allocator<SBJoints> > BJoints; /*!< vector of SBJoints structure : contains all Joints of the ymd file */
}SYmdFile;

/*!
 * \class YamlBodyJoint
 * \brief Class for Yaml type file parse
 * 
 * This class parse the .ymd file and create a hierarchical model with the Joint class.
 * 
 */

class YamlBodyJoint
{
	public:
	
		/*!
		 * \fn YamlBodyJoint(std::string fileName)
		 * \brief YamlBodyJoint constructor
		 * \param fileName Path to the .ymd file.
		 */
		YamlBodyJoint(std::string fileName);
		
		/*!
		 * \fn ~YamlBodyJoint()
		 * \brief YamlBodyJoint destructor
		 */
		~YamlBodyJoint();
		
		/*!
		 * \fn void parseModel()
		 * \brief Parse the .ymd file
		 */
		void parseModel();
		
		/*!
		 * \fn void createModel()
		 * \brief Create the hierarchical structure with Joint class
		 */
		void createModel();
		
		/*!
		 * \fn Joint* getModel()
		 * \brief Return the model and call createModel, if the model is not created
		 * \return A pointer on the root Joint.
		 */
		Joint* getModel();
	
	private:
		
		std::string mFileName; /*!< Name of the .ymd file */
		Joint *mModel; /*!< Pointer on the root Joint */
		SYmdFile mParsedFile; /*!< Structure of the parsed .ymd file */
		
		

};

/*!
 * \fn void operator>> (const YAML::Node& node, SYmdFile& YmdFile)
 * \brief Extract a SYmdFile structure from the corresponding node
 * \param node Node to extract from.
 * \param YmdFile Where to store the result.
 */
void operator>> (const YAML::Node& node, SYmdFile& YmdFile);

/*!
 * \fn void operator>> (const YAML::Node& node, vector<SBJoints, Eigen::aligned_allocator<SBJoints> >& BJoints)
 * \brief Extract a vector of SBJoints structures from the corresponding node
 * \param node Node to extract from.
 * \param BJoints Where to store the result.
 */
void operator>> (const YAML::Node& node, std::vector<SBJoints, Eigen::aligned_allocator<SBJoints> >& BJoints);


/*!
 * \fn void operator>> (const YAML::Node& node, SOffset& Offset)
 * \brief Extract SOffset structure from the corresponding node
 * \param node Node to extract from.
 * \param Offset SOffset structure.
 */
void operator>> (const YAML::Node& node, SOffset& Offset);

/*!
 * \fn void operator>> (const YAML::Node& node, SOrientation& Orientation)
 * \brief Extract SOrientation structure from the corresponding node
 * \param node Node to extract from.
 * \param Orientation SOrientation structure.
 */
void operator>> (const YAML::Node& node, SOrientation& Orientation);

/*!
 * \fn void operator>> (const YAML::Node& node, Eigen::Quaternionf& quat)
 * \brief Extract a quaternion from the corresponding node
 * \param node Node to extract from.
 * \param quat Where to store the result.
 */
void operator>> (const YAML::Node& node, Eigen::Quaterniond& quat);

/*!
 * \fn void operator>> (const YAML::Node& node, vector<float>& Offset)
 * \brief Extract a position vector<float> from the corresponding node
 * \param node Node to extract from.
 * \param Offset Where to store the result.
 */
void operator>> (const YAML::Node& node, std::vector<double>& Offset);


/*!
 * \fn void operator>> (const YAML::Node& node, vector<std::string>& SignConst)
 * \brief Extract a vector of sign constraints from the corresponding node.
 * \param node Node to extract from.
 * \param SignConst Vector of sign constraints.
 */
void operator>> (const YAML::Node& node, std::vector<std::string>& SignConst);

#endif
