#include "YamlBodyJoint.h"

using namespace std;

YamlBodyJoint::YamlBodyJoint(std::string fileName)
{
	mFileName = fileName;
	mModel = NULL;
}

YamlBodyJoint::~YamlBodyJoint()
{
	delete mModel;
}

void YamlBodyJoint::parseModel()
{
	std::ifstream file(mFileName.c_str());
	YAML::Parser parser(file);
	YAML::Node node;
	parser.GetNextDocument(node);
	
	node >> mParsedFile;
	
	file.close();
}

void YamlBodyJoint::createModel()
{
	if (mParsedFile.BJoints.size() <= 0)
		this->parseModel();
	
	if (mParsedFile.BJoints[0].Parent == "_Root")
	{
		mModel = new Joint(mParsedFile.BJoints[0].Joint, NULL, mParsedFile.BJoints[0].Offset.Mean, mParsedFile.BJoints[0].Orientation.Mean);
		mModel->setConstraints(mParsedFile.BJoints[0].Offset.Dof, mParsedFile.BJoints[0].Orientation.Dof)->setLimits(mParsedFile.BJoints[0].Offset.SignConst)
		      ->setPartition(mParsedFile.BJoints[0].Offset.Partition, mParsedFile.BJoints[0].Orientation.Partition);
		
		for (int i=1 ; i<mParsedFile.BJoints.size() ; i++)
		{
			cout << "Searching " << mParsedFile.BJoints[i].Parent << " ..." << endl;
			Joint* parent = mModel->getJointFromName(mParsedFile.BJoints[i].Parent);
			if (parent != NULL)
			{
				parent->addChild(mParsedFile.BJoints[i].Joint, mParsedFile.BJoints[i].Offset.Mean, mParsedFile.BJoints[i].Orientation.Mean)
					  ->setConstraints(mParsedFile.BJoints[i].Offset.Dof, mParsedFile.BJoints[i].Orientation.Dof)
					  ->setLimits(mParsedFile.BJoints[i].Offset.SignConst)
					  ->setPartition(mParsedFile.BJoints[i].Offset.Partition, mParsedFile.BJoints[i].Orientation.Partition);//Create Children
					  
				cout << mParsedFile.BJoints[i].Joint << " added to " << parent->getName() << endl;
			}
			else
			{
				cout << "Error to retrieve parent" << endl;
				cout << "Check yaml file" << endl;
			}
		}
		cout << "*******Skeleton constructed*******" << endl;
	}
	else
	{
		std::cout << "Error : The first joint has to be root" << std::endl;
	}
}

Joint* YamlBodyJoint::getModel()
{
	return mModel;
}

void operator >> (const YAML::Node& node, SYmdFile &YmdFile)
{
	node["NbPoses"] >> YmdFile.NbPoses;
	node["NbJoints"] >> YmdFile.NbJoints;
	node["FirstIndex"] >> YmdFile.FirstIndex;
	node["BJoints"] >> YmdFile.BJoints;
}

void operator >> (const YAML::Node& node, vector<SBJoints, Eigen::aligned_allocator<SBJoints> > &BJoints)
{
	BJoints.resize(node.size());
	for (int i=0 ; i<node.size() ; i++)
	{
		const YAML::Node& joint = node[i];
		joint["Joint"] >> BJoints[i].Joint;
		joint["Parent"] >> BJoints[i].Parent;
		joint["Offset"] >> BJoints[i].Offset;
		joint["Orientation"] >> BJoints[i].Orientation;
		
		if (i==0 && (BJoints[i].Parent != "_Root"))
		{
			std::cout << "Error : The first joint of the Yaml file have to be root" << std::endl;
			BJoints[i].Joint = "_Root";
			std::cout << "The joint : " << BJoints[i].Joint << "have been setted to _Root by default" << std::endl;
		}
	}
}

void operator>> (const YAML::Node& node, Eigen::Quaterniond& quat)
{
	quat = Eigen::Quaterniond(node["W"].to<double>(), node["X"].to<double>(), node["Y"].to<double>(), node["Z"].to<double>());
	quat.normalize();
}

void operator>> (const YAML::Node& node, vector<double>& Offset)
{
	Offset.push_back(node["X"].to<double>());
	Offset.push_back(node["Y"].to<double>());
	Offset.push_back(node["Z"].to<double>());
}

void operator>> (const YAML::Node& node, SOffset& Offset)
{
	node["Mean"] >> Offset.Mean;
	node["Dof"] >> Offset.Dof;
	node["SignConst"] >> Offset.SignConst;
	node["Partition"] >> Offset.Partition;
}

void operator>> (const YAML::Node& node, SOrientation& Orientation)
{
	node["Mean"] >> Orientation.Mean;
	node["Dof"] >> Orientation.Dof;
	node["Partition"] >> Orientation.Partition;
}

void operator>> (const YAML::Node& node, vector<std::string>& SignConst)
{
	SignConst.push_back(node["X"].to<std::string>());
	SignConst.push_back(node["Y"].to<std::string>());
	SignConst.push_back(node["Z"].to<std::string>());
}

