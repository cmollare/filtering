#include "ResultParser.h"

using namespace std;

ResultParser::ResultParser(std::string path)
{
	mPath = path;
}

ResultParser::~ResultParser()
{
}

void ResultParser::saveObs(std::string name, std::vector<double> position)
{
	std::string fileName = mPath + name + ".txt";
	std::fstream file(fileName.c_str(), std::fstream::out | std::fstream::app);

	for (int i=0 ; i<position.size() ; i++)
		file << position[i] << " ";
	file << std::endl;
	
	file.close();
}

void ResultParser::saveJoint(std::string name, Eigen::Vector3d& position, Eigen::Translation3d& offset, Eigen::Quaterniond& orientation)
{
	std::string fileName = mPath + name + ".txt";
	std::fstream file(fileName.c_str(), std::fstream::out | std::fstream::app);
	
	file << position[0] << " " << position[1] << " " << position[2] << " ";
	file << offset.x() << " " << offset.y() << " " << offset.z() << " ";
	file << orientation.w() << " " << orientation.x() << " " << orientation.y() << " " << orientation.z() << std::endl;
	
	file.close();
}
