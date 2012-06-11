#ifndef RESULTPARSER_H
#define RESULTPARSER_H

/*!
 * \file ResultParser.h
 * \brief Parser to save some results.
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

/*!
 * \class ResultParser
 * \brief Class to save some results.
 */
class ResultParser
{
	public:
		ResultParser(std::string path);
		~ResultParser();
		void saveObs(std::string name, std::vector<double> position);
		void saveJoint(std::string name, Eigen::Vector3d& position, Eigen::Translation3d& offset, Eigen::Quaterniond& orientation);
		
	private:
		std::string mPath;
};

#endif
