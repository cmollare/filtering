#ifndef FILEPARSER_H
#define FILEPARSER_H

/*!
 * \file FileParser.h
 * \brief Class to parse Kinect files.
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

/*!
 * \class FileParser
 * \brief class for Kinect files parsing
 */
class FileParser
{
	public:
		
		/*!
		 * \fn FileParser(std::string path, std::string fileBeg, int number)
		 * \brief Constructor of the FileParser class.
		 * \param path Path to kinect files' folder.
		 * \param fileBeg Beginning of the file name (before the index).
		 * \param number Number of file to parse.
		 */
		FileParser(std::string path, std::string fileBeg, int number);
		
		/*!
		 * \fn void readFile(int number)
		 * \brief Parse Kinect's files.
		 * \param number Number of files to parse.
		 */
		void readFile(int number);
		
		/*!
		 * \fn vector<vector<double> > &getFirstFrame()
		 * \brief get first set of observations.
		 * \return vector of 3D points.
		 */
		vector<vector<double> > &getFirstFrame();
		
		/*!
		 * \fn vector<vector<double> > &getNextFrame()
		 * \brief get next set of observations.
		 * \return vector of 3D points.
		 */
		vector<vector<double> > &getNextFrame();
		
		/*!
		 * \fn vector<vector<double> > &getCurrentFrame()
		 * \brief get current set of observations.
		 * \return vector of 3D points.
		 */
		vector<vector<double> > &getCurrentFrame();
		vector<std::string> getJointNames();
		
	private:
		std::string mPath; /*!< Path to files' folder */
		std::string mFileBeg; /*!< Beginning of files' names */
		int mMaxFileNumber; /*!< Number of files */
		vector<vector<vector<double> > > mVideoSequence; /*!< Video sequence : all observations */
		vector<std::string> mJointNames; /*!< Name of observations */
		int mNextFrame; /*!< Index of next frame */
};

#endif
