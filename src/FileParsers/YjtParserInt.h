#ifndef YJTPARSERINT_H
#define YJTPARSERINT_H

#include <string>
#include <iomanip>
#include "YjtParserClasses/VectorsTable_c.h"

class YjtParserInt
{
	public:
		YjtParserInt(std::string fileName);
		YjtParserInt(std::string skel0, std::string skel1, std::string skel2);
		~YjtParserInt();
		std::vector<std::vector<std::vector<double> > >& getFirstFrame();
		std::vector<std::vector<std::vector<double> > >& getCurrentFrame();
		std::vector<std::vector<std::vector<double> > >& getNextFrame();
		std::vector<std::string>& getJointNames();
		
		int getNbFrames();
		
	protected:
		void loadYaml(std::stringstream &yamlHeader, MixedFileManager_c* fileManager);
		
		VectorsTable_c* mRegData;
		MixedFileManager_c *mFileManager0, *mFileManager1, *mFileManager2;
		int mNbObs;
		int mNbPoses;
		bool mUseConfidence;
		std::vector<std::vector<std::string> > mJointNames;
		std::vector<std::vector<std::vector<std::vector<double> > > > mVideoSequence;
		std::vector<std::vector<std::vector<double> > > mCurrentFrame;
		int mFrameInd;
};

#endif
