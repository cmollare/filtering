#ifndef SKELETON_H
#define SKELETON_H

#include <Ogre.h>
#include "FileParser.h"

using namespace Ogre;

class Avatar
{
	public:
		Avatar(Ogre::SceneManager* scMgr);
		~Avatar();
		
		void createSphere(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16);
		void moveNextFrame();
		vector<double> &getCurrentDistances();
		
	private:
		Ogre::SceneManager* mSceneMgr;
		Ogre::SceneNode* mSkelSceneNode;
		FileParser *mFileParser;
		int mScale;
		vector<double> mDistances;
};

#endif
