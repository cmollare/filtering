#include "YjtParserInt.h"

YjtParserInt::YjtParserInt(std::string fileName)
{
	mRegData = new VectorsTable_c();
	mRegData->Import(fileName, false);
	
	mNbObs=1;
	mFrameInd=0;
	
	mFileManager0 = NULL;
	mFileManager1 = NULL;
	mFileManager2 = NULL;
}

YjtParserInt::YjtParserInt(std::string skel0, std::string skel1, std::string skel2)
{
	mVideoSequence.reserve(3);
	mFrameInd=0;
	
	mRegData = new VectorsTable_c();
	mFileManager0 = new MixedFileManager_c();
	mFileManager1 = new MixedFileManager_c();
	mFileManager2 = new MixedFileManager_c();
	std::stringstream YamlHeader0(mFileManager0->Parse(skel0));
	std::stringstream YamlHeader1(mFileManager1->Parse(skel1));
	std::stringstream YamlHeader2(mFileManager2->Parse(skel2));
	
	mNbObs=3;
	
	this->loadYaml(YamlHeader0, mFileManager0);
	this->loadYaml(YamlHeader1, mFileManager1);
	this->loadYaml(YamlHeader2, mFileManager2);
	
	std::cout << mVideoSequence.size() << std::endl;
	std::cout << mVideoSequence[0].size() << std::endl;
	std::cout << mVideoSequence[0][0].size() << std::endl;
	std::cout << mVideoSequence[0][0][0].size() << std::endl;
}

YjtParserInt::~YjtParserInt()
{
	delete mRegData;
	
	if(mFileManager0) delete mFileManager0;
	if(mFileManager1) delete mFileManager1;
	if(mFileManager2) delete mFileManager2;
}

std::vector<std::vector<std::vector<double> > >& YjtParserInt::getFirstFrame()
{
	mCurrentFrame.clear();
	mFrameInd=0;
	for (int i=0 ; i<3 ; i++)
	{
		mCurrentFrame.push_back(mVideoSequence[i][mFrameInd]);
	}
	mFrameInd++;
	
	return mCurrentFrame;
}

std::vector<std::vector<std::vector<double> > >& YjtParserInt::getCurrentFrame()
{
	return mCurrentFrame;
}

std::vector<std::vector<std::vector<double> > >& YjtParserInt::getNextFrame()
{
	for (int i=0 ; i<3 ; i++)
	{
		mCurrentFrame[i] = mVideoSequence[i][mFrameInd];
	}
	mFrameInd;
}

std::vector<std::string>& YjtParserInt::getJointNames()
{
	return mJointNames[0];
}

void YjtParserInt::loadYaml(std::stringstream &yamlHeader, MixedFileManager_c* fileManager)
{
	if(yamlHeader.str().empty())
	{
		std::cout << "BodyJointsPosSeq_c::load() Couldn't load File: " << std::endl;
	}
	
	YAML::Parser parser;
	parser.Load(yamlHeader);
	
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	int nbPoses;
	doc["NbPoses"] >> nbPoses;
	
	bool useConfidence;
	if(doc.FindValue("useConfidence"))//in the case where the motion doesn't start at Frame 0
	{
		doc["useConfidence"] >> useConfidence;
	}
	else
	{
		useConfidence = false;
	}
	
	std::vector<std::string> BJointsNames;
	doc["BJointsNames"] >> BJointsNames;
	/*for (int i=0 ; i<BJointsNames.size() ; i++)
	{
		std::cout << BJointsNames[i] << std::endl;
	}*/
	mJointNames.push_back(BJointsNames);
	
	//AJOUTER les indices éventuellement
	
	int nbJoints = (int)BJointsNames.size();
	VectorsTable_c vectTab;
	
	int nbFloatsPerJoint = 3;
	if(useConfidence)
	{
		nbFloatsPerJoint = 4;
	}
	vectTab.resize(nbJoints*nbFloatsPerJoint,nbPoses);
	
	if(!vectTab.Data.empty())//So that we can load even a header without data
	{
		fileManager->LoadData((char*)&vectTab.Data[0],vectTab.Data.size()*4);
	}
	else
	{
		std::cout << "Warning ! No animations to load..." << std::endl;
	}
	
	//int nbJoints = vectTab.VectSize / nbFloatsPerJoint;
	std::cout << "test : " << vectTab.size() << " nbPoses : " <<  nbPoses << std::endl;
	
	std::vector<std::vector<std::vector<double> > > obsSequence;
	obsSequence.reserve(vectTab.size());
	for(int i=0;i<vectTab.size();i++)
	{
		std::vector<std::vector<double> > frame;
		std::vector<float> temp = vectTab.at(i);
		for(int j=0;j<BJointsNames.size();j++)
		{
			std::vector<double> joint(4,1); //[1 1 1 1]
			joint[0] = temp[nbFloatsPerJoint*j + 0];
			joint[1] = temp[nbFloatsPerJoint*j + 1];
			joint[2] = temp[nbFloatsPerJoint*j + 2];
			if(useConfidence)
			{
				joint[3] = temp[nbFloatsPerJoint*j + 3];
			}
			frame.push_back(joint);
			//std::cout << joint[0] << " " << joint[1] << " " << joint[2] << " " << joint[3] << std::endl;
		}
		obsSequence.push_back(frame);
	}
	mVideoSequence.push_back(obsSequence);
	
	//FromVectTab(vectTab);
	//remplacer si besoin
}
