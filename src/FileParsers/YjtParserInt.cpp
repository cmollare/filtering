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
	
	mSNbPoses=0;
	mSavedData=NULL;
}

YjtParserInt::YjtParserInt(std::string skel0, std::string skel1, std::string skel2)
{
	mSNbPoses=0;
	mSavedData=NULL;
	
	
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
	
}

YjtParserInt::~YjtParserInt()
{
	delete mRegData;
	
	if(mFileManager0) delete mFileManager0;
	if(mFileManager1) delete mFileManager1;
	if(mFileManager2) delete mFileManager2;
	
	if(mSavedData) delete mSavedData;
}

std::vector<std::vector<std::vector<double> > >& YjtParserInt::getFirstFrame(int firstIndex)
{
	mCurrentFrame.clear();
	mFirstInd=firstIndex;
	mFrameInd=mFirstInd;
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
	mFrameInd++;
	
	return mCurrentFrame;
}

std::vector<std::string>& YjtParserInt::getJointNames()
{
	return mJointNames[0];
}

int YjtParserInt::getNbFrames()
{
	return mLastInd-mFirstInd;
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
	mNbPoses = nbPoses;
	
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
	
	if(doc.FindValue("FirstIndex"))//in the case where the motion doesn't start at Frame 0
	{
		doc["FirstIndex"] >> mFirstInd;
	}
	else
	{
		mFirstInd = 0;
	}
	mLastInd = mFirstInd + mNbPoses - 1;
	
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

void YjtParserInt::saveFile(std::string filename, std::vector<std::string> jointNames)
{
	YAML::Emitter Yout;
	Yout << YAML::BeginDoc;
	Yout << YAML::BeginMap;
	Yout << YAML::Key << "NbPoses";
	Yout << YAML::Value	<< mSNbPoses;//Vector<std::string>
	Yout << YAML::Key << "FirstIndex";
	Yout << YAML::Value	<< mFirstInd;
	Yout << YAML::Key << "NbJoints";
	Yout << YAML::Value	<< jointNames.size();
	Yout << YAML::Key << "useConfidence";
	bool useConfidence = false;
	Yout << YAML::Value	<< useConfidence;
	
	Yout << YAML::Key << "BJointsNames";
	Yout << YAML::Value	<< jointNames;//Vector<std::string>
	
	std::vector<cv::Scalar_<unsigned char> > BPartsColors(jointNames.size(), cv::Scalar(150, 150, 150));
	

	Yout << YAML::Key << "BPartsColors";
	Yout << YAML::Value	<< BPartsColors;//Vector<cv::Scalar_<unsigned char> >*/
	
	Yout << YAML::EndDoc;
	
	MixedFileManager_c FManager;
	FManager.Save(filename,(char*)&mSavedData->Data[0],mSavedData->Data.size()*4,  Yout.c_str()  );
}

void YjtParserInt::save(std::vector<std::vector<double> > currentFrame)
{
	if (mSNbPoses==0)
	{
		mSavedData = new VectorsTable_c();
		mSavedData->Reset((int)currentFrame.size()*(int)currentFrame[0].size());
		std::vector<float> nullVec((int)currentFrame.size()*(int)currentFrame[0].size(), 0);
		for(int i=0 ; i<mFirstInd ; i++)
		{
			mSavedData->push(nullVec);
		}
			
	}
	
	std::vector<float> tempo;
	for (int i=0 ; i<currentFrame.size() ; i++)
	{
		for (int j=0 ; j<currentFrame[i].size() ; j++)
		{
			tempo.push_back((float)currentFrame[i][i]);
		}
	}
	mSavedData->push(tempo);
	
	mSNbPoses++;
	
}
