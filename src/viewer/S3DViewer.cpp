#include "S3DViewer.h"

S3DViewer::S3DViewer() : mRoot(0)
{
	mDisplayJoint = true;
	mDisplayBone = true;
	mDisplayAxis = true;
}

S3DViewer::~S3DViewer()
{
	//ajouter les destructions des Line3D !!!
	delete mInputListener;
    delete mRoot;
    delete mLogMgr;
}

bool S3DViewer::init()
{
	mLogMgr = new Ogre::LogManager();
	Ogre::LogManager::getSingleton().createLog("../config/Ogre.log", true, false, false);
	mRoot = new Ogre::Root("../config/plugins.cfg", "../config/ogre.cfg");
	Ogre::ConfigFile configFile;
	configFile.load("../config/resources.cfg");
	Ogre::ConfigFile::SectionIterator seci = configFile.getSectionIterator();
	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
				archName, typeName, secName);
		}
	}
	
	if(!(mRoot->restoreConfig() || mRoot->showConfigDialog()))
	{
		return false;
	}

	mWindow = mRoot->initialise(true, "Viewer");
	
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	mSceneMgr = mRoot->createSceneManager("DefaultSceneManager", "Scene Manager");
	mSceneMgr->setAmbientLight(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

	mCamera = mSceneMgr->createCamera("Default Camera");
	mCamera->setPosition(Ogre::Vector3(0,0,-5));
	mCamera->lookAt(Ogre::Vector3(0,0,0));
	mCamera->setNearClipDistance(0.1);

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
	
	//Load models
	defineMaterials();
	SceneNode *filteringNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Filtering");
	filteringNode->createChildSceneNode("Particles");
	filteringNode->createChildSceneNode("Observations");
	mSceneMgr->getRootSceneNode()->createChildSceneNode("Debug")->attachObject(createAxis("Axis_ref_debug",5));
	
	//mSceneMgr->getRootSceneNode()->createChildSceneNode("lol")->attachObject(createAxis("Axis"));
	//mSceneMgr->getSceneNode("lol")->rotate(Vector3(1,1,1), Radian(0.5));
	
	createFrameListener();
	
	return true;
}
	
bool S3DViewer::start()
{
	
	while(true)
	{
		Ogre::WindowEventUtilities::messagePump();
	 
		if(mWindow->isClosed())
			return false;
	 
		if(!mRoot->renderOneFrame())
			return false;
	}

    return true;
}

bool S3DViewer::isRendering()
{
	Ogre::WindowEventUtilities::messagePump();
	 
	if(mWindow->isClosed())
		return false;
	 
	if(!mRoot->renderOneFrame())
		return false;
		
	return true;
}

void S3DViewer::createFrameListener()
{
    mInputListener = new InputListener(mSceneMgr, mWindow, mCamera);
    mRoot->addFrameListener(mInputListener);
}

void S3DViewer::displaySampling(Eigen::Quaternionf quat)
{
	std::ostringstream oss;
	Ogre::Vector3 vec(quat.x(), quat.y(), quat.z());
	vec.normalise();
	vec=vec*5;
	SceneNode *node = mSceneMgr->getSceneNode("Particles")->createChildSceneNode(vec);
	oss << "Axis_particule_" << node->getName() << endl;
	node->attachObject(createAxis(oss.str(),1));
}

void S3DViewer::setOptions(bool displayJoint, bool displayAxis, bool displayBone)
{
	mDisplayJoint = displayJoint;
	mDisplayBone = displayBone;
	mDisplayAxis = displayAxis;
}

void S3DViewer::initModels(std::vector<S3DModel*>& models)
{
	mLine3DToSNName.clear();
	mModelSNNames.clear();
	int i=0;
	if (models.size()>0)
	{
		for(int i=0 ; i<models.size() ; i++)
		{
			std::map<std::string, std::string> snNames;
			Joint* jt = models[i]->getRootJoint();
			ostringstream oss, ossMod;
			ossMod << "Model_" << i;
			oss << jt->getName() << "_" << i;
			
			//Orientation of Joint
			Eigen::Quaterniond vQuat = *(jt->getOrientation());
			Ogre::Quaternion quat(vQuat.w(), vQuat.x(), vQuat.y(), vQuat.z());

			//offset
			Eigen::Translation3d vOff = *(jt->getOffset());
			Vector3 offset(vOff.x(), vOff.y(), vOff.z());
			
			//Creation of sceneNode with orientation and offset
			SceneNode *modelNode = mSceneMgr->getSceneNode("Particles")->createChildSceneNode(ossMod.str());
			
			SceneNode *node = modelNode->createChildSceneNode(oss.str());
			node->setPosition(offset);
			node->setOrientation(quat);
			node->_updateBounds();

			//cout << oss.str() << " " << node->_getDerivedPosition() << endl;
			//cout << oss.str() << " " << node->_getDerivedOrientation() << endl;

			snNames[jt->getName()]=oss.str();
			
			if (mDisplayAxis)//option to display axis
			{
				ostringstream oss2;
				oss2 << "Axis_" << oss.str();
				node->attachObject(createAxis(oss2.str()));
			}
			if (jt->hasChildren())
			{
				initModels(jt->getChildren(), node, i, snNames);//Recursivity
			}
			mModelSNNames.push_back(snNames);
			
			modelNode->setVisible(models[i]->isVisible());
		}
	}
	
	/*cout << "pouet" << endl;
	Eigen::Vector3d vec = models[0]->getRootJoint()->getJointFromName("HandRight")->getXYZVect();
	cout << vec << endl;
	Ogre::SceneNode *node = mSceneMgr->getSceneNode("Debug")->createChildSceneNode("lol", Ogre::Vector3(vec[0], vec[1], vec[2]));
	node->attachObject(createAxis("Axis_lol",0.1));
	cout << "pouetfin" << endl;//*/
	
	/*for(int i=0 ; i<mModelSNNames.size() ; i++)
	{
		std::map<std::string, std::string>::iterator it;
		for (it = mModelSNNames[i].begin() ; it != mModelSNNames[i].end() ; it++)
		{
			cout << (*it).first << " => " << (*it).second << endl;
		}
	}*/

}

void S3DViewer::initObservations(std::vector<std::string> jtNames, std::vector<std::vector<double> > frame)
{
	mObsNameVec = jtNames;
	mObsCurrentFrame = frame;
	mObservationSNNames.clear();
	
	Ogre::SceneNode *obsNode = mSceneMgr->getSceneNode("Observations");
	
	for (int i=0 ; i<mObsNameVec.size() ; i++)
	{
		ostringstream oss;
		mObsMap[mObsNameVec[i]]=i;
		oss << "obs_" << mObsNameVec[i];
		Ogre::SceneNode *tempoNode = obsNode->createChildSceneNode(oss.str(), Ogre::Vector3(mObsCurrentFrame[i][1], mObsCurrentFrame[i][2], mObsCurrentFrame[i][3]));
		mObservationSNNames.push_back(oss.str());
		oss.clear();
		oss << "axisObs_" << mObsNameVec[i];
		tempoNode->attachObject(createAxis(oss.str()));
	}
}

void S3DViewer::update(std::vector<S3DModel*>& models, std::vector<std::vector<double> >& frame)
{
	for (int i=0 ; i < models.size() ; i++)
	{
		ostringstream oss;
		oss << "Model_" << i;
		Ogre::SceneNode *modelNode = mSceneMgr->getSceneNode(oss.str());
		modelNode->setVisible(models[i]->isVisible());
		
		if (models[i]->isVisible())
		{
			std::vector<std::string> nameVec = models[i]->getNameVec();
			for (int j=0 ; j<nameVec.size() ; j++)
			{
				Eigen::Quaterniond vQuat = *(models[i]->getJoint(nameVec[j])->getOrientation());
				Ogre::Quaternion quat((double)vQuat.w(), (double)vQuat.x(), (double)vQuat.y(), (double)vQuat.z());
				Eigen::Translation3d vOff = *(models[i]->getJoint(nameVec[j])->getOffset());
				
				Vector3 offset(vOff.x(), vOff.y(), vOff.z());

				Ogre::SceneNode *node = mSceneMgr->getSceneNode(mModelSNNames[i][nameVec[j]]);
				node->setOrientation(quat);
				node->setPosition(offset);
			}
		}
	}
	this->updateLine3D();
	this->updateObs(frame);
}

void S3DViewer::initModels(std::vector<Joint*>& jts, SceneNode *node, int modelNum, std::map<std::string, std::string>& snNames)
{
	if (jts.size()>0)
	{
		for (int i=0 ; i<jts.size() ; i++)
		{
			ostringstream oss;
			oss << jts[i]->getName() << "_" << modelNum;
			
			Eigen::Quaterniond vQuat = *(jts[i]->getOrientation());
			//Eigen::Matrix3d temp = jts[i]->getGlobalTransformationMatrix().rotation();
			//Eigen::Quaterniond vQuat(temp);//*/
			//cout << vQuat.w() << " " << vQuat.x() << " " << vQuat.y() << " " << vQuat.z() << endl;
			Ogre::Quaternion quat(vQuat.w(), vQuat.x(), vQuat.y(), vQuat.z());
			
			//offset
			Eigen::Translation3d vOff = *(jts[i]->getOffset());
			//Eigen::Translation3d vOff(jts[i]->getGlobalTransformationMatrix().translation());
			Vector3 offset(vOff.x(), vOff.y(), vOff.z());
			
			//Creation of node with orientation and offset
			SceneNode *childNode = node->createChildSceneNode(oss.str(), offset, quat);
			//au choix
			/*SceneNode *childNode = node->createChildSceneNode(oss.str());
			childNode->_setDerivedOrientation(quat);
			childNode->_setDerivedPosition(offset);//*/
			
			//cout << oss.str() << " " << childNode->_getDerivedPosition() << endl;
			//cout << oss.str() << " " << childNode->_getDerivedOrientation() << endl;
			snNames[jts[i]->getName()]=oss.str();
			
			if (mDisplayAxis)//option to display axis
			{
				ostringstream oss2;
				oss2 << "Axis_" << oss.str();
				childNode->attachObject(createAxis(oss2.str()));
			}
			if (mDisplayBone)//option to display bones
			{
				ostringstream oss2;
				oss2 << "Line_" << oss.str();
				Line3D *line = new Line3D(oss2.str());
				mLine3DToSNName[line] = oss.str();//mapping
				mLine3DToJoint[line] = jts[i];
				line->setLine(childNode->getPosition(), Vector3(0,0,0), jts[i]->getColor());
				node->attachObject(line);
			}
			if (jts[i]->hasChildren())
			{
				initModels(jts[i]->getChildren(), childNode, modelNum, snNames);//Recursivity
			}
		}
	}
}

void S3DViewer::defineMaterials()
{	
	ResourceGroupManager::getSingleton().createResourceGroup("axis");
	MaterialPtr myManualObjectMaterial = MaterialManager::getSingleton().create("Red","axis", true);
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(1,0,0,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(1,0,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1,0,0);
	
	myManualObjectMaterial = MaterialManager::getSingleton().create("Green","axis"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,1,0,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,1,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,1,0); 
	
	myManualObjectMaterial = MaterialManager::getSingleton().create("Blue","axis"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);
	
	ResourceGroupManager::getSingleton().createResourceGroup("bone");
	myManualObjectMaterial = MaterialManager::getSingleton().create("Purple","bone"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(1,0,1,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(1,0,1); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1,0,1);
	
	myManualObjectMaterial = MaterialManager::getSingleton().getByName("BaseWhiteNoLighting");
	myManualObjectMaterial->setTransparencyCastsShadows(true);
	myManualObjectMaterial->setSceneBlending(SBF_SOURCE_ALPHA, SBF_ONE);
}

ManualObject* S3DViewer::createAxis(const std::string& strName, float scale)
{
	ManualObject* manual = mSceneMgr->createManualObject(strName);
 
	// specify the material (by name) and rendering type
	manual->begin("Red", RenderOperation::OT_LINE_LIST);
	manual->position(0, 0, 0);
	manual->position(scale, 0, 0);
	manual->end();
	
	manual->begin("Green", RenderOperation::OT_LINE_LIST);
	manual->position(0, 0, 0);
	manual->position(0, scale, 0);
	manual->end();
	
	manual->begin("Blue", RenderOperation::OT_LINE_LIST);
	manual->position(0, 0, 0);
	manual->position(0, 0, scale);
	manual->end();
	 
	return manual;
}

void S3DViewer::updateLine3D()
{
	std::map<Line3D*, std::string>::iterator it;
	for (it = mLine3DToSNName.begin() ; it != mLine3DToSNName.end() ; it++)
	{
		Line3D* line = (*it).first;
		Ogre::SceneNode* node = line->getParentSceneNode();
		Ogre::SceneNode* childNode = mSceneMgr->getSceneNode((*it).second);
		line->setLine(childNode->getPosition(), Ogre::Vector3(0, 0, 0), mLine3DToJoint[line]->getColor());
	}
}

void S3DViewer::updateObs(std::vector<std::vector<double> >& frame)
{
	mObsCurrentFrame = frame;
	
	Ogre::SceneNode *obsNode = mSceneMgr->getSceneNode("Observations");
	
	for (int i=0 ; i<mObservationSNNames.size() ; i++)
	{
		Ogre::SceneNode *obsNode = mSceneMgr->getSceneNode(mObservationSNNames[i]);
		obsNode->setPosition(Ogre::Vector3(mObsCurrentFrame[i][1], mObsCurrentFrame[i][2], mObsCurrentFrame[i][3]));
	}
}

