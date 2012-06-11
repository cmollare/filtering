#include "AppDemarrage.h"

AppDemarrage::AppDemarrage() : mRoot(0)
{
}

AppDemarrage::~AppDemarrage()
{
	delete mInputListener;
	delete mAvatar;
    delete mRoot;
}

bool AppDemarrage::start()
{
	mRoot = new Ogre::Root("../config/plugins.cfg", "../config/ogre.cfg", "../config/Ogre.log");
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

	mWindow = mRoot->initialise(true, "Une application de zér0");
	
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	mSceneMgr = mRoot->createSceneManager("DefaultSceneManager", "Mon Scene Manager");
	mSceneMgr->setAmbientLight(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

	mCamera = mSceneMgr->createCamera("PlayerCam");
	mCamera->setPosition(Ogre::Vector3(0,0,100));
	mCamera->lookAt(Ogre::Vector3(0,0,0));
	mCamera->setNearClipDistance(5);

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
	
	mAvatar = new Avatar(mSceneMgr);
	
	createFrameListener();
	
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

void AppDemarrage::createFrameListener()
{
    mInputListener = new InputListener(mWindow, mCamera, mAvatar);
    mRoot->addFrameListener(mInputListener);
}

