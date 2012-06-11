#ifndef APPDEMARRAGE_Hc
#define APPDEMARRAGE_H

#include <Ogre.h>
#include "InputListener.h"
#include "Avatar.h"

class AppDemarrage
{
public:
    AppDemarrage();
    ~AppDemarrage();
    
	void createFrameListener();

    bool start();

private:
    Ogre::Root *mRoot;
    Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	InputListener *mInputListener;
	Avatar *mAvatar;

};

#endif
