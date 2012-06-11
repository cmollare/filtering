#ifndef INPUTLISTENER_H
#define INPUTLISTENER_H

#include <Ogre.h>
#include <OIS/OIS.h>
#include "Avatar.h"

class InputListener: public Ogre::FrameListener, public Ogre::WindowEventListener
{
public:
	InputListener(Ogre::RenderWindow *wnd, Ogre::Camera *camera, Avatar* avatar);
	~InputListener();
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	
	//OIS
	void startOIS();
	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);

private:
    Ogre::RenderWindow* mWindow;
    Ogre::Camera*       mCamera;

    OIS::InputManager*  mInputManager;
    OIS::Mouse*         mMouse;
    OIS::Keyboard*      mKeyboard;
    
    Avatar *mAvatar;
    float mCounterAnimAvatar;
};

#endif
