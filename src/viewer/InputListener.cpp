#include "InputListener.h"

InputListener::InputListener(Ogre::SceneManager* scMgr, Ogre::RenderWindow *wnd, Ogre::Camera *camera)
{
    mWindow = wnd;
    mCamera = camera;
    mSceneMgr = scMgr;
    
    startOIS();
    
    mContinue = true;
    
    
    mYaw = 0;
    mPitch = 0;
    mZoom = mCamera->getPosition().z;
    
	mMouseLPressed = false;
    mMouseRPressed = false;
}

InputListener::~InputListener()
{
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
}

bool InputListener::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	
    if(mWindow->isClosed())
        mContinue = false;
        
    if(mMouse)
        mMouse->capture();
    if(mKeyboard)
        mKeyboard->capture();
    
    mCamera->setPosition(Ogre::Vector3(0, 0, mZoom));
    mSceneMgr->getRootSceneNode()->yaw(Ogre::Radian(mYaw));
    mSceneMgr->getRootSceneNode()->pitch(Ogre::Radian(mPitch));
    mYaw = 0;
    mPitch = 0;

	return mContinue;
}

void InputListener::windowResized(Ogre::RenderWindow* wnd)
{
    unsigned int width, height, depth;
    int left, top;
    wnd->getMetrics(width, height, depth, left, top);
 
    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

void InputListener::windowClosed(Ogre::RenderWindow* wnd)
{
    if( wnd == mWindow )
    {
        if( mInputManager )
        {
            mInputManager->destroyInputObject( mMouse );
            mInputManager->destroyInputObject( mKeyboard );
 
            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}

void InputListener::startOIS() 
{
    Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");

    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;
 
    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
 
    mInputManager = OIS::InputManager::createInputSystem( pl );
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));
    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));

    windowResized(mWindow);
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
    
    //Buffer
    mMouse->setEventCallback(this);
	mKeyboard->setEventCallback(this);
}

bool InputListener::mouseMoved(const OIS::MouseEvent &e)
{
	if (mMouseLPressed)
	{
		mPitch = e.state.Y.rel*0.01;
		mYaw = e.state.X.rel*0.01;
	}
	else if(mMouseRPressed)
	{
		mZoom += e.state.Y.rel*0.1;
	}
	return mContinue;
}

bool InputListener::mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
	switch(id)
	{
		case OIS::MB_Left:
			mMouseLPressed = true;
			return mContinue;
			break;
		
		case OIS::MB_Right:
			mMouseRPressed = true;
			return mContinue;
			break;
	}
	return mContinue; 
}

bool InputListener::mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
	switch(id)
	{
		case OIS::MB_Left:
			mMouseLPressed = false;
			return mContinue;
			break;
		
		case OIS::MB_Right:
			mMouseRPressed = false;
			return mContinue;
			break;
	}
	return mContinue;
}
 
bool InputListener::keyPressed(const OIS::KeyEvent &e)
{
	switch(e.key)
    {
    case OIS::KC_ESCAPE:
        mContinue = false;
        break;
    
    default:
		break;
    }

    return mContinue;
}

bool InputListener::keyReleased(const OIS::KeyEvent &e)
{
	return true;
}


