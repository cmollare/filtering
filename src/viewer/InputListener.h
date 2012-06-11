#ifndef INPUTLISTENER_H
#define INPUTLISTENER_H

/*!
 * \file InputListener.h
 * 
 */

#include <Ogre.h>
#include <OIS/OIS.h>

#include <iostream>
using namespace std;

/*!
 * \class InputListener
 * \brief class to handle Mouse and Keyboard events
 * 
 * This class handle mouse and keyboard events, and manage the trackBall camera.
 * 
 */

class InputListener: public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
	/*!
	 * \brief InputListener constructor
	 * \param scMgr Pointer to the Ogre::SceneManager.
	 * \param wnd Pointer to the Ogre::RenderWindow.
	 * \param camera Pointer to the Ogre::Camera.
	 */
	InputListener(Ogre::SceneManager* scMgr, Ogre::RenderWindow *wnd, Ogre::Camera *camera);
	
	/*!
	 * \brief InputListener destructor
	 */
	~InputListener();
	
	/*!
	 * \brief Function called each frame
	 * \param evt Ogre::FrameEvent
	 */
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	
	//OIS
	/*!
	 * \brief Setup OIS environment for input handle
	 */
	void startOIS();
	
	/*!
	 * \brief Called if the window is resized
	 * \param rw Pointer on Ogre::RenderWindow.
	 */
	virtual void windowResized(Ogre::RenderWindow* rw);
	
	/*!
	 * \brief Called if the window is closed (with the cross button)
	 * \param rw Pointer on Ogre::RenderWindow.
	 */
	virtual void windowClosed(Ogre::RenderWindow* rw);

private:

	/*!
	 * \brief Called if the mouse move
	 * \param e Reference to OIS::MouseEvent.
	 */
	virtual bool mouseMoved(const OIS::MouseEvent &e);
	
	/*!
	 * \brief Called if the a mouse button is pressed
	 * \param e Reference to OIS::MouseEvent.
	 * \param id Id of the pressed button.
	 */
	virtual bool mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id);
	
	/*!
	 * \brief Called if the a mouse button is released
	 * \param e Reference to OIS::MouseEvent.
	 * \param id Id of the released button.
	 */
	virtual bool mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id);
	
	/*!
	 * \brief Called if the a heyboard button is pressed
	 * \param e Reference to OIS::KeyEvent.
	 */
	virtual bool keyPressed(const OIS::KeyEvent &e);
	
	/*!
	 * \brief Called if the a heyboard button is released
	 * \param e Reference to OIS::KeyEvent.
	 */
	virtual bool keyReleased(const OIS::KeyEvent &e);

	Ogre::SceneManager* mSceneMgr; /*!< pointer to Ogre::SceneManager */
    Ogre::RenderWindow* mWindow; /*!< pointer to Ogre::RenderWindow */
    Ogre::Camera*       mCamera; /*!< pointer to Ogre::Camera */

    OIS::InputManager*  mInputManager; /*!< pointer to OIS::InputManager */
    OIS::Mouse*         mMouse; /*!< pointer to OIS::Mouse for mouse event handling */
    OIS::Keyboard*      mKeyboard; /*!< pointer to OIS::Keyboard for keyboard event handling */
    
    bool mContinue; /*!< if false, stop the infinite loop */
    
    //Params for trackBallCamera
    float mYaw; /*!< for camera rotation around the Y axis */
    float mPitch; /*!< for camera rotation around the X axis */
    Ogre::Real mZoom; /*!< for camera zoom */
    bool mMouseLPressed; /*!< true if mouse left button is pressed */
    bool mMouseRPressed; /*!< true if mouse right button is pressed */
    
    //Avatar *mAvatar;
    //float mCounterAnimAvatar;
};

#endif
