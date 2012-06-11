#ifndef LINE3D_H
#define LINE3D_H

/*!
 * \file Line3D.h
 */

#include <Ogre.h>
#include <string>
#include <iostream>

/*!
 * \class Line3D
 * \brief Class to draw a 3D line
 * 
 * This class is herited from a ManualObject.
 * 
 */

class Line3D : public Ogre::ManualObject
{
	public:
	
		/*!
		 * \fn Line3D(Ogre::String name)
		 * \brief Constructor of Line3D
		 * \param name Unique name of the line
		 */
		Line3D(Ogre::String name);
		
		/*!
		 * \fn ~Line3D()
		 * \brief Destructor of Line3D
		 */
		~Line3D();
		
		/*!
		 * \fn void setLine(const Ogre::Vector3& start, const Ogre::Vector3& stop, std::vector<int> color)
		 * \brief Draw the line between the two 3D points
		 * \param start First 3D point.
		 * \param stop Second 3D point.
		 * \param color Color of the line.
		 */
		void setLine(const Ogre::Vector3& start, const Ogre::Vector3& stop, std::vector<float> color);
		
	private:
};

#endif
