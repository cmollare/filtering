#include "Line3D.h"

Line3D::Line3D(Ogre::String name) : Ogre::ManualObject(name)
{
	
}

Line3D::~Line3D()
{
}

void Line3D::setLine(const Ogre::Vector3& start, const Ogre::Vector3& stop, std::vector<float> color)
{
	clear();
	begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_LIST);

		position(start[0], start[1], start[2]);
		//colour(0.0f, 0.0f, 1.0f, 0.0f);
		colour(color[0], color[1], color[2], color[3]);

		position(stop[0], stop[1], stop[2]);
		colour(color[0], color[1], color[2], color[3]);

	end();
}
