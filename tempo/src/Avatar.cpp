#include "Avatar.h"

Avatar::Avatar(Ogre::SceneManager* scMgr)
{
	mFileParser = new FileParser("../skel2/", "skel_", 1150);
	mSceneMgr=scMgr;
	mSkelSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mScale=15;
	
	
	//Recuperation du premier squelette
	vector<vector<double> > skelVec = mFileParser->getFirstFrame();
	
	//Creation of sphere Mesh
	createSphere("joint", 0.5);
	
	//Spine
	Entity* tempEntity = mSceneMgr->createEntity("Spine", "joint");
	SceneNode* tempNode = mSkelSceneNode->createChildSceneNode("Spine", Vector3(mScale*skelVec[1][1], mScale*skelVec[1][2], mScale*skelVec[1][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//HipCenter
	tempEntity = mSceneMgr->createEntity("HipCenter", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("HipCenter", Vector3(mScale*skelVec[0][1], mScale*skelVec[0][2], mScale*skelVec[0][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//ShoulderCenter
	tempEntity = mSceneMgr->createEntity("ShoulderCenter", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("ShoulderCenter", Vector3(mScale*skelVec[2][1], mScale*skelVec[2][2], mScale*skelVec[2][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//Head
	tempEntity = mSceneMgr->createEntity("Head", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("Head", Vector3(mScale*skelVec[3][1], mScale*skelVec[3][2], mScale*skelVec[3][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//ShoulderLeft
	tempEntity = mSceneMgr->createEntity("ShoulderLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("ShoulderLeft", Vector3(mScale*skelVec[4][1], mScale*skelVec[4][2], mScale*skelVec[4][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//ElbowLeft
	tempEntity = mSceneMgr->createEntity("ElbowLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("ElbowLeft", Vector3(mScale*skelVec[5][1], mScale*skelVec[5][2], mScale*skelVec[5][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//WristLeft
	tempEntity = mSceneMgr->createEntity("WristLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("WristLeft", Vector3(mScale*skelVec[6][1], mScale*skelVec[6][2], mScale*skelVec[6][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//HandLeft
	tempEntity = mSceneMgr->createEntity("HandLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("HandLeft", Vector3(mScale*skelVec[7][1], mScale*skelVec[7][2], mScale*skelVec[7][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//ShoulderRight
	tempEntity = mSceneMgr->createEntity("ShoulderRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("ShoulderRight", Vector3(mScale*skelVec[8][1], mScale*skelVec[8][2], mScale*skelVec[8][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//ElbowRight
	tempEntity = mSceneMgr->createEntity("ElbowRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("ElbowRight", Vector3(mScale*skelVec[9][1], mScale*skelVec[9][2], mScale*skelVec[9][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//WristRight
	tempEntity = mSceneMgr->createEntity("WristRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("WristRight", Vector3(mScale*skelVec[10][1], mScale*skelVec[10][2], mScale*skelVec[10][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//HandRight
	tempEntity = mSceneMgr->createEntity("HandRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("HandRight", Vector3(mScale*skelVec[11][1], mScale*skelVec[11][2], mScale*skelVec[11][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//HipLeft
	tempEntity = mSceneMgr->createEntity("HipLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("HipLeft", Vector3(mScale*skelVec[12][1], mScale*skelVec[12][2], mScale*skelVec[12][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//KneeLeft
	tempEntity = mSceneMgr->createEntity("KneeLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("KneeLeft", Vector3(mScale*skelVec[13][1], mScale*skelVec[13][2], mScale*skelVec[13][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//AnkleLeft
	tempEntity = mSceneMgr->createEntity("AnkleLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("AnkleLeft", Vector3(mScale*skelVec[14][1], mScale*skelVec[14][2], mScale*skelVec[14][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//FootLeft
	tempEntity = mSceneMgr->createEntity("FootLeft", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("FootLeft", Vector3(mScale*skelVec[15][1], mScale*skelVec[15][2], mScale*skelVec[15][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//HipRight
	tempEntity = mSceneMgr->createEntity("HipRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("HipRight", Vector3(mScale*skelVec[16][1], mScale*skelVec[16][2], mScale*skelVec[16][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//KneeRight
	tempEntity = mSceneMgr->createEntity("KneeRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("KneeRight", Vector3(mScale*skelVec[17][1], mScale*skelVec[17][2], mScale*skelVec[17][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//AnkleRight
	tempEntity = mSceneMgr->createEntity("AnkleRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("AnkleRight", Vector3(mScale*skelVec[18][1], mScale*skelVec[18][2], mScale*skelVec[18][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
	//FootRight
	tempEntity = mSceneMgr->createEntity("FootRight", "joint");
	tempNode = mSkelSceneNode->createChildSceneNode("FootRight", Vector3(mScale*skelVec[19][1], mScale*skelVec[19][2], mScale*skelVec[19][3]));
	tempEntity->setMaterialName("joint/HipCenter");
	tempNode->attachObject(tempEntity);
	
}

Avatar::~Avatar()
{
	delete mFileParser;
}

void Avatar::createSphere(const std::string& strName, const float r, const int nRings, const int nSegments)
 {
     MeshPtr pSphere = MeshManager::getSingleton().createManual(strName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
     SubMesh *pSphereVertex = pSphere->createSubMesh();
 
     pSphere->sharedVertexData = new VertexData();
     VertexData* vertexData = pSphere->sharedVertexData;
 
     // define the vertex format
     VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
     size_t currOffset = 0;
     // positions
     vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
     currOffset += VertexElement::getTypeSize(VET_FLOAT3);
     // normals
     vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
     currOffset += VertexElement::getTypeSize(VET_FLOAT3);
     // two dimensional texture coordinates
     vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
     currOffset += VertexElement::getTypeSize(VET_FLOAT2);
 
     // allocate the vertex buffer
     vertexData->vertexCount = (nRings + 1) * (nSegments+1);
     HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
     VertexBufferBinding* binding = vertexData->vertexBufferBinding;
     binding->setBinding(0, vBuf);
     float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));
 
     // allocate index buffer
     pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
     pSphereVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
     HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
     unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));
 
     float fDeltaRingAngle = (Math::PI / nRings);
     float fDeltaSegAngle = (2 * Math::PI / nSegments);
     unsigned short wVerticeIndex = 0 ;
 
     // Generate the group of rings for the sphere
     for( int ring = 0; ring <= nRings; ring++ ) {
         float r0 = r * sinf (ring * fDeltaRingAngle);
         float y0 = r * cosf (ring * fDeltaRingAngle);
 
         // Generate the group of segments for the current ring
         for(int seg = 0; seg <= nSegments; seg++) {
             float x0 = r0 * sinf(seg * fDeltaSegAngle);
             float z0 = r0 * cosf(seg * fDeltaSegAngle);
 
             // Add one vertex to the strip which makes up the sphere
             *pVertex++ = x0;
             *pVertex++ = y0;
             *pVertex++ = z0;
 
             Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
             *pVertex++ = vNormal.x;
             *pVertex++ = vNormal.y;
             *pVertex++ = vNormal.z;
 
             *pVertex++ = (float) seg / (float) nSegments;
             *pVertex++ = (float) ring / (float) nRings;
 
             if (ring != nRings) {
                                // each vertex (except the last) has six indices pointing to it
                 *pIndices++ = wVerticeIndex + nSegments + 1;
                 *pIndices++ = wVerticeIndex;               
                 *pIndices++ = wVerticeIndex + nSegments;
                 *pIndices++ = wVerticeIndex + nSegments + 1;
                 *pIndices++ = wVerticeIndex + 1;
                 *pIndices++ = wVerticeIndex;
                 wVerticeIndex ++;
             }
         }; // end for seg
     } // end for ring
 
     // Unlock
     vBuf->unlock();
     iBuf->unlock();
     // Generate face list
     pSphereVertex->useSharedVertices = true;
 
     // the original code was missing this line:
     pSphere->_setBounds( AxisAlignedBox( Vector3(-r, -r, -r), Vector3(r, r, r) ), false );
     pSphere->_setBoundingSphereRadius(r);
         // this line makes clear the mesh is loaded (avoids memory leaks)
         pSphere->load();
  }
  
void Avatar::moveNextFrame()
{
	//Recuperation du nouveau squelette
	vector<vector<double> > skelVec = mFileParser->getNextFrame();
	  
	SceneNode* tempNode = mSceneMgr->getSceneNode("HipCenter");
	tempNode->setPosition(Vector3(mScale*skelVec[0][1], mScale*skelVec[0][2], mScale*skelVec[0][3]));
	
	tempNode = mSceneMgr->getSceneNode("Spine");
	tempNode->setPosition(Vector3(mScale*skelVec[1][1], mScale*skelVec[1][2], mScale*skelVec[1][3]));
	
	tempNode = mSceneMgr->getSceneNode("ShoulderCenter");
	tempNode->setPosition(Vector3(mScale*skelVec[2][1], mScale*skelVec[2][2], mScale*skelVec[2][3]));
	
	tempNode = mSceneMgr->getSceneNode("Head");
	tempNode->setPosition(Vector3(mScale*skelVec[3][1], mScale*skelVec[3][2], mScale*skelVec[3][3]));
	
	tempNode = mSceneMgr->getSceneNode("ShoulderLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[4][1], mScale*skelVec[4][2], mScale*skelVec[4][3]));
	
	tempNode = mSceneMgr->getSceneNode("ElbowLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[5][1], mScale*skelVec[5][2], mScale*skelVec[5][3]));
	
	tempNode = mSceneMgr->getSceneNode("WristLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[6][1], mScale*skelVec[6][2], mScale*skelVec[6][3]));
	
	tempNode = mSceneMgr->getSceneNode("HandLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[7][1], mScale*skelVec[7][2], mScale*skelVec[7][3]));
	
	tempNode = mSceneMgr->getSceneNode("ShoulderRight");
	tempNode->setPosition(Vector3(mScale*skelVec[8][1], mScale*skelVec[8][2], mScale*skelVec[8][3]));
	
	tempNode = mSceneMgr->getSceneNode("ElbowRight");
	tempNode->setPosition(Vector3(mScale*skelVec[9][1], mScale*skelVec[9][2], mScale*skelVec[9][3]));
	
	tempNode = mSceneMgr->getSceneNode("WristRight");
	tempNode->setPosition(Vector3(mScale*skelVec[10][1], mScale*skelVec[10][2], mScale*skelVec[10][3]));
	
	tempNode = mSceneMgr->getSceneNode("HandRight");
	tempNode->setPosition(Vector3(mScale*skelVec[11][1], mScale*skelVec[11][2], mScale*skelVec[11][3]));
	
	tempNode = mSceneMgr->getSceneNode("HipLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[12][1], mScale*skelVec[12][2], mScale*skelVec[12][3]));
	
	tempNode = mSceneMgr->getSceneNode("KneeLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[13][1], mScale*skelVec[13][2], mScale*skelVec[13][3]));
	
	tempNode = mSceneMgr->getSceneNode("AnkleLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[14][1], mScale*skelVec[14][2], mScale*skelVec[14][3]));
	
	tempNode = mSceneMgr->getSceneNode("FootLeft");
	tempNode->setPosition(Vector3(mScale*skelVec[15][1], mScale*skelVec[15][2], mScale*skelVec[15][3]));
	
	tempNode = mSceneMgr->getSceneNode("HipRight");
	tempNode->setPosition(Vector3(mScale*skelVec[16][1], mScale*skelVec[16][2], mScale*skelVec[16][3]));
	
	tempNode = mSceneMgr->getSceneNode("KneeRight");
	tempNode->setPosition(Vector3(mScale*skelVec[17][1], mScale*skelVec[17][2], mScale*skelVec[17][3]));
	
	tempNode = mSceneMgr->getSceneNode("AnkleRight");
	tempNode->setPosition(Vector3(mScale*skelVec[18][1], mScale*skelVec[18][2], mScale*skelVec[18][3]));
	
	tempNode = mSceneMgr->getSceneNode("FootRight");
	tempNode->setPosition(Vector3(mScale*skelVec[19][1], mScale*skelVec[19][2], mScale*skelVec[19][3]));
	
	getCurrentDistances();
	//cout << "Distance : " << mDistances[4] << endl;
	
}

vector<double>& Avatar::getCurrentDistances()
{
	vector<vector<double> > skelVec = mFileParser->getCurrentFrame();
	
	double distanceTemp;
	mDistances.clear();
	
	//Head-ShoulderCenter
	distanceTemp=abs(skelVec[3][1]-skelVec[2][1]) + abs(skelVec[3][2]-skelVec[2][2]) + abs(skelVec[3][3]-skelVec[2][3]);
	mDistances.push_back(distanceTemp);
	
	//ShoulderCenter-ShoulderLeft
	distanceTemp=abs(skelVec[4][1]-skelVec[2][1]) + abs(skelVec[4][2]-skelVec[2][2]) + abs(skelVec[4][3]-skelVec[2][3]);
	mDistances.push_back(distanceTemp);
	
	//ShoulderLeft-ElbowLeft
	distanceTemp=abs(skelVec[4][1]-skelVec[5][1]) + abs(skelVec[4][2]-skelVec[5][2]) + abs(skelVec[4][3]-skelVec[5][3]);
	mDistances.push_back(distanceTemp);
	
	//ElbowLeft-WristleLeft
	distanceTemp=abs(skelVec[6][1]-skelVec[5][1]) + abs(skelVec[6][2]-skelVec[5][2]) + abs(skelVec[6][3]-skelVec[5][3]);
	mDistances.push_back(distanceTemp);
	
	//WristleLeft-HandLeft
	distanceTemp=abs(skelVec[6][1]-skelVec[7][1]) + abs(skelVec[6][2]-skelVec[7][2]) + abs(skelVec[6][3]-skelVec[7][3]);
	mDistances.push_back(distanceTemp);
	
	//ShoulderCenter-ShoulderRight
	distanceTemp=abs(skelVec[8][1]-skelVec[2][1]) + abs(skelVec[8][2]-skelVec[2][2]) + abs(skelVec[8][3]-skelVec[2][3]);
	mDistances.push_back(distanceTemp);
	
	//ShoulderRight-ElbowRight
	distanceTemp=abs(skelVec[8][1]-skelVec[9][1]) + abs(skelVec[8][2]-skelVec[9][2]) + abs(skelVec[8][3]-skelVec[9][3]);
	mDistances.push_back(distanceTemp);
	
	//ElbowRight-WristleRight
	distanceTemp=abs(skelVec[10][1]-skelVec[9][1]) + abs(skelVec[10][2]-skelVec[9][2]) + abs(skelVec[10][3]-skelVec[9][3]);
	mDistances.push_back(distanceTemp);
	
	//WristleRight-HandRight
	distanceTemp=abs(skelVec[10][1]-skelVec[11][1]) + abs(skelVec[10][2]-skelVec[11][2]) + abs(skelVec[10][3]-skelVec[11][3]);
	mDistances.push_back(distanceTemp);
	
	//ShoulderCenter-Spine
	distanceTemp=abs(skelVec[1][1]-skelVec[2][1]) + abs(skelVec[1][2]-skelVec[2][2]) + abs(skelVec[1][3]-skelVec[2][3]);
	mDistances.push_back(distanceTemp);
	
	//Spine-HipCenter
	distanceTemp=abs(skelVec[1][1]-skelVec[0][1]) + abs(skelVec[1][2]-skelVec[0][2]) + abs(skelVec[1][3]-skelVec[0][3]);
	mDistances.push_back(distanceTemp);
	
	//HipCenter-HipLeft
	distanceTemp=abs(skelVec[12][1]-skelVec[0][1]) + abs(skelVec[12][2]-skelVec[0][2]) + abs(skelVec[12][3]-skelVec[0][3]);
	mDistances.push_back(distanceTemp);
	
	//HipLeft-KneeLeft
	distanceTemp=abs(skelVec[12][1]-skelVec[13][1]) + abs(skelVec[12][2]-skelVec[13][2]) + abs(skelVec[12][3]-skelVec[13][3]);
	mDistances.push_back(distanceTemp);
	
	//KneeLeft-AnkleLeft
	distanceTemp=abs(skelVec[14][1]-skelVec[13][1]) + abs(skelVec[14][2]-skelVec[13][2]) + abs(skelVec[14][3]-skelVec[13][3]);
	mDistances.push_back(distanceTemp);
	
	//AnkleLeft-FootLeft
	distanceTemp=abs(skelVec[14][1]-skelVec[15][1]) + abs(skelVec[14][2]-skelVec[15][2]) + abs(skelVec[14][3]-skelVec[15][3]);
	mDistances.push_back(distanceTemp);
	
	//HipCenter-HipRight
	distanceTemp=abs(skelVec[16][1]-skelVec[0][1]) + abs(skelVec[16][2]-skelVec[0][2]) + abs(skelVec[16][3]-skelVec[0][3]);
	mDistances.push_back(distanceTemp);
	
	//HipRight-KneeRight
	distanceTemp=abs(skelVec[16][1]-skelVec[17][1]) + abs(skelVec[16][2]-skelVec[17][2]) + abs(skelVec[16][3]-skelVec[17][3]);
	mDistances.push_back(distanceTemp);
	
	//KneeRight-AnkleRight
	distanceTemp=abs(skelVec[18][1]-skelVec[17][1]) + abs(skelVec[18][2]-skelVec[17][2]) + abs(skelVec[18][3]-skelVec[17][3]);
	mDistances.push_back(distanceTemp);
	
	//AnkleRight-FootRight
	distanceTemp=abs(skelVec[18][1]-skelVec[19][1]) + abs(skelVec[18][2]-skelVec[19][2]) + abs(skelVec[18][3]-skelVec[19][3]);
	mDistances.push_back(distanceTemp);
	
	return mDistances;
}
