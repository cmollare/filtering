#ifndef MIXEDFILEMANAGER_C_H
#define MIXEDFILEMANAGER_C_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>


using namespace std;

struct DataInfo_t
{
	size_t DataOffset;	//Bytes
	size_t DataSizeBytes;	//32bits in bytes
	unsigned int CheckSum;//32bits
};

struct Pair
{
	int a;
	int b;
};

int CheckSum(int* pData,int DataSizex4B);

class MixedFileManager_c
{
public:
	DataInfo_t		DataInfo;
	std::string		FileName;
public:
	MixedFileManager_c();
	//In user load() : First Parse(), then LoadData()
	std::string Parse(std::string FileName);		//Get all the info from the File, the most important beeing the DataSize
	void LoadData(char*	pAllocatedDest,size_t DataSizeBytes);//Load with memory ready to receive data as info is got from Parse
	
	//In user save() : Optionally Configure the MEmitter but the Text Header will be there anyway, 
	//Note the DataStartkey can Optionally be modified, then Save()
	void Save(std::string FileName,char*	pData,size_t DataSizeBytes,std::string UserHeader = "");	//YAML Emitter is configured optionally, and DataInfo obligatory
	void SaveHeaderOnly(std::string FileName,std::string UserHeader = "");
	//-------------------------------------------------------------------------------------------
};

//------------------------------------------------------------------------------YAML serialisation
//--------------------------------------------------------------------------------------------------------------
//YAML << Pair(a,b)
YAML::Emitter& operator<<(YAML::Emitter& out, const Pair pair);
//YAML << ScalarC
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar_<unsigned char> &Scalar);
//YAML << vector<Pair>
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<Pair>& pairs);
//YAML << vector<ScalarC>
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar_<unsigned char> >& ScalVect);
//YAML >> Pair(a,b)
void operator >> (const YAML::Node& node, Pair& pair);
//YAML >> ScalarC
void operator >> (const YAML::Node& node, cv::Scalar_<unsigned char> &Scalar);
//YAML >> vector<Pair>
void operator >> (const YAML::Node& node, std::vector<Pair> &pairs);
//YAML >> vector<ScalarC>
void operator >> (const YAML::Node& node, std::vector<cv::Scalar_<unsigned char> > &VScalars);
//cout << YAML::node
std::ostream& operator<<(std::ostream &out, const YAML::Node& node);
//YAML << cv::Scalar
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar &Scalar);
//YAML << vector<cv::Scalar>
//YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar>& ScalVect);
//YAML >> cv::Scalar
void operator >> (const YAML::Node& node, cv::Scalar &Scalar);

//YAML >> vector<cv::Scalar>
void operator >> (const YAML::Node& node, std::vector<cv::Scalar> &VScalars);

//cout << vector<myType>
template<class myType>
std::ostream& operator<<(std::ostream& out, const std::vector<myType>& Vector)
{
	for(int i=0;i<Vector.size();i++)
	{
		out << Vector[i];//The class is free to end its text with endl or not
	}
	return out;
}

#endif
