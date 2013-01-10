#ifndef VECTORSTABLE_C_H
#define VECTORSTABLE_C_H

#include <opencv2/opencv.hpp>

#include "MixedFileManager_c.h"

typedef std::map<std::vector<float>,int> MLMap_t;
typedef std::vector<std::vector<int> > MLCMat_t;

float CompareClasses_Printf(		MLMap_t &RefClassMap,
										MLMap_t &ResClassMap,
										MLCMat_t &ConfMat,
										bool isPrintfTable = true,
										bool isPrintfAverageResult = true,
										vector<string>*ClassesNames = NULL);
float CompareClasses_Stream(		string &Log,
										MLMap_t &RefClassMap,
										MLMap_t &ResClassMap,
										MLCMat_t &ConfMat,
										vector<string>*ClassesNames = NULL);
										
void ParseFloatLine(std::string &line,std::vector<float> &Data);
size_t FindFirstofAny(string &Str,char C1,char C2);

//------------------------------------------------------------------------------------------------------------------
class VectorsTable_c
{
public:
	std::vector< float >	Data;//Array;
public:
	int VectSize;
	size_t NbVects;
	//int NbVectsInFile;
	std::string TypeName;
	//std::string UserHeader;
	std::vector<std::string>	ClassesNames;//Optionnal Use

	//FileInfo_t Info;

	cv::Mat GetMat();
private:
	void Init();
	std::vector<float> RefVector;
public:
	VectorsTable_c();
	VectorsTable_c(int vVectSize);
	VectorsTable_c(std::string FileName,int verb = 0);
	void Reset(int vVectSize);//nasty should be cleaned with multiple constructors

	//--------------------------------------------------------------------------------------Container utilities
	//std::vector<float>& VectorsTable_c::at(size_t i);
	std::vector<float>& at(size_t i);
	void copyto(size_t i,std::vector<float> &Vect);
	void push(float *pVal);
	void push(std::vector<float>&vVect);
	size_t size();
	void resize(int vVectSize,int vNbVects);
	void clear();

	//----------------------------------------------------------------------------------------
	void save(const std::string &FileName);
	void save(const std::string &FileName,const std::string &UserHeader,bool verbose);
	bool load(const std::string &FileName,bool isAppend = false	,int verb=1);
	bool load(const std::string &FileName,bool isAppend			,int verb	,YAML::Node &user_doc);//so that we're shure append is well treated

	void CopyAppend(const VectorsTable_c &Table);

	void Export(std::string TextFileName,const std::string &UserHeader = "");
	void Import(std::string TextFileName,bool isAppend = true);

	//---------------------------------------------------------------------------------------filtering
	void FilterRemoveDuplicates();
	//---------------------------------------------------------------------------------------Testing
	//big issue, the classes are issued at random of their coming in the map construction
	float CompareClasses(VectorsTable_c &RefTable,int MaxClasses = 20,bool isPrintTab = false,bool isPrintAvg = true);
	float CompareClassesStream(string &Log,VectorsTable_c &RefTable,int MaxClasses = 20,bool isPrintTab = false,bool isPrintAvg = true);
	//simple and Fine
	//std::vector<float> VectorsTable_c::CompareClass9(const VectorsTable_c &Table);
	
	void CompareClasses_ComputeMap(		VectorsTable_c &RefTable,int MaxClasses,
										MLMap_t &RefClassMap,
										MLMap_t &ResClassMap);
	void CompareClasses_ComputeConf(	VectorsTable_c &RefTable,
										MLMap_t &RefClassMap,
										MLMap_t &ResClassMap,
										MLCMat_t &ConfMat			);
};

#endif
