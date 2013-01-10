#include "VectorsTable_c.h"

//----------------------------------------------------------------------------------
//								VectorsTable_c
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void VectorsTable_c::Init()
{
	VectSize = 0;//Not 0 so that size() can be returned without additionnal overhead
	NbVects = 0;
	//NbVectsInFile = 0;
	TypeName = "NotInitialized";
}
//----------------------------------------------------------------------------------
VectorsTable_c::VectorsTable_c()
{
	Init();
}
//----------------------------------------------------------------------------------
VectorsTable_c::VectorsTable_c(int vVectSize)
{
	Init();
	VectSize = vVectSize;
}
//----------------------------------------------------------------------------------
VectorsTable_c::VectorsTable_c(std::string FileName,int verb)
{
	Init();
	load(FileName,false,verb);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::Reset(int vVectSize)
{
	VectSize = vVectSize;
	clear();
}
//----------------------------------------------------------------------------------
size_t VectorsTable_c::size()
{
	if(VectSize == 0)
		return Data.size();
	else
		return Data.size() / VectSize;
}
//----------------------------------------------------------------------------------
void VectorsTable_c::resize(int vVectSize,int vNbVects)
{
	VectSize = vVectSize;
	Data.resize(vNbVects * VectSize);
}
//----------------------------------------------------------------------------------
std::vector<float>& VectorsTable_c::at(size_t i)
{
	if(RefVector.size()!=VectSize)
	{
		RefVector.resize(VectSize);
	}
	memcpy(&RefVector[0],&Data[i * VectSize],VectSize * sizeof(float));
	return RefVector;
}
//----------------------------------------------------------------------------------
void VectorsTable_c::push(std::vector<float>&vVect)
{
	//for(int i=0;i<VectSize;i++)Data.push_back(vVect[i]);
	assert(!vVect.empty());
	float *pData = &vVect[0];
	float *pDataAfterLast = pData + VectSize;
	while(pData<pDataAfterLast)
	{
		Data.push_back(*pData++);
	}
}
//----------------------------------------------------------------------------------
//This function still copies the data, for faster access, resize Data[] then take a pointer
void VectorsTable_c::push(float *pVal)
{
	for(int i=0;i<VectSize;i++)Data.push_back(*pVal++);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::copyto(size_t i,std::vector<float> &Vect)
{
	memcpy(&Data[i * VectSize],&Vect[0],VectSize * sizeof(float));
}
//----------------------------------------------------------------------------------
void VectorsTable_c::save(const std::string &FileName,const std::string &UserHeader,bool verbose)// Vect Size and Name Should be initialized
{
	
	//assert(!Data.empty());//Yeah, no empty file, by design => Changed to accepting empty files, box, tables,...

	YAML::Emitter MEmitter;
	if(UserHeader.empty())
	{
		MEmitter << YAML::BeginDoc;
		MEmitter << YAML::EndDoc;
	}
	MEmitter << YAML::BeginDoc;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "TypeName";
	MEmitter << YAML::Value << TypeName;//user TypeName is obligatory while additional Header is optional
	if(!ClassesNames.empty())
	{
		MEmitter << YAML::Key << "ClassesNames";
		MEmitter << YAML::Value << ClassesNames;
	}
	MEmitter << YAML::Key << "NbFloats";
	MEmitter << YAML::Value << Data.size();
	MEmitter << YAML::Key << "VectSize";
	MEmitter << YAML::Value << VectSize;
	MEmitter << YAML::Key << "NbVects";
	MEmitter << YAML::Value << size();
	MEmitter << YAML::EndMap;
	MEmitter << YAML::EndDoc;
	
	MixedFileManager_c FManager;
	if(Data.empty())
	{
		FManager.SaveHeaderOnly(FileName, UserHeader + MEmitter.c_str()  );
	}
	else
	{
		FManager.Save(FileName,(char*)&Data[0],Data.size()*4,  UserHeader + MEmitter.c_str()  );
	}
	//FManager.Save(FileName,(char*)&Data[0],Data.size()*4,  MEmitter.c_str()  );
	

}
//----------------------------------------------------------------------------------
void VectorsTable_c::save(const std::string &FileName)
{
	save(FileName,"",true);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::Export(std::string TextFileName,const std::string &UserHeader)
{
	YAML::Emitter MEmitter;
	if(UserHeader.empty())
	{
		MEmitter << YAML::BeginDoc;
		MEmitter << YAML::EndDoc;
	}
	MEmitter << YAML::BeginDoc;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "TypeName";
	MEmitter << YAML::Value << TypeName;//user TypeName is obligatory while additional Header is optional
	MEmitter << YAML::Key << "NbFloats";
	MEmitter << YAML::Value << Data.size();
	MEmitter << YAML::Key << "VectSize";
	MEmitter << YAML::Value << VectSize;
	MEmitter << YAML::Key << "NbVects";
	MEmitter << YAML::Value << size();
	MEmitter << YAML::EndMap;
	MEmitter << YAML::EndDoc;

	std::ofstream myfile;
	myfile.open(TextFileName.c_str(),ios::out | ios::trunc);
	if(myfile.is_open())
	{
		//cout << "UserHeader: " << UserHeader << "Emitter: " << MEmitter.c_str() << endl;
		myfile << UserHeader << MEmitter.c_str();

		float *pData = &Data[0];
		NbVects = size();
		for(int i=0;i<NbVects;i++)
		{
			myfile << (*pData++);
			for(int j=1;j<VectSize;j++)
			{
				myfile << "," << (*pData++);
			}
			myfile << std::endl;
		}
		myfile.close();
		printf("File (%s) Exported with %d Features\n",TextFileName.c_str(),VectSize);
	}
	else
	{
		printf("Couldn't Export File (%s)\n",TextFileName.c_str());
	}
}
//----------------------------------------------------------------------------------
void VectorsTable_c::Import(std::string TextFileName,bool isAppend)
{
	//-----------------------------Left Issue do not parse the YAML Header to set tge TypeName !!!
	std::ifstream myfile;
	std::string line;
	myfile.open(TextFileName.c_str(),ios::in);
	if(Data.empty())
	{
		isAppend = false;
	}
	else
	{
		if(!isAppend)//if(isReWrite)
		{
			clear();
		}
	}
	if(myfile.is_open())
	{
		//--------------------------remove docs '...'
		int NbTagsFound = 0;
		while((NbTagsFound !=2) && !myfile.eof())
		{
			std::string Line;
			getline(myfile,Line);
			if(Line.compare("...") == 0)
			{
				NbTagsFound++;
			}
			if(Line.compare("TypeName") == 0)
			{
				TypeName = Line.substr( Line.find_last_of( ':' ) +2,Line.length() );//BAD Manually parsing YAML what laziness
				NbTagsFound++;
			}
		}
		if(NbTagsFound == 0)
		{
			//myfile.seekg(0, ios::beg);//just a simple .csv File without header - so restart
			myfile.close();//apparently seekg brakes the text format and getline() works no more after it
			myfile.open(TextFileName.c_str(),ios::in);
		}
		//--------------------------remove docs

		std::vector<float> VData;
		getline(myfile,line);
		ParseFloatLine(line,VData);//Set DataSize
		int NbDataPerLine = (int)VData.size();
		if(isAppend)
		{
			assert(VectSize == NbDataPerLine);
		}
		else
		{
			VectSize = NbDataPerLine;
		}
		push(VData);
		//-----------Start Cycle with getline
		int NbLines = 1;
		getline(myfile,line);
		while(!myfile.eof())
		{
			ParseFloatLine(line,VData);
			assert(VData.size() == NbDataPerLine);
			push(VData);
			getline(myfile,line);
			NbLines++;
		}
		myfile.close();
		printf("Imported %d Samples of VectSize %d (lines %d)\n",size(),VectSize,NbLines);
	}
	else
	{
		printf("Couldn't Import File (%s)\n",TextFileName.c_str());
	}
	
}
//----------------------------------------------------------------------------------
bool VectorsTable_c::load(const std::string &FileName,bool isAppend,int verb,YAML::Node &user_doc)
{
	bool Res = true;

	if(Data.empty())
	{
		isAppend = false;//override it
	}

	MixedFileManager_c FManager;
	std::stringstream YamlHeader(FManager.Parse(FileName));
	if(YamlHeader.str().empty())
	{
		cout << "VectorsTable_c::load() Couldn't load File: " << FileName << endl;
		return false;
	}
	YAML::Parser parser;
	parser.Load(YamlHeader);
	parser.GetNextDocument(user_doc);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	doc["TypeName"] >> TypeName;
	if(doc.FindValue("ClassesNames"))
	{
		doc["ClassesNames"] >> ClassesNames;
	}
	size_t vNbFloats,vVectSize,vNbVects;
	doc["NbFloats"] >> vNbFloats;
	doc["VectSize"] >> vVectSize;
	doc["NbVects"] >> vNbVects;
	assert(vNbVects == vNbFloats / vVectSize);
	assert((vNbFloats % vVectSize) == 0);
	assert(FManager.DataInfo.DataSizeBytes == vNbFloats*4);//Check Coherence between myinfo and FManager Info

	if(isAppend)//Data not empty
	{
		assert(vVectSize == VectSize);
	}
	size_t oldDataSize = Data.size();
	size_t FileDataSize = vNbFloats;

	Data.resize(oldDataSize + FileDataSize);
	float *pNewData = &Data[oldDataSize];
	FManager.LoadData((char*)pNewData,FileDataSize*4);

	VectSize = vVectSize;//Already asserted on Append or Affect it here
	NbVects = Data.size() / VectSize;
	
	return Res;
}
//----------------------------------------------------------------------------------
bool VectorsTable_c::load(const std::string &FileName,bool isAppend,int verb)
{
	YAML::Node ThrowAwayThisDoc;
	return load(FileName,isAppend,verb,ThrowAwayThisDoc);
}
//----------------------------------------------------------------------------------
void VectorsTable_c::CopyAppend(const VectorsTable_c &Table)
{
	if(Data.size() == 0)//just copy and Initialize VectSize and Type
	{
		VectSize = Table.VectSize;
		TypeName = Table.TypeName;
	}
	else//append case
	{
		assert(Table.VectSize == VectSize);
		assert(TypeName.compare(Table.TypeName) == 0);
	}
	size_t OldDataSize = Data.size();
	Data.resize(OldDataSize + Table.Data.size());//here the reallocation magic operates
	float * pNewData = &Data[OldDataSize];
	memcpy(&Data[OldDataSize],&Table.Data[0],Table.Data.size()*sizeof(float));
}
//----------------------------------------------------------------------------------
void VectorsTable_c::clear()
{
	Data.clear();
}
//----------------------------------------------------------------------------------
cv::Mat VectorsTable_c::GetMat()
{
	NbVects = size();
	return cv::Mat((int)NbVects,VectSize,CV_32FC1,&Data[0]);//Shallow copy the constructor in TableMat
}
//----------------------------------------------------------------------------------
void VectorsTable_c::FilterRemoveDuplicates()
{
	std::set<std::vector<float> > TableSet;
	NbVects = size();
	assert(NbVects!=0);
	std::vector<float> Vecti(VectSize);
	for(int i=0;i<NbVects;i++)
	{
		Vecti = at(i);
		TableSet.insert(Vecti);
	}
	//TStop(t,"SetInsertion");// 78 ms
	Data.resize(TableSet.size() * VectSize);
	float *pData = &Data[0];
	//TStop(t,"resize");// 7 ms  ???!!! what for in resize

	std::set<std::vector<float> >::iterator it;
	int index = 0;
	for(it=TableSet.begin(); it!=TableSet.end(); it++)
	{
		for(int i=0;i<VectSize;i++)
		{
			(*pData++) = (*it)[i];//couldn't get a pointer at it as it's a const float*
		}
	}
	//TStop(t,"copy from set to vect");// 1.3 ~ 1.8 ms
}
//---------------------------------------------------------------------------------------------------------------
//CompareClasses_ComputeMap can append the processing
void VectorsTable_c::CompareClasses_ComputeMap(	VectorsTable_c &RefTable,int MaxClasses,
												MLMap_t &RefClassMap,
												MLMap_t &ResClassMap)
{
	assert(RefTable.VectSize == VectSize);
	assert(TypeName.compare(RefTable.TypeName) == 0);
	assert(RefTable.Data.size() == Data.size());
	//------------------------------------------------------------------find the classes map in Reference
	int NbRefClasses = 0;
	size_t NbRefVects = RefTable.size();
	for(size_t i=0;i<NbRefVects;i++)
	{
		std::vector<float> Sample = RefTable.at(i);
		if(RefClassMap.find(Sample) == RefClassMap.end())//Not found
		{
			RefClassMap[Sample] = NbRefClasses++;//New Class Added
		}
	}
	assert(NbRefClasses <= MaxClasses);
	//------------------------------------------------------------------Reaffect Ordered indices
	MLMap_t::iterator Rit;
	int index = 0;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		(*Rit).second = index++;
	}
	//------------------------------------------------------------------find the classes map in Result
	int NbResClasses = 0;
	NbVects = size();
	for(size_t i=0;i<NbVects;i++)
	{
		std::vector<float> Sample = at(i);
		if(ResClassMap.find(Sample) == ResClassMap.end())//Not found
		{
			ResClassMap[Sample] = NbResClasses++;//New Class Added
		}
	}
	assert(NbResClasses <= MaxClasses);
	//------------------------------------------------------------------Reaffect Ordered indices
	index = 0;
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		(*Rit).second = index++;
	}
}
//---------------------------------------------------------------------------------------------------------------
//CompareClasses_ComputeConf also appends the process
void VectorsTable_c::CompareClasses_ComputeConf(	VectorsTable_c &RefTable,
													MLMap_t &RefClassMap,
													MLMap_t &ResClassMap,
													MLCMat_t &ConfMat			)
{
	assert(RefTable.VectSize == VectSize);
	assert(TypeName.compare(RefTable.TypeName) == 0);
	assert(RefTable.Data.size() == Data.size());
	//------------------------------------------------------------------Process Confusion Matrix
	size_t NbRefClasses = RefClassMap.size();
	size_t NbResClasses = ResClassMap.size();
	if(ConfMat.size() != NbRefClasses)//if First time only
	{
		ConfMat = MLCMat_t(NbRefClasses, std::vector<int>(NbResClasses));//Initialized to 0 ??!!
	}
	NbVects = Data.size() / VectSize;
	for(int i=0;i<NbVects;i++)
	{
		std::vector<float> RefSample = RefTable.at(i);
		std::vector<float> ResSample = at(i);
		int RefClassID = RefClassMap[RefSample];
		int ResClassID = ResClassMap[ResSample];
		ConfMat[RefClassID][ResClassID]++;
	}
}
//---------------------------------------------------------------------------------------------------------------
float VectorsTable_c::CompareClasses(VectorsTable_c &RefTable,int MaxClasses,bool isPrintTab,bool isPrintAvg)
{
	MLMap_t RefClassMap;
	MLMap_t ResClassMap;
	MLCMat_t ConfMat;
	//mcv::TabPrintf(ClassesNames,"ClassesNames");
	CompareClasses_ComputeMap(RefTable,MaxClasses,RefClassMap,ResClassMap);
	CompareClasses_ComputeConf(RefTable,RefClassMap,ResClassMap,ConfMat);
	if(!ClassesNames.empty())
	{
		return CompareClasses_Printf(RefClassMap,ResClassMap,ConfMat,isPrintTab,isPrintAvg,&ClassesNames);
	}
	else
	{
		return CompareClasses_Printf(RefClassMap,ResClassMap,ConfMat,isPrintTab,isPrintAvg,&ClassesNames);
	}
}
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
float VectorsTable_c::CompareClassesStream(string &Log,VectorsTable_c &RefTable,int MaxClasses,bool isPrintTab,bool isPrintAvg)
{
	MLMap_t RefClassMap;
	MLMap_t ResClassMap;
	MLCMat_t ConfMat;
	//mcv::TabPrintf(ClassesNames,"ClassesNames");
	CompareClasses_ComputeMap(RefTable,MaxClasses,RefClassMap,ResClassMap);
	CompareClasses_ComputeConf(RefTable,RefClassMap,ResClassMap,ConfMat);
	if(!ClassesNames.empty())
	{
		return CompareClasses_Stream(Log,RefClassMap,ResClassMap,ConfMat,&ClassesNames);
	}
	else
	{
		return CompareClasses_Stream(Log,RefClassMap,ResClassMap,ConfMat,&ClassesNames);
	}
}

float CompareClasses_Printf(	MLMap_t &RefClassMap,
											MLMap_t &ResClassMap,
											MLCMat_t &ConfMat,
											bool isPrintfTable,
											bool isPrintfAverageResult,
											vector<string>*ClassesNames)
{
	size_t NbRefClasses = RefClassMap.size();
	size_t NbResClasses = ResClassMap.size();
	int idx;
	std::vector<std::vector<float> > RefD(NbRefClasses);
	std::vector<std::vector<float> > ResD(NbResClasses);

	bool isT = isPrintfTable;
	if(isT)printf("Confusion Matrix:\n");
	if(isT)printf("Res Classes horz:\n");
	MLMap_t::iterator Rit;
	idx = 0;
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		ResD[idx] = (*Rit).first;
		//mcv::TabPrintf(ResD[idx],NULL,false,' ',0);printf(",");
		if(isT)
		{
			for(int i=0;i<ResD[idx].size();i++)
			{
				if(ClassesNames != NULL)
				{
					int PartIndex = ResD[idx][i];
					cout << (*ClassesNames)[PartIndex] << ",";
				}
				else
				{
					printf("% 6.0f",ResD[idx][i]);printf(",");
				}
			}
		}
		//for(int i=0;i<Vect.size();i++)printf("%1.2f ",ResD[idx][i]);printf(",");
		idx++;
	}
	if(isT)printf("<-- classified as\n");
	//printf("Ref Classes vert: ");
	idx = 0;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		//std::vector<float> Vect = (*Rit).first;
		RefD[idx] = (*Rit).first;
		//mcv::TabPrintf(RefD[idx],NULL,false,' ',0);
		//for(int i=0;i<Vect.size();i++)printf("%1.2f ",Vect[i]);printf(",");
		idx++;
	}
	//printf("\n");
		
	for(size_t ref=0;ref<NbRefClasses;ref++)
	{
		for(size_t res=0;res<NbResClasses;res++)
		{
			if(isT)printf("%6d,",ConfMat[ref][res]);
		}
		if(isT)printf("| ");
		//for(int i=0;i<RefD.size();i++)printf("% 6.0f",RefD[ref][i]);printf(",");
		if(isT) TabPrintf(RefD[ref],NULL,false,' ',0);
		if(ClassesNames != NULL)
		{
			if(!RefD[ref].empty())
			{
				int PartIndex = RefD[ref][0];
				cout << (*ClassesNames)[PartIndex];
			}
		}

		if(isT)printf("\n");
	}
	//Yeah pretty complex : knowning that mapped value is unique, we need reverse mapping of the Ref Res Maps
	std::map<int,std::vector<float> > revRefClassMap;
	std::map<int,std::vector<float> > revResClassMap;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		revRefClassMap[(*Rit).second] = (*Rit).first;
	}
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		revResClassMap[(*Rit).second] = (*Rit).first;
	}


	std::vector<int> ClassTotal(NbRefClasses);
	std::vector<int> ClassNbGood(NbRefClasses);
	std::vector<float> ClassRatio(NbRefClasses);

	for(int ref=0;ref<NbRefClasses;ref++)
	{
		for(int res=0;res<NbResClasses;res++)
		{
			ClassTotal[ref]+=ConfMat[ref][res];
			if(revRefClassMap[ref] == revResClassMap[res])//used reverse mapping to check equality
			{
				ClassNbGood[ref]+=ConfMat[ref][res];
			}
		}
	}

	for(int i=0;i<NbRefClasses;i++)
	{
		ClassRatio[i] = (float)ClassNbGood[i];
		if(ClassTotal[i] == 0)
		{
			ClassRatio[i] = 0;
		}
		else
		{
			ClassRatio[i] /= ClassTotal[i];
		}
	}


	bool isA = isPrintfAverageResult;
	if(isA)
	{
		if(ClassesNames != NULL)//very nasty should use the reference kept classes
		{
			if(!(*ClassesNames).empty())
			{
				if((*ClassesNames)[0].compare("Empty") == 0)
				{
					ClassRatio.erase(ClassRatio.begin());
				}
				if((*ClassesNames)[0].compare("Uncknown") == 0)
				{
					ClassRatio.erase(ClassRatio.begin());
				}
			}
		}
		TabPrintf(ClassRatio,"GoodClassesRatio",false);
		//printf("Good Classes Ratios");
		//for(int j=0;j<ClassRatio.size();j++)printf(",%1.3f",ClassRatio[j]);
	}

	float TAvg = TabAverage(ClassRatio);

	if(isA)printf("Parts Avg,%1.1f%%\n",TAvg*100);
	

	return TAvg;
}

float CompareClasses_Stream(			string &resStr,
											MLMap_t &RefClassMap,
											MLMap_t &ResClassMap,
											MLCMat_t &ConfMat,
											vector<string>*ClassesNames)
{
	stringstream sstr;
	size_t NbRefClasses = RefClassMap.size();
	size_t NbResClasses = ResClassMap.size();
	int idx;
	vector<vector<float> > RefD(NbRefClasses);
	vector<vector<float> > ResD(NbResClasses);

	sstr << "Confusion Matrix:" << endl;
	sstr << "Res Classes horz:" << endl;
	MLMap_t::iterator Rit;
	idx = 0;
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		ResD[idx] = (*Rit).first;
		//mcv::TabPrintf(ResD[idx],NULL,false,' ',0);printf(",");
		for(int i=0;i<ResD[idx].size();i++)
		{
			if(ClassesNames != NULL)
			{
				int PartIndex = ResD[idx][i];
				sstr << (*ClassesNames)[PartIndex] << '\t';
			}
			else
			{
				sstr << setprecision(0) << ResD[idx][i] << '\t' << endl;
			}
		}
		//for(int i=0;i<Vect.size();i++)printf("%1.2f ",ResD[idx][i]);printf(",");
		idx++;
	}
	sstr << "<-- classified as" << endl;
	//printf("Ref Classes vert: ");
	idx = 0;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		//std::vector<float> Vect = (*Rit).first;
		RefD[idx] = (*Rit).first;
		//mcv::TabPrintf(RefD[idx],NULL,false,' ',0);
		//for(int i=0;i<Vect.size();i++)printf("%1.2f ",Vect[i]);printf(",");
		idx++;
	}
	//printf("\n");
		
	for(size_t ref=0;ref<NbRefClasses;ref++)
	{
		for(size_t res=0;res<NbResClasses;res++)
		{
			sstr << ConfMat[ref][res] << '\t';
		}
		sstr << "| ";
		//for(int i=0;i<RefD.size();i++)printf("% 6.0f",RefD[ref][i]);printf(",");
			if(!RefD[ref].empty())
			{
				sstr << RefD[ref][0] << " ";
			}
		if(ClassesNames != NULL)
		{
			if(!RefD[ref].empty())
			{
				int PartIndex = RefD[ref][0];
				sstr << (*ClassesNames)[PartIndex];
			}
		}
		sstr << endl;
	}
	//Yeah pretty complex : knowning that mapped value is unique, we need reverse mapping of the Ref Res Maps
	std::map<int,std::vector<float> > revRefClassMap;
	std::map<int,std::vector<float> > revResClassMap;
	for(Rit = RefClassMap.begin();Rit!=RefClassMap.end();Rit++)
	{
		revRefClassMap[(*Rit).second] = (*Rit).first;
	}
	for(Rit = ResClassMap.begin();Rit!=ResClassMap.end();Rit++)
	{
		revResClassMap[(*Rit).second] = (*Rit).first;
	}


	std::vector<int> ClassTotal(NbRefClasses);
	std::vector<int> ClassNbGood(NbRefClasses);
	std::vector<float> ClassRatio(NbRefClasses);

	for(int ref=0;ref<NbRefClasses;ref++)
	{
		for(int res=0;res<NbResClasses;res++)
		{
			ClassTotal[ref]+=ConfMat[ref][res];
			if(revRefClassMap[ref] == revResClassMap[res])//used reverse mapping to check equality
			{
				ClassNbGood[ref]+=ConfMat[ref][res];
			}
		}
	}

	for(int i=0;i<NbRefClasses;i++)
	{
		ClassRatio[i] = (float)ClassNbGood[i];
		if(ClassTotal[i] == 0)
		{
			ClassRatio[i] = 0;
		}
		else
		{
			ClassRatio[i] /= ClassTotal[i];
		}
	}

	if(ClassesNames != NULL)//very nasty should use the reference kept classes - BAD Dangerous fix, to fix ASAP
	{
		if(!(*ClassesNames).empty())
		{
			if((*ClassesNames)[1].compare("Uncknown") == 0)//Empty Uncknown
			{
				ClassRatio.erase(ClassRatio.begin());
			}
		}
	}
	sstr << "GoodClassesRatio:" << endl << TabStream(ClassRatio,NULL,true,'\t',3);

	float TAvg = TabAverage(ClassRatio);

	sstr << "Parts Avg:" << TAvg*100 << endl;

	resStr = sstr.str();

	return TAvg;
}

void ParseFloatLine(std::string &line,std::vector<float> &Data)
{
	Data.clear();
	//stringstream ss (stringstream::in | stringstream::out);
	//ss << line;
	while(!line.empty())
	{
		float Val;
		//std::string Vs = line.substr( 0, line.find_first_of(',' ) );
		size_t SepPos = FindFirstofAny(line,',',';');
		std::string Vs = line.substr( 0, SepPos );
		if(SepPos != line.npos)
		{
			line = line.substr( SepPos+1,line.length() );
		}
		else
		{
			line = "";
		}
		Val = (float)atof(Vs.c_str());
		Data.push_back(Val);
	}
}

size_t FindFirstofAny(string &Str,char C1,char C2)
{
	size_t PC1 = Str.find_first_of(C1);
	size_t PC2 = Str.find_first_of(C2);
	if(PC1 == Str.npos)
	{
		return PC2;
	}
	else
	{
		if(PC2 == Str.npos)
		{
			return PC1;
		}
		else
		{
			return max(PC1,PC2);
		}
	}
}


