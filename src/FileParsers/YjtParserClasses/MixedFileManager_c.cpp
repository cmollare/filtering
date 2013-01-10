#include "MixedFileManager_c.h"

MixedFileManager_c::MixedFileManager_c()
{
	DataInfo.DataOffset = 0;
	DataInfo.DataSizeBytes = 0;
	DataInfo.CheckSum = 0;
}

std::string MixedFileManager_c::Parse(std::string vFileName)
{
	FileName = vFileName;

	std::ifstream inFile(FileName.c_str(),ios::in | ios::binary);
	//-------------------------------------------------------------------------------------------
	std::string YamlHeader;
	if(inFile.is_open())
	{
		bool isTagFound = false;
		while(!isTagFound && !inFile.eof())
		{
			std::string Line;
			getline(inFile,Line);
			if(Line.compare("***") == 0)
			{
				isTagFound = true;
			}
			else
			{
				//YamlHeader << Line << endl;
				YamlHeader += Line + "\n";
			}
		}
		assert(isTagFound);
		//-------------------------------------------------------------------------------------------
		std::stringstream DataFileHeader;
		isTagFound = false;
		while(!isTagFound && !inFile.eof())
		{
			std::string Line;
			getline(inFile,Line);
			if(Line.compare("***") == 0)
			{
				isTagFound = true;
			}
			else
			{
				DataFileHeader << Line << endl;
			}
		}
		assert(isTagFound);
		unsigned int begin,end;
		begin = inFile.tellg();//might be @ eof()
		inFile.seekg (0, ios::end);
		end = inFile.tellg();	//ifstream::pos_type size;
		inFile.close();
		DataInfo.DataOffset = begin;
		//ull => 18446744072091646652 in stand of 2677062332 and size_t gets negative val
		unsigned int RealDataSizeBytes = end - begin;
		//----------------------------------------------------------Parse DataFileHeader-------------
		YAML::Parser parser;
		//cout << DataFileHeader.str();
		parser.Load(DataFileHeader);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		doc["DataSizeBytes"] >> DataInfo.DataSizeBytes;
		doc["CheckSumXor_32bits"] >> DataInfo.CheckSum;
		//printf("DataInfo %u ; Real %u\n",DataInfo.DataSizeBytes,RealDataSizeBytes);
		unsigned int V1 = DataInfo.DataSizeBytes;
		unsigned int V2 = RealDataSizeBytes;
		//assert(DataInfo.DataSizeBytes == RealDataSizeBytes); //Doesn't pass the comparison test even if equal
		assert(V1 == V2);
		//assert(DataInfo.DataSizeBytes == RealDataSizeBytes);//when we will cross the 2^32 we will feel it here
	}
	return YamlHeader;
}
//--------------------------------------------------------------------------------------------------------------
void MixedFileManager_c::LoadData(char*	pAllocatedDest,size_t DataSizeBytes)
{
	if(FileName.empty())
	{
		printf("Parse a File before loading Data\n");
		return;
	}
	std::ifstream inFile(FileName.c_str(),ios::in | ios::binary);
	inFile.seekg(DataInfo.DataOffset,ios::beg);
	inFile.read(pAllocatedDest,DataSizeBytes);
	int DataCheckSum = CheckSum((int*)pAllocatedDest,DataSizeBytes/4);
	assert(DataCheckSum == DataInfo.CheckSum);
	inFile.close();
}
//--------------------------------------------------------------------------------------------------------------
void MixedFileManager_c::Save(std::string FileName,char*	pData,size_t DataSizeBytes,std::string UserHeader)
{
	std::string DataFileHeader;

	//assert((DataSizeBytes % sizeofOneData) == 0);// why ? Data could be chars[]
	int checksum = CheckSum((int*)pData,DataSizeBytes/4);//The last bytes not checked by this function
	//int checksum = CheckSum((int*)pData,NbData8);//Would Check Sum absolutely all the bytes

	YAML::Emitter MEmitter;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "DataSizeBytes";
	MEmitter << YAML::Value << DataSizeBytes;
	MEmitter << YAML::Key << "CheckSumXor_32bits";
	MEmitter << YAML::Value << YAML::Hex << checksum;
	MEmitter << YAML::EndMap;
	DataFileHeader = MEmitter.c_str();


	std::ofstream outFile(FileName.c_str(),ios::out | ios::binary | ios::trunc);

	outFile << UserHeader << endl;
	outFile << "***" << endl;//Separate between user Header and DataFileHeader
	outFile << DataFileHeader << endl;
	outFile << "***" << endl;//The end of the YAML File
	//outFile << "This is my personal Data Not parsed by YAML" << endl;
	outFile.write(pData,DataSizeBytes);
}
//--------------------------------------------------------------------------------------------------------------
void MixedFileManager_c::SaveHeaderOnly(std::string FileName,std::string UserHeader)
{
	std::string DataFileHeader;

	YAML::Emitter MEmitter;
	MEmitter << YAML::BeginMap;
	MEmitter << YAML::Key << "DataSizeBytes";
	MEmitter << YAML::Value << 0;
	MEmitter << YAML::Key << "CheckSumXor_32bits";
	MEmitter << YAML::Value << YAML::Hex << 0;
	MEmitter << YAML::EndMap;
	DataFileHeader = MEmitter.c_str();


	std::ofstream outFile(FileName.c_str(),ios::out | ios::binary | ios::trunc);

	outFile << UserHeader << endl;
	outFile << "***" << endl;//Separate between user Header and DataFileHeader
	outFile << DataFileHeader << endl;
	outFile << "***" << endl;//The end of the YAML File
}

YAML::Emitter& operator<<(YAML::Emitter& out, const Pair pair)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "a";
	out << YAML::Value << pair.a;
	out << YAML::Key << "b";
	out << YAML::Value << pair.b;
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------

YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar_<unsigned char> &Scalar)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "b";
	out << YAML::Value << (int)(Scalar.val[0]);
	out << YAML::Key << "g";
	out << YAML::Value << (int)(Scalar.val[1]);
	out << YAML::Key << "r";
	out << YAML::Value << (int)(Scalar.val[2]);
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar_<unsigned char> >& ScalVect)
{
	//out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<ScalVect.size();i++)
	{
		out << ScalVect[i];
	}
	out << YAML::EndSeq;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<Pair>& pairs)
{
	//out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<pairs.size();i++)
	{
		out << pairs[i];
	}
	out << YAML::EndSeq;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, Pair& pair)
{
	pair.a = (unsigned char)node["a"].to<int>();
	pair.b = (unsigned char)node["b"].to<int>();
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, cv::Scalar_<unsigned char> &Scalar)
{
	Scalar.val[0] = (unsigned char)node["b"].to<int>();
	Scalar.val[1] = (unsigned char)node["g"].to<int>();
	Scalar.val[2] = (unsigned char)node["r"].to<int>();
	Scalar.val[3] = 0;//Yes Not used and 0 for distance
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, std::vector<cv::Scalar_<unsigned char> > &VScalars)
{
	int NbVals = node.size();
	VScalars.resize(NbVals);
	for(int i=0;i<NbVals;i++)
	{
		node[i] >> VScalars[i];
	}
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, std::vector<Pair> &pairs)
{
	int NbVals = node.size();
	pairs.resize(NbVals);
	for(int i=0;i<NbVals;i++)
	{
		node[i] >> pairs[i];
	}
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &out, const YAML::Node& node)
{
	YAML::Emitter Emit;
	Emit << node;
	out << Emit.c_str() << std::endl;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const cv::Scalar &Scalar)
{
	out << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << Scalar.val[0];
	out << YAML::Key << "y";
	out << YAML::Value << Scalar.val[1];
	out << YAML::Key << "z";
	out << YAML::Value << Scalar.val[2];
	out << YAML::Key << "w";
	out << YAML::Value << Scalar.val[3];
	out << YAML::EndMap;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
YAML::Emitter& operator<<(YAML::Emitter& out, const std::vector<cv::Scalar>& ScalVect)
{
	//out << YAML::Flow;
	out << YAML::BeginSeq;
	for(int i=0;i<ScalVect.size();i++)
	{
		out << ScalVect[i];
	}
	out << YAML::EndSeq;
	return out;
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, cv::Scalar &Scalar)
{
	node["x"] >> Scalar.val[0];
	node["y"] >> Scalar.val[1];
	node["z"] >> Scalar.val[2];
	node["w"] >> Scalar.val[3];
}
//--------------------------------------------------------------------------------------------------------------
void operator >> (const YAML::Node& node, std::vector<cv::Scalar> &VScalars)
{
	size_t NbVals = node.size();
	VScalars.resize(NbVals);
	for(size_t i=0;i<NbVals;i++)
	{
		node[i] >> VScalars[i];
	}
}

//--------------------------------------------------------------------------------------------------------------
int CheckSum(int* pData,int DataSizex4B)
{
	int XorRes = 0;
	int *pDataAfterLast = pData + DataSizex4B;
	while(pData<pDataAfterLast)
	{
		XorRes+=(*pData++);
	}
	return XorRes;
}
