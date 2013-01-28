#ifndef FILTERINT_H
#define FILTERINT_H

#include "../Control.h"

#include <iostream>
#include <vector>
#include <time.h>
#include <map>

#include "../viewer/S3DViewer.h"

#include "../FileParsers/YamlBodyJoint.h"
#include "../FileParsers/FileParser.h"

#include "../solver/IKSolverPFOrient.h"
#include "../solver/IKSolverMathias.h"

#include "../3DModel/S3DModel.h"
#include "../3DModel/S3DModelQRS.h"

#include "../filter/SIR.h"
#include "../filter/Partitionned.h"
#include "../filter/PartitionnedMMSE.h"
#include "../tools/_Stats.h"

#include "Config.h"

template<class Observations, class Particles>
class FilterInt
{
	public:
		FilterInt(int argc, char** argv);
		~FilterInt();
		void init(Observations firstFrame, std::vector<std::string>& posNames);
		void update(Observations frame);
		std::vector<std::vector<double> > getPosture();
		bool isEnvOk();
		
		Config* _env;
		_Filter<Particles, Observations> *filter;
		S3DViewer<Particles, Observations> *viewer;
		Particles *mods;

		std::string filterType;
		std::map<std::string, std::string> jtsToPos;
		std::vector<std::string> jointVector;
};

#include "FilterInt.cpp" //To separate implementation from declaration

#endif
