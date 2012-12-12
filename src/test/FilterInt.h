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

#include "../3DModel/S3DModel.h"
#include "../3DModel/S3DModelQRS.h"

#include "../filter/SIR.h"
#include "../filter/Partitionned.h"
#include "../filter/PartitionnedMMSE.h"
#include "../tools/_Stats.h"

#include "Config.h"

class FilterInt
{
	public:
		FilterInt(int argc, char ** argv);
		~FilterInt();
		void init(std::vector<std::vector<double> >& firstFrame, std::vector<std::string>& posNames);
		void update(std::vector<std::vector<double> >& frame);
		std::vector<std::vector<double> > getPosture();
		bool isEnvOk();
		
		Config* _env;
		_Filter<S3DModel, std::vector<std::vector<double> > > *filter;
		_Filter<S3DModelQRS, std::vector<std::vector<double> > > *filterQRS;
		S3DModel *mods;
		S3DModelQRS *modsQRS;
		std::string filterType;
};

#endif
