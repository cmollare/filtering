
/*#include "../Control.h"

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

#include "Config.h"*/

#include "../observations/ObsMonoKinect.h"
#include "../3DModel/S3DModelQRS.h"
#include "FilterInt.h"

using namespace std;

//../bin/filtering -N 500 -type "partMMSE"
int main(int argc, char** argv)
{
	FilterInt<ObsMonoKinect, S3DModel<ObsMonoKinect> > filter(argc, argv);
	
	if (filter.isEnvOk())
	{
		int nbFrames = 795;
		FileParser *fileParser = new FileParser("../skel/", "skel_", nbFrames);//Animation file parser
		
		filter.init(fileParser->getFirstFrame(), fileParser->getJointNames());
		
		for (int i=0 ; i<nbFrames ; i++)
		{
			std::cout << "frame : " << i << std::endl;
			filter.update(fileParser->getNextFrame());
		}
	}
	return 0;
}

/*int main(int argc, char ** argv)
{
	
	Config env(argc, argv);
	
	if (env.confOk())
	{
		env.printConf(); //Display the chosen configuration
	
	srand (time(NULL));
	
	YamlBodyJoint ymlBJ("../Model_simple.ymd");//Yaml parser
	ymlBJ.createModel();
	
	int nbFrames = 795;
	FileParser *fileParser = new FileParser("../skel/", "skel_", nbFrames);//Animation file parser
	//int nbFrames = 1150;
	//FileParser *fileParser = new FileParser("../skel2/", "skel_", 1150);//Animation file parser
	
	Joint* model = ymlBJ.getModel();//temporary model
	
	
	//******************************************
	//*************INITIALISATION***************
	//******************************************
	
	
	std::vector<std::vector<double> > frame = fileParser->getFirstFrame();
	std::map<std::string, std::string> jtsToPos; //A mettre dans un fichier
	
	
	//
	jtsToPos["Spine"] = "Spine";
	jtsToPos["Head"] = "Head";
	jtsToPos["ShoulderCenter"] = "ShoulderCenter";
	jtsToPos["ShoulderLeft"] = "ShoulderLeft";
	jtsToPos["ShoulderRight"] = "ShoulderRight";
	jtsToPos["ElbowLeft"] = "ElbowLeft";
	jtsToPos["ElbowRight"] = "ElbowRight";
	jtsToPos["WristLeft"] = "WristLeft";
	jtsToPos["WristRight"] = "WristRight";
	jtsToPos["HipCenter"] = "HipCenter";
	jtsToPos["HipLeft"] = "HipLeft";
	jtsToPos["HipRight"] = "HipRight";
	jtsToPos["KneeLeft"] = "KneeLeft";
	jtsToPos["KneeRight"] = "KneeRight";
	jtsToPos["AnkleLeft"] = "AnkleLeft";
	jtsToPos["AnkleRight"] = "AnkleRight";//
	
	
	

	
	int nbParticles = env.getParticleNumber();
	std::string filterType = env.getFilterType();
	if ((filterType.compare("part") == 0) || (filterType.compare("partMMSE") == 0))
	{
		S3DViewer<S3DModel> viewer;//Declaration of viewer
		viewer.setOptions(true, false, true);
		
		S3DModel *mods= new S3DModel(model);
		mods->mapJointToObs(fileParser->getJointNames(), jtsToPos);
		_Filter<S3DModel, std::vector<std::vector<double> > > *filter;
		
		if (filterType.compare("partMMSE") == 0)
		{
			filter = new PartitionnedMMSE<S3DModel, std::vector<std::vector<double> > >(nbParticles, *mods);
		}
		else if (filterType.compare("part") == 0)
		{
			filter = new Partitionned<S3DModel, std::vector<std::vector<double> > >(nbParticles, *mods);
		}
		std::vector<S3DModel*> particles = filter->getParticleVector();
		
		IKSolverPFOrient<S3DModel> iksol(particles, fileParser->getJointNames(), frame);//Declaration of solver
		
		iksol.mapJointToObs(jtsToPos);
		iksol.initFilter();
		viewer.init();
		
		viewer.initModels(particles);
		viewer.initObservations(fileParser->getJointNames(), frame);
		iksol.computeLikelihood();
		
		bool continuer = true;
		std::string step = "InitFilter";
		while (continuer)
		{
			//frame = fileParser->getNextFrame();//Observation update
			if (step == "IK")
			{
				if (iksol.stepAlt() < 0.60)
				{
					iksol.save();
					step = "InitFilter";
				}
				viewer.update(particles, frame);
				continuer = viewer.isRendering();
			}
			else if (step == "InitFilter")
			{
				filter->init(frame);
				step = "Filter";
				nbFrames--;
				viewer.update(particles, frame);
				continuer = viewer.isRendering();
			}
			else if (step == "Filter")
			{
				frame = fileParser->getNextFrame();//Observation update
				filter->step(frame);
				nbFrames--;

				viewer.update(particles, frame);
				if (nbFrames==0)
				{
					step="End";
					cout << "End of sequence" << endl;
				}
				continuer = viewer.isRendering();
			}
			else if (step == "End")
			{
				continuer = viewer.isRendering();
			}
			else
			{
				cout << "State unknown" << endl;
				continuer = viewer.isRendering();
			}
		}
		
		delete filter;
		delete mods;
		delete fileParser;
	}
	else if ((filterType.compare("partQRS") == 0) || (filterType.compare("partMMSEQRS") == 0))
	{
		S3DViewer<S3DModelQRS> viewer;//Declaration of viewer
		viewer.setOptions(true, false, true);
	
		S3DModelQRS *mods= new S3DModelQRS(model);
		mods->mapJointToObs(fileParser->getJointNames(), jtsToPos);
		
		_Filter<S3DModelQRS, std::vector<std::vector<double> > > *filter;
		if (filterType.compare("partMMSEQRS") == 0)
		{
			filter = new PartitionnedMMSE<S3DModelQRS, std::vector<std::vector<double> > >(nbParticles, *mods);
		}
		else if (filterType.compare("partQRS") == 0)
		{
			filter = new Partitionned<S3DModelQRS, std::vector<std::vector<double> > >(nbParticles, *mods);
		}
		std::vector<S3DModelQRS*> particles = filter->getParticleVector();
		
		IKSolverPFOrient<S3DModelQRS> iksol(particles, fileParser->getJointNames(), frame);//Declaration of solver
		
		iksol.mapJointToObs(jtsToPos);
		iksol.initFilter();
		viewer.init();
		
		viewer.initModels(particles);
		viewer.initObservations(fileParser->getJointNames(), frame);
		iksol.computeLikelihood();
		
		bool continuer = true;
		std::string step = "InitFilter";
		while (continuer)
		{
			//frame = fileParser->getNextFrame();//Observation update
			if (step == "IK")
			{
				if (iksol.stepAlt() < 0.60)
				{
					iksol.save();
					step = "InitFilter";
				}
				viewer.update(particles, frame);
				continuer = viewer.isRendering();
			}
			else if (step == "InitFilter")
			{
				filter->init(frame);
				step = "Filter";
				nbFrames--;
				viewer.update(particles, framn, pour lever toute incertitude!
;-)e);
				continuer = viewer.isRendering();
			}
			else if (step == "Filter")
			{
				frame = fileParser->getNextFrame();//Observation update
				filter->step(frame);
				nbFrames--;

				viewer.update(particles, frame);
				if (nbFrames==0)
				{
					step="End";
					cout << "End of sequence" << endl;
				}
				continuer = viewer.isRendering();
			}
			else if (step == "End")
			{
				continuer = viewer.isRendering();
			}
			else
			{
				cout << "State unknown" << enn, pour lever toute incertitude!
;-)dl;
				continuer = viewer.isRendering();
			}
		}
		
		delete filter;
		delete mods;
		delete fileParser;
	}
	
	
	cout << "Program ended successfully !!!" << endl;
	}
	else
	{
		env.printHelp();
	}
	return 0;
}*/

