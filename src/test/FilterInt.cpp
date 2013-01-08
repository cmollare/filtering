#ifdef FILTERINT_H

template<class Observations>
FilterInt<Observations>::FilterInt(int argc, char ** argv)
{
	_env = new Config(argc, argv);
	
	if (_env->confOk())
	{
		_env->printConf(); //Display the chosen configuration
		srand (time(NULL));
	}
	else
	{
		_env->printHelp();
	}
	
	filter=NULL;
	filterQRS=NULL;
	mods=NULL;
	modsQRS=NULL;
}

template<class Observations>
FilterInt<Observations>::~FilterInt()
{
	if (filter) delete filter;
	if (filterQRS) delete filterQRS;
	if (mods) delete mods;
	if (modsQRS) delete modsQRS;
	
	delete _env;
}

template<class Observations>
void FilterInt<Observations>::init(Observations firstFrame, std::vector<std::string>& posNames)
{
	YamlBodyJoint ymlBJ("../Model_simple.ymd");//Yaml parser
	ymlBJ.createModel();
	
	Joint* model = ymlBJ.getModel();//temporary model
	
	//std::vector<std::vector<double> > frame = fileParser->getFirstFrame();
	Observations frame = firstFrame;
	std::map<std::string, std::string> jtsToPos; //A mettre dans un fichier
	
	
	//*	
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
	jtsToPos["AnkleRight"] = "AnkleRight";//*/
	
	int nbParticles = _env->getParticleNumber();
	filterType = _env->getFilterType();
	
	if ((filterType.compare("part") == 0) || (filterType.compare("partMMSE") == 0))
	{
		//S3DViewer<S3DModel> viewer;//Declaration of viewer
		//viewer.setOptions(true, false, true);
		mods= new S3DModel(model);
		mods->mapJointToObs(posNames, jtsToPos);
		
		
		if (filterType.compare("partMMSE") == 0)
		{
			filter = new PartitionnedMMSE<S3DModel, Observations>(nbParticles, *mods);
		}
		else if (filterType.compare("part") == 0)
		{
			filter = new Partitionned<S3DModel, Observations>(nbParticles, *mods);
		}
		std::vector<S3DModel*> particles = filter->getParticleVector();
		
		IKSolverPFOrient<S3DModel> iksol(particles, posNames, frame.getFrame());//Declaration of solver
		
		iksol.mapJointToObs(jtsToPos);
		iksol.initFilter();
		//viewer.init();
		
		//viewer.initModels(particles);
		//viewer.initObservations(fileParser->getJointNames(), frame);
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
				//viewer.update(particles, frame);
				//continuer = viewer.isRendering();
			}
			else if (step == "InitFilter")
			{
				filter->init(frame);
				step = "Filter";
				//nbFrames--;
				//viewer.update(particles, frame);
				//continuer = viewer.isRendering();
				continuer=false;
			}
		}
	}
	else if ((filterType.compare("partQRS") == 0) || (filterType.compare("partMMSEQRS") == 0))
	{
		//S3DViewer<S3DModelQRS> viewer;//Declaration of viewer
		//viewer.setOptions(true, false, true);
	
		modsQRS= new S3DModelQRS(model);
		modsQRS->mapJointToObs(posNames, jtsToPos);
		
		
		if (filterType.compare("partMMSEQRS") == 0)
		{
			filterQRS = new PartitionnedMMSE<S3DModelQRS, Observations>(nbParticles, *modsQRS);
		}
		else if (filterType.compare("partQRS") == 0)
		{
			filterQRS = new Partitionned<S3DModelQRS, Observations>(nbParticles, *modsQRS);
		}
		std::vector<S3DModelQRS*> particles = filterQRS->getParticleVector();
		
		IKSolverPFOrient<S3DModelQRS> iksol(particles, posNames, frame.getFrame());//Declaration of solver
		
		iksol.mapJointToObs(jtsToPos);
		iksol.initFilter();
		//viewer.init();
		
		//viewer.initModels(particles);
		//viewer.initObservations(fileParser->getJointNames(), frame);
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
				//viewer.update(particles, frame);
				//continuer = viewer.isRendering();
			}
			else if (step == "InitFilter")
			{
				filterQRS->init(frame);
				step = "Filter";
				//nbFrames--;
				//viewer.update(particles, frame);
				//continuer = viewer.isRendering();
				continuer = false;
			}
		}
	}
}

template<class Observations>
void FilterInt<Observations>::update(Observations frame)
{
	if ((filterType.compare("part") == 0) || (filterType.compare("partMMSE") == 0))
	{
		filter->step(frame);
	}
	else if ((filterType.compare("partQRS") == 0) || (filterType.compare("partMMSEQRS") == 0))
	{
		filterQRS->step(frame);

		//viewer.update(particles, frame);
		//continuer = viewer.isRendering();
	}
}

template<class Observations>
std::vector<std::vector<double> > FilterInt<Observations>::getPosture()
{
	std::vector<std::vector<double> > posture;
	
	std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > orient;
	std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > offset;
	std::map<std::string, int> map;
	
	if ((filterType.compare("part") == 0) || (filterType.compare("partMMSE") == 0))
	{
		//filter->getParticleVector().back() => get last particle = MMSE estimate of filter
		orient = filter->getParticleVector().back()->getOrientationVec();
		offset = filter->getParticleVector().back()->getOffsetVector();
		map = filter->getParticleVector().back()->getJointToIntMap();//Mapping between joint names and index in vector
		
		
		posture.resize(map.size());
		std::map<std::string, int>::iterator it;
		for (it=map.begin() ; it!=map.end() ; it++)
		{
			Eigen::Vector3d pos = filter->getParticleVector().back()->getJoint((*it).first)->getXYZVect();
			posture[(*it).second].push_back(pos[0]);
			posture[(*it).second].push_back(pos[1]);
			posture[(*it).second].push_back(pos[2]);
			posture[(*it).second].push_back(offset[(*it).second]->x());
			posture[(*it).second].push_back(offset[(*it).second]->y());
			posture[(*it).second].push_back(offset[(*it).second]->z());
			posture[(*it).second].push_back(orient[(*it).second]->w());
			posture[(*it).second].push_back(orient[(*it).second]->x());
			posture[(*it).second].push_back(orient[(*it).second]->y());
			posture[(*it).second].push_back(orient[(*it).second]->z());
		}
	}
	else if ((filterType.compare("partQRS") == 0) || (filterType.compare("partMMSEQRS") == 0))
	{
		orient = filterQRS->getParticleVector().back()->getOrientationVec();
		offset = filterQRS->getParticleVector().back()->getOffsetVector();
		map = filterQRS->getParticleVector().back()->getJointToIntMap();
		
		posture.resize(map.size());
		std::map<std::string, int>::iterator it;
		for (it=map.begin() ; it!=map.end() ; it++)
		{
			Eigen::Vector3d pos = filter->getParticleVector().back()->getJoint((*it).first)->getXYZVect();
			posture[(*it).second].push_back(pos[0]);//position 3D
			posture[(*it).second].push_back(pos[1]);
			posture[(*it).second].push_back(pos[2]);
			posture[(*it).second].push_back(offset[(*it).second]->x());//offset d'un joint par rapport à son parent
			posture[(*it).second].push_back(offset[(*it).second]->y());
			posture[(*it).second].push_back(offset[(*it).second]->z());
			posture[(*it).second].push_back(orient[(*it).second]->w());//orientation des joints
			posture[(*it).second].push_back(orient[(*it).second]->x());
			posture[(*it).second].push_back(orient[(*it).second]->y());
			posture[(*it).second].push_back(orient[(*it).second]->z());
		}
	}
	return posture;
}

template<class Observations>
bool FilterInt<Observations>::isEnvOk()
{
	return _env->confOk();
}

#endif
