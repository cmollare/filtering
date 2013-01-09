#ifdef FILTERINT_H

template<class Observations, class Particles>
FilterInt<Observations, Particles>::FilterInt(int argc, char ** argv)
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
	mods=NULL;
	viewer=NULL;
}

template<class Observations, class Particles>
FilterInt<Observations, Particles>::~FilterInt()
{
	if (filter) delete filter;
	if (mods) delete mods;
	if (viewer) delete viewer;
	
	delete _env;
}

template<class Observations, class Particles>
void FilterInt<Observations, Particles>::init(Observations firstFrame, std::vector<std::string>& posNames)
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
	

	viewer = new S3DViewer<Particles, Observations>();//Declaration of viewer
	viewer->setOptions(true, false, true);
	mods= new Particles(model);
	mods->mapJointToObs(posNames, jtsToPos);
		
		
	if (filterType.compare("partMMSE") == 0)
	{
		filter = new PartitionnedMMSE<Particles, Observations>(nbParticles, *mods);
	}
	else if (filterType.compare("part") == 0)
	{
		filter = new Partitionned<Particles, Observations>(nbParticles, *mods);
	}
	std::vector<Particles*> particles = filter->getParticleVector();
	
	IKSolverPFOrient<Particles> iksol(particles, posNames, frame.getFrame());//Declaration of solver
	
	iksol.mapJointToObs(jtsToPos);
	iksol.initFilter();
	viewer->init();
	
	viewer->initModels(particles);
	//viewer->initObservations(posNames, frame);
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
template<class Observations, class Particles>
void FilterInt<Observations, Particles>::update(Observations frame)
{
	filter->step(frame);
}

template<class Observations, class Particles>
std::vector<std::vector<double> > FilterInt<Observations, Particles>::getPosture()
{
	std::vector<std::vector<double> > posture;
	
	std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > orient;
	std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > offset;
	std::map<std::string, int> map;
	

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
	return posture;
}

template<class Observations, class Particles>
bool FilterInt<Observations, Particles>::isEnvOk()
{
	return _env->confOk();
}

#endif
