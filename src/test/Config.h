#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <map>
#include <stdlib.h>

using namespace std;

class Config
{	
	public:
		Config(int argc, char ** argv);
		
		void printHelp();
		void printConf();
		bool confOk();
		std::string getFilterType();
		int getParticleNumber();
		
	protected:
		bool _confok;
		int _nbParticles;
		std::string _typeFilter;
		
};

#endif
