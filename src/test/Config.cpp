#include "Config.h"

Config::Config(int argc, char ** argv)
{
	if (argc<=4)
		_confok = false;
	else
	{
		_confok = true;
		int numArg=1;
		
		while (numArg < argc && _confok)
		{
			std::string str(argv[numArg]);
			if (str.compare("-N") == 0) // number of particles
			{
				_nbParticles = atoi(argv[numArg+1]);
			}
			else if (str.compare("-type") == 0) //type of Filter
			{
				_typeFilter=argv[numArg+1];
				if ((_typeFilter.compare("part") != 0) && (_typeFilter.compare("partQRS") != 0) && (_typeFilter.compare("partMMSE") != 0) && (_typeFilter.compare("partMMSEQRS") != 0))
				{
					cout << "Unknown parameter : " << _typeFilter << endl;
					_confok=false;
					break;
				}
			}
			else
			{
				cout << "Unknown parameter" << endl;
				_confok=false;
				break;
			}
			
			numArg+=2;
		}
	}
	
	
	//cout << "argc = " << argc << endl;
}

void Config::printHelp()
{
	cout << "Please enter the following parameters :" << endl;
	cout << "-N <number of particules>" << endl;
	cout << "-type <type>" << endl;
	
	cout << "*************************************************************" << endl;
	cout << "<type> = \"part\", \"partQRS\", \"partMMSE\", \"partMMSEQRS\"" << endl;
}

void Config::printConf()
{
	cout << "***********************************************************" << endl;
	cout << "Starting filtering with " << _nbParticles << "particles" << endl;
	cout << "Filtering type : " << _typeFilter << endl;
	cout << "***********************************************************" << endl;
}

bool Config::confOk()
{
	return _confok;
}
