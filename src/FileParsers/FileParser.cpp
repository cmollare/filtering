#include "FileParser.h"

FileParser::FileParser(std::string path, std::string fileBeg, int number)
{
	mPath = path;
	mFileBeg = fileBeg;
	mMaxFileNumber = number;
	mNextFrame=0;
	
	cout << "Lecture des fichier Skel" << endl;
	
	for (int i=0 ; i<mMaxFileNumber ; i++)
		readFile(i);
		
	cout << "lecture des fichiers terminee" << endl;
		
}

void FileParser::readFile(int number)
{
	ostringstream oss;
	oss << mPath << mFileBeg << number << ".txt";
	std::ifstream file(oss.str().c_str(), ios::in);
	
	if (file)
	{
		string name, tracked, tempo;
		double x, y, z;
		vector<vector<double> > frame;
		vector<double> joint;
		
		for (int i=0 ; i<20 ; i++)
		{
			vector<double> joint;
			
			file >> name >> tempo >> x >> tempo >> y >> tempo >> z >> tempo >> tracked;
			
			if (number==0)
			{
				mJointNames.push_back(name);
			}
			
			if (tracked == "Tracked")
				joint.push_back(1);
			else
				joint.push_back(0);
				
			#ifdef NOISE
			x+= (double) this->noise(0.02)*0.02;
			y+= (double) this->noise(0.02)*0.02;
			z+= (double) this->noise(0.02)*0.02;
			
			joint.push_back(x);
			joint.push_back(y);
			joint.push_back(z);
			#else
			joint.push_back(x);
			joint.push_back(y);
			joint.push_back(z);
			#endif
			
			frame.push_back(joint);
			//cout << "name : " << joint[0] << " x : " << joint[1] << " y : " << joint[2] << " z : " << joint[3] << " tracked : " << tracked << endl;//*/
		}
		
		mVideoSequence.push_back(frame);
		
		file.close();
	}
	else
	{
		cout << "erreur de lecture du fichier : " << oss.str() << endl;
	}
}

std::vector<std::vector<double> > &FileParser::getFirstFrame()
{
	mNextFrame=1;

	return mVideoSequence[0];
}

std::vector<std::vector<double> > &FileParser::getNextFrame()
{
	int currentFrame=mNextFrame;
	mNextFrame++;
	if (mNextFrame>(mMaxFileNumber-1))
		mNextFrame=0;
	
	return mVideoSequence[currentFrame];
}

std::vector<std::vector<double> > &FileParser::getCurrentFrame()
{
	int currentFrame=mNextFrame;
	if (currentFrame<0)
		currentFrame=mMaxFileNumber-1;
	
	return mVideoSequence[currentFrame];
}

std::vector<std::string> FileParser::getJointNames()
{
	return mJointNames;
}

double FileParser::noise(double sigma)
{
	double U1, U2, X, Y;
	do
	{
		int U1int = rand()%10001;
		double U1 = (double)U1int / 10001.;
		int U2int = rand()%10001;
		double U2 = (double)U2int / 10001.;
		
		X = sqrt(-2*log(U1))*cos(2*3.14*U2);
		Y = sqrt(-2*log(U1))*sin(2*3.14*U2);
		X *= sigma;
		Y *= sigma;
	}
	while(X != X);
	return X;
}
