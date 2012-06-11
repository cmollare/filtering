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
			
			if (tracked == "Tracked")
				joint.push_back(1);
			else
				joint.push_back(0);
			joint.push_back(x);
			joint.push_back(y);
			joint.push_back(z);
			
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

vector<vector<double> > &FileParser::getFirstFrame()
{
	mNextFrame=1;

	return mVideoSequence[0];
}

vector<vector<double> > &FileParser::getNextFrame()
{
	int currentFrame=mNextFrame;
	mNextFrame++;
	if (mNextFrame>(mMaxFileNumber-1))
		mNextFrame=0;
	
	return mVideoSequence[currentFrame];
}

vector<vector<double> > &FileParser::getCurrentFrame()
{
	int currentFrame=mNextFrame;
	if (currentFrame<0)
		currentFrame=mMaxFileNumber-1;
	
	return mVideoSequence[currentFrame];
}
