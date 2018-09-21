#include "CarReader.h"
#include <iostream>
#include <fstream>
#include <QtCore/QDir>
#include <dirent.h>



CarReader::CarReader(){

}

void CarReader::initialize(const QString& directory, std::shared_ptr<std::map<std::string, Car>> cars_){
	QDir base_dir(directory);
	autoauto_dir = base_dir.filePath("autoautos.txt").toStdString();
	//autoautos = autoautos_;
	cars = cars_;
}

void CarReader::load(std::vector<Car>& results){
	std::ifstream infile(autoauto_dir.c_str());
	for( std::string line; getline( infile, line ); )
	{
	    //std::cout<<line<<std::endl;
		Eigen::Matrix4f pose_;
		Eigen::Vector4f dir_;
		std::string ps;
		std::string model;

		std::istringstream ssline(line);
		ssline >>
			model >>
			pose_(0,0) >>
			pose_(1,0) >>
			pose_(2,0) >>
			pose_(0,1) >>
			pose_(1,1) >>
			pose_(2,1) >>
			pose_(0,2) >>
			pose_(1,2) >>
			pose_(2,2) >>
			pose_(0,3) >>
			pose_(1,3) >>
			pose_(2,3) >>
			dir_(0) >>
			dir_(1) >>
			dir_(2) >>
			ps;
		pose_(3,0) = 0;
		pose_(3,1) = 0;
		pose_(3,2) = 0;
		pose_(3,3) = 1;
		//selectedpts = pointStringToGlowVector(ps, pose_);
		//dir = dir_;
		Car c(cars->find(model)->second);
		c.setPosition(pose_);
		results.push_back(c);

		//std::cout<<ps<<std::endl;
	}
	infile.close();
}