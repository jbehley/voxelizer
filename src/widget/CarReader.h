#ifndef CAR_READER_H_
#define CAR_READER_H_

#include "Car.h"
#include <iostream>
#include <fstream>
#include <QtCore/QDir>

class CarReader{
public:
	CarReader();
	void initialize(const QString& directory, std::shared_ptr<std::map<std::string, Car>> cars_);
	//void save();
	void load(std::vector<Car>& results);
private:
	//std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> autoautos;
	std::string autoauto_dir;
	std::shared_ptr<std::map<std::string, Car>> cars;

};

#endif
