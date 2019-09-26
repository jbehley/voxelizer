#include "Car.h"
#include <dirent.h>
#include <QtCore/QDir>
#include <fstream>
#include <iostream>

Car::~Car() {}

std::shared_ptr<std::map<std::string, Car>> Car::loadCarModels(const std::string& path) {
  auto dirp = opendir(path.c_str());  //"../cars"
  if (dirp == NULL) {
    std::cout << "Cars-Folder not found." << std::endl;
    return nullptr;
  }

  struct dirent* dp;
  auto loadedCars = std::make_shared<std::map<std::string, Car>>();
  while ((dp = readdir(dirp)) != NULL) {
    if (std::string(dp->d_name).length() < 4) continue;
    if (0 != std::string(dp->d_name).compare(std::string(dp->d_name).length() - 4, 4, ".xyz")) continue;

    std::basic_string<char> filepath = (std::string(path) + "/" + (dp->d_name)).c_str();

    std::ifstream infile(filepath);
    float x, y, z, a, b, c;

    std::shared_ptr<std::vector<glow::vec4>> v = std::make_shared<std::vector<glow::vec4>>();

    while (infile >> x >> y >> z >> a >> b >> c) {
      v->push_back(glow::vec4(x, y, z, 1));
    }
    if (v->size() == 0) {
      std::cout << filepath << ": File corrupted or empty. SKIP" << std::endl;
      continue;
    }
    std::cout << filepath << " loaded" << std::endl;

    loadedCars->insert(std::pair<std::string, Car>(dp->d_name, Car(std::string(dp->d_name), v)));
    // std::cout<<filepath<<": "<<v->size()<<" Points"<<std::endl;
  }
  (void)closedir(dirp);
  return loadedCars;
}

void Car::setPosition(Eigen::Matrix4f position_) { position = position_; };

Eigen::Matrix4f Car::getPosition() const { return position; }
std::string Car::getModel() const { return model; }
std::shared_ptr<std::vector<glow::vec4>> Car::getPoints() const { return points; }
std::shared_ptr<std::vector<glow::vec4>> Car::getGlobalPoints() const {
  auto gp = std::make_shared<std::vector<glow::vec4>>();
  for (const auto& value : *points) {
    Eigen::Vector4f v;
    v << value.x, value.y, value.z, 1;
    v = position * v;
    gp->push_back(glow::vec4(v.x(), v.y(), v.z(), 1));
  }
  return gp;
}
void Car::setInlier(uint32_t i) { inlier = i; }
uint32_t Car::getInlier() const { return inlier; }
