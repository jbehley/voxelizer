#include <stdint.h>
#include <iostream>
#include <matio.h>

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>

#include <QLabel>
#include <QDebug>

#include <data/voxelize_utils.h>
#include <widget/Mainframe.h>

#include <chrono>
#include <thread>


template<typename T>
void displayContentOfMatFile(unsigned xSize, T* xData, matvar_t* matVar){
    for(int i = 0; i < xSize; ++i)
    {
        if (xData[i] > 0) std::cout << "\tx[" << i <<"] = "<< xData[i] << "\n" ;
    }
    std::cout<<"\n" ;
    for(int i = 0; i< matVar->rank; ++i)
    {
        std::cout << "\tdim[" << i << "] == " << matVar->dims[i] << "\n" ;
    }
}

void readMatFile(const char* filename, int32_t*& xData, unsigned& xSize){
    // int32_t *xData;
    mat_t *mat = Mat_Open(filename, MAT_ACC_RDONLY);
    if(mat){
        matvar_t * matVar = Mat_VarRead(mat,"data");
        if (matVar){
            xSize = matVar->nbytes / matVar->data_size;
            xData = static_cast<int32_t*>(matVar->data);
            // displayContentOfMatFile(xSize, xData, matVar);
        }
        else{
            std::cout << "Error: Check if variable name is 'data' " << std::endl;
        }
    }
    else{
        std::cout << "Error: Could not open Mat file" << std::endl;
    }
}


int main(int argc, char** argv) {

  const char* filename = "../assets/voxelData.mat";
  int32_t* inVoxels; 
  unsigned numVoxels;
  readMatFile(filename, inVoxels, numVoxels);
  float voxelSize{0.6};                                       // size of a voxel
  Eigen::Vector4f minExtent{Eigen::Vector4f(0, -20, -2, 1)};  // minimum coordinate to consider for voxelgrid creation
  Eigen::Vector4f maxExtent{Eigen::Vector4f(40, 20, 1, 1)}; 
  VoxelGrid voxelGrid;
  voxelGrid.initialize(voxelSize, minExtent, maxExtent);
  voxelGrid.clear();
  fillVoxelGridMat(inVoxels, voxelGrid);
  std::cout << "Size VoxelGrid = " << voxelGrid.num_elements() << std::endl;



  QApplication app(argc, argv);
  Mainframe frame(voxelGrid);
  frame.show();

  Viewport* widget;
  widget = frame.ui.mViewportXYZ;
  widget->show();

  QApplication::processEvents();
  QApplication::processEvents();

  widget->updateGL();
  frame.saveScreenshot();

  return 0;
}



