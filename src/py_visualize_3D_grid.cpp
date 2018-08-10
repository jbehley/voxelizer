#include <boost/python.hpp>
#include <iostream>

#include <matio.h>

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>

#include <data/voxelize_utils.h>
#include <widget/Mainframe.h>


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

void plot(const char* filename)
{

  std::cout << "Visualize 3D Grid New2" << std::endl;
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



  int argc = 1;
  char* n_argv[] = { "py_visualize_3D_grid"};
  char ** argv = n_argv;
  std::cout << argv[0] << std::endl;

  QApplication app(argc, argv); // TODO: This constructor expects argc and argv from the main function
  std::cout << "Mainframe" << std::endl;
  Mainframe frame(voxelGrid);
  frame.show();


  Viewport* widget;
  widget = frame.ui.mViewportXYZ;
  widget->show();
  std::cout << "Process Events" << std::endl;

// while(true) {QApplication::processEvents(); }
for ( int i = 0; i < 300000; i++){
  if (i % 100000 == 0) std::cout << i << std::endl;
  QApplication::processEvents();
  QApplication::processEvents();
  QApplication::processEvents();
  QApplication::processEvents();
  QApplication::processEvents();
  QApplication::processEvents();
}
  std::cout << "UpdateGL" << std::endl;
  widget->updateGL();
  frame.saveScreenshot();
}


BOOST_PYTHON_MODULE(py_visualize_3D_grid)
{
  // import_array();
  // boost::python::numeric::array::set_module_and_type("numpy", "ndarray");
  boost::python::def("plot",plot);
}
