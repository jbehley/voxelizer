# Voxelize point clouds

 Tool to voxelize annotated point clouds. 
 
## Dependencies

* catkin
* Eigen >= 3.2
* boost >= 1.54
* QT >= 5.2
* OpenGL >= 3.3
* [glow](https://github.com/jbehley/glow) (catkin package)
 
## Build
  
On Ubuntu 16.04, most of the dependencies can be installed from the package manager:
```bash
sudo apt install git libeigen3-dev libboost-all-dev qtbase5-dev libglew-dev catkin
```

Additionally, make sure you have [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) and the [fetch](https://github.com/Photogrammetry-Robotics-Bonn/catkin_tools_fetch) verb installed:
```bash
sudo apt install python-pip
sudo pip install catkin_tools catkin_tools_fetch empy
```

And Matio for reading / writing Matlab mat-files
```
sudo apt install p7zip-full

mkdir -p ~/libs/matio && cd ~/libs/matio
wget https://kent.dl.sourceforge.net/project/matio/matio/1.5.12/matio-1.5.12.7z
7z x matio-1.5.12.7z
cd matio-1.5.12
chmod +x configure
./configure
make -j4
make check
sudo make install

```


If you do not have a catkin workspace already, create one:
```bash
cd
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin init
cd src
git clone https://github.com/ros/catkin.git
```
Clone the repository in your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/jbehley/voxelizer.git
```
Download the additional dependencies:
```bash
catkin deps fetch
```
Then, build the project:
```bash
catkin build voxelizer
```
Now the project root directory (e.g. `~/catkin_ws/src/voxelizer`) should contain a `bin` directory containing the voxelizer.

## Usage


In the `bin` directory, just run `./voxelizer` to start the voxelizer. 


In the `settings.cfg` files you can change the followings options:

<pre>

max scans: 500    # number of scans to load for a tile. (should be maybe 1000), but this currently very memory consuming.
min range: 2.5    # minimum distance of points to consider.
max range: 50.0   # maximum distance of points in the point cloud.
ignore: 0,250,251,252,253,254  # label ids of labels that should be ignored when building a voxelgrid.

</pre>




 
## Folder structure

When loading a dataset, the data must be organized as follows:

<pre>
point cloud folder
├── velodyne/             -- directory containing ".bin" files with Velodyne point clouds.   
├── labels/               -- label directory, will be generated if not present.    
├── calib.txt             -- calibration of velodyne vs. camera. needed for projection of point cloud into camera.  
└── poses.txt             -- file containing the poses of every scan.
</pre>

 

## TODO

 See TODO.md.
