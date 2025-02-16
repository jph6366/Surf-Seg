# 3D Surface Segmentation

This is a repo containing code to to analyze a list of 3D triangles, find the number N of
disconnected closed surfaces and visualize the result as N surfaces where each surface is
rendered in a different color. I compare the use of Octrees and KDTrees to perform a 
nearest neighbor search using built-in VTK and PCL libraries.

## Install

CMake | https://cmake.org/download/
VTK | https://vtk.org/download/
PCL | https://pointclouds.org/downloads/


## Usage

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ..
make --debug=v
./surf-seg vertices.txt
```

## To Dos

- Incorporate PCL Octree
- Improve PCL Visualizer to integrate with Qt UI
- Union Find option to use Inverse Ackermann function
- Add New Spatial Data Partition Data Structures

## License

MIT © Jackson Hardee
