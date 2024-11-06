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

The included file vertices.txt contains a list of the triangles. Each line of the file represents a
triangle in ASCII CSV format using 9 floating point numbers to represent the three vertices of
the triangle as x0, y0, z0, x1, y1, z1, x2, y2, z2.

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Ninja" ..
ninja
./surf-seg  [path to txt file]
```

## Contributing

PRs accepted.

## License

MIT © Jackson Hardee
