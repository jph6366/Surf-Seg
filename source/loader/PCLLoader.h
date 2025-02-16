#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "../utils/PCLKdTreeFLANN.h"
#include "../utils/UnionFind.h"
#include "../spec/Triangle.h"



void readTriangles(const std::string& filename, std::vector<Triangle>& triangles) {
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "Failed to open file." << std::endl;
        return;
    }
    std::string line;
    while (std::getline(fin, line)) {
        std::stringstream s(line);
        std::vector<float> vertices;
        std::string word;
        while (std::getline(s, word, ',')) {
            vertices.push_back(std::stof(word));
        }
        if (vertices.size() == 9) {
            Vertex v0 {vertices[0], vertices[1], vertices[2]};
            Vertex v1 {vertices[3], vertices[4], vertices[5]};
            Vertex v2 {vertices[6], vertices[7], vertices[8]};

            triangles.push_back(Triangle(v0, v1, v2));
        } else {
            std::cerr << "Invalid line format: " << line << std::endl;
        }
    }
}


void renderVertices(const std::string& parseArg)
{
    std::vector<Triangle> triangles;
    readTriangles(parseArg, triangles);

    std::cout << "Loaded " << triangles.size() << " triangles\n";

    std::unordered_map<int, std::unordered_set<int>> adjacencyList;

    for (int i = 0; i < triangles.size(); ++i) {
        for (int j = i + 1; j < triangles.size(); ++j) {
            if (triangles[i].sharesVertexWith(triangles[j])) {
                adjacencyList[i].insert(j);
                adjacencyList[j].insert(i);
            }
        }
    }

    UnionFind surfaceUF;
    for (int i = 0; i < triangles.size(); ++i) {
        surfaceUF.add(i);
    }

    for (const auto& entry : adjacencyList) {
        int triIndex = entry.first;
        for (int neighborIndex : entry.second) {
            surfaceUF.unite(triIndex, neighborIndex);
        }
    }

    std::unordered_set<int> uniqueSurfaces;
    for (int i = 0; i < triangles.size(); ++i) {
        uniqueSurfaces.insert(surfaceUF.find(i));
    }

    // get the number of points in each unique surface
    std::unordered_map<int, int> surfaceSizes;
    for (int i = 0; i < triangles.size(); ++i) {
        surfaceSizes[surfaceUF.find(i)] += 3;
    }
    std::cout << "Loaded " << surfaceSizes.size() << " closed surfaces\n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& entry : surfaceSizes) {
        pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kDTree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighborPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
        initializeSpace(neighborPoints, triangles, surfaceUF, entry.first);
        buildTree(neighborPoints, kDTree);
        nearestNeighborColorMap(points, kDTree, triangles, surfaceUF, entry.first);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(points, "surface segmentation");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "surface segmentation");
    viewer->initCameraParameters();
    viewer->spin();
    
}