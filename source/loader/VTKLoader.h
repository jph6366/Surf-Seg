#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
// #include "../utils/OctreeFLANN.h"
#include "../utils/VTKKdTreeFLANN.h"
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
    std::cout << "Loaded " << surfaceSizes.size() << " surfaces\n";

    // vtkNew means “I own this pointer, which must remain the same for my lifetime”,
    //  vtkSmartPointer means “I use this pointer, which came from somewhere else and might change,”

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> surfaces;
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(surfaceSizes.size());
    colors->SetName("Colors");

    vtkNew<vtkKdTree> kDTree;
    for (const auto& entry : surfaceSizes) {
        vtkNew<vtkPoints> neighborPoints;
        initializeSpace(neighborPoints, triangles, surfaceUF, entry.first);
        buildTree(neighborPoints, kDTree);
        nearestNeighborColorMap(points, colors, kDTree, triangles, surfaceUF, entry.first);
        clearTree(kDTree);
    }


    for(size_t i = 0; i < triangles.size(); i++) {
        vtkNew<vtkTriangle> triangle;
        triangle->GetPointIds()->SetId(0, i*3);
        triangle->GetPointIds()->SetId(1, i*3 + 1);
        triangle->GetPointIds()->SetId(2, i*3 + 2);
        surfaces->InsertNextCell(triangle);
    };

    vtkNew<vtkPolyData> surfacePolyData;
    surfacePolyData->SetPoints(points);
    surfacePolyData->SetPolys(surfaces);
    surfacePolyData->GetPointData()->SetScalars(colors);

    vtkNew<vtkPolyDataMapper> surfaceMapper;
    surfaceMapper->SetInputData(surfacePolyData);

    vtkNew<vtkActor> surfaceActor;
    surfaceActor->SetMapper(surfaceMapper);

    // Set up Phong lighting model for the surface
    surfaceActor->GetProperty()->SetInterpolationToPhong();
    surfaceActor->GetProperty()->SetDiffuse(0.8);
    surfaceActor->GetProperty()->SetSpecular(0.5);
    surfaceActor->GetProperty()->SetSpecularPower(30.0);

    vtkNew<vtkRenderer> surfaceRenderer;
    vtkNew<vtkRenderWindow> surfaceRenderWindow;
    surfaceRenderWindow->AddRenderer(surfaceRenderer);
    vtkNew<vtkRenderWindowInteractor> surfaceRenderWindowInteractor;
    surfaceRenderWindowInteractor->SetRenderWindow(surfaceRenderWindow);
    surfaceRenderer->AddActor(surfaceActor);
    surfaceRenderer->SetBackground(0.1, 0.2, 0.3);
    surfaceRenderWindow->Render();
    surfaceRenderWindowInteractor->Start();
    
}