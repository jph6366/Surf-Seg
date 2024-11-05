#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include "utils/UnionFind.h"
#include "loader/ASCIILoader.h"
#include "utils/KDTreeFLANN.h"


int main(int, char**){

    std::vector<Triangle> triangles;
    readTriangles("/home/jphardee/Desktop/PointCloudLibrary/vertices.txt", triangles);

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

    // vtkNew means “I own this pointer, which must remain the same for my lifetime”,
    //  vtkSmartPointer means “I use this pointer, which came from somewhere else and might change,”
    vtkNew<vtkPoints> torus1;
    vtkNew<vtkPoints> torus2;
    vtkNew<vtkPoints> cylinder1;

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> surfaces;
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
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
