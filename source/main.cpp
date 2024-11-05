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
    // print the number of points in each unique surface
    for (const auto& entry : surfaceSizes) {
        std::cout << "Surface " << entry.first << " has " << entry.second << " points." << std::endl;
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

    for(int i = 0; i < triangles.size(); ++i) {
        if (surfaceUF.find(i) ==  *uniqueSurfaces.begin()) {
            const Triangle& tri = triangles[i];
            double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
            double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
            double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};
            cylinder1->InsertNextPoint(x);
            cylinder1->InsertNextPoint(y);
            cylinder1->InsertNextPoint(z);
            torus2->InsertNextPoint(x);
            torus2->InsertNextPoint(y);
            torus2->InsertNextPoint(z);
        } else if (surfaceUF.find(i) == *std::next(uniqueSurfaces.begin())) { 
            const Triangle& tri = triangles[i];
            double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
            double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
            double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};
            torus1->InsertNextPoint(x);
            torus1->InsertNextPoint(y);
            torus1->InsertNextPoint(z);
            cylinder1->InsertNextPoint(x);
            cylinder1->InsertNextPoint(y);
            cylinder1->InsertNextPoint(z);
        } else {
            const Triangle& tri = triangles[i];
            double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
            double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
            double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};
            torus1->InsertNextPoint(x);
            torus1->InsertNextPoint(y);
            torus1->InsertNextPoint(z);
            torus2->InsertNextPoint(x);
            torus2->InsertNextPoint(y);
            torus2->InsertNextPoint(z);
        }
    };

    vtkNew<vtkKdTree> kDTree_t1;
    vtkNew<vtkKdTree> kDTree_t2;
    vtkNew<vtkKdTree> kDTree_c1;

    buildTree(torus1, kDTree_t1);
    buildTree(torus2, kDTree_t2);
    buildTree(cylinder1, kDTree_c1);

    unsigned char r,g,b = 0;
    double NNDist;
    for(int i = 0; i < triangles.size(); ++i) {
        if (surfaceUF.find(i) ==  *uniqueSurfaces.begin()) {

            const Triangle& tri = triangles[i];    
            double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
            double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
            double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};    

            points->InsertNextPoint(x);
            NNDist = proximityFind(x, kDTree_t1);
            if (NNDist > 0) {
                // sigmoid function creates a more gradual gradient, 
                // emphasizing points close to the surface and creating 
                // a smooth transition for farther points.
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            unsigned char b = 255 - r;
            colors->InsertNextTuple3(r,g,b);

            points->InsertNextPoint(y);
            NNDist = proximityFind(y, kDTree_t1);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            b = 255 - r;
            colors->InsertNextTuple3(r,g,b);
            
            points->InsertNextPoint(z);
            NNDist = proximityFind(z, kDTree_t1);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            b = 255 - r;
            colors->InsertNextTuple3(r,g,b);

        } else if (surfaceUF.find(i) == *std::next(uniqueSurfaces.begin())) { 

            const Triangle& tri = triangles[i];    
            double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
            double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
            double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};    

            points->InsertNextPoint(x);
            NNDist = proximityFind(x, kDTree_t2);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 * (NNDist/20));
            } else {
                r = 0;
            };
            unsigned char b = 255 - r;
            colors->InsertNextTuple3(r,g,b);

            points->InsertNextPoint(y);
            NNDist = proximityFind(y, kDTree_t2);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            b = 255 - r;
            colors->InsertNextTuple3(r,g,b);
            
            points->InsertNextPoint(z);
            NNDist = proximityFind(z, kDTree_t2);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            b = 255 - r;
            colors->InsertNextTuple3(r,g,b);
        } else {
            const Triangle& tri = triangles[i];    
            double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
            double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
            double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};    

            points->InsertNextPoint(x);
            NNDist = proximityFind(x, kDTree_c1);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            unsigned char b = 255 - r;
            colors->InsertNextTuple3(r,g,b);

            points->InsertNextPoint(y);
            NNDist = proximityFind(y, kDTree_c1);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            b = 255 - r;
            colors->InsertNextTuple3(r,g,b);
            
            points->InsertNextPoint(z);
            NNDist = proximityFind(z, kDTree_c1);
            if (NNDist > 0) {
                r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
            } else {
                r = 0;
            };
            b = 255 - r;
            colors->InsertNextTuple3(r,g,b);
        }
    };


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
