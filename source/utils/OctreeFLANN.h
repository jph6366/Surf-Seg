#include <vtkNew.h>
#include <vtkOctreePointLocator.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <cmath>  // For std::sqrt


void buildTree(vtkPoints* points, vtkOctreePointLocator* tree) 
{
    vtkNew<vtkPolyData> polydata;
    polydata->SetPoints(points);
    tree->SetDataSet(polydata);
    tree->BuildLocator();

}

double proximityFind(double* queryPoint, vtkKdTree* tree) {
    vtkIdType iD = tree->FindClosestPoint(queryPoint);

    double neighborPoint[3];
    tree->GetDataSet()->GetPoint(iD, neighborPoint);
    // d = √((x2 - x1)² + (y2 - y1)² + (z2 - z1)²)
    double shortestDist = std::sqrt(std::pow(queryPoint[0] - neighborPoint[0], 2) + std::pow(queryPoint[1] - neighborPoint[1], 2) + std::pow(queryPoint[2] - neighborPoint[2], 2));

    return shortestDist;
}

void initializeSpace(vtkPoints* points,  std::vector<Triangle> triangles, UnionFind uf, int surface ) {
    
    // print the number of points in each unique surface
    std::cout << "Surface " << surface << std::endl;
    std::vector<int> sameSet = uf.getAllNotInSameSet(surface);
    for (int j : sameSet) {
        const Triangle& tri = triangles[j];
        double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
        double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
        double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};  
        points->InsertNextPoint(x);
        points->InsertNextPoint(y);
        points->InsertNextPoint(z);
    }
}