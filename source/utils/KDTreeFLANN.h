#include <vtkDataSetCollection.h>
#include <vtkKdTree.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTriangle.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>


void clearTree(vtkKdTree* tree) {
    tree->RemoveAllDataSets();
};

// The asterisk (*) used to declare a pointer goes 
// before the variable name
void buildTree(vtkPoints* points, vtkKdTree* tree) {
    tree->BuildLocatorFromPoints(points);
}

double proximityFind(double* queryPoint, vtkKdTree* tree) {
    double shortestDist;
    vtkIdType id = tree->FindClosestPoint(queryPoint, shortestDist);
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

void nearestNeighborColorMap(vtkPoints* points, vtkUnsignedCharArray* colors, vtkKdTree* kDTree, std::vector<Triangle> triangles, UnionFind uf, int surface ) {
        // print the number of points in each unique surface
    std::cout << "Surface " << surface << std::endl;
    std::vector<int> sameSet = uf.getAllInSameSet(surface);
    unsigned char r,g,b;
    double NNDist;
    std::cout << sameSet.size() << endl;
    for (int j : sameSet) {
        const Triangle& tri = triangles[j];
        double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
        double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
        double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z}; 

        points->InsertNextPoint(x);
        NNDist = proximityFind(x, kDTree);
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
        NNDist = proximityFind(y, kDTree);
        if (NNDist > 0) {
            r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
        } else {
            r = 0;
        };
        b = 255 - r;
        colors->InsertNextTuple3(r,g,b);
        
        points->InsertNextPoint(z);
        NNDist = proximityFind(z, kDTree);
        if (NNDist > 0) {
            r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
        } else {
            r = 0;
        };
        b = 255 - r;
        colors->InsertNextTuple3(r,g,b);
    }
}
