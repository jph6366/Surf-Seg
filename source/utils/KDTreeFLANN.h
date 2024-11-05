#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

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


void clearTree(vtkKdTree tree) {
    tree.RemoveAllDataSets();
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

