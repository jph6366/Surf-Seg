#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/octree/octree_search.h>



void buildTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* tree) 
{
  tree->setInputCloud(cloud);
  tree->addPointsFromInputCloud();

}

double proximityFind(pcl::PointXYZ queryPoint, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* tree) {

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    tree->nearestKSearch(queryPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance)
    std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[0] ].x << " " << (*cloud)[ pointIdxNKNSearch[0] ].y << " " << (*cloud)[ pointIdxNKNSearch[0] ].z << std::endl;
    
    // d = √((x2 - x1)² + (y2 - y1)² + (z2 - z1)²)

    return pointNKNSquaredDistance[0];
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