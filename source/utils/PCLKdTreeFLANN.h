#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.12/pcl/PolygonMesh.h>
#include <pcl-1.12/pcl/Vertices.h>
#include <pcl-1.12/pcl/visualization/pcl_visualizer.h>
#include "../spec/Triangle.h"
#include "../utils/UnionFind.h"


void clearTree(pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree) 
{
    tree->setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()));
}

void buildTree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree) 
{
    tree->setInputCloud(cloud);
}

double proximityFind(pcl::PointXYZRGB queryPoint, pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree) {

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    tree->nearestKSearch(queryPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    
    // d = √((x2 - x1)² + (y2 - y1)² + (z2 - z1)²)

    return pointNKNSquaredDistance[0];
}

void initializeSpace(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,  std::vector<Triangle> triangles, UnionFind uf, int surface ) {
    
    // print the number of points in each unique surface
    std::cout << "Get all not in Surface " << surface <<  std::endl;
    std::vector<int> sameSet = uf.getAllNotInSameSet(surface);
    for (int j : sameSet) {
        const Triangle& tri = triangles[j];
        double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
        double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
        double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z};
        pcl::PointXYZRGB point;
        point.x = x[0];
        point.y = x[1];
        point.z = x[2];  
        points->push_back(point);
        point.x = y[0];
        point.y = y[1];
        point.z = y[2];  
        points->push_back(point);
        point.x = z[0];
        point.y = z[1];
        point.z = z[2];  
        points->push_back(point);
    }
}


void nearestNeighborColorMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kDtree, std::vector<Triangle> triangles, UnionFind uf, int surface ) {
        // print the number of points in each unique surface
    std::cout << "FLANN on Surface " << surface << std::endl;
    std::vector<int> sameSet = uf.getAllInSameSet(surface);
    unsigned char r,g,b;
    double NNDist;
    std::cout << "Triangles in Surface " << sameSet.size() << endl;
    for (int j : sameSet) {
        const Triangle& tri = triangles[j];
        double x[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
        double y[3] = {tri.v2.x, tri.v2.y, tri.v2.z};
        double z[3] = {tri.v3.x, tri.v3.y, tri.v3.z}; 

        pcl::PointXYZRGB pointx;
        pointx.x = x[0];
        pointx.y = x[1];
        pointx.z = x[2];
        NNDist = proximityFind(pointx, kDtree);
        if (NNDist > 0) {
            // sigmoid function creates a more gradual gradient, 
            // emphasizing points close to the surface and creating 
            // a smooth transition for farther points.
            r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
        } else {
            r = 0;
        };
        unsigned char b = 255 - r;
        pointx.r = r; //  the data array where all points of type PointT are stored.
        pointx.g = g;
        pointx.b = b;
        points->push_back(pointx);



        pcl::PointXYZRGB pointy;
        pointy.x = y[0];
        pointy.y = y[1];
        pointy.z = y[2];
        NNDist = proximityFind(pointy, kDtree);
        if (NNDist > 0) {
            r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
        } else {
            r = 0;
        };
        b = 255 - r;
        pointy.r = r; //  the data array where all points of type PointT are stored.
        pointy.g = g;
        pointy.b = b;
        points->push_back(pointy);


        pcl::PointXYZRGB pointz;
        pointz.x = z[0];
        pointz.y = z[1];
        pointz.z = z[2];
        points->push_back(pointz);        
        NNDist = proximityFind(pointz, kDtree);
        if (NNDist > 0) {
            r = static_cast<unsigned char>(255 / (1 + exp(-0.1 * (NNDist - 100))));
        } else {
            r = 0;
        };
        b = 255 - r;
        pointz.r = r; //  the data array where all points of type PointT are stored.
        pointz.g = g;
        pointz.b = b;  
        points->push_back(pointz);        
    }
}
