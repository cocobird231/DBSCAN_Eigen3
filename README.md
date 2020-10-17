# DBSCAN_Eigen3
DBSCAN clustering algorithm based on Eigen3.

This code is to solve the clustering of 3D point cloud using C++, with help of Eigen, this code runs very fast!

To preserve each clustered group's boundary, this code didn't cluster the border points.

# Usage
Use vector of Eigen::Vector3f to store core points, border points and noises. The core points includes clustered label, so we use 2D vector of Eigen::Vector3f to store it. The first dimension of vector stores the points with same label.
```
std::vector<std::vector<Eigen::Vector3f> > corePoints
std::vector<Eigen::Vector3f> borderPoints
std::vector<Eigen::Vector3f> noises
```

Another way to store the points is using DBPoint, includes the point location, point type and clustered label.
```
std::vector<DBPoint> corePoints
std::vector<DBPoint> borderPoints
std::vector<DBPoint> noises
```

Calling function is simple, just call DBSCAN(), input raw point cloud and vector of core points, border points and noises. And don't forget the parameters of DBSCAN.
```
//The corePoints, borderPoints, noises can be the vector of Eigen::Vector3f or vector of DBPoint.
DBSCAN::DBSCAN(pointCloud, corePoints, borderPoints, noises, eps, MinPts);
```
