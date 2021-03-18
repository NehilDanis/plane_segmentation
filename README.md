# Plane Segmentation using PCL

![alt text](https://github.com/NehilDanis/plane_segmentation/raw/main/blob/example_gif.gif "segmentation example")

In this repository PCL library is used to segment the planer structures from a pointcloud. In my case I was
 trying to segment the table under the object.

The planar segmentation with RANSAC is used from PCL to detect the indices which represents the plane.
Later on the ExtractIndices object is generated to extract the indices which represent the plane from the 
input point cloud. 

Here are the steps of RANSAC plane detection method.

1) Select three random points from your point cloud that will form a plane.
2) Find the parameters of the plane.
3) Then using point to plane equation calculate the distance between the points in the point cloud to this plane
4) If the distance is within the specified threshold then these points will be considered as inliners.
5) Repeat the process max_iterations time.

Lastly the points which represents the plane and the object of interest is visualized 
using a PCL viewer.

## Requirements 
PCL library is required to be able to use this repo.

Cheers :)



