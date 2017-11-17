#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/vfh.h>

int main(int argc, char* argv[])
{
    if(argc != 3){
        std::cout <<"Usage: " << argv[0] << "Path2PCD Path2VFH" << std::endl;
        return 0;
    }
    //--Simple cloud creation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //--load pcd from h.d.d
    pcl::io::loadPCDFile (argv[1], *cloud);
    cloud->width = (int) cloud->points.size ();
    cloud->height = 1;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree_normal);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);


    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);
    pcl::io::savePCDFileASCII (argv[2], *vfhs);
    return 0;
}