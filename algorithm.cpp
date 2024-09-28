#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <thread>
#include <pcl/common/angles.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <iostream>
#include <random>
#include <ctime>

using namespace std;

using namespace std::chrono_literals;


std::vector<pcl::PointXYZ> selected_points;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void find_path(pcl::visualization::PCLVisualizer *viewer){
	//SHOW INPUT BOX FOR WIDTH INPUT
	if(viewer->contains("Path")){
		viewer->removePointCloud("Path");
	}
  	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_input(input_cloud);
  	// viewer->addPointCloud<pcl::PointXYZRGB> (input_cloud, rgb_input, "Path");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Path");

}

void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    float x, y, z;
	std::cout<<"picking pt";
	pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    if (event.getPointIndex() == -1)
    {
        return;
    }
    event.getPoint(x, y, z);

	
	selected_points.push_back(pcl::PointXYZ(x, y, z));

	pcl::PointXYZRGB selection;

	selection.x = x;
	selection.y = y;
	selection.z = z;

	selection.r = 255;
	selection.g = 0;
	selection.b = 0;

	input_cloud->points.push_back(selection);


	if(viewer->contains("Input Points")){
		viewer->removePointCloud("Input Points");
	}
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_input(input_cloud);
  	viewer->addPointCloud<pcl::PointXYZRGB> (input_cloud, rgb_input, "Input Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Input Points");

    std::cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)

{

  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

  if (event.getKeySym () == "c" && event.keyDown ())

  {

    std::cout << "c was pressed => clearing all inputs" << std::endl;
	selected_points.clear();
	input_cloud->points.clear();
    

  }else if(event.getKeySym () == "g" && event.keyDown ()){
    std::cout << "g was pressed => finding path" << std::endl;
	find_path(viewer);
  }

}



pcl::visualization::PCLVisualizer::Ptr createView(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)

{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "Original Surface");
  

//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_path(path_cloud);
//   viewer->addPointCloud<pcl::PointXYZRGB> (path_cloud, rgb_path, "Path Points");

//   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Input Points");
  

//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  viewer->addCoordinateSystem (1.0);

  viewer->initCameraParameters ();

  viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)viewer.get());

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get());



  return (viewer);

}



int main(int argc, char** argv)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZ> ("input.pcd", *cloud) == -1)
    {
        
        PCL_ERROR ("Couldn't read file input.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
                <<cloud -> width * cloud-> height
                << " data points from input.pcd";

 
	pcl::visualization::PCLVisualizer::Ptr viewer = createView(cloud);

    while (!viewer->wasStopped ())

  	{
    	viewer->spinOnce (100);
    	std::this_thread::sleep_for(100ms);
  	}



    // // Step 3: Create a convex hull from these randomly selected points
    // pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::ConvexHull<pcl::PointXYZ> chull;
    // chull.setInputCloud(random_interior_points);
    // chull.setDimension(3);  // Set to 3D

    // std::vector<pcl::Vertices> hull_vertices;
    // chull.reconstruct(*convex_hull, hull_vertices);

    // // Step 4: Crop the original point cloud using the convex hull
    // pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    // cropHullFilter.setInputCloud(cloud);       // The original point cloud to crop
    // cropHullFilter.setHullCloud(convex_hull);  // The convex hull created from random points
    // cropHullFilter.setHullIndices(hull_vertices);
    // cropHullFilter.setDim(3);  // 3D cropping

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // cropHullFilter.filter(*cropped_cloud);

    // // Step 5: Save the cropped point cloud
    // pcl::io::savePCDFileASCII("cropped_cloud.pcd", *cropped_cloud);
    // std::cout << "Cropped surface saved" << std::endl;

    return 0;
}


// void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
// {
//     struct callback_args* data = (struct callback_args*)args;
//     if (event.getPointIndex() == -1)
//         return;
//     PointT current_point;
//     event.getPoint(current_point.x, current_point.y, current_point.z);
//     data->clicked_points_3d->clear();//将上次选的点清空
//     data->clicked_points_3d->points.push_back(current_point);//添加新选择的点
//     // 设置屏幕渲染属性，红色显示选择的点
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
//     data->viewerPtr->removePointCloud("clicked_points");
//     data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
//     data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
//     std::cout <<"点的坐标为:"<< "x="<<current_point.x << "y= " << current_point.y << "z= " << current_point.z << std::endl;