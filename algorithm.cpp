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
#include <Eigen/Dense>

using namespace std;

using namespace std::chrono_literals;

std::vector<int> selectedPoints;

std::set<int> perimeter;
std::vector<int> startLine;

double stepSize;
double width;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(double searchRadius)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch(0.05);

	ne.compute(*cloudNormals);

	return cloudNormals;
}
Eigen::Vector3f projectVector(Eigen::Vector3f dirVector, pcl::Normal pointNormal)
{
	Eigen::Vector3f normalVector(pointNormal.normal_x, pointNormal.normal_y, pointNormal.normal_z);

	return dirVector - (dirVector.dot(normalVector) / normalVector.dot(normalVector)) * normalVector;
}
double distEuc(pcl::PointXYZ pt1, pcl::PointXYZ pt2)
{
	return sqrt(
		(pt1.x - pt2.x) * (pt1.x - pt2.x) +
		(pt1.y - pt2.y) * (pt1.y - pt2.y) +
		(pt1.z - pt2.z) * (pt1.z - pt2.z));
}
void calculateStepSize(int numMoves = 150)
{
	std::cout << "Calculating step size";
	if (cloud->points.empty() || numMoves <= 0)
	{
		stepSize = 0;
		return;
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	pcl::PointXYZ currentPoint = cloud->points[0];
	std::set<int> visited;
	visited.insert(0);

	double cumulativeStep = 0.0;

	for (int move = 0; move < numMoves; ++move)
	{
		std::vector<int> nearestIndices(1);
		std::vector<float> nearestDistances(1);

		if (kdtree.nearestKSearch(currentPoint, 1, nearestIndices, nearestDistances) > 0)
		{
			int nearestIndex = nearestIndices[0];

			while (visited.find(nearestIndex) != visited.end() && visited.size() < cloud->points.size())
			{
				currentPoint = cloud->points[nearestIndex];
				kdtree.nearestKSearch(currentPoint, 1, nearestIndices, nearestDistances);
				nearestIndex = nearestIndices[0];
			}
			if (visited.find(nearestIndex) == visited.end())
			{
				pcl::PointXYZ nearestPoint = cloud->points[nearestIndex];
				double stepDistance = distEuc(nearestPoint, currentPoint);
				cumulativeStep += stepDistance;
				visited.insert(nearestIndex);
				currentPoint = nearestPoint;
			}
			else
			{
				break;
			}
		}
	}

	stepSize = visited.size() > 1 ? cumulativeStep / double(visited.size() - 1) : 0;
}

vector<int> findNearest(pcl::PointXYZ searchPoint, int K = 1)
{
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	{
		return pointIdxKNNSearch;
	}
	return std::vector<int>();
};

void findPerimeterLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr perimeterCloud,
					   pcl::PointCloud<pcl::Normal>::Ptr normals,
					   int index1,
					   int index2)
{
	pcl::PointXYZRGB rgbPt;
	pcl::PointXYZ pt1(cloud->points[index1]);
	pcl::PointXYZ pt2(cloud->points[index2]);
	while (distEuc(pt1, pt2) >= width)
	{

		Eigen::Vector3f direction(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
		Eigen::Vector3f projected = projectVector(direction, normals->points[index1]);
		pt1.x += projected.x();
		pt1.y += projected.y();
		pt1.z += projected.z();
		vector<int> newPtIdx = findNearest(pt1);
		pt1 = cloud->points[newPtIdx[0]];
		rgbPt.x = pt1.x;
		rgbPt.y = pt1.y;
		rgbPt.z = pt1.z;

		rgbPt.r = 0;
		rgbPt.g = 255;
		rgbPt.b = 0;
		perimeterCloud->points.push_back(rgbPt);
	}
}

void createPerimeter(pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl::Normal>::Ptr normals, double stepSize)

{
	std::cout << "Creating Perimeter";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr perimeterCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < selectedPoints.size(); ++i)
	{
		findPerimeterLine(perimeterCloud, normals, selectedPoints[i - 1], selectedPoints[i]);
	}
	std::cout << "Displaying Perimeter";
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbPerm(perimeterCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(inputCloud, rgbPerm, "Perimeter Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Perimeter Points");
}
void findPath(pcl::visualization::PCLVisualizer *viewer)
{
	std::cout << "Finding Path";

	if (viewer->contains("Path"))
	{
		viewer->removePointCloud("Path");
	}

	pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(0.05);

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");

	calculateStepSize();

	createPerimeter(viewer, normals, stepSize);

	std::vector<pcl::PointXYZ> tragectory;
	pcl::PointCloud<pcl::PointXYZRGB> tragCloud;
	std::vector<pcl::Normal> tragNormals;
}

void promptWidthInput(pcl::visualization::PCLVisualizer *viewer)
{
	std::cout << "Please enter the width value: ";
	std::cin >> width;
	std::cout << "You have entered width: " << width << std::endl;
}

int findPointIndex(float x, float y, float z)
{
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		const pcl::PointXYZ &point = cloud->points[i];
		if (point.x == x && point.y == y && point.z == z)
		{
			return i;
		}
	}
	return -1;
}

void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void *viewer_void)
{
	float x, y, z;
	std::cout << "picking pt";
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
	if (event.getPointIndex() == -1)
	{
		return;
	}
	event.getPoint(x, y, z);

	selectedPoints.push_back(findPointIndex(x, y, z));

	pcl::PointXYZRGB selection;

	selection.x = x;
	selection.y = y;
	selection.z = z;

	selection.r = 255;
	selection.g = 0;
	selection.b = 0;

	inputCloud->points.push_back(selection);

	if (viewer->contains("Input Points"))
	{
		viewer->removePointCloud("Input Points");
	}
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_input(inputCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(inputCloud, rgb_input, "Input Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Input Points");

	std::cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void)

{

	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
	if (event.getKeySym() == "c" && event.keyDown())

	{

		std::cout << "c was pressed => clearing all inputs" << std::endl;
		selectedPoints.clear();
		inputCloud->points.clear();
		if (viewer->contains("Input Points"))
		{
			viewer->removePointCloud("Input Points");
		}
	}
	else if (event.getKeySym() == "k" && event.keyDown())
	{
		std::cout << "k was pressed => finding path" << std::endl;
		if (selectedPoints.size() > 2)
		{
			promptWidthInput(viewer);
			std::cout << "calling findPath";
			findPath(viewer);
		}
		else
		{
			std::cout << selectedPoints.size() << " points were selected" << std::endl;
			std::cout << "Please select 3 or more points" << std::endl;
		}
	}
}

pcl::visualization::PCLVisualizer::Ptr createView(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)

{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Original Surface");

	viewer->addCoordinateSystem(1.0);

	viewer->initCameraParameters();

	viewer->addText("Choose points to create the boundary by using 'shift + left-mouse-button'", 10, 80, "instruction1");
	viewer->addText("Points should be chosen in the order that they will be connected to create the boundary'", 10, 60, "instruction2");
	viewer->addText("Press 'c' to clear selected points", 10, 40, "instruction3");
	viewer->addText("Press 'k' to confirm selected points and continue to width input", 10, 20, "instruction4");

	viewer->registerPointPickingCallback(pointPickingEventOccurred, (void *)viewer.get());

	viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)viewer.get());

	return (viewer);
}

int main(int argc, char **argv)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1)
	{

		PCL_ERROR("Couldn't read file input.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
			  << cloud->width * cloud->height
			  << " data points from input.pcd";

	pcl::visualization::PCLVisualizer::Ptr viewer = createView(cloud);

	while (!viewer->wasStopped())

	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

	return 0;
}
