#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <thread>
#include <pcl/common/angles.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_hull.h>

#include <pcl/console/parse.h>
#include <iostream>
#include <random>
#include <ctime>
#include <Eigen/Dense>

using namespace std;

using namespace std::chrono_literals;

/*
ASSUMPTIONS:

area to generally trapazoidal (for finding inwards position)






*/
std::vector<int> selectedPoints;

std::set<int> perimeter;
std::vector<int> startLine;

double stepSize;
double width;
bool orientation_ccw;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr perimeterCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::CropHull<pcl::PointXYZRGB> cropHullPerm;

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(double searchRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud(inCloud);

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

template <typename PointT>
double distEuc(PointT &pt1, PointT &pt2)
{
	return sqrt(
		(pt1.x - pt2.x) * (pt1.x - pt2.x) +
		(pt1.y - pt2.y) * (pt1.y - pt2.y) +
		(pt1.z - pt2.z) * (pt1.z - pt2.z));
}
void calculateStepSize(int numMoves = 150)
{
	std::cout << "Calculating step size" << std::endl;
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
		std::vector<int> nearestIndices(10);
		std::vector<float> nearestDistances(10);

		if (kdtree.nearestKSearch(currentPoint, 10, nearestIndices, nearestDistances) > 0)
		{
			int nearestIndex = nearestIndices[1];
			int i = 2;
			while (visited.find(nearestIndex) != visited.end() && visited.size() < cloud->points.size())
			{
				if (nearestIndices.size() > i)
				{
					nearestIndex = nearestIndices[i];
					i++;
				}
				else
				{
					break;
				}

				// Break if all points have been visited
				if (visited.size() == cloud->points.size())
				{
					std::cout << "All points have been visited." << std::endl;
					break;
				}
			}

			if (visited.find(nearestIndex) == visited.end())
			{
				pcl::PointXYZ nearestPoint = cloud->points[nearestIndex];
				double stepDistance = distEuc<pcl::PointXYZ>(nearestPoint, currentPoint);
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

	stepSize = (visited.size() > 1) ? (cumulativeStep / (visited.size() - 1)) : 0;
}

template <typename PointT>
vector<int> findNearest(pcl::PointXYZ searchPoint, typename pcl::PointCloud<PointT>::Ptr searchCloud, int K = 1)
{
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(searchCloud);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	{
		return pointIdxKNNSearch;
	}
	return std::vector<int>();
};

void findPerimeterLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr perimeterCloud,
					   pcl::PointCloud<pcl::Normal>::Ptr normals,
					   std::vector<int> &permIdxVec,
					   int index1,
					   int index2)
{
	pcl::PointXYZRGB rgbPt;
	pcl::PointXYZ pt1(cloud->points[index1]);
	pcl::PointXYZ pt2(cloud->points[index2]);
	permIdxVec.push_back(index1);
	int idx = index1;
	while (distEuc(pt1, pt2) > stepSize)
	{
		Eigen::Vector3f direction(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
		Eigen::Vector3f projected = projectVector(direction, normals->points[idx]).normalized();
		pt1.x += projected.x() * stepSize;
		pt1.y += projected.y() * stepSize;
		pt1.z += projected.z() * stepSize;
		vector<int> newPtIdx = findNearest<pcl::PointXYZ>(pt1, cloud);
		idx = newPtIdx[0];
		pt1 = cloud->points[idx];
		rgbPt.x = pt1.x;
		rgbPt.y = pt1.y;
		rgbPt.z = pt1.z;

		rgbPt.r = 255;
		rgbPt.g = 0;
		rgbPt.b = 0;

		// std::cout << "pt1.x" << pt1.x << "pt1.y" << pt1.y << "pt1.y" << pt1.y << std::endl;

		perimeterCloud->points.push_back(rgbPt);
		permIdxVec.push_back(idx);
		// std::cout << distEuc(pt1, pt2) << std::endl;
	}
}

void createPerimeter(pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl::Normal>::Ptr normals, std::vector<int> &permIdxVec)

{
	std::cout << "Creating Perimeter" << std::endl;
	for (int i = 1; i < selectedPoints.size(); i++)
	{
		findPerimeterLine(perimeterCloud, normals, permIdxVec, selectedPoints[i - 1], selectedPoints[i]);
	}
	findPerimeterLine(perimeterCloud, normals, permIdxVec, *(selectedPoints.end() - 1), selectedPoints[0]);

	std::cout << "Displaying Perimeter" << std::endl;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbPerm(perimeterCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(perimeterCloud, rgbPerm, "Perimeter Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "Perimeter Points");
}

Eigen::Vector3f findStepDir(Eigen::Vector3f permDir, Eigen::Vector3f normal)
{
	Eigen::Vector3f dir;
	//  || (!orientation_ccw && normal.z() < 0)
	// if ((orientation_ccw && normal.z() < 0) || (!orientation_ccw && normal.z() < 0))
	// {
	dir = (normal.cross(permDir)).normalized();
	// }
	// else
	// {
	// 	dir = ((-normal).cross(permDir)).normalized();
	// // }
	// std::cout << "Directoin vector x: " << dir.x() << " y: " << dir.y() << " z: " << dir.z() << std::endl;

	return dir;
}

template <typename PointT>
Eigen::Vector3f subtractPoints(PointT &a, PointT &b)
{
	Eigen::Vector3f solution;
	solution.x() = a.x - b.x;
	solution.y() = a.y - b.y;
	solution.z() = a.z - b.z;
	return solution;
}

void convertXYZRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
						pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

	for (size_t i = 0; input->points.size(); i++)
	{
		pcl::PointXYZ pt;
		pt.x = input->points[i].x;
		pt.y = input->points[i].y;
		pt.z = input->points[i].z;
		output->points.push_back(pt);
	}
}

int findPointInCloud(double x, double y, double z, pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud)
{

	// Iterate through the new cloud to find the point
	for (size_t i = 0; i < searchCloud->size(); ++i)
	{
		// Compare points (considering a threshold for floating point comparison)
		if (searchCloud->points[i].x == x &&
			searchCloud->points[i].y == y &&
			searchCloud->points[i].z == z)
		{
			return i; // Return the index in the new cloud
		}
	}

	std::cout << "Point not found in the new cloud." << std::endl;
	return -1; // Point not found
}

void generateGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloud,
				  std::vector<int> &permIdxVec)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr permCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	convertXYZRGBtoXYZ(perimeterCloud, permCloudXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
	// Calculate Hull based on perimeterCloud
	pcl::ConcaveHull<pcl::PointXYZ> hull_calculator;
	std::vector<pcl::Vertices> polygons; // This will hold the hull indices

	hull_calculator.setInputCloud(permCloudXYZ);  // Set the input as perimeterCloud
	hull_calculator.setAlpha(20);				  // Set the alpha value, adjust as needed for concave hull
	hull_calculator.reconstruct(*hull, polygons); // Compute the hull and get the polygons (hull indices)
	int dim = hull_calculator.getDimension();	  // Get the dimension of the hull (likely 3)

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCropped(new pcl::PointCloud<pcl::PointXYZ>);
	// Crop Hull using the calculated perimeterCloud hull
	pcl::CropHull<pcl::PointXYZ> crop_filter;
	crop_filter.setInputCloud(cloud);	  // Set the cloud you want to crop
	crop_filter.setHullCloud(hull);		  // Set the hull calculated from perimeterCloud
	crop_filter.setHullIndices(polygons); // Set the hull indices (polygons) from the hull calculation
	crop_filter.setDim(dim);			  // Set the dimension of the hull (3 for 3D)
	crop_filter.filter(*cloudCropped);

	pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(0.05, cloudCropped);

	std::cout << "inside create grid" << std::endl;
	std::set<int> areaCovered;
	std::set<int> permSet(permIdxVec.begin(), permIdxVec.end());
	// want permindex to  be ordered so I replicate the perimeter untill an "edge"
	// reminder i is the index of permIdxVec not the index in the cloud
	double offset;
	int currentCloudIdx;
	int newCloudIdx;
	pcl::PointXYZRGB addPt;
	pcl::PointXYZ startPt;
	Eigen::Vector3f permDir;
	pcl::PointXYZ searchPt;
	Eigen::Vector3f normal;
	Eigen::Vector3f stepDir;

	for (int i = 0; i < permIdxVec.size(); i++)
	{
		offset = 0;
		startPt = cloud->points[permIdxVec[i]];
		currentCloudIdx = findPointInCloud(startPt.x, startPt.y, startPt.z, cloudCropped);
		if (currentCloudIdx == -1)
		{
			continue;
		}
		if (i == 0)
		{
			permDir = subtractPoints<pcl::PointXYZ>(startPt, cloud->points[permIdxVec[permIdxVec.size() - 1]]) +
					  (subtractPoints<pcl::PointXYZ>(cloud->points[permIdxVec[i + 1]], startPt));
		}
		else if (i == permIdxVec.size() - 1)
		{
			permDir = subtractPoints<pcl::PointXYZ>(startPt, cloud->points[permIdxVec[i - 1]]) +
					  (subtractPoints<pcl::PointXYZ>(cloud->points[permIdxVec[0]], startPt));
		}
		else
		{
			permDir = subtractPoints<pcl::PointXYZ>(startPt, cloud->points[permIdxVec[i - 1]]) +
					  (subtractPoints<pcl::PointXYZ>(cloud->points[permIdxVec[i + 1]], startPt));
		}

		// if current point is on the perimeter or
		while (offset == 0 ||
			   (permSet.end() == permSet.find(currentCloudIdx) &&
				areaCovered.end() == areaCovered.find(currentCloudIdx)))
		{
			startPt = cloudCropped->points[newCloudIdx];

			normal.x() = normals->points[permIdxVec[i]].normal_x;
			normal.y() = normals->points[permIdxVec[i]].normal_y;
			normal.z() = normals->points[permIdxVec[i]].normal_z;

			if (startPt.x < 0)
			{
				stepDir = findStepDir(permDir, normal);
			}
			else
			{
				stepDir = findStepDir(permDir, -normal);
			}

			std::cout << "Directoin vector x: " << stepDir.x() << " y: " << stepDir.y() << " z: " << stepDir.z() << std::endl;

			searchPt.x = startPt.x + stepDir.x() * stepSize;
			searchPt.y = startPt.y + stepDir.y() * stepSize;
			searchPt.z = startPt.z + stepDir.z() * stepSize;

			std::cout << "searchpt vector x: " << searchPt.x << " y: " << searchPt.y << " z: " << searchPt.z << std::endl;

			newCloudIdx = findNearest<pcl::PointXYZ>(searchPt, cloudCropped)[0];

			// find new which is the closest point within step size in thedirection above
			// add new - current distance to offset
			offset += distEuc<pcl::PointXYZ>(cloudCropped->points[newCloudIdx], cloudCropped->points[currentCloudIdx]);
			if (offset >= width)
			{

				addPt.x = startPt.x;
				addPt.y = startPt.y;
				addPt.z = startPt.z;

				addPt.r = 0;
				addPt.g = 255;
				addPt.b = 0;

				gridCloud->points.push_back(addPt);
				offset = 0;
			}
			areaCovered.insert(currentCloudIdx);
			currentCloudIdx = newCloudIdx;
		}
	}
	std::cout << "grid created" << gridCloud->points.size() << std::endl;
}

void findPath(pcl::visualization::PCLVisualizer *viewer)
{
	std::cout << "Finding Path" << std::endl;

	if (viewer->contains("Path"))
	{
		viewer->removePointCloud("Path");
	}

	pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(0.05, cloud);

	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");

	calculateStepSize();
	std::cout << "Step Size: " << stepSize << std::endl;
	std::vector<int> permIdxVec;
	createPerimeter(viewer, normals, permIdxVec);

	std::cout << "Create Grid" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	generateGrid(gridCloud, permIdxVec);

	std::cout << "Displayy grid" << std::endl;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbGrid(gridCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(gridCloud, rgbGrid, "Grid Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "Grid Points");

	std::vector<pcl::PointXYZRGB> tragectory;
	pcl::PointCloud<pcl::PointXYZRGB> tragCloud;
	std::vector<pcl::Normal> tragNormals;
}

void promptInput(pcl::visualization::PCLVisualizer *viewer)
{
	int orientation;
	std::cout << "Please enter the width value: ";
	std::cin >> width;
	std::cout << "You have entered width: " << width << std::endl;
	std::cout << "Were the points selected clockwise (enter 0) or counterclockwise (enter 1): ";
	std::cin >> orientation;
	if (orientation == 1)
	{

		std::cout << "Points selected CCW" << std::endl;
		orientation_ccw = true;
		findPath(viewer);
	}
	else if (orientation == 0)
	{
		std::cout << "Points selected CW" << std::endl;
		orientation_ccw = false;
		findPath(viewer);
	}
	else
	{
		std::cout << "Invalid orientation, try again" << std::endl;
		promptInput(viewer);
	}
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

	selection.r = 0;
	selection.g = 0;
	selection.b = 255;

	inputCloud->points.push_back(selection);

	if (viewer->contains("Input Points"))
	{
		viewer->removePointCloud("Input Points");
	}
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_input(inputCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(inputCloud, rgb_input, "Input Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "Input Points");

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
			promptInput(viewer);
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
