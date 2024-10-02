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
space is not a critical issue





*/
std::vector<int> selectedPoints;

std::vector<int> startLine;

double stepSize;
double width;
bool orientation_ccw;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>); // for visualization only
pcl::PointCloud<pcl::PointXY>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXY>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

/**
 * @brief Structure to hold a trajectory point.
 *
 * Contains the index of the point, its coordinates, and the normal.
 */
struct TrajPt
{
	int index;
	pcl::PointXYZ point;
	pcl::Normal normal;

	/**
	 * @brief Constructor for the TrajPt structure.
	 *
	 * @param i Index of the point.
	 * @param pt The point's coordinates.
	 * @param norm The point's normal.
	 */
	TrajPt(int i, pcl::PointXYZ pt, pcl::Normal norm) : index(i), point(pt), normal(norm) {}
};

/**
 * @brief Computes normals for the input point cloud.
 *
 * This function uses a KdTree to search for neighboring points and compute the normal vectors.
 *
 * @param searchRadius The radius used to search for neighboring points.
 * @param inCloud Input point cloud.
 * @param cloudNormals Output cloud of computed normals.
 */
void computeNormals(double searchRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud(inCloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setSearchMethod(tree);

	ne.setRadiusSearch(0.05);

	ne.compute(*cloudNormals);
}

/**
 * @brief Projects a vector onto a 2D plane defined by a normal.
 *
 * @param dirVector Direction vector to project.
 * @param pointNormal The normal of the point defining the projection plane.
 * @return The projected vector.
 */
Eigen::Vector3f projectVector(Eigen::Vector3f dirVector, pcl::Normal pointNormal)
{
	Eigen::Vector3f normalVector(pointNormal.normal_x, pointNormal.normal_y, pointNormal.normal_z);

	return dirVector - (dirVector.dot(normalVector) / normalVector.dot(normalVector)) * normalVector;
}

/**
 * @brief Computes the Euclidean distance between two 3D points.
 *
 * This function calculates the distance between two 3D points using their x, y, and z coordinates.
 *
 * @tparam PointT Type of the point (should have x, y, z fields).
 * @param pt1 First point.
 * @param pt2 Second point.
 * @return The Euclidean distance as a double.
 */
template <typename PointT>
double distEuc(const PointT &pt1, const PointT &pt2)
{
	return sqrt(
		(pt1.x - pt2.x) * (pt1.x - pt2.x) +
		(pt1.y - pt2.y) * (pt1.y - pt2.y) +
		(pt1.z - pt2.z) * (pt1.z - pt2.z));
}

/**
 * @brief Computes the Euclidean distance between two 2D points.
 *
 * This specialization of distEuc computes the distance between 2D points ussing x and y coordinates.
 *
 * @param pt1 First 2D point.
 * @param pt2 Second 2D point.
 * @return The Euclidean distance as a double.
 */
template <>
double distEuc<pcl::PointXY>(const pcl::PointXY &pt1, const pcl::PointXY &pt2)
{
	return sqrt(
		(pt1.x - pt2.x) * (pt1.x - pt2.x) +
		(pt1.y - pt2.y) * (pt1.y - pt2.y));
}

/**
 * @brief Finds the nearest K points to a given point in a point cloud.
 *
 * @tparam PointT Type of the points in the cloud.
 * @param searchPoint The point to find neighbors for.
 * @param searchCloud The point cloud to search within.
 * @param K The number of nearest neighbors to find.
 * @return A vector of indices of the nearest points.
 */
template <typename PointT>
vector<int> findNearest(PointT searchPoint, typename pcl::PointCloud<PointT>::Ptr searchCloud, int K = 1)
{
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(searchCloud);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	{
		return pointIdxKNNSearch;
	}
	return std::vector<int>();
};

bool checkIntersection(pcl::PointXY p, pcl::PointXY a, pcl::PointXY b)
{
	cout << "point in poly" << endl;
	cout << "p.x: " << p.x << "p.y: " << p.y << endl;
	cout << "a.x: " << a.x << "a.y: " << a.y << endl;
	cout << "b.x: " << b.x << "b.y: " << b.y << endl;
	// horizontal rray from, -infinity
	// check is ray crosses above/below line
	if (p.y < min(a.y, b.y) || p.y > max(a.y, b.y))
	{
		return false;
	}
	// chekc is point is far left or right of line
	else if (p.x <= min(a.x, b.x))
	{
		return false;
	}
	// add = in both cases because if its on the boundary its either on the line (we dont care if
	// it is included or not) or its on the boundary meaning it is technically left or right of line
	// if m=0
	else if (p.x >= max(a.x, b.x))
	{
		cout << "1" << endl;
		return true;
	}

	// at this point the point is within the boundarys of the rectrangle created by line ab

	// use y = mx+d
	int m;
	m = (a.y - b.y) / (a.x - b.x); // we knwo den wont be 0 because of above checks
	double d = a.y - m * a.x;
	// no check for y = mx+b becasuse we dont care if its on the line
	// because if on line we can make an argument that it is both inside and outside so doesnt matter
	if (m > 0)
	{
		if (p.y < m * p.x + d)
		{
			cout << "2" << endl;
			return true;
		}
		else // y > mx+b
		{
			return false;
		}
	}
	else // m < 0
	{
		if (p.y < m * p.x + d)
		{
			return false;
		}
		else
		{
			cout << "3" << endl;
			return true;
		}
	}
}

bool pointInPoly(int pointIdx, vector<int> &poly)
{
	cout << "checking point in poly" << endl;
	bool inside = false;
	pcl::PointXY pt;
	pt.x = cloud2d->points[pointIdx].x;
	pt.y = cloud2d->points[pointIdx].y;

	for (int i = 0; i < poly.size(); i++)
	{
		pcl::PointXY checkPt;
		checkPt.x = cloud2d->points[pointIdx].x;
		checkPt.y = cloud2d->points[pointIdx].y;

		pcl::PointXY pt1;
		pcl::PointXY pt2;
		pt2.x = cloud2d->points[poly[i]].x;
		pt2.y = cloud2d->points[poly[i]].y;

		if (i == 0)
		{
			pt1.x = cloud2d->points[*(poly.end() - 1)].x;
			pt1.y = cloud2d->points[*(poly.end() - 1)].y;
		}
		else
		{
			pt1.x = cloud2d->points[poly[i - 1]].x;
			pt1.y = cloud2d->points[poly[i - 1]].y;
		}
		// check if ray interesct the line, if it does change the inside boolean
		if (checkIntersection(checkPt, pt1, pt2))
		{
			inside = !inside;
			cout << "ppoint in poly" << inside << endl;
		}
	}
	return inside;
}

void create2dCloud()
{
	for (pcl::PointXYZ pt3d : cloud->points)
	{
		pcl::PointXY pt2d;
		pt2d.x = pt3d.x;
		pt2d.y = pt3d.y;
		cloud2d->points.push_back(pt2d);
	}
}

/**
 * @brief Calculates the step size for the zigzag path based on neighboring point distances.
 *
 * @param numMoves Number of steps to average over for computing the step size.
 */
void calculateStepSize(int numMoves = 150)
{
	std::cout << "Calculating step size" << std::endl;
	if (cloud2d->points.empty() || numMoves <= 0)
	{
		std::cout << "cloud2d empty" << std::endl;
		stepSize = 0;
		return;
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cloud2d);

	pcl::PointXY currentPoint = cloud2d->points[0];
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
				pcl::PointXY nearestPoint = cloud2d->points[nearestIndex];
				double stepDistance = distEuc<pcl::PointXY>(nearestPoint, currentPoint);
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

void connectPts(vector<TrajPt> &output,
				pcl::PointXY &pt1,
				pcl::PointXY &pt2)
{
	Eigen::Vector2f dir;
	dir.x() = pt2.x - pt1.x;
	dir.y() = pt2.y - pt1.y;

	// this connects the points exluding pt1 and pt2

	for (int i = 0; i * stepSize < dir.norm(); i++)
	{
		pcl::PointXY searchPt;
		searchPt.x = pt1.x + stepSize * i * dir.normalized().x();
		searchPt.y = pt1.y + stepSize * i * dir.normalized().y();
		int nearest = findNearest<pcl::PointXY>(searchPt, cloud2d)[0];

		if (output.back().index != nearest)
		{
			TrajPt trajPt(nearest, cloud->points[nearest], normals->points[nearest]);
			output.push_back(trajPt);
		}
	}
}

void addTrajPt(int index,
			   vector<TrajPt> &output,
			   Eigen::Vector2f &dir,

			   bool addToFront = false)
{
	pcl::PointXY prevPt;
	int prevNearest = 0;
	int nearest = index;

	double gap = 0;
	// if the gap  > width and the nearest (aka previous index which you be within width)
	// is in the polygon add the point toth output vector
	while (gap < width)
	{
		prevPt.x = cloud2d->points[nearest].x;
		prevPt.y = cloud2d->points[nearest].y;

		pcl::PointXY searchPt;
		searchPt.x = prevPt.x + stepSize * dir.normalized().x();
		searchPt.y = prevPt.y + stepSize * dir.normalized().y();
		prevNearest = nearest;
		nearest = findNearest<pcl::PointXY>(searchPt, cloud2d)[0];

		gap += sqrt(pow(cloud2d->points[nearest].x - prevPt.x, 2) +
					pow(cloud2d->points[nearest].y - prevPt.y, 2)) /
			   normals->points[nearest].normal_z; // using the z of the normal project distance into 3D
	}
	if (pointInPoly(nearest, selectedPoints))
	{
		TrajPt trajPt(prevNearest, cloud->points[prevNearest], normals->points[prevNearest]);
		if (addToFront)
		{
			output.insert(output.begin(), trajPt);
		}
		else
		{
			output.push_back(trajPt);
			cout << "Point added to output: " << trajPt.index << endl;
		}
	}
}

void checkWiden(vector<TrajPt> &line, Eigen::Vector2f &gridLineDir)
{
	// traverse parrellel to the line away from the back until the end of the line is within a distance of 'width' from the edge;
	int count = line.size() - 1;
	while (line.size() > count)
	{
		count++;
		addTrajPt(line.back().index, line, gridLineDir);
	}

	// do the ssame for the front of the line
	count = line.size() - 1;

	while (line.size() > count)
	{
		Eigen::Vector2f oppGridLineDir(-gridLineDir.x(), -gridLineDir.y());
		addTrajPt(line.front().index, line, oppGridLineDir, true);
	}
}

void createGrid(vector<vector<TrajPt>> &output,
				vector<TrajPt> &initialLine,
				Eigen::Vector2f &gridNormal,
				Eigen::Vector2f &gridLineDir)
{
	// for each point in the line find the next point that is normal to the grid line that is the last point
	// within the width
	output.emplace_back();
	for (TrajPt trajPt : initialLine)
	{
		addTrajPt(trajPt.index, output[0], gridNormal);
	}
	checkWiden(output.back(), gridLineDir);

	// while the last vector had points inside the polygon add another gridline
	while (!output.back().empty())
	{
		output.emplace_back();
		for (TrajPt trajPt : *(output.end() - 1))
		{
			addTrajPt(trajPt.index, output.back(), gridNormal);
		}
	}

	if (output.empty())
	{
		output.push_back(initialLine);
	}
}

void connectTrajectory(vector<TrajPt> &vecTrajectory, vector<vector<TrajPt>> &vecGrid)
{
	for (TrajPt trajPt : vecGrid[0])
	{
		vecTrajectory.push_back(trajPt);
	}
	for (int i = 1; i < vecGrid.size(); i++)
	{
		// if even connect by right side, if odd connect by left
		pcl::PointXY gridPt1;
		pcl::PointXY gridPt2;
		if (i % 2 == 0)
		{
			gridPt1.x = vecGrid[i - 1].front().point.x;
			gridPt1.y = vecGrid[i - 1].front().point.y;

			gridPt2.x = vecGrid[i].front().point.x;
			gridPt2.y = vecGrid[i].front().point.y;
		}
		else
		{

			gridPt1.x = vecGrid[i - 1].back().point.x;
			gridPt1.y = vecGrid[i - 1].back().point.y;

			gridPt2.x = vecGrid[i].back().point.x;
			gridPt2.y = vecGrid[i].back().point.y;
		}
		connectPts(vecTrajectory, gridPt1, gridPt2);
		// add the grid line to the vecTrajectory
		for (TrajPt trajPt : vecGrid[i])
		{
			vecTrajectory.push_back(trajPt);
		}
	}
}

/**
 * @brief Finds the path and visualizes it in the PCL viewer.
 *
 * This function computes the normals, generates a 2D point cloud, calculates the step size,
 * and constructs the trajectory grid. It then connects the grid lines into a trajectory
 * and visualizes the trajectory points in green.
 *
 * @param viewer The PCLVisualizer pointer to display the trajectory.
 */
void findPath(pcl::visualization::PCLVisualizer *viewer)
{
	std::cout << "Finding Path" << std::endl;

	computeNormals(0.05, cloud, normals);

	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");

	// create  a 2d point cloud on plane (0, 0, 1), working in 2D will make the search algorithms fasteer
	// ccloud2d indicies will match cloud
	create2dCloud();

	// create an estimate for the step size between points in the 2D plane
	// this will determine distance to step from a specific to find the next point
	calculateStepSize();
	std::cout << "Step Size: " << stepSize << std::endl;

	// direction the grid will progress in normal to the initial line
	Eigen::Vector2f gridNormal;

	gridNormal.x() = cloud->points[selectedPoints[0]].x - cloud->points[selectedPoints[1]].x;
	gridNormal.y() = cloud->points[selectedPoints[1]].y - cloud->points[selectedPoints[0]].y;

	Eigen::Vector2f gridLineDir;

	gridLineDir.x() = -gridNormal.x();
	gridLineDir.y() = gridNormal.y();

	// find a line between the first two points to create the Trajectory grid from

	vector<TrajPt> initialLine;
	// add points to start and eend of line as connectPts for versitility reasons does not add the start and end pts
	TrajPt trajPt1(selectedPoints[0], cloud->points[selectedPoints[0]], normals->points[selectedPoints[0]]);
	initialLine.push_back(trajPt1);
	cout << "coonnectiong initial points" << endl;
	connectPts(initialLine, cloud2d->points[selectedPoints[0]], cloud2d->points[selectedPoints[1]]);

	if (initialLine.back().index != selectedPoints[1])
	{

		TrajPt trajPt2(selectedPoints[1], cloud->points[selectedPoints[1]], normals->points[selectedPoints[1]]);
		initialLine.push_back(trajPt2);
	}

	// create a set of lines that coves the surface
	vector<vector<TrajPt>> vecGrid;
	cout << "creating grid" << endl;
	createGrid(vecGrid, initialLine, gridNormal, gridLineDir);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectoryCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (vector<TrajPt> vect : vecGrid)
	{
		for (TrajPt trajPt : vect)
		{
			pcl::PointXYZRGB pt;

			pt.x = trajPt.point.x;
			pt.y = trajPt.point.y;
			pt.z = trajPt.point.z;

			pt.r = 0;
			pt.g = 255;
			pt.b = 0;

			trajectoryCloud->points.push_back(pt);
		}
	}

	if (trajectoryCloud->points.empty())
	{
		cout << "cloud empty" << endl;
	}
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTraj(trajectoryCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(trajectoryCloud, rgbTraj, "Trajectory Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTraj(trajectoryCloud);
	// viewer->addPointCloud<pcl::PointXYZRGB>(trajectoryCloud, rgbTraj, "Trajectory Points");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

	// // connect the lines into one Trajectory
	// // vecTrajectory gives the indices of the Trajectory in order
	// vector<TrajPt> vecTrajectory;
	// cout << "connecting tragectory" << endl;
	// connectTrajectory(vecTrajectory, vecGrid);

	// // populate 3d Trajectory into PointXYZRGB pointcloud for visualization

	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectoryCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// for (TrajPt trajPt : vecTrajectory)
	// {
	// 	pcl::PointXYZRGB pt;

	// 	pt.x = trajPt.point.x;
	// 	pt.y = trajPt.point.y;
	// 	pt.z = trajPt.point.z;

	// 	pt.r = 0;
	// 	pt.g = 255;
	// 	pt.b = 0;

	// 	trajectoryCloud->points.push_back(pt);
	// }

	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTraj(trajectoryCloud);
	// viewer->addPointCloud<pcl::PointXYZRGB>(trajectoryCloud, rgbTraj, "Trajectory Points");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);
}

/**
 * @brief Prompts the user to input the width value and initiates the path-finding process.
 *
 * This function collects user input for width, displays the entered value,
 * and then calls the findPath function.
 *
 * @param viewer The PCLVisualizer pointer to display the path.
 */
void promptInput(pcl::visualization::PCLVisualizer *viewer)
{
	int orientation;
	std::cout << "Please enter the width value: ";
	std::cin >> width;
	std::cout << "You have entered width: " << width << std::endl;
	findPath(viewer);
}

/**
 * @brief Finds the index of a point in the point cloud given its coordinates.
 *
 * This function iterates through the point cloud and returns the index of the point
 * that matches the provided x, y, and z coordinates. If no match is found, it returns -1.
 *
 * @param x The x coordinate of the point.
 * @param y The y coordinate of the point.
 * @param z The z coordinate of the point.
 * @return The index of the point if found, otherwise -1.
 */
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

/**
 * @brief Handles the event when a point is selected by the user in the PCLVisualizer.
 *
 * This function captures the coordinates of the picked point and adds it to the selectedPoints vector.
 * It also visualizes the selected point in blue on the viewer.
 *
 * @param event The point picking event object containing point selection details.
 * @param viewer_void A void pointer to the PCLVisualizer object.
 */
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void *viewer_void)
{
	float x, y, z;
	std::cout << "picking pt" << endl;

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

/**
 * @brief Handles keyboard events in the PCLVisualizer.
 *
 * This function listens for specific keyboard inputs:
 * - Press 'c' to clear all selected points.
 * - Press 'k' to confirm the selected points and proceed with pathfinding if the correct number of points is selected.
 *
 * @param event The keyboard event object containing keypress details.
 * @param viewer_void A void pointer to the PCLVisualizer object.
 */
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
		if (selectedPoints.size() > 2 && selectedPoints.size() < 5)
		{
			promptInput(viewer);
		}
		else
		{
			std::cout << selectedPoints.size() << " points were selected" << std::endl;
			std::cout << "Please select 3-4 points" << std::endl;
		}
	}
}

/**
 * @brief Creates a PCLVisualizer view and configures the visual settings.
 *
 * This function initializes the 3D viewer, sets the background color, adds the original
 * point cloud, and sets up text instructions. It also registers callbacks for point picking
 * and keyboard events.
 *
 * @param cloud The input point cloud (PointXYZ format) to be displayed.
 * @return A pointer to the initialized PCLVisualizer object.
 */
pcl::visualization::PCLVisualizer::Ptr createView(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)

{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Original Surface");

	viewer->addCoordinateSystem(1.0);

	viewer->initCameraParameters();

	viewer->addText("Choose points to create the boundary by using 'shift + left-mouse-button'", 10, 80, "instruction1");
	viewer->addText("Points should be chosen in the order that they will be connected to create the boundary (Clockwise)'", 10, 60, "instruction2");
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
