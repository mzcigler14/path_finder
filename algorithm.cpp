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
though of at the end just find the normal with the smallest z on each line and use that to mvoe all the points on the line


area to generally trapazoidal (for finding inwards position)
space is not a critical issue





*/
vector<int> selectedPoints;

double stepSize;
double width;

const double NORMAL_ESTIMATION_RADIUS = 0.05;
const int STEP_SIZE_MOVES = 150;

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
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud(inCloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setSearchMethod(tree);

	ne.setRadiusSearch(NORMAL_ESTIMATION_RADIUS);

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
 * @return A vector of indices(integers with reference to cloud or cloud2d) of the nearest points.
 */
template <typename PointT>
vector<int> findNearest(PointT searchPoint, typename pcl::PointCloud<PointT>::Ptr searchCloud, int K = 1)
{
	vector<int> pointIdxKNNSearch(K);
	vector<float> pointKNNSquaredDistance(K);
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(searchCloud);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	{
		return pointIdxKNNSearch;
	}
	return vector<int>{-1};
};

/**
 * @brief Checks if a horizontal ray from -infinity to point p intersects the line segment ab.
 *
 * This function checks if a horizontal ray extending from point p intersects the line segment defined
 * by points a and b. It first verifies if the point p is within the vertical bounds of the line segment ab,
 * then checks if p is to the left or right of the line segment. Finally, it uses the line equation to determine
 * if the point p is below or above the line segment, thereby detecting the intersection.
 *
 * @param p The point from which the ray extends.
 * @param a The starting point of the line segment.
 * @param b The ending point of the line segment.
 * @return True if the ray to point p intersects the line segment, otherwise false.
 */
bool checkIntersection(pcl::PointXY p, pcl::PointXY a, pcl::PointXY b)
{
	// Check if point p is above or below the line
	if (p.y < std::min(a.y, b.y) || p.y >= std::max(a.y, b.y))
	{
		return false; // p is outside the vertical bounds of the line segment
	}

	// Check if the ray to the right from p intersects with the line segment ab
	if (a.y == b.y)
	{
		// The line segment is horizontal
		// true if p is on the line
		return p.y == a.y && (p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x));
	}

	// Calculate the intersection points x
	double m = (b.x - a.x) / (b.y - a.y);
	double xInter = a.x + m * (p.y - a.y); // calculate the x value where the ray intersects the line segment

	// Return true if the intersection point is to the right of point p
	return p.x <= xInter;
}

/**
 * @brief Determines if a point is inside a polygon using the ray-casting method.
 *
 * This function checks whether a 2D point, defined by its index in 'cloud', lies inside a polygon.
 * It casts a horizontal ray to the point and counts how many times this ray intersects the polygon's edges.
 * If the number of intersections is odd, the point is inside the polygon; otherwise, it is outside.
 *
 * @param pointIdx The index of the point in cloud2d/cloud.
 * @param poly A vector of indices(integers representing an index to cloud) representing the vertices
 *  of the polygon in the point cloud.
 * @return True if the point is inside the polygon, otherwise false.
 */
bool pointInPoly(int pointIdx, vector<int> &poly)
{

	bool inside = false;
	pcl::PointXY pt;
	pt.x = cloud2d->points[pointIdx].x;
	pt.y = cloud2d->points[pointIdx].y;

	// for each point in poly check if the ray crosses the vertex that proceeds it
	for (int i = 0; i < poly.size(); i++)
	{
		pcl::PointXY pt1;
		pcl::PointXY pt2;
		pt2.x = cloud2d->points[poly[i]].x;
		pt2.y = cloud2d->points[poly[i]].y;

		if (i == 0)
		{
			// if i is  0 use the line from the last point in the poly
			pt1.x = cloud2d->points[poly[poly.size() - 1]].x;
			pt1.y = cloud2d->points[poly[poly.size() - 1]].y;
		}
		else
		{
			// otherwise use the point from the proceeding one
			pt1.x = cloud2d->points[poly[i - 1]].x;
			pt1.y = cloud2d->points[poly[i - 1]].y;
		}
		// check if ray interesct the line, if it does change the inside boolean
		if (checkIntersection(pt, pt1, pt2))
		{
			inside = !inside;
			// string str = inside ? "true" : "false";
			// cout << "inside changed to " << str << endl;
		}
	}
	// string str1 = inside ? "inside" : "outside";
	// cout << "Point is " << str1 << " the poly" << endl;
	return inside;
}

/**
 * @brief Converts a 3D point cloud into a 2D point cloud by discarding the z-axis.
 *
 * @note This conversion is useful for performing 2D geometric operations such as checking point inclusion in polygons.
 */
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
 * @brief Calculates the average 2d euclidean distance between 150 points.
 *
 */
void calculateStepSize()
{

	// if the cloud is empty, return step size of 1
	cout << "Calculating step size" << endl;
	if (cloud2d->points.empty() || STEP_SIZE_MOVES <= 0)
	{
		cout << "cloud2d empty" << endl;
		stepSize = 1;
		return;
	}

	// create search tree to find nearest points
	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(cloud2d);

	pcl::PointXY currentPoint = cloud2d->points[0];
	set<int> visited;
	visited.insert(0);

	double cumulativeStep = 0.0;

	// for 150 moves calculate the distance
	for (int move = 0; move < STEP_SIZE_MOVES; ++move)
	{
		vector<int> nearestIndices(10);
		vector<float> nearestDistances(10);
		if (kdtree.nearestKSearch(currentPoint, 10, nearestIndices, nearestDistances) > 0)
		{

			int nearestIndex = nearestIndices[0];
			int i = 1;
			// while the nearest nearestIndex has already been visited find the next nearest
			// if visited points exceeds the cloud size also end loop
			while (visited.find(nearestIndex) != visited.end() && visited.size() < cloud2d->points.size())
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
			}

			// if 'nearest' isnt in the visited add the distance to the culmulative
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
	// if visited has more than one point divide by the number of visited points
	stepSize = (visited.size() > 1) ? (cumulativeStep / (visited.size())) : 1;
}

/**
 * @brief Connects two points by interpolating intermediate points along the line between them.
 *
 *
 * @param output The vector of trajectory points to which the interpolated points will be added.
 * @param pt1 The starting point of the line.
 * @param pt2 The ending point of the line.
 */
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
		// step in directon of point 2 by the step size each loop
		searchPt.x = pt1.x + stepSize * i * dir.normalized().x();
		searchPt.y = pt1.y + stepSize * i * dir.normalized().y();
		int nearest = findNearest<pcl::PointXY>(searchPt, cloud2d)[0];
		if (nearest == -1)
		{
			cout << "No nearest found when connecting points" << endl;
			return;
		}

		// if the point has not already been added, add it too the connection
		if (output.back().index != nearest)
		{
			TrajPt trajPt(nearest, cloud->points[nearest], normals->points[nearest]);
			output.push_back(trajPt);
		}
	}
}

/**
 * @brief Adds trajectory points along a specified direction until a gap condition is met.
 *
 * This function moves along the specified direction from a given point, adding trajectory points
 * to the output (if it is used by the widening operation) vector while checking the gap between the
 * current point and the next one.
 * The function stops when the gap exceeds the defined width and if not widening adds the point to the output
 *
 * @param index The index of the starting point in the point cloud.
 * @param output The vector where new trajectory points will be added.
 * @param dir The direction in which to move from the starting point.
 * @param widening If true, adds each new point to the output
 * @param addToFront If true, and widening is true adds the trajectory point to the front of the output vector, otherwise to the back.
 */
void addTrajPt(int index,
			   vector<TrajPt> &output,
			   Eigen::Vector2f &dir,
			   bool widening = false,
			   bool addToFront = false)
{
	pcl::PointXY prevPt;
	int prevNearest = 0;
	int nearest = index;

	double gap = 0;
	double gapOld = 1;
	// if the gap  > width and the nearest (aka previous index woudl then be within width)
	// is in the polygon add the point both output vector
	while (gap < width)
	{
		// if the gap is not changing return, avoid infinite loop
		if (gap == gapOld)
		{
			// cout << "Gap not changing" << endl;
			return;
		}
		else
		{
			gapOld = gap;
		}

		// previous point is the nearest from the last loop
		prevPt.x = cloud2d->points[nearest].x;
		prevPt.y = cloud2d->points[nearest].y;

		pcl::PointXY searchPt;
		// search one step in the specified direction
		searchPt.x = prevPt.x + stepSize * dir.normalized().x();
		searchPt.y = prevPt.y + stepSize * dir.normalized().y();
		prevNearest = nearest;
		nearest = findNearest<pcl::PointXY>(searchPt, cloud2d)[0];
		if (nearest == -1)
		{
			cout << "No nearest found when finding next point in addTrajPt" << endl;
			return;
		}

		double normalZ = normals->points[nearest].normal_z;
		// add the 3 dimensional distance to the gap
		gap += sqrt(pow(cloud2d->points[nearest].x - prevPt.x, 2) +
					pow(cloud2d->points[nearest].y - prevPt.y, 2)) /
			   abs((normalZ != 0.0) ? normalZ : MAXFLOAT); // using the z of the normal project distance into 3D

		if (widening && nearest != prevNearest)
		{
			if (pointInPoly(prevNearest, selectedPoints))
			{
				TrajPt trajPt(nearest, cloud->points[nearest], normals->points[nearest]);
				// if adding to front of line insert
				if (addToFront)
				{
					output.insert(output.begin(), trajPt);
				}
				else
				{
					output.push_back(trajPt);
					// cout << "Point added to output: " << trajPt.index << endl;
				}
			}
			else
			{
				break;
			}
		}
	}
	// not widening because pt already added
	if (!widening && pointInPoly(prevNearest, selectedPoints))
	{
		TrajPt trajPt(prevNearest, cloud->points[prevNearest], normals->points[prevNearest]);
		output.push_back(trajPt);
		// cout << "Point added to output: " << trajPt.index << endl;
	}
}

/**
 * @brief Expands a grid line by adding points in both directions until line hits boundary.
 *
 * This function traverses the line in both forward and backward directions, adding trajectory points
 * along the gridline direction until the line hits the boundary.
 *
 * @param line The grid line of trajectory points to be expanded.
 * @param gridLineDir The direction of the gridline (points to line.back()).
 */
void checkWiden(vector<TrajPt> &line, Eigen::Vector2f &gridLineDir)
{
	// traverse parrellel to the line away from the back until the end of the line is within a distance of 'width' from the edge;
	int count = line.size() - 1;
	while (line.size() > count && (line.end() - 2)->index != (line.end() - 1)->index)
	{
		count++;
		addTrajPt(line.back().index, line, gridLineDir, true, false);
	}

	// do the ssame for the front of the line
	count = line.size() - 1;

	while (line.size() > count && (line.begin())->index != (line.begin() + 1)->index)
	{
		count++;
		Eigen::Vector2f oppGridLineDir(-gridLineDir.x(), -gridLineDir.y());
		addTrajPt(line.front().index, line, oppGridLineDir, true, true);
	}
}

/**
 * @brief Creates a grid of trajectory points, expanding from an initial line while keeping within polygon boundaries.
 *
 * This function generates a grid of points by iteratively adding points normal to the starting line
 * until no more gridlines can be added within the specified polygon. It also ensures that each grid line
 * is widened according to the provided direction. If the grid exceeds a limit of 100 lines, the loop exits
 * to prevent infinite iteration.
 *
 * @param output A reference to a 2D vector of TrajPt that stores the generated grid lines.
 * @param initialLine A reference to the vector of TrajPt that defines the starting line for the grid to be built off of.
 * @param gridNormal A reference to an Eigen::Vector2f that specifies the normal direction in which to add a new gridline.
 * @param gridLineDir A reference to an Eigen::Vector2f that specifies the direction in which the grid lines should be expanded.
 */
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

	// while the last vector had points inside the polygon add another gridline

	while (!output.back().empty())
	{
		cout << "Line: " << output.size() << endl;
		checkWiden(output.back(), gridLineDir);
		// avoid infinite loop in case of hidden bug
		if (output.size() > 100)
		{
			return;
		}
		output.emplace_back();
		for (TrajPt trajPt : output[output.size() - 2])
		{
			addTrajPt(trajPt.index, output[output.size() - 1], gridNormal);
		}
	}

	if (output.empty())
	{
		output.push_back(initialLine);
	}

	if (output.back().empty())
	{
		output.pop_back();
	}
}
/**
 * @brief Connects a grid of trajectory points into a continuous trajectory.
 *
 * This function takes the grid of trajectory points and connects them into a single continuous trajectory.
 * It alternates the connection direction between even and odd rows of the grid to form a smooth path.
 *
 * @param vecTrajectory The vector where the connected trajectory points will be stored.
 * @param vecGrid A 2D vector containing the grid of trajectory points.
 */
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
	// cout << "Finding Path" << endl;

	computeNormals(cloud, normals);

	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");

	// create a 2d point cloud on plane(0, 0, 1), working in 2D will make the search algorithms fasteer
	//    ccloud2d indicies will match cloud
	create2dCloud();

	// create an estimate for the step size between points in the 2D plane
	// this will determine distance to step from a specific to find the next point
	calculateStepSize();
	cout << "Step Size: " << stepSize << endl;

	// direction the grid will progress in normal to the initial line
	Eigen::Vector2f gridLineDir;

	gridLineDir.x() = cloud->points[selectedPoints[1]].x - cloud->points[selectedPoints[0]].x;
	gridLineDir.y() = cloud->points[selectedPoints[1]].y - cloud->points[selectedPoints[0]].y;

	Eigen::Vector2f gridNormal;
	// in clockwise dir x = - y and y = -x
	gridNormal.x() = -gridLineDir.y();
	gridNormal.y() = gridLineDir.x();

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

	// connect the lines into one Trajectory
	// vecTrajectory gives the indices of the Trajectory in order
	vector<TrajPt> vecTrajectory;
	cout << "connecting tragectory" << endl;
	connectTrajectory(vecTrajectory, vecGrid);

	// populate 3d Trajectory into PointXYZRGB pointcloud for visualization
	cout << "Displaying results" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectoryCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (TrajPt trajPt : vecTrajectory)
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

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTraj(trajectoryCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(trajectoryCloud, rgbTraj, "Trajectory Points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Trajectory Points");
}

/**
 * @brief Prompts the user to input the width value and initiates the path-finding process.
 *
 *
 * @param viewer The PCLVisualizer pointer to display the path.
 */
void promptInput(pcl::visualization::PCLVisualizer *viewer)
{
	cout << "Please enter the width value: ";
	while (!(cin >> width) || width <= 0)
	{
		cin.clear();										 // Clear the error flag
		cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard invalid input
		cout << "Invalid input. Please enter a positive numeric value for width: ";
	}
	cout << "You have entered width: " << width << endl;
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
	cout << "picking pt" << endl;

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

	cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << endl;
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

		cout << "c was pressed => clearing all inputs" << endl;
		selectedPoints.clear();
		inputCloud->points.clear();
		if (viewer->contains("Input Points"))
		{
			viewer->removePointCloud("Input Points");
		}
	}
	else if (event.getKeySym() == "k" && event.keyDown())
	{
		cout << "k was pressed => finding path" << endl;
		if (selectedPoints.size() > 2 && selectedPoints.size() < 5)
		{
			promptInput(viewer);
		}
		else
		{
			cout << selectedPoints.size() << " points were selected" << endl;
			cout << "Please select 3-4 points" << endl;
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
	cout << "Loaded "
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
