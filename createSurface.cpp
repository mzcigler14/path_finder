// #include <iostream>
// #include <cmath>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>

// using namespace std;

// int main()
// {
//     int width = 100;        // Grid size in x-direction
//     int height = 100;       // Grid size in y-direction
//     float frequency = 0.2f; // Frequency of the sine waves
//     float amplitude = 5.0f; // Amplitude (height of the wave)

//     // Create a point cloud object
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//     // Set the point cloud dimensions
//     cloud->width = width * height;
//     cloud->height = 1;
//     cloud->is_dense = false;
//     cloud->points.resize(cloud->width);

//     // Generate points on the rolling surface
//     int idx = 0;
//     for (int x = 0; x < width; x++)
//     {
//         for (int y = 0; y < height; y++)
//         {
//             // Scale x and y to be between -1 and 1
//             float scaled_x = 2.0f * (static_cast<float>(x) / (width - 1)) - 1.0f;  // Maps to [-1, 1]
//             float scaled_y = 2.0f * (static_cast<float>(y) / (height - 1)) - 1.0f; // Maps to [-1, 1]

//             // Calculate z using the sine wave function
//             float z = amplitude * sin(frequency * scaled_x) * cos(frequency * scaled_y);

//             // Set the point's x, y, z values
//             cloud->points[idx].x = scaled_x;
//             cloud->points[idx].y = scaled_y;
//             cloud->points[idx].z = z;
//             idx++;
//         }
//     }

//     string filename = "test2.pcd";
//     // Save the generated point cloud to the appropriate PCD file
//     pcl::io::savePCDFileASCII(filename, *cloud);
//     cout << "Saved " << cloud->points.size() << " points to " << filename << std::endl;

//     return 0;
// }

#include <iostream>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

using namespace std;

int main()
{
    int width = 250;          // Grid size in x-direction
    int height = 250;         // Grid size in y-direction
    float frequency1 = 0.25f; // Very low frequency for gentle waves
    float frequency2 = 0.1f;  // Very low frequency for gentle waves
    float amplitude1 = 0.1f;  // Very low amplitude for gentle waves
    float amplitude2 = 0.05f; // Very low amplitude for gentle waves

    // Create a point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Set the point cloud dimensions
    cloud->width = width * height;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width);

    // Generate points on the rolling surface
    int idx = 0;
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            // Scale x and y to be between -1 and 1
            float scaled_x = 2.0f * (static_cast<float>(x) / (width - 1)) - 1.0f;  // Maps to [-1, 1]
            float scaled_y = 2.0f * (static_cast<float>(y) / (height - 1)) - 1.0f; // Maps to [-1, 1]

            // Calculate z using multiple sine and cosine waves for very gentle peaks and troughs
            float z = amplitude1 * sin(frequency1 * M_PI * scaled_x) * cos(frequency1 * M_PI * scaled_y) +
                      amplitude2 * sin(frequency2 * M_PI * scaled_x) * cos(frequency2 * M_PI * scaled_y);

            // Set the point's x, y, z values
            cloud->points[idx].x = scaled_x;
            cloud->points[idx].y = scaled_y;
            cloud->points[idx].z = z;
            idx++;
        }
    }

    string filename = "test_very_gentle_peaks_troughs.pcd";
    // Save the generated point cloud to the appropriate PCD file
    pcl::io::savePCDFileASCII(filename, *cloud);
    cout << "Saved " << cloud->points.size() << " points to " << filename << std::endl;

    return 0;
}
