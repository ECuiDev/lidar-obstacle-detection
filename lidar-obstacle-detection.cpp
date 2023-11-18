// lidar-obstacle-detection.cpp : Defines the entry point for the application.
//

#include "lidar-obstacle-detection.h"

using namespace std;

// Function to read a CSV file into a vector of vectors
bool readCSV(const std::string& file_path, std::vector<std::vector<std::string>>& data) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error opening the file: " << file_path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> tokens;
        size_t pos = 0;
        while ((pos = line.find(',')) != std::string::npos) {
            tokens.push_back(line.substr(0, pos));
            line.erase(0, pos + 1);
        }
        tokens.push_back(line);
        data.push_back(tokens);
    }

    file.close();
    return true;
}

// Function to filter and downsample the point cloud
void filterAndDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud) {
    // Implement filtering criteria here (base on intensity, range, region of interest, etc.?)

    // Keep all points for now
    *filtered_cloud = *input_cloud;

    // Downsample using VoxelGrid (not finalized)
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(0.1, 0.1, 0.1);
    vg.filter(*filtered_cloud);
}

// Function to perform segmentation into plane and obstacle clouds
void segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& plane_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // Set the input cloud for segmentation
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the plane points
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    // Extract the obstacle points
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
}

// Function to perform clustering on the obstacle cloud
void clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(obstacle_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.3); // Adjust as needed
    ec.setMinClusterSize(100);  // Adjust as needed
    ec.setMaxClusterSize(50000); // Adjust as needed
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacle_cloud);
    ec.extract(cluster_indices);
}

// Function to draw bounding boxes around clusters
void drawBoundingBoxes(pcl::visualization::PCLVisualizer::Ptr& viewer, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters) {
    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster = clusters[i];

        // Compute bounding box
        pcl::PointXYZI min_point, max_point;
        pcl::getMinMax3D(*cluster, min_point, max_point);

        // Add bounding box to viewer
        viewer->addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, 1.0, 0.0, 0.0, "bbox_" + std::to_string(i));
    }
}

// Function to process LiDAR data
void processLiDARData(const std::vector<std::vector<std::string>>& data, pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud) {
    // Ensure the point cloud is empty before populating it
    point_cloud->clear();

    for (const auto& row : data) {
        if (row.size() < 9) {
            std::cerr << "Invalid data format in CSV file." << std::endl;
            return;
        }

        // Extract X, Y, and Z coordinates, intensity, and range from the CSV data
        float x = std::stof(row[3]); // X coordinate
        float y = std::stof(row[4]); // Y coordinate
        float z = std::stof(row[5]); // Z coordinate
        float intensity = std::stof(row[6]); // Intensity
        float range = std::stof(row[7]); // Range, not too sure what to do with this yet

        // Create a PCL point with intensity and add it to the point cloud
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = intensity;
        point_cloud->push_back(point);
    }
}

int main() {
    std::string directory_path = "C:/Users/Edward/Downloads/sydney-urban-objects-dataset/objects/trees";

    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.path().extension() == ".csv") {
            std::string csv_file_path = entry.path().string();

            // Read CSV file
            std::vector<std::vector<std::string>> data;
            if (readCSV(csv_file_path, data)) {
                // Process LiDAR data
                pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                processLiDARData(data, point_cloud);

                // Filter and downsample
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                filterAndDownsample(point_cloud, filtered_cloud);

                // Segmentation
                pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                segmentCloud(filtered_cloud, plane_cloud, obstacle_cloud);

                // Clustering
                std::vector<pcl::PointIndices> cluster_indices;
                clusterCloud(obstacle_cloud, cluster_indices);

                // Extract individual clusters
                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
                for (const auto& indices : cluster_indices) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
                    for (const auto& index : indices.indices) {
                        cluster->points.push_back(obstacle_cloud->points[index]);
                    }
                    cluster->width = cluster->points.size();
                    cluster->height = 1;
                    cluster->is_dense = true;
                    clusters.push_back(cluster);
                }

                // Visualization
                pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("LiDAR Viewer"));
                viewer->setBackgroundColor(0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI>(point_cloud, "original_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(filtered_cloud, "filtered_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(plane_cloud, "plane_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(obstacle_cloud, "obstacle_cloud");

                drawBoundingBoxes(viewer, clusters);

                while (!viewer->wasStopped()) {
                    viewer->spinOnce();
                }
                
            }
            else {
                std::cerr << "Error processing CSV file: " << csv_file_path << std::endl;
            }
        }
    }

    return 0;
}
