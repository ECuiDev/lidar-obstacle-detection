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

// Function to process LiDAR data and apply tree detection
void processLiDARData(const std::vector<std::vector<std::string>>& data, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud) {
    // Ensure the point cloud is empty before populating it
    point_cloud->clear();

    for (const auto& row : data) {
        if (row.size() < 9) {
            std::cerr << "Invalid data format in CSV file." << std::endl;
            return;
        }

        // Extract X, Y, and Z coordinates from the CSV data
        float x = std::stof(row[3]); // X coordinate (index 3)
        float y = std::stof(row[4]); // Y coordinate (index 4)
        float z = std::stof(row[5]); // Z coordinate (index 5)

        // Create a PCL point and add it to the point cloud
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        point_cloud->push_back(point);
    }
}

int main() {
    std::string directory_path = "C:/Users/Edward/Downloads/sydney-urban-objects-dataset/objects/trees";

    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.path().extension() == ".csv") {
            std::string csv_file_path = entry.path().string();
            std::cout << csv_file_path << std::endl;
            
            // Read CSV file
            std::vector<std::vector<std::string>> data;
            if (readCSV(csv_file_path, data)) {
                // Process LiDAR data
                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                processLiDARData(data, point_cloud);
                // COMMENT HERE
                // Preprocessing
                pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setInputCloud(point_cloud);
                vg.setLeafSize(0.1, 0.1, 0.1);
                vg.filter(*downsampled_cloud);

                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(downsampled_cloud);
                sor.setMeanK(50);
                sor.setStddevMulThresh(1.0);
                sor.filter(*filtered_cloud);

                // Feature extraction
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
                ne.setInputCloud(filtered_cloud);
                ne.setKSearch(20); // Adjust as needed
                ne.compute(*normals);

                // Tree-specific feature extraction and segmentation
                pcl::PointCloud<pcl::PointXYZ>::Ptr tree_points(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
                std::vector<pcl::PointIndices> tree_clusters;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(0.5); // Adjust as needed
                ec.setMinClusterSize(100);  // Adjust as needed
                ec.setMaxClusterSize(50000); // Adjust as needed
                ec.setSearchMethod(tree_kdtree);
                ec.setInputCloud(filtered_cloud);
                ec.extract(tree_clusters);

                // Classification
                // classify clusters as trees or non-trees based on size, shape, or other features?
                // or maybe just assume single trunk and make model based off directly facing a tree?
                // COMMENT HERE
            } else {
                std::cerr << "Error processing CSV file: " << csv_file_path << std::endl;
            }
        }
    }

    return 0;
}
