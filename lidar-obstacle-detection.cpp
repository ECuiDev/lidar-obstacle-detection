#include "lidar-obstacle-detection.h"

using namespace std;

/**
 * @brief Read a CSV file and store its contents in a vector of vectors.
 *
 * This function reads a CSV file, where each line represents a row of data with comma-separated values,
 * and stores the data in a vector of vectors. Each inner vector represents a row, and its elements are the
 * values from the corresponding CSV line.
 *
 * @param file_path the path to the CSV file
 * @param data a reference to a vector of vectors to store the CSV data
 * @return true if the file is successfully read, false otherwise
 */
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

/**
 * @brief Filter and downsample the input point cloud.
 *
 * This function applies filtering criteria (e.g., intensity, range, region of interest) and downsamples
 * the input point cloud using VoxelGrid.
 *
 * @param input_cloud the input point cloud
 * @param filtered_cloud the filtered and downsampled point cloud
 */
void filterAndDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud) {
    /*
     * Initialize a temporary point cloud (temp_cloud) to store the filtered results.
     * 
     * Implement your filtering criteria (e.g., intensity, range, region of interest, etc.).
     * Currently defines an intensity threshold value (0.5) to filter points based on intensity.
     * 
     * Create a condition (intensity_condition) for intensity filtering using the defined threshold.
     * Add a comparison condition to the intensity filter for points with intensity greater than the specified threshold.
     * 
     * Instantiate a conditional removal filter (intensity_filter) to apply the intensity condition.
     * Specify the input cloud for the intensity filter to be the original input cloud.
     * Set the intensity condition for the conditional removal filter.
     * Apply the intensity filter to the input cloud, storing the results in temp_cloud.
     * 
     * Create a VoxelGrid filter (vg) for downsampling.
     * Set the input cloud for VoxelGrid downsampling to be the filtered temp_cloud.
     * Define the leaf size for downsampling (e.g., 0.1, 0.1, 0.1).
     * Apply VoxelGrid downsampling to the filtered cloud and store the result in the output cloud (filtered_cloud).
     * 
     * Note: Adjust the intensity threshold and VoxelGrid leaf size as needed. Also consider adding other filtering criteria.
     */

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    float intensity_threshold = 0.5;

    pcl::ConditionAnd<pcl::PointXYZI>::Ptr intensity_condition(new pcl::ConditionAnd<pcl::PointXYZI>());
    intensity_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::GT, intensity_threshold)));

    pcl::ConditionalRemoval<pcl::PointXYZI> intensity_filter;
    intensity_filter.setInputCloud(input_cloud);
    intensity_filter.setCondition(intensity_condition);
    intensity_filter.filter(*temp_cloud);

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(temp_cloud);
    vg.setLeafSize(0.1, 0.1, 0.1);
    vg.filter(*filtered_cloud);
}


/**
 * @brief Segment the input point cloud into plane and obstacle clouds.
 *
 * This function uses RANSAC segmentation to separate the input point cloud into a plane cloud
 * (representing the ground or a large flat surface) and an obstacle cloud (representing potential objects).
 *
 * @param input_cloud the input point cloud
 * @param plane_cloud the segmented plane cloud
 * @param obstacle_cloud the segmented obstacle cloud
 */
void segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& plane_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud) {
    /*
     * Smart pointers (coefficients and inliers) are declared to store model
     * coefficients and point indices obtained during plane segmentation.
     * 
     * A SACSegmentation object (seg) is created for plane segmentation. Parameters such as optimization,
     * model type (plane), method type (RANSAC), maximum iterations, and distance threshold are set.
     * 
     * The input cloud for segmentation is set, and the segmentation is executed,
     * storing the inliers and coefficients of the segmented plane.
     * 
     * An ExtractIndices object (extract) is initialized, and the points of the segmented plane are extracted into plane_cloud.
     * 
     * The ExtractIndices object is configured to extract the non-plane (obstacle) points into obstacle_cloud.
     */
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
}

/**
 * @brief Perform clustering on the obstacle cloud.
 *
 * This function applies Euclidean clustering to the obstacle cloud, identifying clusters of points
 * representing potential objects. The cluster indices are returned.
 *
 * @param obstacle_cloud the obstacle point cloud
 * @param cluster_indices the indices of point clusters
 */
void clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, std::vector<pcl::PointIndices>& cluster_indices) {
    /*
     * A KdTree search object (tree) is created and set to use obstacle_cloud as its input cloud.
     * 
     * An EuclideanClusterExtraction object (ec) is created for clustering with parameters cluster tolerance, 
     * minimum cluster size, and maximum cluster size.
     * The cluster_tolerance determines the maximum distance between points to be considered part of the same cluster.
     * The min_cluster_size and max_cluster_size parameters determine the allowed size range for clusters.
     * 
     * The tree is set as the search method for clustering.
     * 
     * The input cloud for clustering is set to obstacle_cloud, and the clustering
     * is executed, storing the resulting cluster indices in cluster_indices.
     */

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

/**
 * @brief Draw bounding boxes around clusters in a PCL visualizer.
 *
 * This function takes a PCL visualizer and a vector of point cloud clusters, and draws bounding boxes
 * around each cluster in the visualizer. Each cluster is assigned a unique color.
 *
 * @param viewer a PCL visualizer pointer
 * @param clusters a vector of point cloud clusters
 */
void drawBoundingBoxes(pcl::visualization::PCLVisualizer::Ptr& viewer, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters) {
    /*
     * For each cluster:
     *   - A custom color handler is created for the cluster, setting the color to red (1.0, 0.0, 0.0).
     *   - The cluster is visualized in the PCL visualizer with the specified color and a unique identifier.
     *   - The rendering properties for the point size of the cluster are set.
     *   - The minimum and maximum points of the cluster are computed.
     *   - A bounding box (cube) is drawn around the cluster using the computed min and max points, with a red color.
     * The visualization is updated in the PCL visualizer.
     */

    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster = clusters[i];

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cluster, 1.0, 0.0, 0.0);

        viewer->addPointCloud<pcl::PointXYZI>(cluster, single_color, "cluster_" + std::to_string(i));

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster_" + std::to_string(i));

        pcl::PointXYZI min_point, max_point;
        pcl::getMinMax3D(*cluster, min_point, max_point);

        viewer->addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, 1.0, 0.0, 0.0, "bbox_" + std::to_string(i));
    }
}

/**
 * @brief Process LiDAR data from a CSV file and convert it into a point cloud.
 *
 * This function reads LiDAR data from a CSV file, extracts X, Y, Z coordinates, intensity, and range
 * from each row, and creates a point cloud with intensity values.
 *
 * @param data a vector of vectors representing the CSV data
 * @param point_cloud the resulting point cloud
 */
void processLiDARData(const std::vector<std::vector<std::string>>& data, pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud) {
    /*
     * The point_cloud is cleared to ensure it's empty before populating.
     * 
     * For each row in the CSV data:
     *   - If the row size is less than 9, an error message is printed, indicating an invalid data format, and the function returns.
     *   - X, Y, Z coordinates, intensity, and range are extracted from the CSV row.
     *   - A PCL point (PointXYZI) is created with X, Y, Z coordinates and intensity.
     *   - The point is added to the point cloud.
     *   - Range is also extracted but I'm not too sure what to do with it yet.
     * 
     * Range is mentioned in the comment but not used in the current code; you might decide how to use it based on your specific needs.
     */

    point_cloud->clear();

    for (const auto& row : data) {
        if (row.size() < 9) {
            std::cerr << "Invalid data format in CSV file." << std::endl;
            return;
        }

        float x = std::stof(row[3]); // X coordinate
        float y = std::stof(row[4]); // Y coordinate
        float z = std::stof(row[5]); // Z coordinate
        float intensity = std::stof(row[6]); // Intensity
        float range = std::stof(row[7]); // Range

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

            // Read CSV file into a 2D vector (data)
            std::vector<std::vector<std::string>> data;
            if (readCSV(csv_file_path, data)) {
                /// Process LiDAR data from the CSV
                pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                processLiDARData(data, point_cloud);

                // Filter and downsample the point cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                filterAndDownsample(point_cloud, filtered_cloud);

                // Segmentation: Separate the point cloud into a plane and obstacle cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                segmentCloud(filtered_cloud, plane_cloud, obstacle_cloud);

                // Clustering: Identify clusters in the obstacle cloud
                std::vector<pcl::PointIndices> cluster_indices;
                clusterCloud(obstacle_cloud, cluster_indices);

                /*
                 * Declare a vector to store pointers to individual clusters.
                 * 
                 * For each cluster indice obtained from clustering the obstacle cloud:
                 *   - Create a new PointCloud pointer for the current cluster.
                 *   - For each indices of points belonging to the current cluster:
                 *     - Add the corresponding point from the obstacle cloud to the current cluster.
                 *   - Set the width of the cluster point cloud to the number of points it contains.
                 *   - Set the height of the cluster 1 (since the cluster is an unorganized point cloud and not structured in a grid)
                 *   - Set is_dense to true, indicating that there are no invalid points in the cluster.
                 *   - Add the pointer to the current cluster to the vector of clusters.
                 */
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

                // Visualization: Set up a PCL viewer and visualize the original, filtered, plane, and obstacle point clouds
                pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("LiDAR Viewer"));
                viewer->setBackgroundColor(0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI>(point_cloud, "original_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(filtered_cloud, "filtered_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(plane_cloud, "plane_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(obstacle_cloud, "obstacle_cloud");

                // Set visualization parameters for each point cloud
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "original_cloud"); // White
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "filtered_cloud"); // Green
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "plane_cloud"); // Blue
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "obstacle_cloud"); // Red

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
