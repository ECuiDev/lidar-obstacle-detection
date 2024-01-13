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
 * @brief Read a PLY file and store its contents in a cevtor of vectors.
 *
 * This function reads a PLY file, where each line represents a row of data separated by spaces,
 * and stores the data in a vector of vectors. The first 17 lines are meta data.
 * Each inner vector represents a row, and its elements are the
 * values from the corresponding PLY line.
 *
 * @param file_path the path to the PLY file
 * @param data a reference to a vector of vectors to store the PLY data
 * @return true if the file is successfully read, false otherwise
 */
bool readPLY(const std::string& file_path, std::vector<std::vector<std::string>>& data) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error opening the file: " << file_path << std::endl;
        return false;
    }

    std::string line;
    int lineCount = 0;

    // Skip the first 17 lines of metadata
    while (lineCount < 17 && std::getline(file, line)) {
        lineCount++;
    }

    // Read the remaining lines and extract tokens
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;

        // Extract tokens separated by spaces
        while (iss >> token) {
            tokens.push_back(token);
        }

        // Add the tokens to the data vector
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
 * @brief Segments an input point cloud into plane and obstacle clouds.
 *
 * Utilizing RANSAC (Random Sample Consensus) segmentation, this function divides the input point cloud into two parts:
 * a 'plane cloud' and an 'obstacle cloud'. The 'plane cloud' consists of points that form a large flat surface,
 * typically representing the ground or similar structures. The 'obstacle cloud' includes points that are not part of
 * these large flat surfaces, indicating potential objects or irregularities in the terrain.
 *
 * @param input_cloud A shared pointer to the input point cloud.
 * @param planes A vector of shared pointers, each pointing to a segmented plane cloud.
 * @param remaining_cloud A shared pointer to the point cloud representing non-plane elements.
 */
void segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& planes, pcl::PointCloud<pcl::PointXYZI>::Ptr& remaining_cloud) {
    /*
     * The function initializes a SACSegmentation object for plane segmentation.
     * It sets necessary parameters like optimization, model type, method type,
     * maximum iterations, and distance threshold for the segmentation process.
     *
     * The input cloud is repeatedly segmented to extract plane surfaces. For each iteration:
     * - A new set of model coefficients and inlier indices are allocated.
     * - The segmentation is executed on the remaining (non-extracted) part of the cloud.
     * - If a plane is found, its points are extracted into a separate cloud.
     * - The remaining cloud is updated to exclude the extracted plane.
     *
     * This process continues until the remaining cloud is smaller than a set fraction of the original cloud.
     */

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(5000);
    seg.setDistanceThreshold(0.01);

    *remaining_cloud = *input_cloud;

    int nr_points = static_cast<int>(input_cloud->points.size());

    // Threshold to ensure segmentation stops when remaining cloud contains only minor planes, reducing over-segmentation and computational load.
    while (remaining_cloud->points.size() > 0.1 * nr_points) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        planes.push_back(plane_cloud);

        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*temp_cloud);
        remaining_cloud.swap(temp_cloud);
    }
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
 * @struct BoundingBoxData
 * @brief Holds information about a 3D Bounding Box in a point cloud, using Cartesian coordinates.
 *
 * This structure is designed to represent a 3D bounding box, typically used for encapsulating
 * a set of points or an object within a point cloud. It includes information about the vertices
 * of the box, its centroid, dimensions (width, height, depth), and orientation.
 *
 * @param width The width of the bounding box (extent in the x-dimension).
 * @param height The height of the bounding box (extent in the y-dimension).
 * @param depth The depth of the bounding box (extent in the z-dimension).
 * @param orientation The orientation of the bounding box represented as a quaternion (Eigen::Quaternionf).
 *                    This defines the rotation of the box with respect to a reference frame.
 * @param position The position of the bounding box's center, represented as an Eigen::Vector3f point.
 */
struct BoundingBoxInfo {
    float width, height, depth, distance;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f position;
};

/**
 * @brief Draw bounding boxes around clusters in a PCL visualizer.
 *
 * This function takes a PCL visualizer and a vector of point cloud clusters, and draws bounding boxes
 * around each cluster in the visualizer. Each cluster is assigned a unique color.
 *
 * @param viewer a PCL visualizer pointer
 * @param clusters a vector of point cloud clusters
 */
void drawBoundingBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& planes, std::vector<BoundingBoxInfo> boundingBoxes) {
    for (size_t i = 0; i < planes.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane = planes[i];

        // Compute the PCA
        pcl::PCA<pcl::PointXYZI> pca;
        pca.setInputCloud(plane);

        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane, centroid);

        // Transform the original cloud to the origin where the principal components correspond to the axes.
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::demeanPointCloud(*plane, centroid, *transformedCloud);
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigen_vectors.transpose();
        pcl::transformPointCloud(*transformedCloud, *transformedCloud, projectionTransform);

        // Get the min and max points of the transformed cloud
        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(*transformedCloud, minPoint, maxPoint);

        // The position of the box
        Eigen::Vector3f position = eigen_vectors * (maxPoint.getVector3fMap() + minPoint.getVector3fMap()) / 2 + centroid.head<3>();

        // The dimensions of the box
        float width = maxPoint.x - minPoint.x;
        float height = maxPoint.y - minPoint.y;
        float depth = maxPoint.z - minPoint.z;

        Eigen::Quaternionf orientation(eigen_vectors);

        // Draw the bounding box
        viewer->addCube(position, orientation, width, height, depth, "obb_" + std::to_string(i));

        // Fill bbox object
        BoundingBoxInfo bbox;
        bbox.position = position;
        bbox.orientation = orientation;
        bbox.width = width;
        bbox.height = height;
        bbox.depth = depth;
        boundingBoxes.push_back(bbox);
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
        if (row.size() < 10) { // Adjusted for the new data format
            std::cerr << "Invalid data format in PLY file." << std::endl;
            return;
        }

        float x = std::stof(row[0]); // X coordinate
        float y = std::stof(row[1]); // Y coordinate
        float z = std::stof(row[2]); // Z coordinate

        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 1;

        point_cloud->push_back(point);
    }
}

int main() {
    std::string directory_path = "/Users/andreasboscariol/Desktop/UAS";
    std::vector<BoundingBoxInfo> boundingBoxes;

    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.path().extension() == ".ply") {
            std::string ply_file_path = entry.path().string();

            // Read PLY file into a 2D vector (data)
            std::vector<std::vector<std::string>> data;
            if (readPLY(ply_file_path, data)) {
                /// Process LiDAR data from the PLY
                pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                processLiDARData(data, point_cloud);

                // Filter and downsample the point cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                filterAndDownsample(point_cloud, filtered_cloud);

                // Segment the point cloud into multiple planes
                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> planes;
                pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                segmentCloud(filtered_cloud, planes, remaining_cloud);

                // Visualization: Set up a PCL viewer and visualize the original, filtered, plane, and obstacle point clouds
                pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("LiDAR Viewer"));
                viewer->setBackgroundColor(0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI>(point_cloud, "original_cloud");
                viewer->addPointCloud<pcl::PointXYZI>(filtered_cloud, "filtered_cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "filtered_cloud"); // Green
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "filtered_cloud"); // Increase point size

                // Visualize each plane cloud
                float point_size = 10;
                for (size_t i = 0; i < planes.size(); ++i) {
                    std::string plane_id = "plane_" + std::to_string(i);
                    viewer->addPointCloud<pcl::PointXYZI>(planes[i], plane_id);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, plane_id); // Blue
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, plane_id); // Increase point size
                }

                // Visualize obstacle cloud
                viewer->addPointCloud<pcl::PointXYZI>(remaining_cloud, "remaining_cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "remaining_cloud"); // Red
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "remaining_cloud"); // Increase point size

                // Draw bounding boxes for each plane
                drawBoundingBox(viewer, planes, boundingBoxes);

                while (!viewer->wasStopped()) {
                    viewer->spinOnce();
                }
            }
            else {
                std::cerr << "Error processing PLY file: " << ply_file_path << std::endl;
            }
        }
    }

    return 0;
}
