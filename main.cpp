#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <chrono>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// 计算点云的协方差矩阵
Eigen::Matrix3f computeCovarianceMatrix(const PointCloud::Ptr &cloud) {
    auto start = std::chrono::high_resolution_clock::now();
    
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();

    // 计算均值
    for (const auto &point : cloud->points) {
        mean += Eigen::Vector3f(point.x, point.y, point.z);
    }
    mean /= cloud->points.size();

    // 计算协方差矩阵
    for (const auto &point : cloud->points) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        covariance_matrix += (p - mean) * (p - mean).transpose();
    }
    covariance_matrix /= cloud->points.size() - 1;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Covariance matrices computed in " << duration.count() << " seconds." << std::endl;

    return covariance_matrix;
}

// 计算两个协方差矩阵的相关性系数
float computeCorrelationCoefficient(const Eigen::Matrix3f &cov1, const Eigen::Matrix3f &cov2) {
    float numerator = (cov1.array() * cov2.array()).sum();
    float denominator = std::sqrt((cov1.array() * cov1.array()).sum() * (cov2.array() * cov2.array()).sum());
    return numerator / denominator;
}

// 加载点云并可选地进行降采样
PointCloud::Ptr loadAndDownsamplePointCloud(const std::string &filename, bool downsample = false, float leaf_size = 0.1f) {
    PointCloud::Ptr cloud(new PointCloud);
    
    // 记录文件加载时间
    auto start = std::chrono::high_resolution_clock::now();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", filename.c_str());
        return nullptr;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "File " << filename << " loaded successfully in " << duration.count() << " seconds." << std::endl;

    if (downsample) {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);

        start = std::chrono::high_resolution_clock::now();
        PointCloud::Ptr cloud_filtered(new PointCloud);
        sor.filter(*cloud_filtered);
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        std::cout << "Downsampling completed with leaf size " << leaf_size 
                  << " in " << duration.count() << " seconds." << std::endl;
        return cloud_filtered;
    }

    return cloud;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <file1.pcd> <file2.pcd> [downsample] [leaf_size]" << std::endl;
        return -1;
    }

    std::string file1 = argv[1];
    std::string file2 = argv[2];
    bool downsample = (argc > 3) ? std::stoi(argv[3]) : false;
    float leaf_size = (argc > 4) ? std::stof(argv[4]) : 0.1f;//默认降采样大小0.1

    // 加载并可选地降采样点云
    PointCloud::Ptr cloud1 = loadAndDownsamplePointCloud(file1, downsample, leaf_size);
    PointCloud::Ptr cloud2 = loadAndDownsamplePointCloud(file2, downsample, leaf_size);

    if (!cloud1 || !cloud2) {
        return -1;
    }

    // 计算协方差矩阵并输出协方差矩阵
    Eigen::Matrix3f cov1 = computeCovarianceMatrix(cloud1);
    std::cout << "Covariance Matrix of Point Cloud 1:\n" << cov1 << std::endl;
    Eigen::Matrix3f cov2 = computeCovarianceMatrix(cloud2);
    std::cout << "Covariance Matrix of Point Cloud 2:\n" << cov2 << std::endl;

    // 计算相关性系数
    float correlation_coefficient = computeCorrelationCoefficient(cov1, cov2);
    std::cout << "Correlation Coefficient between the two point clouds: " << correlation_coefficient << std::endl;

    return 0;
}
