#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class LaserScanMerger : public rclcpp::Node {
public:
    LaserScanMerger() : Node("laser_scan_merger") {
        // 订阅两个 LaserScan 话题
        scan1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan1", 10, std::bind(&LaserScanMerger::scan1Callback, this, std::placeholders::_1));

        scan2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan2", 10, std::bind(&LaserScanMerger::scan2Callback, this, std::placeholders::_1));

        // 发布合并后的 LaserScan 话题
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/merged_scan", 10);
    }

private:
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double x_offset, double y_offset, double z_offset) {
        for (auto& point : cloud->points) {
            point.x += x_offset;
            point.y += y_offset;
            point.z += z_offset;
        }
    }
    // 回调函数，处理第一个 LaserScan 话题
    void scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr scan1) {
        projector_.projectLaser(*scan1, cloud1_);  // 将 LaserScan 转换为 PointCloud2
        pcl::fromROSMsg(cloud1_, *pcl_cloud1_);    // 将 PointCloud2 转换为 PCL 点云
        transformPointCloud(pcl_cloud1_, -0.23, 0.0, 0.0); // 对点云进行平移变换，x轴向左平移23cm
        mergeAndPublish();                         // 尝试合并点云并发布
    }

    // 回调函数，处理第二个 LaserScan 话题
    void scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr scan2) {
        projector_.projectLaser(*scan2, cloud2_);  // 将 LaserScan 转换为 PointCloud2
        pcl::fromROSMsg(cloud2_, *pcl_cloud2_);    // 将 PointCloud2 转换为 PCL 点云
        //transformPointCloud(pcl_cloud1_, 0.23, 0.0, 0.0); // 对点云进行平移变换，x轴向右平移23cm
        mergeAndPublish();                         // 尝试合并点云并发布
    }

    // 合并两个点云并发布为新的 LaserScan
    void mergeAndPublish() {
        if (pcl_cloud1_->empty() || pcl_cloud2_->empty()) {
            //RCLCPP_WARN(this->get_logger(), "One or both point clouds are empty, skipping merge.");
            return;  // 等待两个点云都接收到后再合并
        }
        RCLCPP_INFO(this->get_logger(), "Point Cloud1 size: %zu", pcl_cloud1_->size());
        RCLCPP_INFO(this->get_logger(), "Point Cloud2 size: %zu", pcl_cloud2_->size());

        // 合并两个点云
        pcl::PointCloud<pcl::PointXYZ> merged_pcl_cloud;
        merged_pcl_cloud += *pcl_cloud1_;
        merged_pcl_cloud += *pcl_cloud2_;
        // 打印合并后点云的大小
        RCLCPP_INFO(this->get_logger(), "Merged Point Cloud size: %zu", merged_pcl_cloud.size());

        // 将合并后的点云转换回 PointCloud2
        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(merged_pcl_cloud, merged_cloud_msg);
        merged_cloud_msg.header = cloud1_.header;  // 保持与第一个点云的时间戳和坐标系一致

        // 将合并后的 PointCloud2 转换为 LaserScan
        sensor_msgs::msg::LaserScan merged_scan;
        pointcloudToLaserScan(merged_cloud_msg, merged_scan);
        merged_scan.header.frame_id = "merged_laser";
        // 发布合并后的 LaserScan
        scan_pub_->publish(merged_scan);
        RCLCPP_INFO(this->get_logger(), "publish merged_scan");
    }

    // 从 PointCloud2 转换为 LaserScan
    void pointcloudToLaserScan(const sensor_msgs::msg::PointCloud2& cloud, sensor_msgs::msg::LaserScan& laser_scan) {
        // 示例：将 PointCloud2 转换为 LaserScan
        laser_scan.header = cloud.header;
        laser_scan.angle_min = -M_PI;
        laser_scan.angle_max = M_PI;
        laser_scan.angle_increment = 2 * M_PI / 360; // 假设360度扫描
        laser_scan.range_min = 0.0;
        laser_scan.range_max = 360.0;

        laser_scan.ranges.resize(360, std::numeric_limits<float>::infinity());

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);

    for (const auto& point : pcl_cloud.points) {
        // 计算点的角度
        float angle = atan2(point.y, point.x);
        // 将角度转换为范围数组的索引
        int index = static_cast<int>((angle - laser_scan.angle_min) / laser_scan.angle_increment);
        if (index >= 0 && index < 360) {
            // 计算点的距离
            float range = sqrt(point.x * point.x + point.y * point.y);
            // 仅在新距离小于当前值时更新距离（保留最近的障碍物）
            if (range < laser_scan.ranges[index]) {
                laser_scan.ranges[index] = range;
            }
        }
    }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud1_, cloud2_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanMerger>());
    rclcpp::shutdown();
    return 0;
}
