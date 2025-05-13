#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <deque>

class CSM3DNode : public rclcpp::Node {
public:
    CSM3DNode() : Node("csm_3d_node") {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth/color/points", 10,
            std::bind(&CSM3DNode::pointCloudCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("csm_3d/pose", 10);
        RCLCPP_INFO(this->get_logger(), "3D CSM Node started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer_;
    const int MAX_BUFFER = 10;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud_in);

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_in);
        voxel.setLeafSize(0.05f, 0.05f, 0.05f);  // Downsample
        voxel.filter(*cloud_in);

        if (cloud_buffer_.empty()) {
            cloud_buffer_.push_back(cloud_in);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received cloud with %ld points", cloud_in->size());

        // Concatenate buffer into a reference map
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& c : cloud_buffer_) *cloud_ref += *c;

        // Perform ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_ref);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged()) {
            Eigen::Matrix4f tf = icp.getFinalTransformation();
            publishPose(tf);
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }

        // Update buffer
        if (cloud_buffer_.size() >= MAX_BUFFER) cloud_buffer_.pop_front();
        cloud_buffer_.push_back(cloud_in);
    }

    void publishPose(const Eigen::Matrix4f& tf) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = tf(0, 3);
        pose.pose.position.y = tf(1, 3);
        pose.pose.position.z = tf(2, 3);

        Eigen::Matrix3f rot = tf.block<3,3>(0,0);
        Eigen::Quaternionf q(rot);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose_pub_->publish(pose);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CSM3DNode>());
    rclcpp::shutdown();
    return 0;
}
