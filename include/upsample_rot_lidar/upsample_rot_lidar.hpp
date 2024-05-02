// Headers in Boost
#include <boost/optional.hpp>

// Headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
// #include <quaternion_operation/quaternion_operation.h>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

#include <upsample_rot_lidar/kdtree.h>

// https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html
pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_down_sampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float x, float y, float z){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(x, y, z);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}


class MyPoint : public std::array<double, 2>
{
public:

	// dimension of space (or "k" of k-d tree)
	// KDTree class accesses this member
	static const int DIM = 2;

	// the constructors
	MyPoint() {}
	MyPoint(double x, double y)
	{ 
		(*this)[0] = x;
		(*this)[1] = y;
	}
};

class UpsampleRotLidar
{
public:
    UpsampleRotLidar(ros::NodeHandle nh, ros::NodeHandle pnh):tfListener(tfBuffer)
    {
        nh_ = nh;
        pnh_ = pnh;
        pnh_.param<std::string>("input_topic", input_topic_, "/vlp32/velodyne_points");
        pnh_.param<double>("weight_coefficient_gain", weight_coefficient_gain, 0.5);
        weight_coefficient = -1.0 * weight_coefficient_gain;
        pnh_.param<int>("grid_num_x", grid_num_x, 360);
        pnh_.param<int>("grid_num_y", grid_num_y, 40);
        pnh_.param<double>("ignore_range", ignore_range, 1.5);
        // pnh_.param<int>("resolution_x", resolution_x, 360);
        // pnh_.param<int>("resolution_y", resolution_y, 40);
        pc_sub_ = nh_.subscribe(input_topic_, 1, &UpsampleRotLidar::pcCallback, this);
        pc_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("upsample_points", 1);
        alpha_resolution = abs(deg2rad(360)/(double)grid_num_x);
        omega_resolution = abs(deg2rad(40)/(double)grid_num_y);
        // 
        depth_img_from_lidar.resize(grid_num_x);
        for (int i = 0; i < depth_img_from_lidar.size(); i++)
        {
            depth_img_from_lidar[i].resize(grid_num_y); 
        }
    }

    ~UpsampleRotLidar()
    {

    }

    void pcCallback(const sensor_msgs::PointCloud2::ConstPtr msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->clear();
        /////////////////////////////////////////////////////////////////
        // transform point cloud
        // geometry_msgs::TransformStamped transformStamped;
        // try
        // {
        //     // transformStamped = tfBuffer.lookupTransform("footprint", msg->header.frame_id, ros::Time(0.0), ros::Duration(0.0));
        // }
        // catch (tf2::TransformException& ex)
        // {
        //     ROS_WARN("%s", ex.what());
        //     return;
        // }
        // sensor_msgs::PointCloud2 conv_cloud_msg_2;
        // Eigen::Matrix4f mat = tf2::transformToEigen(transformStamped.transform).matrix().cast<float>();
        // pcl_ros::transformPointCloud(mat, *msg, conv_cloud_msg_2);
        // pcl::fromROSMsg(conv_cloud_msg_2, *cloud); 
        /////////////////////////////////////////////////////////////////
        pcl::fromROSMsg(*msg, *cloud);
        cloud = voxel_grid_down_sampling(cloud, 0.05, 0.05, 0.05);

        std::vector<MyPoint> points;
        for (auto pc : *cloud){
            double r = sqrt(pow(pc.x, 2.0) + pow(pc.y, 2.0) + pow(pc.z, 2.0));
            if (r < 1.5) continue;
            double alpha = atan2(pc.y, pc.x);
            double omega = atan2(pc.z, sqrt(pow(pc.x, 2.0) + pow(pc.y, 2.0)));
            int alpha_g, omega_g;
            transformRealToGrid(alpha, omega, deg2rad(-180), deg2rad(-25), 
                                alpha_resolution, omega_resolution, alpha_g, omega_g);
            if (alpha_g < 0 || grid_num_x <= alpha_g || omega_g < 0 || grid_num_y <= omega_g) continue;
            depth_img_from_lidar[alpha_g][omega_g] = r;
            MyPoint a(alpha, omega);
            points.push_back(a);
        }

        // // output file name
        // std::ofstream outputfile("/home/tomson784/catkin_ws/test.csv");
        // // output data
        // for (auto alpha : depth_img_from_lidar) {
        //     for (auto omega : alpha) {
        //         outputfile << omega;
        //         outputfile << ',';
        //     }
        //     outputfile << '\n';
        // }
        // outputfile.close();

        // build k-d tree
        // kdt::KDTree<MyPoint> kdtree(points);

        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        new_cloud->clear();
        for (int i=0; i<depth_img_from_lidar.size(); i++){
            for (int j=0; j<depth_img_from_lidar[0].size(); j++){
                if (depth_img_from_lidar[i][j] < 0.0001){
                    double r_alp, r_omg;
                    transformGridToReal(i, j, deg2rad(-180), deg2rad(-25), 
                                        alpha_resolution, omega_resolution, r_alp, r_omg);
                    ///////////////////////////////////////////////////////////////////////////////////////////////
                    // k nearest neightbor search by k-d tree
                    // MyPoint query(r_alp, r_omg);
                    // const std::vector<int> knnIndices = kdtree.knnSearch(query, 6);
                    // double up = 0;
                    // double down = 0;
                    // for (auto idx : knnIndices){
                    //     double d_i = sqrt(pow(points[idx][0] - query[0], 2.0) + pow(points[idx][1] - query[1], 2.0));
                    //     double w_i = exp(weight_coefficient * d_i);
                    //     int alpha_g, omega_g;
                    //     transformRealToGrid(points[idx][0], points[idx][1], deg2rad(-180), deg2rad(-25), deg2rad(1), alpha_g, omega_g);
                    //     up += w_i * depth_img_from_lidar[alpha_g][omega_g];
                    //     down += w_i;
                    // }
                    ///////////////////////////////////////////////////////////////////////////////////////////////
                    std::vector<std::vector<int>> indexes = vertical_search(depth_img_from_lidar, i, j);
                    double up = 0;
                    double down = 0;
                    for (auto idx_pair : indexes){
                        double p_i_pos_alpha, p_i_pos_omega;
                        transformGridToReal(idx_pair[0], idx_pair[1], deg2rad(-180), deg2rad(-25), 
                                            alpha_resolution, omega_resolution, p_i_pos_alpha, p_i_pos_omega);
                        double d_i = sqrt(pow(p_i_pos_alpha - r_alp, 2.0) + pow(p_i_pos_omega - r_omg, 2.0));
                        double w_i = exp(weight_coefficient * d_i);
                        up += w_i * depth_img_from_lidar[idx_pair[0]][idx_pair[1]];
                        down += w_i;
                    }
                    ///////////////////////////////////////////////////////////////////////////////////////////////
                    const double P = up/down;
                    pcl::PointXYZ a_pc;
                    a_pc.x = P*cos(r_omg)*cos(r_alp);
                    a_pc.y = P*cos(r_omg)*sin(r_alp);
                    a_pc.z = P*sin(r_omg);
                    new_cloud->points.push_back(a_pc);
                }
            }
        }
        *new_cloud += *cloud;
        sensor_msgs::PointCloud2 output_upsampling_cloud;
        pcl::toROSMsg(*new_cloud, output_upsampling_cloud);
        output_upsampling_cloud.header.frame_id =  msg->header.frame_id;
        pc_pub_.publish(output_upsampling_cloud);

    }
    
    std::vector<std::vector<int>> vertical_search(std::vector<std::vector<double>> array, const int alpha_g, const int omega_g){

        std::vector<std::vector<int>> alpha_omega_g;
        int alpha_id = alpha_g;
        int alpha_increment_id = -1;
        while (alpha_omega_g.size() < 6)
        {
            // alpha_increment_id
            alpha_increment_id++;
            if (alpha_increment_id == 0){
            }
            // 奇数
            else if ((int)(alpha_increment_id % 2) == 1){
                alpha_id += alpha_increment_id;
            }
            // 偶数
            else if ((int)(alpha_increment_id % 2) == 0){
                alpha_id -= alpha_increment_id;
            }
            if (alpha_id < 0 || array.size() <= alpha_id) continue;
            // 縦方向の探索
            int omega_id = omega_g;
            while (omega_id < array[0].size())
            {
                omega_id++;
                if (array[alpha_id][omega_id] > 1e-3){
                    alpha_omega_g.push_back({alpha_id, omega_id});
                    break;
                }
            }
            omega_id = omega_g;
            while (0 <= omega_id)
            {
                omega_id--;
                if (array[alpha_id][omega_id] > 1e-3){
                    alpha_omega_g.push_back({alpha_id, omega_id});
                    break;
                }
            }
        }
        return alpha_omega_g;
    }

    void transformGridToReal(const int grid_x, const int grid_y, const double map_origin_x, const double map_origin_y, 
                             const double grid_resolution, double &real_x, double &real_y){
        real_x = (double)grid_x*grid_resolution + map_origin_x + grid_resolution/2.0;
        real_y = (double)grid_y*grid_resolution + map_origin_y + grid_resolution/2.0;
    }

    void transformRealToGrid(const double real_x, const double real_y, const double map_origin_x, const double map_origin_y, 
                             const double grid_resolution, int &grid_x, int &grid_y){
        grid_x = (int)((real_x - map_origin_x)/grid_resolution);
        grid_y = (int)((real_y - map_origin_y)/grid_resolution);
    }

    void transformGridToReal(const int grid_x, const int grid_y, const double map_origin_x, const double map_origin_y, 
                             const double grid_reso_x, const double grid_reso_y, double &real_x, double &real_y){
        real_x = (double)grid_x*grid_reso_x + map_origin_x + grid_reso_x/2.0;
        real_y = (double)grid_y*grid_reso_y + map_origin_y + grid_reso_y/2.0;
    }

    void transformRealToGrid(const double real_x, const double real_y, const double map_origin_x, const double map_origin_y, 
                             const double grid_reso_x, const double grid_reso_y, int &grid_x, int &grid_y){
        grid_x = (int)((real_x - map_origin_x)/grid_reso_x);
        grid_y = (int)((real_y - map_origin_y)/grid_reso_y);
    }

    inline double deg2rad(const double deg) { return deg * M_PI / 180.0; }
    inline double rad2deg(const double rad) { return rad * 180.0 / M_PI; }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pc_pub_;
    ros::Subscriber pc_sub_;
    std::string input_topic_;
    std::vector<std::vector<double>> depth_img_from_lidar;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    double alpha_resolution;
    double omega_resolution;
    double weight_coefficient_gain;
    double weight_coefficient;
    int grid_num_x;
    int grid_num_y;
    // double resolution_x;
    // double resolution_y;
    double ignore_range;
};
