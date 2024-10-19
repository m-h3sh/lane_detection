#include <math.h>
#include <functional>
#include <memory>
#include <string>
#include "cuda_runtime.h"
#include "rclcpp/rclcpp.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Dense>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define THREADS 512 
using namespace std;
using namespace Eigen;
using std::placeholders::_1;
template <typename T>
__global__ void dev_matmul(const T *a, const T *b, T *output, int rows)
{
    // a is 3x3 matrix
    // b is 3x1 set of matrices
    // output is 3x1 set of matrices
    int thread_id = threadIdx.x;
    int block_id = blockIdx.x;
    int offset = (block_id * THREADS + thread_id) * 3;

    if (offset < rows * 3)
    {
#pragma unroll 3
        for (int i = 0; i < 3; ++i)
        {
            double temp = 0;
#pragma unroll 3
            for (int k = 0; k < 3; ++k)
            {
                temp += a[i * 3 + k] * b[offset + k];
            }
            output[offset + i] = temp;
        }
    }
}

__global__ void dot(double *a, double *b, double *rot_mat, double camera_height, int rows)
{
    int thread_id = threadIdx.x;
    int block_id = blockIdx.x;
    int offset = (block_id * THREADS + thread_id) * 3;

    if (offset < rows * 3)
    {
        double temp = 0;
#pragma unroll 3
        for (int i = 0; i < 3; ++i)
        {
            temp += a[i] * b[offset + i];
        }
	// temp will be the denom[i]
	// we transform the point in place
	// i.e. b will now contain points in the 
	// camera frame, in the base link axes. 
	double swap_temp = b[offset + 2];
	double factor = camera_height/temp;
	b[offset + 2] = -factor* b[offset + 1];
	b[offset + 1] = -factor * b[offset];
	b[offset] = factor * swap_temp;

	// transfrom to the road frame 
	// using the rotation matrix
	double x=0.0, y=0.0, z=0.0;
#pragma unroll 3
	for(int i=0;i < 3; ++i){
		x += rot_mat[i] * b[offset + i];
		y += rot_mat[3 + i] * b[offset + i];
		z += rot_mat[6 + i] * b[offset + i];
	}
	b[offset] = x;
	b[offset + 1] = y;
	b[offset + 2] = z;

    }
}

void log(cudaError_t &&error, int line = 0)
{
    // cout << cudaGetErrorString(error) << "line : " << line << '\n' << flush;
}

#define log(x) log(x, __LINE__)

class IPM : public rclcpp::Node
{
  public:
    IPM() : Node("ipm")
    {
        // NOTE: this is important for simulation, else rviz cups
	this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        left_caminfo_subscription =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera1/camera_info", 10,
                std::bind(&IPM::left_info_callback, this, _1));

        left_img_subscription =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/model_lanes", 10,
                std::bind(&IPM::left_img_callback, this, _1));
		

        right_caminfo_subscription =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/short_1_camera/camera_info", 10,
                std::bind(&IPM::right_info_callback, this, _1));

        right_img_subscription =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/model_lanes2", 10,
                std::bind(&IPM::right_img_callback, this, _1));

	publisher_left = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ipm_left", 10);
 
	publisher_right = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ipm_right", 10);

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	while(true){
		std::string toFrameRel = "base_link";
		try 
		{
			t1 = tf_buffer_->lookupTransform(
					toFrameRel, "camera_short_link",
					tf2::TimePointZero);
			t2 = tf_buffer_->lookupTransform(
					toFrameRel, "camera_link",
					tf2::TimePointZero);

			RCLCPP_INFO(this->get_logger(), "transform received! starting ipm...");
			break;
		} 
		catch (const tf2::TransformException & ex) 
		{
	//	RCLCPP_INFO(
	//			this->get_logger(), "ERROR: %s, transform from %s to %s not available",
	//			ex.what(), toFrameRel.c_str(), "camera frame");
		}
	}
    }

  private:
    void right_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        this->right_camera_info = *msg;
    }
    void left_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        this->left_camera_info = *msg;
    }
    void right_img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
	    process_img(msg, "camera_short_link", right_camera_info, publisher_right);
    }
    void left_img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
	    process_img(msg, "camera_link", left_camera_info, publisher_left);
    }
    void process_img(const sensor_msgs::msg::Image::SharedPtr msg, 
		    std::string &&frame, 
		    sensor_msgs::msg::CameraInfo &camera_info, 
		    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_)
    {
	// processing recieved image
	sensor_msgs::msg::PointCloud2 pub_pointcloud;
	unique_ptr<PointCloud> cloud_msg  = std::make_unique<PointCloud>();
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	cv::Mat gray_image;
	cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);
	cv::inRange(gray_image, cv::Scalar(245), cv::Scalar(255), gray_image);
	cv::Mat nonZeroCoordinates;
	cv::findNonZero(gray_image, nonZeroCoordinates);

	/*
	 * ipm code: refer to https://thomasfermi.github.io/Algorithms-for-Automated-Driving/LaneDetection/InversePerspectiveMapping.html
	 */

	auto t = (frame == "camera_link")?t2:t1;
	// There are two different axes, one followed 
	// by the base link and the other used in the code
	// for ipm calculation. 
	// assume xyz to be base link and x'y'z' to 
	// be ipm axes. then the relation holds:
	//
	// x == z'
	// y == -x'
	// z == -y'
	//
	// these quaternions are in the base_link axes 
	double q0 = t.transform.rotation.w;
	double q1 = t.transform.rotation.x;
	double q2 = t.transform.rotation.y;
	double q3 = t.transform.rotation.z;

	Eigen::Matrix<double, 3, 3, RowMajor> k;
	k(0, 0) = 2 * (q0 * q0 + q1 * q1) - 1;
	k(0, 1) = 2 * (q1 * q2 - q0 * q3);
	k(0, 2) = 2 * (q1 * q3 + q0 * q2);
	k(1, 0) = 2 * (q1 * q2 + q0 * q3);
	k(1, 1) = 2 * (q0 * q0 + q2 * q2) - 1;
	k(1, 2) = 2 * (q2 * q3 - q0 * q1);
	k(2, 0) = 2 * (q1 * q3 - q0 * q2);
	k(2, 1) = 2 * (q2 * q3 + q0 * q1);
	k(2, 2) = 2 * (q0 * q0 + q3 * q3) - 1;

	auto k_inv = k.inverse();
	// normal vector to road in base link axes
	Eigen::Matrix<double, 3, 1> nor;
	nor(0,0) = 0.0f;
	nor(1,0) = 0.0f;
	nor(2,0) = 1.0f;

	Eigen::Matrix<double, 1, 3, RowMajor> nT_base_link_axes = (k_inv * nor).transpose();
								
	// transformed normal vector( ncT ) in ipm axes
	Eigen::Matrix<double, 1, 3, RowMajor> nT;
	// x' == -y
	// y' == -z
	// z' ==  x
	nT[0] = -nT_base_link_axes[1];
	nT[1] = -nT_base_link_axes[2];
	nT[2] = nT_base_link_axes[0];

	// no of points to map
	cv::Size s = nonZeroCoordinates.size();
	int rows = s.height;
	std::cout << "rows : " << rows << '\n';
	auto caminfo = camera_info.k;
	Eigen::Map<Matrix<double, 3, 3, RowMajor>> mat(caminfo.data());
	mat = mat.inverse();
	double *inv_caminfo = mat.data();
	vector<double> kin_uv(3 * rows), uv_hom(3 * rows), denom(rows);

	// figure out how to get raw pointer from nonZeroCoordinates
	for (int i = 0; i < rows; ++i)
	{
	    int x = nonZeroCoordinates.at<cv::Point>(i).x;
	    int y = nonZeroCoordinates.at<cv::Point>(i).y;
	    uv_hom[i * 3] = x;
	    uv_hom[i * 3 + 1] = y;
	    uv_hom[i * 3 + 2] = 1;
	}

        // device
        double *d_uv_hom, *d_kin_uv, *d_caminfo, *d_uv, *d_k;
        log(cudaMalloc((void **)&d_uv_hom, sizeof(double) * 3 * rows));
        log(cudaMalloc((void **)&d_kin_uv, sizeof(double) * 3 * rows));
        log(cudaMalloc((void **)&d_caminfo, sizeof(double) * 9));
        log(cudaMalloc((void **)&d_k, sizeof(double) * 9));
        log(cudaMalloc((void **)&d_uv, sizeof(double) * 3));

        // copying to device
        log(cudaMemcpy(d_caminfo, inv_caminfo, sizeof(double) * 9,
                   cudaMemcpyHostToDevice));
        log(cudaMemcpy(d_k, k.data(), sizeof(double) * 9,
                   cudaMemcpyHostToDevice));
        log(cudaMemcpy(d_uv_hom, uv_hom.data(), sizeof(double) * 3 * rows,
                   cudaMemcpyHostToDevice));
        log(cudaMemcpy(d_uv, nT.data(), sizeof(double) * 3, cudaMemcpyHostToDevice));

	// NOTE: place this somewhere else
	// also, add x and y translation as well 
	double camera_height = t.transform.translation.z;
	double camera_x_offset = t.transform.translation.x;
	double camera_y_offset = t.transform.translation.y;

        // batch multiplication
        dim3 dim_grid((rows + THREADS -1) / THREADS, 1);
        dim3 dim_block(THREADS, 1);
        dev_matmul<<<dim_grid, dim_block>>>(d_caminfo, d_uv_hom, d_kin_uv, rows);
        cudaDeviceSynchronize();
        dot<<<dim_grid, dim_block>>>(d_uv, d_kin_uv, d_k, camera_height, rows);
        cudaDeviceSynchronize();

	// copying back to host
	// kin_uv contains the point in the road frame in base link axes
        log(cudaMemcpy(kin_uv.data(), d_kin_uv, sizeof(double) * 3 * rows,cudaMemcpyDeviceToHost));

        for (int i = 0; i < rows; ++i)
        {
		// vec is points in road frame
		pcl::PointXYZ vec;
		// transforming from camera frame to base link frame
		vec.x = kin_uv[i * 3] - camera_x_offset;
		vec.y = kin_uv[i * 3 + 1] - camera_y_offset;
		// there is a z translation between camera frame and 
		// base link
		vec.z = kin_uv[i * 3 + 2] - camera_height;
		cloud_msg->points.push_back(vec);

        }

        cudaFree(d_uv_hom);
        cudaFree(d_uv);
	cudaFree(d_k);
        cudaFree(d_kin_uv);
        cudaFree(d_caminfo);
        cloud_msg->height = 1;
        cloud_msg->width = cloud_msg->points.size();
        cloud_msg->is_dense = false;
        pcl::toROSMsg(*cloud_msg, pub_pointcloud);
        pub_pointcloud.header.frame_id = "base_link";
        // use the internal clock, will use gazebo's clock
        // when use_sim_time set true
        pub_pointcloud.header.stamp = this->get_clock()->now();
	
        // Publishing our cloud image
        publisher_->publish(pub_pointcloud);
		
        cloud_msg->points.clear();
    }

	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_caminfo_subscription;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_img_subscription;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_caminfo_subscription;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_img_subscription;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_left;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_right;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	geometry_msgs::msg::TransformStamped t1, t2;
	sensor_msgs::msg::CameraInfo right_camera_info;
	sensor_msgs::msg::CameraInfo left_camera_info;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IPM>());
    rclcpp::shutdown();
    return 0;
}
