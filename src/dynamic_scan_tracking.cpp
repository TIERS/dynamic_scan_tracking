// Include the ROS library
#include <ros/ros.h>

// Include ROS message types
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Include the LIVOX LIDAR drivers library
#include <livox_ros_driver/CustomMsg.h>

// Include the PCL library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

// Include tf2 library to use quaternions in pose calculation
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  

#include <dynamic_scan_tracking/utils.h>
#include "kalman_filter.cpp"

typedef pcl::PointXYZINormal PointType;

int MIN_INTEGRATION_TIME_AST = 1; // minimum integration time for Adaptive Sparse Tracking values
int MAX_INTEGRATION_TIME_AST = 5;
int MIN_INTEGRATION_TIME_ADT = 10; // minimum integration time for Adaptive Dense Tracking values
int MAX_INTEGRATION_TIME_ADT = 50;

float MAX_DISTANCE_AST = 25.0; // maximum distance at which the LiDAR detects at least 4 points
float MAX_DISTANCE_ADT = 60.0;

class DynamicScanTrackingNode
{

    private:

        // NodeHandle is the main access point to communications with the ROS system.
        // Start the node by initialising a node handle
        ros::NodeHandle nh;
        ros::NodeHandle pnh; // A private NodeHandle makes the nodes name part of the namespace. Good for parameters

        // Define Subscribers
        ros::Subscriber sub_livox_lidar;
        ros::Subscriber sub_initial_position;

        // Define Publishers
        ros::Publisher pub_object_pose; // Final pose from intersection AST and ADT
        ros::Publisher pub_object_velocity;
        ros::Publisher pub_target_pcl;
        
        // Define vector used to transform livox custom msg into PointCloud2
        std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

        // Initialize search radius for object extraction in KDTree
        float search_radius = 0;

        // Define initial position used in Kalman filter
        PointType obj_position; // Defined only to retrieve position from parameters
        bool position_initialized;
        bool drone_detection_enabled; // Detect drone using range images

        // Define vector containing object pose sequence
        std::vector< geometry_msgs::PoseStamped > pose_vector_ast;
        std::vector< geometry_msgs::PoseStamped > pose_vector_adt;
        std::vector< geometry_msgs::PoseStamped > pose_vector_final; // intersection AST and ADT

        // Define current and previous pose (previous pose used to compute velocity and orientation)
        geometry_msgs::PoseStamped current_pose_ast;
        geometry_msgs::PoseStamped current_pose_adt;
        geometry_msgs::PoseStamped current_pose_ground_truth;
        geometry_msgs::PoseStamped final_pose; // Final pose intersecrtion AST and ADT

        geometry_msgs::TwistStamped final_velocity;

        geometry_msgs::PoseStamped previous_pose_ast;
        geometry_msgs::PoseStamped previous_pose_adt;
        geometry_msgs::PoseStamped previous_pose_ground_truth;
        geometry_msgs::PoseStamped previous_pose_final; // final intersected value

        // Define delta t used in the motion velocity model
        // It is computed when velocity is calculated
        double delta_t_pose = 0.01; // based on livox LiDAR frequency

        // Define predicted future positions for each integration time. Used in KDTree to search for new object position
        PointType future_position_ast;
        PointType future_position_adt;

        // Define the point cloud objects to store up to N scans
        pcl::PointCloud<PointType>::Ptr point_cloud_current_scan; // current livox message used to update point clouds at different integration times
        pcl::PointCloud<PointType>::Ptr point_cloud_ast;
        pcl::PointCloud<PointType>::Ptr point_cloud_adt;

        pcl::PointCloud<PointType>::Ptr obj_cloud_ast;
        pcl::PointCloud<PointType>::Ptr obj_cloud_adt;

        // Define the deques to store the last N scans
        std::deque<pcl::PointCloud<PointType>> deque_ast;
        std::deque<pcl::PointCloud<PointType>> deque_adt;

        float gamma; // constant used to compute weight for each point

        // Define Kalman Filter object, related matrices and parameters
        KalmanFilter kf_ast; // kalman filter for position estimate using small integration times  
        KalmanFilter kf_adt;

        VectorXd x_in; // Initial state
        MatrixXd P_in; // Initial state covariance
        MatrixXd F_in; // Transition matrix
        MatrixXd H_in; // Measurement matrix
        MatrixXd R_in; // Measurement covariance ma
        MatrixXd Q_in; // Process covariance matrix

        float sigma_p44;  // initial setting for estimation error covariance P entry for vx
        float sigma_p55; // initial setting for estimation error covariance P entry for vy
        float sigma_p66;  // initial setting for estimation error covariance P entry for vz

        float sigma_lidar_x;  // measurement noise standard deviation for lidar x position
        float sigma_lidar_y;  // measurement noise standard deviation for lidar y position
        float sigma_lidar_z;  // measurement noise standard deviation for lidar z position

        // Define optimal integration time
        int optimal_integration_time_ast;
        int optimal_integration_time_adt;

        // Define object distance
        float distance_ast;
        float distance_adt;

        // Define fused final pose, velocity and covariance
        VectorXd pose_fused;
        MatrixXd cov_fused;
        double omega; // weight in adaptive covariance intersection

    public:

        // Constructor. Initializes the node with regular and private NodeHandle
        DynamicScanTrackingNode():nh(), pnh("~")
        {

            // Define subscribers and publishers
            sub_livox_lidar = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 0, &DynamicScanTrackingNode::livoxLidarCallback, this); // LiDAR point cloud
            sub_initial_position = nh.subscribe<geometry_msgs::PoseStamped>("/init_position", 0, &DynamicScanTrackingNode::droneDetectionCallback, this); // initial position

            // Publishers for poses calculated for different integration times
            pub_object_pose = nh.advertise<geometry_msgs::PoseStamped>("/dynamic_scan_tracking/object_pose", 1000);
            pub_object_velocity = nh.advertise<geometry_msgs::TwistStamped>("/dynamic_scan_tracking/object_velocity", 1000);
            pub_target_pcl = nh.advertise<sensor_msgs::PointCloud2>("/target_pcl", 1000); // used to visualize current livox point cloud scan

            // Initialize position and drone detection to false (changed later according to parameters)
            position_initialized = false;
            drone_detection_enabled = false;

            // Retrieve parameters
            if(!pnh.getParam("search_radius", search_radius))
            {
                search_radius = 0.2; // Initialize search radius used in KDTree
            }

            if(!pnh.getParam("drone_detection_enabled", drone_detection_enabled))
            {
                drone_detection_enabled = false;
            }

            // If drone detection is not enabled, initialize object position from yaml config file
            if(!drone_detection_enabled)
            {
                if(!pnh.getParam("initial_position_x", obj_position.x)
                    || !pnh.getParam("initial_position_y", obj_position.y) 
                    || !pnh.getParam("initial_position_z", obj_position.z))
                {
                    obj_position.x = 0.0;
                    obj_position.y = 0.0;
                    obj_position.z = 0.0;
                }
                else
                {
                    position_initialized = true; // initial position defined from yaml config file
                }
            }
            

            if(!pnh.getParam("gamma", gamma))
            {
                gamma = 0.000000005; // Initialize constant for weighted sum
            }

            if(!pnh.getParam("sigma_p44", sigma_p44)
                || !pnh.getParam("sigma_p55", sigma_p55)
                || !pnh.getParam("sigma_p66", sigma_p66))
            {
                sigma_p44 = 50;
                sigma_p55 = 50;
                sigma_p66 = 5;
            }

            if(!pnh.getParam("sigma_lidar_x", sigma_lidar_x)
                || !pnh.getParam("sigma_lidar_y", sigma_lidar_y)
                || !pnh.getParam("sigma_lidar_z", sigma_lidar_z))
            {
                sigma_lidar_x = 0.1;
                sigma_lidar_y = 0.1;
                sigma_lidar_z = 0.1;
            }

            // Initialize point cloud objects
            point_cloud_current_scan.reset(new pcl::PointCloud<PointType>()); //full point cloud of current scan
            obj_cloud_ast.reset(new pcl::PointCloud<PointType>());
            obj_cloud_adt.reset(new pcl::PointCloud<PointType>());
            point_cloud_ast.reset(new pcl::PointCloud<PointType>());
            point_cloud_adt.reset(new pcl::PointCloud<PointType>());

            // Initialize Kalman Filter objects
            x_in = VectorXd::Zero(6);
            x_in << obj_position.x, obj_position.y, obj_position.z, 0, 0, 0;
            
            P_in = MatrixXd::Zero(6, 6); // initialize covariance error matrix P with zeros
            P_in(0, 0) = 1;
            P_in(1, 1) = 1;
            P_in(2, 2) = 1;
            P_in(3, 3) = sigma_p44*sigma_p44; // fill velocity covariance error part using lidar parameters
            P_in(4, 4) = sigma_p55*sigma_p55;
            P_in(5, 5) = sigma_p66*sigma_p66;

            F_in = MatrixXd::Identity(6,6);
            F_in(0, 3) = 1;
            F_in(1, 4) = 1;
            F_in(2, 5) = 1;
            H_in = MatrixXd::Zero(3,6); // the matrix has this shape because the sensor is a LiDAR
            H_in(0, 0) = 1;
            H_in(1, 1) = 1;
            H_in(2, 2) = 1;
            
            R_in = MatrixXd::Zero(3,3);
            R_in(0, 0) = sigma_lidar_x * sigma_lidar_x;
            R_in(1, 1) = sigma_lidar_y * sigma_lidar_y;
            R_in(2, 2) = sigma_lidar_z * sigma_lidar_z;
            Q_in = Eigen::MatrixXd::Zero(6,6); 

            kf_ast.init(x_in, P_in, F_in, H_in, R_in, Q_in); // AST kalman filter
            kf_adt.init(x_in, P_in, F_in, H_in, R_in, Q_in); // ADT kalman filter

            // Initialize optimal integration time with max integration time in each range
            optimal_integration_time_ast = MAX_INTEGRATION_TIME_AST;
            optimal_integration_time_adt = MAX_INTEGRATION_TIME_ADT;

            // Print parameters values
            ROS_INFO("Starting Dynamic Scan Tracking Node");
            ROS_INFO_STREAM("Namespace of public nh = " << nh.getNamespace());
            ROS_INFO_STREAM("Namespace of private pnh = " << pnh.getNamespace());
            ROS_INFO_STREAM("Drone detection enabled = " << drone_detection_enabled);
            ROS_INFO_STREAM("Search radius = " << search_radius);
            ROS_INFO_STREAM("Initial position = " << obj_position);
        }

    void publishPointCloud(pcl::PointCloud<PointType>::Ptr& pcl_ptr, ros::Publisher& publisher)
    {
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*pcl_ptr.get(), pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "livox_frame";
        pcl_ros_msg.header.stamp = ros::Time::now();
        publisher.publish(pcl_ros_msg);
    }

    void droneDetectionCallback(const geometry_msgs::PoseStamped initial_position)
    {
        if(!position_initialized && drone_detection_enabled)
        {
            x_in << initial_position.pose.position.x, initial_position.pose.position.y, initial_position.pose.position.z, 0, 0, 0;
            kf_ast.init(x_in, P_in, F_in, H_in, R_in, Q_in);
            kf_adt.init(x_in, P_in, F_in, H_in, R_in, Q_in);

            position_initialized = true;
        }
        
    }

    void livoxLidarCallback(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {

        if (position_initialized)
        {
            // Clear Point Cloud of previous message's data
            point_cloud_current_scan->clear();

            // Convert Livox Message to PointCloud2
            livoxMsgToPCL(livox_msg_in, point_cloud_current_scan, livox_data);

            // Retrieve timestamp in nanoseconds from livox data
            // Used in compute pose to define each point's weight
            unsigned long msg_timebase_ns = livox_data[0]->timebase;

            publishPointCloud(obj_cloud_ast, pub_target_pcl); // uncomment to visualize tracked target point cloud

            trackObjectAdaptiveIntegrationTime(point_cloud_ast, point_cloud_current_scan, obj_cloud_ast, deque_ast, future_position_ast,
                                            kf_ast, search_radius, msg_timebase_ns, MAX_INTEGRATION_TIME_AST, optimal_integration_time_ast,
                                            pose_vector_ast, current_pose_ast, previous_pose_ast);

            trackObjectAdaptiveIntegrationTime(point_cloud_adt, point_cloud_current_scan, obj_cloud_adt, deque_adt, future_position_adt,
                                            kf_adt, search_radius, msg_timebase_ns, MAX_INTEGRATION_TIME_ADT, optimal_integration_time_adt,
                                            pose_vector_adt, current_pose_adt, previous_pose_adt);

            // Fuse values obtained by both integration times
            fuseValuesICI(kf_ast.x_, kf_ast.P_, kf_adt.x_, kf_adt.P_, pose_fused, cov_fused, omega);

            // Compute and publish final pose (intersection AST and ADT)
            computePose(pose_vector_final, final_pose, previous_pose_final, pose_fused);

            // Adjust ADT and AST integartion times based on new state estimation
            adjustIntegrationTime(MIN_INTEGRATION_TIME_AST, MAX_DISTANCE_AST, distance_ast, MAX_INTEGRATION_TIME_AST, optimal_integration_time_ast, pose_fused);
            adjustIntegrationTime(MIN_INTEGRATION_TIME_ADT, MAX_DISTANCE_ADT, distance_adt, MAX_INTEGRATION_TIME_ADT, optimal_integration_time_adt, pose_fused);

            // Publish current pose and velocity
            geometry_msgs::TwistStamped current_velocity;
            current_velocity.header.frame_id = "livox_frame";
            current_velocity.header.stamp = ros::Time::now();
            current_velocity.twist.linear.x = pose_fused[3];
            current_velocity.twist.linear.y = pose_fused[4];
            current_velocity.twist.linear.z = pose_fused[5];

            pub_object_pose.publish(final_pose);
            pub_object_velocity.publish(current_velocity);

            // Clear vector containing current livox message data. Each time we only use the current scan msg
            livox_data.clear();
        }
        
    }

    void trackObjectAdaptiveIntegrationTime(pcl::PointCloud<PointType>::Ptr& integrated_cloud, pcl::PointCloud<PointType>::Ptr& new_msg_cloud, 
                                            pcl::PointCloud<PointType>::Ptr& object_cloud, std::deque<pcl::PointCloud<PointType>>& point_cloud_deque,
                                            PointType& future_position, KalmanFilter& kalman_filter, float search_radius, unsigned long msg_timebase_ns,
                                            int max_scans, int& optimal_integration_time,
                                            std::vector< geometry_msgs::PoseStamped >& pose_vector, geometry_msgs::PoseStamped& current_pose, geometry_msgs::PoseStamped& previous_pose)
    {
        // Combine the new scan with the different point clouds for up to "max_scans" scans
        combinePointCloudsAdaptiveIntegrationTime(integrated_cloud, new_msg_cloud, max_scans, optimal_integration_time, point_cloud_deque);

        // Extract object cloud from point cloud integrated from "optimal_integration_time" number of scans based on search_radius using KDTree
        extractObjectCloud(integrated_cloud, object_cloud, future_position, kalman_filter, search_radius);

        // Update object position based on average of points in extracted point cloud
        updatePosition(object_cloud, kalman_filter, msg_timebase_ns);

        // Compute current pose based on kalman filter
        computePose(pose_vector, current_pose, previous_pose, kalman_filter.x_);

    }

    void combinePointCloudsAdaptiveIntegrationTime(pcl::PointCloud<PointType>::Ptr& current_cloud, pcl::PointCloud<PointType>::Ptr& new_msg_cloud,
                            int max_scans, int& optimal_integration_time, std::deque<pcl::PointCloud<PointType>>& point_cloud_deque)
    {

        // Make a standalone copy of the new_msg_cloud object before storing it in the cloud
        // The deque contains the point cloud itself, not a pointer. Pointers end up being
        // updated to the same last PointCloud object
        pcl::PointCloud<PointType> new_cloud(*new_msg_cloud);

        // Add the new point cloud to the deque
        point_cloud_deque.push_front(new_cloud);

        // Clear the current point cloud
        current_cloud->clear();

        // Define number of scans to include in point cloud
        // Smallest value between optimal integration time and size of the deque
        int total_num_scans = (optimal_integration_time < point_cloud_deque.size()) ? optimal_integration_time : point_cloud_deque.size();

        // Add the points from the last max_scans point clouds to the current point cloud
        for (int i = 0; i <= total_num_scans; ++i)
        {
            *current_cloud += point_cloud_deque[i];
        }

        // If the deque has more than max_scans elements, remove the oldest element
        if (point_cloud_deque.size() >= max_scans)
        {
            point_cloud_deque.pop_back();
        }
    }

    void extractObjectCloud(pcl::PointCloud<PointType>::Ptr& point_cloud, 
                            pcl::PointCloud<PointType>::Ptr& object_cloud,
                            PointType& future_position,
                            KalmanFilter& kalman_filter,
                            float search_radius)
    {

        // Initialize KDTree for searching object in point cloud
        pcl::KdTreeFLANN<PointType>::Ptr kd_tree (new pcl::KdTreeFLANN<PointType>());
        kd_tree->setInputCloud(point_cloud);

        // Define KDTree search parameters
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // Clear object cloud of previous message data
        object_cloud->clear();

        // Define object point cloud's timestamp and frame id
        object_cloud->header.frame_id = "livox_frame";
        object_cloud->header.stamp = ros::Time::now().toNSec()/1e3;

        // Calculate predicted position using kalman filter
        // Use future position in KDTree to find object in point cloud
        // kalman_filter.predictEKF(delta_t_pose);
        kalman_filter.predict(delta_t_pose); // Linear Kalman filter prediction
        future_position.x = kalman_filter.x_[0];
        future_position.y = kalman_filter.x_[1];
        future_position.z = kalman_filter.x_[2];
        
        // If neighbors within radius are found
        if(kd_tree->radiusSearch(future_position, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            {
                // Retrieve point belonging to object from point cloud based on index
                PointType pt = (*point_cloud)[pointIdxRadiusSearch[i]];

                // Add point to object point cloud
                object_cloud->points.push_back(pt); 
            }
        }

    }

    void updatePosition(pcl::PointCloud<PointType>::Ptr& object_cloud, KalmanFilter& kalman_filter, unsigned long msg_timebase_ns)
    {
        // Check that object point cloud is not empty
        if(!object_cloud->points.empty()){

            // Define average point position
            PointType avg_pts_position;

            // Define sum of weights for weighted average
            float weights_sum = 0.0;

            // Iterate over points in object point cloud
            for (std::size_t i = 0; i < object_cloud->size(); ++i){
                
                // Retrieve point in object point cloud
                PointType pt = object_cloud->points[i];

                // Compute weight for each point based on timestamp stored in curvature
                float weight = exp(-gamma*(msg_timebase_ns - pt.curvature));
                
                // Update average point position with new point coordinates weighted
                avg_pts_position.x += pt.x * weight;
                avg_pts_position.y += pt.y * weight;
                avg_pts_position.z += pt.z * weight;

                // Sum weights
                weights_sum += weight;
            }

            // Compute average
            avg_pts_position.x /= weights_sum;
            avg_pts_position.y /= weights_sum;
            avg_pts_position.z /= weights_sum;


            // Update observation with
            // average of points detected in point cloud
            VectorXd observation = VectorXd::Zero(3);
            observation << avg_pts_position.x, avg_pts_position.y, avg_pts_position.z;
            kalman_filter.update(observation);
        }

    }

    void computePose(std::vector< geometry_msgs::PoseStamped >& pose_vector,
                     geometry_msgs::PoseStamped& current_pose, geometry_msgs::PoseStamped& previous_pose,
                     VectorXd& pose)
    {

        // Define pose frame id and timestamp (used to compute velocity later)
        current_pose.header.frame_id = "livox_frame";
        current_pose.header.stamp = ros::Time::now();

        // Compute position
        current_pose.pose.position.x = pose[0];
        current_pose.pose.position.y = pose[1];
        current_pose.pose.position.z = pose[2];

        // Compute orientation
        // Check that there is more than one pose before computing orientation
        if( pose_vector.size() > 0){
            
            // Retrieve previous pose
            previous_pose = pose_vector.back();

            // Compute orientation angle (in radians) between current and last pose
            double theta = atan2((current_pose.pose.position.y - previous_pose.pose.position.y),
                                 (current_pose.pose.position.x - previous_pose.pose.position.x));
    
            // Define orientation quaternion
            tf2::Quaternion orientation;
            orientation.setRPY( 0, 0, theta ); // Create quaternion from roll/pitch/yaw (in radians)
            geometry_msgs::Quaternion quaternion_msg; 
            quaternion_msg = tf2::toMsg(orientation); 
    
            current_pose.pose.orientation = quaternion_msg;

        }

        // Add current pose to pose vector
        pose_vector.push_back(current_pose);
        
    }

    void adjustIntegrationTime(int min_integration_time, float max_distance, float& distance, int max_scans, int& optimal_integration_time, VectorXd& pose)
    {
        // Compute object distance
        distance = sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]);
        
        // Compute new optimal integration time based on distance
        optimal_integration_time = std::round(min_integration_time +
                                             (max_scans - min_integration_time) *
                                             (distance / max_distance));
    }

};

int main(int argc, char** argv)
{
    // Initialize the node
    ros::init(argc, argv, "dynamic_scan_tracking_node"); 

    DynamicScanTrackingNode dynamic_scan_tracking_node;

    // Spin as a multi-threaded node 
    ros::MultiThreadedSpinner spinner(15);
    // ros::spin();
    spinner.spin();
 
    // Main has ended, return 0
    return 0;
}
