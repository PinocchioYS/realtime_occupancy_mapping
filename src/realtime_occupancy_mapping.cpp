#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <superray_gridmap3d/CullingRegionGrid3D.h>

class MappingServer : public rclcpp::Node {
public:
    MappingServer(const std::string& name = "mapping_server") : rclcpp::Node(name), occupancy_gridmap(nullptr) { 
        declare_parameter("resolution", 0.1);       // defalut value: 0.1 [m]
        declare_parameter("max_range", 100.0);      // defalut value: 100 [m]
        declare_parameter("hit_prob", 0.0);         // defalut value: 0.7
        declare_parameter("miss_prob", 0.0);        // defalut value: 0.4
        declare_parameter("fixed_frame_id", "map"); // defalut value: map
    }

    ~MappingServer() {
        delete occupancy_gridmap;
    }

    bool initialize(bool verbose = true) {
        // Initialize parameters
        RESOLUTION = get_parameter("resolution").as_double();
        MAXIMUM_RANGE = get_parameter("max_range").as_double();
        HIT_PROB = get_parameter("hit_prob").as_double();
        MISS_PROB = get_parameter("miss_prob").as_double();
        FIXED_FRAME_ID = get_parameter("fixed_frame_id").as_string();

        if(RESOLUTION < 10e-5)                  { RCLCPP_ERROR_STREAM(get_logger(), "Invalid voxel size: " << RESOLUTION << " [m] < 0"); return false; }
        if(MAXIMUM_RANGE < 10e-5)               { RCLCPP_ERROR_STREAM(get_logger(), "Invalid maximum range: " << MAXIMUM_RANGE << " [m] < 0"); return false; }
        if(HIT_PROB < 0.0 || HIT_PROB > 1.0)    { RCLCPP_ERROR_STREAM(get_logger(), "Invalid hit prob.: " << HIT_PROB << (HIT_PROB < 0.0 ? " < 0" : " > 1")); return false; }
        if(MISS_PROB < 0.0 || MISS_PROB > 1.0)  { RCLCPP_ERROR_STREAM(get_logger(), "Invalid miss prob.: " << MISS_PROB << (MISS_PROB < 0.0 ? " < 0" : " > 1")); return false; }
        if(HIT_PROB < MISS_PROB)                { RCLCPP_ERROR_STREAM(get_logger(), "Invalid hit/miss prob.: " << HIT_PROB << " < " << MISS_PROB); return false; }
        if(FIXED_FRAME_ID.empty())              { FIXED_FRAME_ID = "map"; }

        if(verbose) {
            RCLCPP_INFO_STREAM(get_logger(), "MappingServer Parameters");
            RCLCPP_INFO_STREAM(get_logger(), "   " << "Voxel size: " << RESOLUTION << " [m]");
            RCLCPP_INFO_STREAM(get_logger(), "   " << "Max. range: " << MAXIMUM_RANGE << " [m]");
            RCLCPP_INFO_STREAM(get_logger(), "   " << "Hit Prob. : " << HIT_PROB);
            RCLCPP_INFO_STREAM(get_logger(), "   " << "Miss Prob.: " << MISS_PROB);
        }

        // Create a grid map
        occupancy_gridmap = new gridmap3d::CullingRegionGrid3D(RESOLUTION);
        occupancy_gridmap->setProbHit(HIT_PROB);
        occupancy_gridmap->setProbMiss(MISS_PROB);

        pointcloud_subscriber = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, "/realtime_occupancy_mapping/pointcloud_in"
        );
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()
        );
        tf_buffer->setCreateTimerInterface(timer_interface);
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_pointcloud_subscriber = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            *pointcloud_subscriber, *tf_buffer, FIXED_FRAME_ID, 1, get_node_logging_interface(), get_node_clock_interface()
        );
        tf_pointcloud_subscriber->registerCallback(std::bind(&MappingServer::update_occupancy_map, this, std::placeholders::_1));

        // Visualization Publishers
        occupied_cells_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/realtime_occupancy_mapping/occupied_cells", rclcpp::QoS(1));

        return true;
    }

    /*
     * Update the occupancies of grid map from given pointcloud.
     * A pointcloud subscriber registers this callback function to update the map
     * when a new sensor measurement is observed.
     *
     * @param src_pc: a pointcloud in the sensor coordinate
     */
    void update_occupancy_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr _src_pc) {
        gridmap3d::Pointcloud pointcloud;
        gridmap3d::point3d origin;
        if(!parse_valid_sensor_measurement(*_src_pc, pointcloud, origin)) {
            RCLCPP_ERROR_STREAM(get_logger(), "Invalid sensor measurement.");
            return;
        }

        occupancy_gridmap->insertPointCloudRays(pointcloud, origin);

        if(occupied_cells_publisher->get_subscription_count() > 0)
            publish_occupied_cells();
    }

    /*
     * Publish the occupied cells for visualization in RViz.
     * As a simple message type, the function publishes a set of centers of occupied cells.
     *
     */
    void publish_occupied_cells() {
        // Find a set of the occupied cells and collect the center points.
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
        for(const auto& node : *(occupancy_gridmap->getGrid())) {
            if(occupancy_gridmap->isNodeOccupied(node.second)) {
                gridmap3d::point3d center = occupancy_gridmap->keyToCoord(node.first);
                pcl_pointcloud.emplace_back(center.x(), center.y(), center.z());
            }
        }

        // Convert the centers to ROS message; pcl::PointCloud --> sensor_msgs::PointCloud2
        sensor_msgs::msg::PointCloud2 msg_pointcloud;
        pcl::toROSMsg(pcl_pointcloud, msg_pointcloud);
        msg_pointcloud.header.frame_id = FIXED_FRAME_ID;
        msg_pointcloud.header.stamp = get_clock()->now();

        // Publish the message
        occupied_cells_publisher->publish(msg_pointcloud);
    }


protected:
    // Transform and pointcloud subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_subscriber;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_pointcloud_subscriber;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    // Options =========================================================================================================
    std::string FIXED_FRAME_ID;
    double RESOLUTION;
    double MAXIMUM_RANGE;
    double HIT_PROB;
    double MISS_PROB;

    // Map publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cells_publisher;

    // Occupancy grid
    gridmap3d::CullingRegionGrid3D* occupancy_gridmap;

    /*
     * Parse the valid points in the sensor measurement
     * and transform the data from the sensor coordinate to the world coordinate.
     *
     * @param ros_pc: [input]  point cloud in the sensor coordinate
     * @param pc:     [output] point cloud consisting of valid points in the world coordinate
     * @param origin: [output] sensor origin in the world coordinate
     * @return validity of the sensor measurement
     */
    bool parse_valid_sensor_measurement(const sensor_msgs::msg::PointCloud2& _ros_pc, gridmap3d::Pointcloud& _pc, gridmap3d::point3d& _origin) {
        // Pose of the sensor frame
        geometry_msgs::msg::TransformStamped sensor_to_world;
        try{
            sensor_to_world = tf_buffer->lookupTransform(FIXED_FRAME_ID, _ros_pc.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::TransformException& e) {
            RCLCPP_ERROR_STREAM(get_logger(), "Cannot find a transform from sensor to world: " << _ros_pc.header.frame_id << " --> " << FIXED_FRAME_ID);
            return false;
        }

        // in the sensor coordinate ====================================================================================
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
        pcl::fromROSMsg(_ros_pc, pcl_pointcloud);

        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_sensor_coordinate;
        for(const auto& point : pcl_pointcloud) {
            // Remove the invalid data: NaN
            if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
                continue;

            // Remove the invalid data: out of sensing range
            double l = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if(l > MAXIMUM_RANGE)
                continue;

            pcl_pointcloud_in_sensor_coordinate.push_back(point);
        }

        // in the world coordinate =====================================================================================
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_world_coordinate;
        pcl_ros::transformPointCloud(pcl_pointcloud_in_sensor_coordinate, pcl_pointcloud_in_world_coordinate, sensor_to_world);

        _pc.reserve(pcl_pointcloud_in_world_coordinate.size());
        for(const auto& point : pcl_pointcloud_in_world_coordinate)
            _pc.push_back(point.x, point.y, point.z);

        // sensor origin
        _origin.x() = sensor_to_world.transform.translation.x;
        _origin.y() = sensor_to_world.transform.translation.y;
        _origin.z() = sensor_to_world.transform.translation.z;

        return _pc.size() != 0;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<MappingServer> mapping_server = std::make_shared<MappingServer>("realtime_occupancy_mapping");

    bool success = mapping_server->initialize(true);
    if(success)
        rclcpp::spin(mapping_server);

    rclcpp::shutdown();
    
    return 0;
}