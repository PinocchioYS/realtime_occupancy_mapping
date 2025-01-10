#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <superray_gridmap3d/CullingRegionGrid3D.h>

class MappingServer {
public:
    MappingServer() : occupancy_gridmap(nullptr) { }

    ~MappingServer() {
        delete pointcloud_subscriber;
        delete occupancy_gridmap;
    }

    bool initialize(bool verbose = true) {
        // Initialize parameters
        if(!nh.getParam("resolution", RESOLUTION))          RESOLUTION = 0.1;       // defalut value: 0.1 [m]
        if(!nh.getParam("max_range", MAXIMUM_RANGE))        MAXIMUM_RANGE = 100.0;  // defalut value: 100 [m]
        if(!nh.getParam("hit_prob", HIT_PROB))              HIT_PROB = 0.7;         // defalut value: 0.7
        if(!nh.getParam("miss_prob", MISS_PROB))            MISS_PROB = 0.4;        // defalut value: 0.4
        if(!nh.getParam("fixed_frame_id", FIXED_FRAME_ID))  FIXED_FRAME_ID = "map"; // defalut value: map

        if(RESOLUTION < 10e-5)                  { ROS_ERROR_STREAM("Invalid voxel size: " << RESOLUTION << " [m] < 0"); return false; }
        if(MAXIMUM_RANGE < 10e-5)               { ROS_ERROR_STREAM("Invalid maximum range: " << MAXIMUM_RANGE << " [m] < 0"); return false; }
        if(HIT_PROB < 0.0 || HIT_PROB > 1.0)    { ROS_ERROR_STREAM("Invalid hit prob.: " << HIT_PROB << (HIT_PROB < 0.0 ? " < 0" : " > 1")); return false; }
        if(MISS_PROB < 0.0 || MISS_PROB > 1.0)  { ROS_ERROR_STREAM("Invalid miss prob.: " << MISS_PROB << (MISS_PROB < 0.0 ? " < 0" : " > 1")); return false; }
        if(HIT_PROB < MISS_PROB)                { ROS_ERROR_STREAM("Invalid hit/miss prob.: " << HIT_PROB << " < " << MISS_PROB); return false; }
        if(FIXED_FRAME_ID.empty())              { FIXED_FRAME_ID = "map"; }

        if(verbose) {
            ROS_INFO_STREAM("MappingServer Parameters");
            ROS_INFO_STREAM("   " << "Voxel size: " << RESOLUTION << " [m]");
            ROS_INFO_STREAM("   " << "Max. range: " << MAXIMUM_RANGE << " [m]");
            ROS_INFO_STREAM("   " << "Hit Prob. : " << HIT_PROB);
            ROS_INFO_STREAM("   " << "Miss Prob.: " << MISS_PROB);
        }

        // Create a grid map
        occupancy_gridmap = new gridmap3d::CullingRegionGrid3D(RESOLUTION);
        occupancy_gridmap->setProbHit(HIT_PROB);
        occupancy_gridmap->setProbMiss(MISS_PROB);

        pointcloud_subscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/realtime_occupancy_mapping/pointcloud_in", 1);
        tf_pointcloud_subscriber = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_subscriber, tf_listener, FIXED_FRAME_ID, 1);
        tf_pointcloud_subscriber->registerCallback(boost::bind(&MappingServer::update_occupancy_map, this, _1));

        // Visualization Publishers
        occupied_cells_publisher = nh.advertise<sensor_msgs::PointCloud2>("/realtime_occupancy_mapping/occupied_cells", 1);

        return true;
    }

    /*
     * Update the occupancies of grid map from given pointcloud.
     * A pointcloud subscriber registers this callback function to update the map
     * when a new sensor measurement is observed.
     *
     * @param src_pc: a pointcloud in the sensor coordinate
     */
    void update_occupancy_map(const sensor_msgs::PointCloud2ConstPtr& _src_pc) {
        gridmap3d::Pointcloud pointcloud;
        gridmap3d::point3d origin;
        if(!parse_valid_sensor_measurement(*_src_pc, pointcloud, origin)) {
            ROS_ERROR_STREAM("Invalid sensor measurement.");
            return;
        }

        occupancy_gridmap->insertPointCloudRays(pointcloud, origin);

        if(occupied_cells_publisher.getNumSubscribers() > 0)
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
        sensor_msgs::PointCloud2 msg_pointcloud;
        pcl::toROSMsg(pcl_pointcloud, msg_pointcloud);
        msg_pointcloud.header.frame_id = FIXED_FRAME_ID;
        msg_pointcloud.header.stamp = ros::Time::now();
        msg_pointcloud.header.seq = 0;

        // Publish the message
        occupied_cells_publisher.publish(msg_pointcloud);
    }


protected:
    // Node handle
    ros::NodeHandle nh;

    // Transform and pointcloud subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud2>*  pointcloud_subscriber;
    tf::MessageFilter<sensor_msgs::PointCloud2>*            tf_pointcloud_subscriber;
    tf::TransformListener                                   tf_listener;

    // Options =========================================================================================================
    std::string FIXED_FRAME_ID;
    double RESOLUTION;
    double MAXIMUM_RANGE;
    double HIT_PROB;
    double MISS_PROB;

    // Map publisher
    ros::Publisher occupied_cells_publisher;

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
    bool parse_valid_sensor_measurement(const sensor_msgs::PointCloud2& _ros_pc, gridmap3d::Pointcloud& _pc, gridmap3d::point3d& _origin) {
        // Pose of the sensor frame
        tf::StampedTransform sensor_to_world;
        try{
            tf_listener.lookupTransform(FIXED_FRAME_ID, _ros_pc.header.frame_id, _ros_pc.header.stamp, sensor_to_world);
        }
        catch(tf::TransformException& e) {
            ROS_ERROR_STREAM("Cannot find a transform from sensor to world: " << _ros_pc.header.frame_id << " --> " << FIXED_FRAME_ID);
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
        Eigen::Matrix4f transform;
        pcl_ros::transformAsMatrix(sensor_to_world, transform);
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_world_coordinate;
        pcl::transformPointCloud(pcl_pointcloud_in_sensor_coordinate, pcl_pointcloud_in_world_coordinate, transform);

        _pc.reserve(pcl_pointcloud_in_world_coordinate.size());
        for(const auto& point : pcl_pointcloud_in_world_coordinate)
            _pc.push_back(point.x, point.y, point.z);

        // sensor origin
        _origin.x() = sensor_to_world.getOrigin().x();
        _origin.y() = sensor_to_world.getOrigin().y();
        _origin.z() = sensor_to_world.getOrigin().z();

        return _pc.size() != 0;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "realtime_occupancy_mapping");

    std::shared_ptr<MappingServer> mapping_server = std::make_shared<MappingServer>();

    bool success = mapping_server->initialize(true);
    if(success) {
        try{
            ros::spin();
        }
        catch(std::runtime_error& e) {
            ROS_ERROR("Exception: %s", e.what());
            return -1;
        }
    }
    
    return 0;
}