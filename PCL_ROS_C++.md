# Point Cloud Library (PCL) in ROS C++

### Useful Links
ROS Industrial: [Building a Perception Pipeline](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html)

ROS Wiki: [PCL Overview](http://wiki.ros.org/pcl/Overview)

ROS Wiki: [PCL Tutorials](http://wiki.ros.org/pcl/Tutorials)

[turtlebot2-tutorials: PCL with ROS using C++](https://dabit-industries.github.io/turtlebot2-tutorials/13-ROSPCL.html)

-----

#### Use `typedef` to hold the point cloud type
```cpp
typedef  pcl::PointXYZI  PointT;
```
### Calculate Centroid
```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
// Object to store the centroid coordinates.
Eigen::Vector4f centroid;
pcl::compute3DCentroid(*cloud, centroid);
std::cout << "The XYZ coordinates of the centroid are: ("
		  << centroid[0] << ", "
		  << centroid[1] << ", "
		  << centroid[2] << ")." << std::endl;
```

## ROS

### Always `#include` the `pcl_ros/point_cloud.h` file in the ROS node
```cpp
#include <pcl_ros/point_cloud.h>
```

### Convert ROS->PCL
```cpp
sensor_msgs::PointCloud2::Ptr cloudROSPtr (new sensor_msgs::PointCloud2);

pcl::PointCloud<PointT>  cloudPCL;
pcl::fromROSMsg (*cloudROSPtr, cloudPCL);
pcl::PointCloud<PointT>::Ptr cloudPCLPtr (new pcl::PointCloud<PointT> (cloudPCL));
```

### Convert PCL->ROS
```cpp
pcl::PointCloud<PointT> cloudPCL;
pcl::PointCloud<PointT>::Ptr cloudPCLPtr (new pcl::PointCloud<PointT>  (cloudPCL));

sensor_msgs::PointCloud2::Ptr cloudROSPtr (new sensor_msgs::PointCloud2);
pcl::toROSMsg(*cloudPCLPtr, *cloudROSPtr);
```

### Publisher
> The publisher takes care of the conversion (serialization) between sensor_msgs::PointCloud2 and pcl::PointCloud<T> where needed.
```cpp
ros::Publisher pcl_pub
pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_topic", int queue_size);
```

### Subscriber
> When subscribing to a pcl::PointCloud<T> topic with a sensor_msgs::PointCloud2 subscriber or viceversa, the conversion (deserialization) between the two types sensor_msgs::PointCloud2 and pcl::PointCloud<T> is done on the fly by the subscriber.
```cpp
ros::Subscriber  pclSub  =  nh.subscribe("cloud_topic", int queue_size,  cloud_callback);
```

### Callback
```cpp
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
// ......
}
```

### Transform PointCloud from Camera Frame to World Frame
```cpp
sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

tf::TransformListener listener;
tf::StampedTransform stamped_transform;

try {
  listener.waitForTransform(world_frame, recent_cloud->header.frame_id, ros::Time::now(),  ros::Duration(6.0));
  listener.lookupTransform(world_frame, recent_cloud->header.frame_id, ros::Time(0),  stamped_transform);
} 
catch (tf::TransformException ex) {
  ROS_ERROR("%s",ex.what());
}

sensor_msgs::PointCloud2 transformed_cloud;

pcl_ros::transformPointCloud(world_frame, stamped_transform, *recent_cloud,  transformed_cloud);
```

### Example PCL Node with Publisher, Subscriber and Callback
```cpp
/*  ========================================
*  Point Cloud Callback
*  ========================================*/
void  cloud_cb(const  sensor_msgs::PointCloud2ConstPtr  &msg){
  /*  ========================================
  *  CONVERT POINTCLOUD ROS->PCL
  *  ========================================*/
  pcl::PointCloud<PointT>  cloudPCL;
  pcl::fromROSMsg  (*msg,  cloudPCL);
  pcl::PointCloud<PointT>::Ptr  cloudPCLPtr  (new  pcl::PointCloud<PointT>  (cloudPCL));

  // ..........

  /*  ========================================
  *  CONVERT POINTCLOUD PCL->ROS
  *  ========================================*/
  sensor_msgs::PointCloud2::Ptr  cloudROSPtr  (new  sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloudPCLPtr,  *cloudROSPtr);
  cloudROSPtr->header.frame_id=world_frame;
  cloudROSPtr->header.stamp=ros::Time::now();
  pub.publish(cloudROSPtr);
}

int  main(int  argc,  char  **argv){
  ros::init(argc,  argv,  "line_detection");
  ros::NodeHandle  nh;
  param  =  nh.param<type>("param_name",  int default_val);

  /*  ========================================
  *  Point Cloud SUBSCRIBER
  *  ========================================*/
  ros::Subscriber sub = nh.subscribe("topic_name", int queue_size, callback);

  /*  ========================================
  *  Point Cloud SUBSCRIBER
  *  ========================================*/
  pubPCLTemplate    =  nh.advertise<pcl::PointCloud<PointT>>("topic_name",  int queue_size);
  pubROSPointCloud2 =  nh.advertise<sensor_msgs::PointCloud2>("topic_name",  int queue_size);

  ros::spin();
  return  0;
}
```


