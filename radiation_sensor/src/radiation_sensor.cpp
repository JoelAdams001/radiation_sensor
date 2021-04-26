#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "ground_truth");
  ros::NodeHandle n;
  ros::Publisher ground_truth_pub = n.advertise<visualization_msgs::Marker>("ground_truth", 10);
  ros::Rate ros(30);

  double res = 0.3048;
  float legnth = 10;
  float width = 10;
  float height = 10;
  double intensity = 1.625e-10;
  std::vector<float> source_loc{2,2,3};
  while (ros::ok())
  {
    visualization_msgs::Marker points;
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    int size = (legnth / res)*(width / res)*(height / res);
    ROS_INFO_STREAM(size);
    geometry_msgs::Point p;
    double intens[size];
    int iter = 0;
    for(int i = 0; i < legnth; i = i + res) {
        for(int j = 0; j < width; j = j + res) {
            for(int k = 0; k < height; k = k + res) {
                double dis = sqrt(pow((i-source_loc[0]),2)+pow((j-source_loc[1]),2)+pow((k-source_loc[2]),2)); //Distance from source to point
                //double I = intensity/(4*M_PI*pow(dis,2));
                //intens[iter] = I;
                p.x = i;
                p.y = j;
                p.z = k;
                points.points.push_back(p);
                iter++;
            }
        }
    }
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    points.header.frame_id = "/odom";
    points.header.stamp = ros::Time::now();
    points.ns = "ground_truth";
    points.id = 0;
    points.scale.x = res;
    points.scale.y = res;
    points.pose.orientation.w = 1.0;
    
    points.lifetime = ros::Duration();
    
    ground_truth_pub.publish(points);

    ros.sleep();
  }
}
