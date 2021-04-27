#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <math.h>
#include <vector>

bool getHeatMapColor(float value, float *red, float *green, float *blue) {
    const int NUM_COLORS = 4;
    static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0}};
    //Static array of 4 colors: blue, green, yellow, red
    int idx1, idx2;
    float fractBetween = 0; //Fraction between idx1 and idx2 where value is
    
    if(value <= 0){idx1 = idx2 = 0;}
    else if(value >= 1) {idx1 = idx2 = NUM_COLORS-1;}
    else {
     value = value * (NUM_COLORS-1);
     idx1 = floor(value);
     idx2 = idx1+1;
     fractBetween = value - float(idx1);
    }
    
    *red = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
    *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
    *blue = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "ground_truth");
    ros::NodeHandle n;
    ros::Publisher ground_truth_pub = n.advertise<visualization_msgs::Marker>("ground_truth", 10);
    ros::Rate ros(4000);

    double res = 0.3048;
    float legnth = 10;
    float width = 10;
    float height = 4;
    double intensity = 4; //1.625e-10;
    std::vector<float> source_loc{5,3,0.5};
    float dx = 0;
    
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "ground_truth";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    
    points.scale.x = res/2;
    points.scale.y = res/2;
    //points.scale.z = res; //For CUBE_LIST
    
    int size = (legnth / res)*(width / res)*(height / res);
    geometry_msgs::Point p;
    std_msgs::ColorRGBA rgb_value;
    int iter = 0;
    float r,g,b;
    for(double i = 0; i < legnth; i = i + res) {
        for(double j = 0; j < width; j = j + res) {
            for(double k = 0; k < height; k = k + res) {
                double dis = sqrt(pow((i-source_loc[0]),2)+pow((j-source_loc[1]),2)+pow((k-source_loc[2]),2)); //Distance from source to point
                double I = intensity/(4*M_PI*pow(dis,2));
                p.x = i+dx;
                p.y = j;
                p.z = k;
                double ratio;
                if(I < 0.0018){
                    ratio = 0;
                }
                else{
                    ratio = abs(I - 0)/(0.018-0);  
                }
                getHeatMapColor(ratio, &r, &g, &b);
                rgb_value.r = r;
                rgb_value.g = g;
                rgb_value.b = b;
                rgb_value.a = 0.20;
                points.points.push_back(p);
                points.colors.push_back(rgb_value);
                iter++;
            }
        }
    }
    while (ros::ok()){
        ground_truth_pub.publish(points);
        ros.sleep();
    }
}
