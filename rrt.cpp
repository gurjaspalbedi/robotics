/*
Gurjapsal Singh Bedi
Intelligent Systems and Engineering, Indiana University

Program Description: Implementation of RRT algorithm
https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <iostream>
#include <utility>
#include <random>
#include <cmath>
#include <list>


using namespace std;

//Using two point system for saving parent and child nodes
class TwoPoints {
public:
  std::pair<float, float> firstPoint;
  std::pair<float, float> SecondPoint;


};

//Single point system with id
class CordinatewithId {
public:
  std::pair<float, float> cordinates;
  int id;
};

//This class is used to get the obstacles on the world. 
// A object has its center, scale x and scale y. Using 0 for the z
class obstacles {
public:
  std::pair<float, float> _center;
  float _scalex;
  float _scaley;

  obstacles(std::pair<float, float> center, float scalex, float scaley)
  {
    _center = center;
    _scalex = scalex;
    _scaley = scaley;
  }


};

//Class to assign the start point and the goal point on the grid.
class declare_start_goal {
public:
  std::pair<float, float> _goal_point;
  std::pair<float, float> _start_point;


  declare_start_goal(std::pair<float, float> start, std::pair<float, float> goal)
  {

    _start_point = start;
    _goal_point = goal;
  }


};


//Class with all the major RRT operations.
class RRToperations {
public:

  //A vector of two points on the graph. This is used to see the parent child relationship
  vector<TwoPoints> path;

  //All the vertices on the graph are inserted in this vector, although I am not using the ID part of the class
  std::vector<CordinatewithId> rrt_vertices;

  //Stores the list of all the object
  std::vector<obstacles> obs_list;

  //USed to draw the line after we reach the goal
  std::vector<std::pair<float, float>> rrt_pathline;

  //This is the epsilon value with I am using in my RRT algorithm. I tried many different values to test the algorithm.
  static constexpr  float rrt_delta = 0.07;

  //Defining the world cordinates.
  static constexpr std::pair<float, float> robo_World_Begin = { 0,0 };
  static constexpr std::pair<float, float> robo_World_End = { 5,5 };

  RRToperations()
  {
  }

  //Returns the minimum value that can be there for x-cordinate.
  static float getMinimumWorldX() {
    return robo_World_Begin.first;
  }

  //Returns the maximum value that can be there for x-cordinate.
  static float getMaximumWorldX() {
    return robo_World_End.first;
  }

  //Returns the minimum value that can be there for y-cordinate.
  static float getMinimumWorldY() {
    return robo_World_Begin.second;
  }

  //Returns the maximum value that can be there for y-cordinate.
  static float getMaximumWorldY() {

    return robo_World_End.second;

  }

  //To gat the random point in space
  static std::pair<float, float> getRandomPoint()
  {

    std::pair<float, float> randomPoint;

    //I tried many random engines but this one worked well for this problem
    static std::default_random_engine rand;


    //Gettubg random values upto the two decimal places
    std::uniform_int_distribution<int> real_random(getMinimumWorldX() * 100, getMaximumWorldY() * 100);
    std::uniform_int_distribution<int> real_randomY(getMinimumWorldX() * 100, getMaximumWorldY() * 100);
    randomPoint.first = RRToperations::getTwoDecimals(real_random(rand) / 100.00);
    randomPoint.second = RRToperations::getTwoDecimals(real_randomY(rand) / 100.00);
    return randomPoint;
  }


  //To find the distance between the two points
  static float distance_between_points(std::pair<float, float> firstPoint, std::pair<float, float> secondPoint)
  {
    float x_difference = secondPoint.first - firstPoint.first;
    float y_difference = secondPoint.second - firstPoint.second;
    float dist = sqrt(pow(x_difference, 2) + pow(y_difference, 2));
    return dist;

  }

  //To check if the slope is infinite or not
  bool slope_issue_infinite(std::pair<float, float> first, std::pair<float, float> second) {

    if (second.first - first.first == 0)
      return true;
    return false;
  }

  // Gets the closes vertex in graph from the random point which we generate.
  std::pair<float, float> getClosestVertice(std::pair<float, float> randomPoint) {
    float min = 1000;
    std::pair<float, float> closest_vertice;
    for (int i = 0; i < rrt_vertices.size(); i++)
    {
      float distance = distance_between_points(rrt_vertices[i].cordinates, randomPoint);
      if (distance < min && !slope_issue_infinite(randomPoint, rrt_vertices[i].cordinates))
      {
        min = distance;
        closest_vertice.first = getTwoDecimals(rrt_vertices[i].cordinates.first);
        closest_vertice.second = getTwoDecimals(rrt_vertices[i].cordinates.second);
      }
    }
    return closest_vertice;
  }


  // Get new vertice
  static std::pair<float, float> getNewVertice(std::pair<float, float> verticePoint, std::pair<float, float> randomPoint, std::vector<obstacles> ob) {
    std::pair<float, float> newVertice;
    if (randomPoint.first == verticePoint.first)
    {
      randomPoint.first = randomPoint.first = 0.01;
    }
    float slope = (randomPoint.second - verticePoint.second) / (randomPoint.first - verticePoint.first);
    float part = (sqrt(1 + pow(slope, 2)));

    if (rand() % 2 == 1)
    {
      newVertice.first = RRToperations::getTwoDecimals(verticePoint.first - (rrt_delta / part));
      newVertice.second = RRToperations::getTwoDecimals((verticePoint.second - ((slope * rrt_delta) / part)));
    }
    else
    {
      newVertice.first = RRToperations::getTwoDecimals((verticePoint.first + (rrt_delta / part)));
      newVertice.second = RRToperations::getTwoDecimals(verticePoint.second + ((slope * rrt_delta) / part));
    }
    if (not_on_any_obstacle(newVertice, ob) && not_outside_world(newVertice))
    {
      return newVertice;
    }
    return { -100,-100 };

  };

  //To determine if point is not outside our range. 
  bool static not_outside_world(std::pair<float, float> newVertice) {
    if (newVertice.first > getMinimumWorldX() && newVertice.first < getMaximumWorldX() && newVertice.second > getMinimumWorldY() && newVertice.second < getMaximumWorldY())
      return true;
    else
      return false;
  }

  //Get floating point with two digits after decimal.
  static float getTwoDecimals(float number) {
    return std::round((number * 100) + .01) / 100.00;
  }

  //To check if the point is not on any of the obstacles
  static bool not_on_any_obstacle(std::pair<float, float> point, std::vector<obstacles> obs_list) {
    for (int g = 0; g < obs_list.size(); g++)
    {
      if (!RRToperations::is_not_on_obstacle(point, obs_list[g]._scalex, obs_list[g]._scaley, obs_list[g]._center))
        return false;
    }
    return true;
  }

  //This function check if the point is not on the obstacle and also it takes into consideration the dimenstion of the ROBOT
  //We add the dimension of ROBOT in such way that the point we get does not collides the ROBOT with obstacle.
  static bool is_not_on_obstacle(std::pair<float, float> point, float scalex, float scaley, std::pair<float, float> obstacle_centre) {

    float xDirectionMin = obstacle_centre.first - (scalex / 2) - (0.25 / 2); //0.25 dimension of robo
    float xDirectionMax = obstacle_centre.first + (scalex / 2) + (0.25 / 2);

    float yDirectionMin = obstacle_centre.second - (scaley / 2) - (0.25 / 2);
    float yDirectionMax = obstacle_centre.second + (scaley / 2) - (0.25 / 2);

    if ((point.first > xDirectionMin && point.first < xDirectionMax) && (point.second > yDirectionMin && point.second < yDirectionMax))
      return false;
    else
      return true;
  }
};

int main(int argc, char **argv)
{
  //Below given code is taken from the code provided
  ros::init(argc, argv, "ros_demo");
  //create a ros handle (pointer) so that you can call and use it
  ros::NodeHandle n;

  //in <>, it specified the type of the message to be published
  //in (), first param: topic name; second param: size of queued messages, at least 1 
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //each second, ros "spins" and draws 20 frames
  ros::Rate loop_rate(20);

  int frame_count = 0;
  float f = 0.0;

  RRToperations rrt;

  //Using three obstacles
  obstacles obs1({ 3,3 }, 1, 1);
  obstacles obs2({ 2,2 }, 0.5, 1.5);
  obstacles obs3({ 1.5,2 }, 0.25, 1.5);
  rrt.obs_list.push_back(obs1);
  rrt.obs_list.push_back(obs2);
  rrt.obs_list.push_back(obs3);

  //Defined the start and the goal point
  declare_start_goal start_goal({ 1,2 }, { 4.73,1.36 });

  int found_goal = 0;
  while (ros::ok())
  {
    //first create a string typed (std_msgs) message
    std_msgs::String msg;

    std::stringstream ss;
    //ss << "Frame index: " << frame_count;
    msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str()); //printing on screen

    //publisher publishes messages, the msg type must be consistent with definition advertise<>(); 
    chatter_pub.publish(msg);

    /******************** From here, we are defining and drawing two obstacles in the workspace **************************/
    //rrt.doRRT(marker_pub);
    // define two obstacles
    visualization_msgs::Marker obst1, obst2, obst3;

    // Set obst1 and obst2 as a Cube and Cylinder, respectively
    obst1.type = visualization_msgs::Marker::CUBE;
    obst2.type = visualization_msgs::Marker::CUBE;
    obst3.type = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obst1.header.frame_id = obst2.header.frame_id = obst3.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz
    obst1.header.stamp = obst2.header.stamp = obst3.header.stamp = ros::Time::now();

    // Set the namespace and id 
    obst1.ns = obst2.ns = obst3.ns = "obstacles";
    obst1.id = 0;
    obst2.id = 1;
    obst3.id = 2;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    obst1.action = obst2.action = obst3.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker 
    obst1.scale.x = rrt.obs_list[0]._scalex;
    obst1.scale.y = rrt.obs_list[0]._scaley;
    obst1.scale.z = 1.0; //1x1x1 here means each side of the cube is 1m long
    obst2.scale.x = rrt.obs_list[1]._scalex;
    obst2.scale.y = rrt.obs_list[1]._scaley;
    obst2.scale.z = 1.0; //1x1x1 here means the cylinder as diameter 1m and height 1m
    obst3.scale.x = rrt.obs_list[2]._scalex;
    obst3.scale.y = rrt.obs_list[2]._scaley;
    obst3.scale.z = 1.0;


    // Set the pose of the marker. since a side of the obstacle obst1 is 1m as defined above, now we place the obst1 center at (1, 2, 0.5). z-axis is height
    obst1.pose.position.x = rrt.obs_list[0]._center.first;
    obst1.pose.position.y = rrt.obs_list[0]._center.second;
    obst1.pose.position.z = 1;
    obst1.pose.orientation.x = 0.0;
    obst1.pose.orientation.y = 0.0;
    obst1.pose.orientation.z = 0.0;
    obst1.pose.orientation.w = 1.0; //(x, y, z, w) is a quaternion, ignore it here (quaternion can be converted to angle and converted back, ros can do it)

    obst2.pose.position.x = rrt.obs_list[1]._center.first;
    obst2.pose.position.y = rrt.obs_list[1]._center.second;
    obst2.pose.position.z = 1;
    obst2.pose.orientation = obst1.pose.orientation;

    obst3.pose.position.x = rrt.obs_list[2]._center.first;
    obst3.pose.position.y = rrt.obs_list[2]._center.second;
    obst3.pose.position.z = 1;
    obst3.pose.orientation = obst1.pose.orientation;

    // Set the color red, green, blue. if not set, by default the value is 0
    obst1.color.r = 0.0f;
    obst1.color.g = 1.0f;
    obst1.color.b = 0.0f;
    obst1.color.a = 1.0;    //be sure to set alpha to something non-zero, otherwise it is transparent
    obst2.color = obst1.color;
    obst3.color = obst1.color;
    obst1.lifetime = obst2.lifetime = obst3.lifetime = ros::Duration();


    visualization_msgs::Marker vertices, edges;
    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = edges.ns = "vertices_and_lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;
    //// POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.01;
    vertices.scale.y = 0.01;

    //// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.01;

    //// Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    //// Line list is red
    edges.color.r = 1.0;
    edges.color.a = 1.0;


    //Set the start point
    CordinatewithId p;
    p.cordinates.first = start_goal._start_point.first;
    p.cordinates.second = start_goal._start_point.second;
    p.id = 0;

    //RRT vertice holds all the points generated
    rrt.rrt_vertices.push_back(p);
    for (int i = 0; i < rrt.rrt_vertices.size(); i++)
    {

      marker_pub.publish(obst1);
      marker_pub.publish(obst2);
      marker_pub.publish(obst3);

      std::pair<float, float> randomPoint;
      std::pair<float, float> nearestVertice;
      std::pair<float, float> newPoint;
      bool got_safe_point = false;

      //Keep in loop until you get the safe new point.
      while (!got_safe_point) {
        randomPoint = RRToperations::getRandomPoint();
        nearestVertice = rrt.getClosestVertice(randomPoint);
        newPoint = RRToperations::getNewVertice(nearestVertice, randomPoint, rrt.obs_list);

        if (newPoint.first != -100)

          got_safe_point = true;
      }

      CordinatewithId cor;
      cor.cordinates = newPoint;
      rrt.rrt_vertices.push_back(cor);

      TwoPoints e;
      e.firstPoint = nearestVertice;
      e.SecondPoint = newPoint;
      rrt.path.push_back(e);
      int goalAchieved = 0;
      static visualization_msgs::Marker path;


      //Goal found
      if (newPoint.first == start_goal._goal_point.first && newPoint.second == start_goal._goal_point.second)
      {

        std::pair<float, float> ultimateParent = { -100,-100 };

        //Looping through the path having parent and child to get the path final path.
        while (frame_count % 100 == 0)
        {
          vector <std::pair<float, float>> finalPath;
          std::pair<float, float> child = newPoint;

          for (int m = rrt.path.size(); m >= 0; m--)
          {
            if (child.first == rrt.path[m].SecondPoint.first && child.second == rrt.path[m].SecondPoint.second)
            {
              ultimateParent = rrt.path[m].firstPoint;
              finalPath.push_back(ultimateParent);
              child = ultimateParent;
            }
          }

          //Creating the path 
          path.type = visualization_msgs::Marker::LINE_STRIP;
          path.header.frame_id = "map";
          path.id = 9;
          path.color.b = 1.0;
          path.color.a = 1.0;
          path.scale.x = 0.08;
          path.pose.orientation.w = 1.0;

          for (int k = 0; k < finalPath.size(); k++)
          {
            geometry_msgs::Point p;
            p.x = finalPath[k].first;
            p.y = finalPath[k].second;
            p.z = 0;
            path.points.push_back(p);

          }

          const std::vector<std::pair<float, float>> roboPath = finalPath;
          marker_pub.publish(path);


          //Moving the ROBO
          visualization_msgs::Marker robo;
          robo.type = visualization_msgs::Marker::CUBE;
          robo.header.frame_id = "map";
          robo.ns = "robo";
          robo.id = 10;
          robo.action = visualization_msgs::Marker::ADD;
          robo.scale.x = 0.25;
          robo.scale.y = 0.25;
          robo.scale.z = 1;
          robo.pose.position.x = start_goal._start_point.first;
          robo.pose.position.y = start_goal._start_point.second;
          robo.pose.position.z = 0;
          robo.pose.orientation.x = 0.0;
          robo.pose.orientation.y = 0.0;
          robo.pose.orientation.z = 0.0;
          robo.pose.orientation.w = 1.0;
          robo.color.r = 1.0f;
          robo.color.g = 0.0f;
          robo.color.b = 0.0f;
          robo.color.a = 1.0;
          marker_pub.publish(robo);

          for (int f = roboPath.size() - 1; f >= 0; f = f - 5)
          {
            robo.pose.position.x = finalPath[f].first;
            robo.pose.position.y = finalPath[f].second;
            robo.pose.position.z = 0;
            marker_pub.publish(robo);
            sleep(1);
          }
          //Sleeping to see the output after the ROBOT has travelled
          sleep(100);
        }
      }

      vertices.id = 4;
      edges.id = 5;

      geometry_msgs::Point p0;  // root vertex
      p0.x = rrt.getTwoDecimals(newPoint.first);
      p0.y = rrt.getTwoDecimals(newPoint.second);
      p0.z = 0;

      geometry_msgs::Point p;
      p.x = rrt.getTwoDecimals(nearestVertice.first);
      p.y = rrt.getTwoDecimals(nearestVertice.second);
      p.z = 0;

      std_msgs::String msg;

      //Just extra checks for debuggin the program.
      if (isnan(p0.x)) {
        std::stringstream ss2;
        ss2 << "P ZERO X" << p0.x;
        msg.data = ss2.str();
        ROS_INFO("%s", msg.data.c_str());
        break;

      }
      if (isnan(p0.y)) {

        std::stringstream ss2;
        ss2 << "P ZERO Y" << p0.y;
        msg.data = ss2.str();
        ROS_INFO("%s", msg.data.c_str());
        break;

      }
      if (isnan(p.x)) {
        std::stringstream ss2;
        ss2 << "P X" << p.x;
        msg.data = ss2.str();
        ROS_INFO("%s", msg.data.c_str());
        break;

      }
      if (isnan(p.y)) {
        std::stringstream ss2;
        ss2 << "P Y" << p.y;
        msg.data = ss2.str();
        ROS_INFO("%s", msg.data.c_str());
        break;

      }

      vertices.points.push_back(p); //for drawing vertices
      edges.points.push_back(p0); //for drawing edges. The line list needs two points for each line
      edges.points.push_back(p);
      marker_pub.publish(vertices);
      marker_pub.publish(edges);
    }

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please run Rviz in another terminal.");
      sleep(1);
    }

    //ros spins, force ROS frame to refresh/update once
    ros::spinOnce();

    loop_rate.sleep();
    ++frame_count;
  }

  return 0;
}
