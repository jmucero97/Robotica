

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>

#include <stdio.h>

#include <math.h>
#include <vector>
#include <fstream>

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>



/**
* Our class to control the robot
* It has members to store the robot pose, and
* methods to control the robot by publishing data
*/

class Turtlebot
{
public:
  Turtlebot();

  /*
   * This function should command the robot to reach the goal
   * It should compute the commands to the robot by knowing the current position
   * and the goal position.
   * This function will return true only if the goal has been reached.
   */
  bool command(double goal_x, double goal_y);

private:
  ros::NodeHandle nh_;

  double DISTANCE;

  //2D robot pose
  double x, y, theta;

  int counter;
  int contadorDecisionGiro;
  int decisionGiro;
  // Scan
  sensor_msgs::LaserScan data_scan;
  ros::Subscriber kinect_sub_;

  //Transform listerner to obtain the transform between the world frame (odom) and the robot frame (base_link)
  tf::TransformListener listener;

  //Publisher and subscribers
  ros::Publisher vel_pub_;

  //!Publish the command to the turtlebot
  void publish(double angular_vel, double linear_vel);

  //!Callback for kinect
  void receiveKinect(const sensor_msgs::LaserScan &laser_kinect);

  bool doAllRaysDetectObstacle();
  float primerAnguloSinObstaculo();
};

Turtlebot::Turtlebot()
{
  DISTANCE = 0.8;
  counter = 0;
  contadorDecisionGiro = 0;
  decisionGiro = 0;
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  kinect_sub_ = nh_.subscribe("/scan", 1, &Turtlebot::receiveKinect, this);
}

bool Turtlebot::doAllRaysDetectObstacle()
{

  bool allDetect = true;
  int numAngles = (data_scan.angle_max - data_scan.angle_min) / data_scan.angle_increment;
  for (int i = 0; i < numAngles; i++)
  {
    allDetect = allDetect && !std::isnan(data_scan.ranges.at(i)) && data_scan.ranges.at(i) < DISTANCE;
  }

  return allDetect;
}

float Turtlebot::primerAnguloSinObstaculo()
{
  int direccionGiro = 0;
  int numAngles = (data_scan.angle_max - data_scan.angle_min) / data_scan.angle_increment;
  int quinto = numAngles/5;
  int indiceMayorProfundidad = 0;
  float mayorProfundidad = 0;
  int forwardIndex = 0;
  int backwardsIndex = numAngles-1;
  size_t arraySize = numAngles;
  std::vector<float> forwardArray(arraySize);
  std::vector<float> backwardsArray(arraySize);
  std::vector<float> sumArray(arraySize);

  for(int i = 0; i < numAngles; i++){
    
    
    /***
     * Hay 2 Bloques de IF, uno para el FORWARDINDEX y otro para el BACKWARDINDEX.
     * En cada bloque IF se realiza lo siguiente:
     * 1º Comprobar si no es un NAN, dentro de los NAN, distinguir NAN reales de aleatorios
     * 2º Si no es NAN, añadir distancia al vector
     * Disclaimer: se distinguen NANs reales de falsos, porque en algunas lecturas de laser aparecen
     * NaNs solitarios, rodeados de distancias muy cortas. (Comprobado experimentalmente).
     * A los NANs se les considera distancia 6 porque Los NANs aparecen a partir de 5.6m
    **/

    //BLOQUE FORWARD
    if(std::isnan(data_scan.ranges.at(forwardIndex)) && forwardIndex == 0){
      //Primer elem array, comprobar si a la derecha tiene NAN, si lo tiene es un NAN real
      if(std::isnan(data_scan.ranges.at(forwardIndex+1))){
        forwardArray[forwardIndex++] = 6;
      }else{
        forwardArray[forwardIndex++] = data_scan.ranges.at(forwardIndex+1);
      }
    }else if(std::isnan(data_scan.ranges.at(forwardIndex)) && forwardIndex > 0 && forwardIndex < numAngles-1){
      //NAN intermedio, comprobar a derecha e izquierda, si a derecha o izquierda tiene NAN, es un NAN real
      if(std::isnan(data_scan.ranges.at(forwardIndex+1)) || std::isnan(data_scan.ranges.at(forwardIndex-1))){
        forwardArray[forwardIndex++] = 6 + forwardArray[forwardIndex-1];
      }else{
        forwardArray[forwardIndex++] = data_scan.ranges.at(forwardIndex+1) + forwardArray[forwardIndex-1];
      }
    }else if(std::isnan(data_scan.ranges.at(forwardIndex)) && forwardIndex == numAngles -1){
      //Ultimo elem array, comprobar si a la izquierda tiene NAN, si lo tiene, es un NAN real
      if(std::isnan(data_scan.ranges.at(forwardIndex-1))){
        forwardArray[forwardIndex++] = 6 + forwardArray[forwardIndex-1];
      }else{
        forwardArray[forwardIndex++] = data_scan.ranges.at(forwardIndex-1) + forwardArray[forwardIndex-1];
      }
    }else{
      // Cubiertos todos los casos de NAN, lo que queda ya no puede ser NAN, se añaden a los vectores.
      //Hay que comprobar si es el 0 para saber si hay que hacer sumatorio (cuando es la primera iteracion, no hay un elemento anterior para sumar)
      if(forwardIndex == 0){
        forwardArray[forwardIndex++] = data_scan.ranges.at(forwardIndex);
      }else{
        forwardArray[forwardIndex++] = data_scan.ranges.at(forwardIndex) + forwardArray[forwardIndex-1];
      }
    }
    
    //BLOQUE BACKWARDS
    //ROS_INFO("BACKWARDS: %d\nFORWARD: %d\nI: %d, NUMANGLES: %d",backwardsIndex,forwardIndex,i,numAngles);
    
    if(std::isnan(data_scan.ranges.at(backwardsIndex)) && backwardsIndex == 0){
      //Primer elem array, comprobar si a la derecha tiene NAN, si lo tiene es un NAN real
      if(std::isnan(data_scan.ranges.at(backwardsIndex+1))){
        backwardsArray[backwardsIndex--] = 6 + backwardsArray[backwardsIndex + 1];
      }else{
        backwardsArray[backwardsIndex--] = data_scan.ranges.at(backwardsIndex+1) + backwardsArray[backwardsIndex + 1];
      }
    }else if(std::isnan(data_scan.ranges.at(backwardsIndex)) && backwardsIndex > 0 && backwardsIndex < numAngles-1){
      //NAN intermedio, comprobar a derecha e izquierda, si a derecha o izquierda tiene NAN, es un NAN real
      if(std::isnan(data_scan.ranges.at(backwardsIndex+1)) || std::isnan(data_scan.ranges.at(backwardsIndex-1))){
        backwardsArray[backwardsIndex--] = 6 + backwardsArray[backwardsIndex + 1];
      }else{
        backwardsArray[backwardsIndex--] = data_scan.ranges.at(backwardsIndex+1) + backwardsArray[backwardsIndex + 1];
      }
      
    }else if(std::isnan(data_scan.ranges.at(backwardsIndex)) && backwardsIndex == numAngles -1){
      //Ultimo elem array, comprobar si a la izquierda tiene NAN, si lo tiene, es un NAN real
      if(std::isnan(data_scan.ranges.at(backwardsIndex-1))){
        backwardsArray[backwardsIndex--] = 6;
      }else{
        backwardsArray[backwardsIndex--] = data_scan.ranges.at(backwardsIndex-1);
      }
    }else{
      // Cubiertos todos los casos de NAN, lo que queda ya no puede ser NAN, se añaden a los vectores.
      //Hay que comprobar si es el numAngles -1 para saber si hay que hacer sumatorio (cuando es la primera iteracion, no hay un elemento anterior para sumar)
      if(backwardsIndex == numAngles - 1){
        backwardsArray[backwardsIndex--] = data_scan.ranges.at(backwardsIndex);
      }else{
        //ROS_INFO("BACKWARDS INDEX DONDE PETA: %d | BI + 1 = %d",backwardsIndex, backwardsIndex+1);
        //DATA_SCAN.RANGES.AT NO FUNCIONA CON EL 0!!!
        backwardsArray[backwardsIndex--] = data_scan.ranges.at(backwardsIndex+1) + backwardsArray[backwardsIndex + 1];
      }
      
    }

  }

  // Sum both arrays and get highest number
  for(int i = 0; i < numAngles; i++){
    sumArray[i] = forwardArray[i] + backwardsArray[i];
    if(sumArray[i] > mayorProfundidad){
      mayorProfundidad = sumArray[i];
      indiceMayorProfundidad = i;
    }
  }

  //Decidir giro dirección
  if(indiceMayorProfundidad < 2*quinto){
    //Girar a derecha
    direccionGiro = -1;
  }else if(indiceMayorProfundidad > 3*quinto){
    direccionGiro = 1;
  }else{
	direccionGiro = 0;	
}

  ROS_INFO("\n\tIndice profundidad: %d; Direccion Giro: %d",indiceMayorProfundidad,direccionGiro);

  return direccionGiro;
}

bool Turtlebot::command(double gx, double gy)
{

  double linear_vel = 0.4;
  double angular_vel = 0.0;

  bool ret_val = false;

  //Transform the goal to the local frame
  geometry_msgs::PointStamped goal;
  geometry_msgs::PointStamped base_goal;

  goal.header.frame_id = "odom";

  //we'll just use the most recent transform available for our simple example
  goal.header.stamp = ros::Time();

  //just an arbitrary point in space
  goal.point.x = gx;
  goal.point.y = gy;
  goal.point.z = 0.0;

  try
  {
    listener.transformPoint("base_link", goal, base_goal);

    /**ROS_INFO("goal: (%.2f, %.2f. %.2f) -----> base_goal: (%.2f, %.2f, %.2f) at time %.2f",
             goal.point.x, goal.point.y, goal.point.z,
             base_goal.point.x, base_goal.point.y, base_goal.point.z, base_goal.header.stamp.toSec());**/
  }
  catch (tf::TransformException &ex)
  {
    //ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    return ret_val;
  }

  /**
	* This should be completed. You should use a proportional controller
	* The linear velocity should be proportional to the distance to the goal
	* The angular velocity should be proportional to the difference in the angle towards
	* the goal and the current angle of the robot. You should check if you reached the goal
	*/
  int numAngles = ceil((data_scan.angle_max - data_scan.angle_min) / data_scan.angle_increment);
  bool obstacle = false;

  for (int i = 0; i < numAngles; i++)
  {
    if (!std::isnan(data_scan.ranges.at(i)) && data_scan.ranges.at(i) < DISTANCE)
    {
      obstacle = true;
      counter = 70;
    }
  }

  float alfa_in_rad = atan2(base_goal.point.y, base_goal.point.x); //Desviación de orientación respecto al goal en radianes
  float alfa = 180 * alfa_in_rad / M_PI;                           //Ángulo en grados
  float distance = sqrt(pow(base_goal.point.x, 2) + pow(base_goal.point.y, 2));
  /**
  ROS_INFO("vector final: {%f, %f}->{2,2}  distancia: %f   alfa: %f", vectorFinal[0], vectorFinal[1], distance, alfa);
**/

  //NORMAL EXECUTION
  angular_vel = 0.5;

  if (abs(alfa) > 10)
  {
    angular_vel = 0.5 * alfa_in_rad / 2 * M_PI;
  }
  else
  {
    angular_vel = 0.1 * alfa_in_rad / 2 * M_PI;

    linear_vel = 0.2 * distance;
    if (linear_vel > 0.5)
    {
      linear_vel = 0.5;
    }
  }

  if (obstacle || counter > 0)
  {
    if (doAllRaysDetectObstacle())
    {
      angular_vel = 0.5;
      ROS_INFO("TODOS DETECTAN OBSTACULO");
    }
    else
    {
      linear_vel = 0.2;
      if (obstacle)
      {
      	if(contadorDecisionGiro < 0){
        	float direccionGiro = primerAnguloSinObstaculo();
  		decisionGiro = direccionGiro;
  		contadorDecisionGiro = 100;
        }
        linear_vel = 0.1;
        angular_vel = 0.8 * decisionGiro;
      }
      else
      {
        ROS_INFO("SIGUE RECTO");
        angular_vel = 0;
      }
    }
    counter--;
    contadorDecisionGiro--;
  }

  if (distance < 0.1)
  {
    linear_vel = 0;
    angular_vel = 0;
    ret_val = true;
  }


  /**angular_vel = 0;
  linear_vel = 0;**/

  publish(angular_vel, linear_vel);

  return ret_val;
}

//Publish the command to the turtlebot
void Turtlebot::publish(double angular, double linear)
{
  geometry_msgs::Twist vel;
  vel.angular.z = angular;
  vel.linear.x = linear;

  //std::cout << "Velocidades: " << vel.linear.x << ", " << vel.angular.z << std::endl;

  vel_pub_.publish(vel);

  return;
}

//Callback for robot position SUBSCRIPTION TO SCAN
void Turtlebot::receiveKinect(const sensor_msgs::LaserScan &msg)
{
  data_scan = msg;
  /**
  // Different variables used to detect obstacles
  float size_lazer;
  data_scan.angle_min;
  size_lazer = data_scan.ranges.size();
  
 ROS_INFO("Angle Min: %f\nAngle max: %f\nSize lazer: %f", data_scan.angle_min, data_scan.angle_max, size_lazer);
 // ROS_INFO("Time Increment: %f\n", data_scan.time_increment);
 // ROS_INFO("Angle Increment: %f\n", data_scan.angle_increment);
  ROS_INFO("Range Min: %f\n", data_scan.range_min);
  ROS_INFO("Range Max: %f\n", data_scan.range_max);
  //ROS_INFO("Ranges: %f\n", sizeof(data_scan.ranges));
  //ROS_INFO("Intensities: %f\n", sizeof(data_scan.intensities));
  int numAngles = ceil((data_scan.angle_max-data_scan.angle_min)/data_scan.angle_increment);
  for (int i = 0; i < numAngles; i++)
  {
    if (!std::isnan(data_scan.ranges.at(i)))
    {
      ROS_INFO("Range %d: %f", i, data_scan.ranges.at(i)); //A partir de 1.8 detecta que hay un obstáculo
    }
  }**/
}

void visualizePlan(const std::vector<geometry_msgs::Pose> &plan, ros::Publisher &marker_pub);

std::vector<geometry_msgs::Pose> loadPlan(const char *filename);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_control");
  Turtlebot robot;
  ros::NodeHandle n;

  if (argc < 2)
  {
    std::cout << "Insufficient number of parameters" << std::endl;
    std::cout << "Usage: robot_control <filename>" << std::endl;
    return 0;
  }

  std::vector<geometry_msgs::Pose> plan = loadPlan(argv[1]);
  unsigned int cont_wp = 0;

  ros::Rate loop_rate(20);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 4);

  /**
  * Complete the code to follow the path,
  * calling adequately to the command function
  */

  int i = 0;
  while (ros::ok() && i < plan.size())
  {
    ros::spinOnce();
    //Llamar funcion command, recorriendo plan cargado y la x,y está dentro del geometry_msgs Pose
    visualizePlan(plan, marker_pub);
    if (robot.command(plan[i].position.x, plan[i].position.y))
    {
      i++;
    }

    loop_rate.sleep();
  }

  return 0;
}

std::vector<geometry_msgs::Pose> loadPlan(const char *filename)
{
  std::vector<geometry_msgs::Pose> plan;
  double x, y;

  std::ifstream is(filename);

  while (is.good())
  {
    is >> x;
    if (is.good())
    {
      is >> y;
      geometry_msgs::Pose curr_way;
      curr_way.position.x = x;
      curr_way.position.y = y;
      plan.push_back(curr_way);
      ROS_INFO("Loaded waypoint (%f, %f).", x, y);
    }
  }
  ROS_INFO("Plan loaded successfully.");
  return plan;
}

void visualizePlan(const std::vector<geometry_msgs::Pose> &plan, ros::Publisher &marker_pub)
{
  ros::NodeHandle n;

  uint32_t shape = visualization_msgs::Marker::CUBE;

  for (unsigned int i = 0; i < plan.size(); i++)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom"; // This is the default fixed frame in order to show the move of the robot
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "plan";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = plan[i].position.x;
    marker.pose.position.y = plan[i].position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(); // Eternal marker

    // Publish the marker
    marker_pub.publish(marker);
  }
}

