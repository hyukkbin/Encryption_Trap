#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include "../include/Dyers/he1n.hpp"
#include "../include/Dyers/wide_data.h"
#include "../include/Dyers/get_prime.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lab_interfaces/msg/command.hpp"


/**
 * Current Code Summary
 * 
 * "labdemonode" - ROS Node
 * Publishers: command velocity
 * Subscribers: odometry
 * 
 * Callbacks: Timer -- runs the controller.
 * 
 * Ultimate goal: Apply encryption to controller.
 * 
 * I need: 
 * 
 * new message format for sending through encrypted information.
 * 
 * 
 * 
 * To robot : command velocity, (2) + keys
 * 
 * To controller : system position (3) + stuff needed for error computation ( cos/sin ) + keys
 * 
 * publisher/subscriber for the encryption keys.
 * 
 * node that will run on the robot and handle encryption.
 * 
 * the encrypted version of the controller.
 * 
 * encrypted version of ATPA and CA
 * 
 */ 

using namespace dyers;
using namespace std::chrono_literals;
using std::placeholders::_1;

struct posture 
{
  float x;
  float y;
  float theta;
};


class LabDemoNode : public rclcpp::Node
{
  public:
    //Node for controller
    LabDemoNode()
    : Node("controller"), count_(0)
    {
      //Publisher to /encnum topic this is one encrypted number. Should be changed to a message of vector of encnum for use in controller. 
      publisher_enc = this -> create_publisher<std_msgs::msg::UInt8MultiArray>("/encnum",10);


      // These were the publishers for the command velocity and reference. ** I don't remember what ref is for..  I feel like this is not needed
      // publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);      
      // ref_pub_ = this->create_publisher<lab_interfaces::msg::Command>("/ref", 10);      

      // This subscribes to the odometry message of the robot. This should be changed to a vector of needed information that is under HE.
      sub_odom_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, std::bind(&LabDemoNode::odom_callback, this, _1));  

      // Timer for callback. This is setting the update frequency of the controller.
      timer_ = this->create_wall_timer(
      10ms, std::bind(&LabDemoNode::timer_callback, this));
    }


  private:
    //this is the controller. 
    void timer_callback()
    {

      // auto message = geometry_msgs::msg::Twist();
      auto ref = refUpdate();
      auto refmsg = lab_interfaces::msg::Command();
      refmsg.x = ref.x;
      refmsg.y = ref.y;
      refmsg.theta = ref.theta;
      //calling the function that gets the error
      auto err = errorUpdate(ref);

      //calling the function that gets the controll commands
      message = controlCommand(err);
      count_++;
      // RCLCPP_INFO(this->get_logger(), "count: '%i', x_r: '%f', y_r: '%f', th_r: '%f'", (int)count_ , ref.x,ref.y,ref.theta);
      if(count_%20 == 0){
      // RCLCPP_INFO(this->get_logger(), " Command :: v: '%f', w: '%f'", message.linear.x,message.angular.z);
      // RCLCPP_INFO(this->get_logger(), "Error :: x: '%f', y: '%f', th: '%f'", err.x,err.y,err.theta);
      }


      // This was demoing ability to send ciphertext.
      std_msgs::msg::UInt8MultiArray sendingThis;
      sendingThis.data = encNum;

      publisher_enc -> publish( sendingThis );


      // This publishes to cmd_val topic for turtlebot.
      // publisher_->publish(message);

      // Still not sure what this does
      // ref_pub_->publish(refmsg);
    }

    // Computes reference based on timer count. 
    posture refUpdate()
    {
    
      float x,y,theta,v_x,v_y;

      // computes current time based on timer frequency.
      float sec = count_/100.00;
      float period = 0.15;

      //This is the actual trajectory
      x =  0.01 * sec;
      y =  0.025 * std::sin(sec * period) + 0.05;

      //Analytic derivative for desired velocity and orientation.
      v_x = 0.01*(0.01) / 0.01;
      v_y = 0.025 * period *std::cos(period*sec);

      //Get orientation based on tangential velocity
      theta = std::atan2(v_y,v_x);

      //Set desired velocities
      v_r = std::sqrt(v_x*v_x+v_y*v_y);
      w_r = (v_x * v_y) / (v_r*v_r);

      //Return reference every 20 runs
    if(count_%20 == 0)
        // RCLCPP_INFO(this->get_logger(), " Observed x: '%f', y: '%f', th: '%f'", robotPose_.x,robotPose_.y,robotPose_.theta);       

      return (posture){x,y,theta};
    }
    float w_r;
    float v_r;
    
    // This function computes the error from the reference and the subscribed odometry
    posture errorUpdate(const posture ref_)
    {
      posture err_ = ref_;
      float c = std::cos(robotPose_.theta);
      float s = std::sin(robotPose_.theta);
      posture temp = {err_.x - robotPose_.x,err_.y - robotPose_.y,err_.theta - robotPose_.theta};
      err_ = (posture){temp.x*c+temp.y*s,-temp.x*s+temp.y*c,temp.theta};
      return err_;
    }

    // This function is the actual controller that takes in the errors from the previous function.
    geometry_msgs::msg::Twist controlCommand(posture err_)
    {
      float K_x, K_y, K_th;

      K_x = 2;
      K_y = 2000;
      K_th = 100; encNum

      auto command_ = geometry_msgs::msg::Twist();
      // linear
      float v = v_r * std::cos(err_.theta) + K_x * err_.x;      
      //angular
      float w = w_r + v_r * (K_y * err_.y + K_th * std::sin(err_.theta));
      // auto [v_m, w_m] = attackU(v, w, count_);

      // This is where FDIA used to be applied. Should change to encrypted version
      float v_m = v;
      float w_m = w;

      command_.linear.x = v_m;
      command_.angular.z = w_m;
      return command_;
    }

    int lim = 210;
    float delta = std::atan(1) / lim;

    //This grabs the odometry data from the robot and translates it to global pose.
    void odom_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {


      auto tfs = (*msg).transforms;

      for(auto tf:tfs){
        if(tf.child_frame_id == "base_footprint"){
          pose_ = tf.transform.translation;
          quat_ = tf.transform.rotation;
          found = true;
          break;
        }
        
        if(tf.child_frame_id == "odom"){
          od_pose_ = tf.transform.translation;
          od_quat_ = tf.transform.rotation;
          found_od = true;
        }
      }

      if(found == false ) return;
      

      tf2::Quaternion q_(quat_.x,quat_.y,quat_.z,quat_.w);
      tf2::Quaternion od_q_(od_quat_.x,od_quat_.y,od_quat_.z,od_quat_.w);
      auto qmap_ = q_;
      tf2::Matrix3x3 m(qmap_);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      auto qp_ = od_q_;
      tf2::Matrix3x3 ma(qp_);
      double oroll, opitch, oyaw;
      ma.getRPY(oroll, opitch, oyaw);

      float rxx = cos(-oyaw)*pose_.x + sin(-oyaw)*pose_.y + od_pose_.x;
      float ryy = -sin(-oyaw)*pose_.x + cos(-oyaw)*pose_.y + od_pose_.y;
      float rth = yaw + oyaw;
      // RCLCPP_INFO(this->get_logger(), " Adjusted X: '%f', Adjusted Y: '%f'",robotPose_.x,robotPose_.y);    
      

      //Base case
      robotPose_.theta = rth;
      robotPose_.x = rxx;
      robotPose_.y = ryy;
     
      found = false;
      found_od = false;
    }

    /**
     * Called before controller is run.
     * 
     * updates two quantities mu and gamma
     * 
     * input is last controller command and feedback output.
     * Also need last iteration mu an gamma.
     * 1. use approximation for each plant model.
     * 2. use mu and gamma to run simulation for both plants.
     * 3. use output of simulations to compute gamma.
     * 4. use gamma in the controller to compute mu.
     * 5. update gamma and mu.
     */
    void covert_attack(){

      //original signal
      auto th = robotPose_.theta;
      auto xx = robotPose_.x;
      auto yy = robotPose_.y;

      posture poseCopy = robotPose_;

      auto vv = message.linear.x;
      auto ww = message.angular.z;
      
      //modified signal

      posture obsrv_td;
      obsrv_td.theta = poseCopy.theta + gamma.theta;
      obsrv_td.x = poseCopy.x + gamma.x;
      obsrv_td.y = poseCopy.y + gamma.y;
      auto v_tilde = vv + mu_v;
      auto w_tilde = ww + mu_w;
      
      float ts = 0.02;

      //simulation 1

      auto gam1_x = vv * std::cos(obsrv_td.theta);
      auto gam1_y = vv * std::sin(obsrv_td.theta);
      auto gam1_th = ww;
      
      //simulation 2

      auto gam2_x = v_tilde * std::cos(th);
      auto gam2_y = v_tilde * std::sin(th);
      auto gam2_th = w_tilde;
      
      //compute gamma
      gamma.x = ts * (gam1_x - gam2_x);
      gamma.y = ts * (gam1_y - gam2_y);
      gamma.theta= ts * (gam1_th - gam2_th);

      //run covert controller.
      // desired disturbance - x, y no change, th + pi.
      posture err_cov = {0,0,0};
      // compute error
      float c = std::cos(gamma.theta);
      float s = std::sin(gamma.theta);
      posture temp = {err_cov.x - robotPose_.x,err_cov.y - robotPose_.y,err_cov.theta - robotPose_.theta};
      err_cov = (posture){temp.x*c+temp.y*s,-temp.x*s+temp.y*c,temp.theta};

      // compute mu
      // linear
      mu_v = v_r_cov * std::cos(err_cov.theta) + K_x * err_cov.x;      
      //angular
      mu_w = w_r_cov + v_r_cov * (K_y * err_cov.y + K_th * std::sin(err_cov.theta));


    }



      geometry_msgs::msg::Vector3 pose_;
      geometry_msgs::msg::Quaternion quat_;

      geometry_msgs::msg::Vector3 od_pose_;
      geometry_msgs::msg::Quaternion od_quat_;

      bool found = false;
      bool found_od = false;


    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<lab_interfaces::msg::Command>::SharedPtr ref_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_enc;

    float mu_v = 0;
    float mu_w = 0;
    posture gamma;

    struct posture robotPose_;
    geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();

    size_t count_;
    float K_x, K_y, K_th;

    int bit_length = 256;
    int rho = 1;
    int rho_ = 32;
    
    PKey key = dyers::keygen(bit_length,rho,rho_);
    wide_uint_t mod = pgen(bit_length,rho_,key.p);
    dyers::cipher_text c1 = dyers::encrypt(24,key,mod);
    //I need to run this function ctext2arr to be able to save a ciphertext to a 8bit array and send
    std::vector<uint8_t> encNum = c1.ctext2arr();


};


int main(int argc, char*argv[])
{

  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<LabDemoNode>());
  rclcpp::shutdown();


  return 0;
}