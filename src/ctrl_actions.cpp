#include "controller.h"

using namespace Eigen;
using namespace std;

void HL_CONTROLLER::takeoff( const double altitude ) {

    //Set up control mode
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //---

    //---Arm
    if( _arming_client.call(arm_cmd) && arm_cmd.response.success){
    }
    
    while(!_mstate.armed ) usleep(0.1*1e6);
    ROS_INFO("Vehicle armed");
    //---

    if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
      ROS_INFO("Manual OFFBOARD enabled");
    }

    _p_cmd(2) = altitude;
    Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Vector4d(_w_q(0), _w_q(1), _w_q(2), _w_q(3)) ) );
    //_yaw_des =  rpy(2);


    while( ( _p_des - _p_cmd).norm() > 0.1 ) 
        usleep(0.1*1e6);


    ROS_INFO("Takeoff completed");
    _in_flight = true;



}

void HL_CONTROLLER::look_around(const double ang){
    
    ROS_INFO("looking around");
    _yaw_des += ang;
}

void HL_CONTROLLER::velocity_controller( ) {

    ros::Rate r(10);
  
    Vector3d kd;
    Vector3d dir_vec;
    kd << 0.5, 0.5, 1;
    while( ros::ok() ) {
        if ( _ctrl_mode == velocity ) {
            dir_vec = _p_des - _w_p;
            dir_vec = dir_vec / dir_vec.norm();
            _dp_des = dir_vec.cwiseProduct(kd); 
           
        }
        r.sleep();
    }

}

void HL_CONTROLLER::manual_land( ) {
    bool done = false;
    Vector3d l_p = _w_p;
    l_p(2) = 0.0;
    ros::Rate r(10);

    Vector3d ep = l_p - _w_p;

    Vector3d kd;
    Vector3d dir_vec;
    kd << 0.5, 0.5, 1;

    ROS_INFO("Landing");
    //Vector3d eu = utilities::R2XYZ ( utilities::QuatToMat(  _w_q ) );
    while ( !done ) {
        ep = l_p - _w_p;
        dir_vec = l_p - _w_p;
        dir_vec = dir_vec / dir_vec.norm();
        _dp_des = dir_vec.cwiseProduct(kd); 
        if( ep.norm() < 0.1 ) done = true;
        r.sleep();
    }
}