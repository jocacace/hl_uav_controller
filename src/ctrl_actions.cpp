#include "controller.h"

using namespace Eigen;
using namespace std;

void HL_CONTROLLER::takeoff( double altitude ) {

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

    _p_des(2) += altitude;
    _q_des = _w_q;

    //ROS_INFO("Takeoff required");
    //ros::Rate r(10);
    //double tc = 1.0/10.0;
//
    //bool done = false;
    //Vector3d to_p = _w_p;
    //to_p(2) += altitude;
//
    //Vector3d ep = to_p - _w_p;
//
    //Vector3d kp;
    //kp << 3, 3, 3;

    //Vector3d eu = utilities::R2XYZ ( utilities::QuatToMat(  _w_q ) );
    
    //_ctrl_mode = position;    
    //while ( !done ) {
//
    //    ep = to_p - _w_p;
    //    _dp_des = Vector3d::Zero();
    //    _p_des = to_p;
    //    _q_des = _w_q;
//
    //    if( ep.norm() < 0.1 ) done = true;
    //    r.sleep();
    //}
    //cout << "Take off completed!" << endl;
//
    //_ctrl_mode = velocity;
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