#include "controller.h"

using namespace Eigen;
using namespace std;

void HL_CONTROLLER::move_drone(const double x, const double y, const double z, const double yaw) {

    ros::ServiceClient client = _nh.serviceClient<motion_planner::generate_plan>("generate_tarjectory");

    //while(ros::ok()) {

        _plan.request.p_i.position.x = _w_p(0);
        _plan.request.p_i.position.y = _w_p(1);
        _plan.request.p_i.position.z = _w_p(2);
        Eigen::Vector3d eu = utilities::R2XYZ( utilities::QuatToMat ( Eigen::Vector4d( _w_q(0), _w_q(1), _w_q(2), _w_q(3) ) ) );
        
        tf::Quaternion q;
        q.setRPY(0, 0, eu(2));
        q = q.normalize();

        _plan.request.p_i.orientation.w = q.w();
        _plan.request.p_i.orientation.x = q.x();
        _plan.request.p_i.orientation.y = q.y();
        _plan.request.p_i.orientation.z = q.z();

        _plan.request.p_f.position.x = x;
        _plan.request.p_f.position.y = y;
        _plan.request.p_f.position.z = z;

        Eigen::Vector4d dq = utilities::rot2quat ( utilities::XYZ2R( Eigen::Vector3d( 0, 0, yaw) ) );

        _plan.request.p_f.orientation.w = dq[0];
        _plan.request.p_f.orientation.x = dq[1];
        _plan.request.p_f.orientation.y = dq[2];
        _plan.request.p_f.orientation.z = dq[3];

        if( client.call( _plan ) ) {
            _new_plan = true;
            _moved = false;
        }
    //}
}

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

    //_p_des(2) += altitude;
    //_q_des = _w_q;
    move_drone(0.0, 0.0, 3.0, 0.0);
    _in_flight = true;

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

void HL_CONTROLLER::look_around(){
    
    ROS_INFO("looking around");
    move_drone(_w_p(0), _w_p(1), _w_p(2), 3.14);
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