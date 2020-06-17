#include "controller.h"


HL_CONTROLLER::HL_CONTROLLER() {
    
    //---Request service
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    //---

    //---Sub/pub  
    _local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 0);
    _local_vel_pub = _nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 0);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 0, &HL_CONTROLLER::mavros_state_cb, this);
    _localization_sub = _nh.subscribe( "/mavros/vision_pose/pose", 0, &HL_CONTROLLER::localization_cb, this);
    //---

    _rate = 100;
    _ctrl_mode = position;
    _first_w_mes = false;
}

void HL_CONTROLLER::mavros_state_cb( mavros_msgs::State mstate) {
    _mstate = mstate;
}


void HL_CONTROLLER::publish_control() {

    ros::Rate r(_rate);
    geometry_msgs::PoseStamped p_ctrl;
    geometry_msgs::Twist vel_ctrl;

    while (ros::ok()) {
        if( _ctrl_mode == position ) {
            p_ctrl.pose.position.x = _p_des(0);
            p_ctrl.pose.position.y = _p_des(1);
            p_ctrl.pose.position.z = _p_des(2);
            p_ctrl.pose.orientation.w = _q_des(0);
            p_ctrl.pose.orientation.x = _q_des(1);
            p_ctrl.pose.orientation.y = _q_des(2);
            p_ctrl.pose.orientation.z = _q_des(3);
            _local_pos_pub.publish( p_ctrl );
        }
        else if( _ctrl_mode == velocity ) {           
            vel_ctrl.linear.x = _dp_des(0);
            vel_ctrl.linear.y = _dp_des(1);
            vel_ctrl.linear.z = _dp_des(2);
            _local_vel_pub.publish( vel_ctrl );
        }
        r.sleep();
    }
}

void HL_CONTROLLER::localization_cb ( geometry_msgs::PoseStampedConstPtr msg ) {
    _w_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    _w_q << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    _first_w_mes = true;
}

void HL_CONTROLLER::load_param() {
    /*
    //Load params
    if( !_nh.getParam("model_name", _model_name) ) {
        _model_name =  "iris_smc";
    }
    if( !_nh.getParam("control_rate", _ctrl_rate) ) {
        _ctrl_rate =  100;
    }
    if( !_nh.getParam("motor_num", _motor_num) ) {
        _motor_num =  4;
    }
    
    vector<double> inertia;
    if( !_nh.getParam("inertia", inertia) ) {
        inertia.resize(3);
        inertia[0] = 1.0;
        inertia[1] = 1.0;
        inertia[2] = 1.0;
    }
    _inertia = Eigen::Matrix3d( Eigen::Vector3d( inertia[0], inertia[1], inertia[2] ).asDiagonal() );
    */
}


void HL_CONTROLLER::state_machine() {

    while(!_first_w_mes) sleep(1);
    _ctrl_mode = position;
    _p_des = _w_p;
    takeoff(3.5);
    
    //manual_land();

}

void HL_CONTROLLER::run() {
    boost::thread publish_control_t( &HL_CONTROLLER::publish_control, this );
    boost::thread state_machine_t( &HL_CONTROLLER::state_machine, this);
    //boost::thread velocity_controller_t( &HL_CONTROLLER::velocity_controller, this);
    ros::spin();
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "hl_uav_controller");
    HL_CONTROLLER hc;
    hc.run();

    return 0;
}