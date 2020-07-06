#include "controller.h"


HL_CONTROLLER::HL_CONTROLLER() {
    
    //---Request service
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    //---

    //---Sub/pub  
    _local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 0);
    _local_vel_pub = _nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 0);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 0, &HL_CONTROLLER::mavros_state_cb, this);
    _localization_sub = _nh.subscribe( "/mavros/local_position/pose", 0, &HL_CONTROLLER::localization_cb, this);
    _qr_d_cbs[0] = _nh.subscribe( "/d/qrcodes", 0, &HL_CONTROLLER::qr_d_cb, this);
    _qr_d_cbs[1] = _nh.subscribe( "/f1/qrcodes", 0, &HL_CONTROLLER::qr_d_cb, this);
    _qr_d_cbs[2] = _nh.subscribe( "/f2/qrcodes", 0, &HL_CONTROLLER::qr_d_cb, this);

    //---


     if( !_nh.getParam("fixed_frame", _fixed_frame)) {
        _fixed_frame = "odom";
    }

    if( !_nh.getParam("camera1_frame", _camera_frame[0])) {
        _camera_frame[0] = "rgbd_camera_optical_frame";
    }
    if( !_nh.getParam("camera2_frame", _camera_frame[1])) {
        _camera_frame[1] = "rgbd_camera_optical_frame";
    }
    if( !_nh.getParam("camera3_frame", _camera_frame[2])) {
        _camera_frame[2] = "rgbd_camera_optical_frame";
    }

    /*if( !_nh.getParam("camera_frame", _camera_frame)) {
        _camera_frame = " ";
    }*/
    

    load_targets();

    _rate = 100;
    _ctrl_mode = position;
    _first_w_mes = false;

    _in_flight = false;
    _finish = false;
    _moved = false;

    //--- Octomap ---
    _tree = new octomap::OcTree("/home/salvatore/catkin_ws/src/hl_uav_controller/map/ch1.binvox.bt");
    
}

void HL_CONTROLLER::mavros_state_cb( mavros_msgs::State mstate) {
    _mstate = mstate;

}

void HL_CONTROLLER::pfilter(){
//Params
    double ref_jerk_max;
    double ref_acc_max;
    double ref_vel_max;
    double ref_omega;
    double ref_zita;

    double ref_o_jerk_max;
    double ref_o_acc_max;
    double ref_o_vel_max;

   
    if( !_nh.getParam("ref_jerk_max", ref_jerk_max)) {
        ref_jerk_max = 0.35;
    }
    if( !_nh.getParam("ref_acc_max", ref_acc_max)) {
        ref_acc_max = 0.75;
    }
    if( !_nh.getParam("ref_vel_max", ref_vel_max)) {
        ref_vel_max = 1.5;
    }
    if( !_nh.getParam("ref_omega", ref_omega)) {
        ref_omega = 1.0;
    }
    if( !_nh.getParam("ref_zita", ref_zita)) {
        ref_zita = 0.5;
    }


   if( !_nh.getParam("ref_o_jerk_max", ref_o_jerk_max)) {
        ref_o_jerk_max = 0.35;
    }
    if( !_nh.getParam("ref_o_acc_max", ref_o_acc_max)) {
        ref_o_acc_max = 0.75;
    }
    if( !_nh.getParam("ref_o_vel_max", ref_o_vel_max)) {
        ref_o_vel_max = 1.5;
    }



    ros::Rate r(100);
    double ref_T = 1.0/100.0;

    _p_cmd = _w_p;
    Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Vector4d(_w_q(0), _w_q(1), _w_q(2), _w_q(3)) ) );
    _yaw_cmd = rpy(2);


    cout << "_yaw_cmd: " << _yaw_cmd << endl;
    

    _p_des = _p_cmd;
    _yaw_des = _yaw_cmd;

    Vector3d ddp;
    ddp << 0.0, 0.0, 0.0;
    Vector3d dp;  
    dp << 0.0, 0.0, 0.0;

    Vector3d ref_dp;
    Vector3d ref_ddp;
    ref_dp << 0.0, 0.0, 0.0;  
    ref_ddp << 0.0, 0.0, 0.0;  
    double ref_dyaw = 0;
    double ref_ddyaw = 0;

    double ddyaw = 0.0;
    double dyaw = 0.0;


    Vector3d ep;
    ep << 0.0, 0.0, 0.0; 
    Vector3d jerk;
    jerk << 0.0, 0.0, 0.0;
            
    while( ros::ok() ) {

        ep = _p_cmd - _p_des;

        double eyaw = _yaw_cmd - _yaw_des;
        if(fabs(eyaw) > M_PI)
            eyaw = eyaw - 2*M_PI* ((eyaw>0)?1:-1);


        for(int i=0; i<3; i++ ) {
            ddp(i) = ref_omega*ref_omega * ep(i) - 2.0 * ref_zita*ref_omega*ref_dp(i);
        
            jerk(i) = (ddp(i) - ref_ddp(i))/ref_T;
            if( fabs( jerk(i) > ref_jerk_max) ) {
                if( jerk(i) > 0.0 ) jerk(i) = ref_jerk_max;
                else jerk(i) = -ref_jerk_max;
            } 

            ddp(i) = ref_ddp(i) + jerk(i)*ref_T;
            if( fabs( ddp(i)) > ref_acc_max   ) {
                if( ddp(i) > 0.0 )
                    ref_ddp(i) = ref_acc_max;
                else 
                    ref_ddp(i) = -ref_acc_max;
            }
            else {
                ref_ddp(i) = ddp(i);
            }


            dp(i) = ref_dp(i) + ref_ddp(i) * ref_T;
            if( fabs( dp(i) ) > ref_vel_max )  {
                if( dp(i) > 0.0 ) ref_dp(i) = ref_vel_max;
                else ref_dp(i) = -ref_vel_max;
            }
            else 
                ref_dp(i) = dp(i);

            _p_des(i) += ref_dp(i)*ref_T;

        }

        
        double ddyaw = ref_omega*ref_omega * eyaw - 2.0 * ref_zita*ref_omega*ref_dyaw;
        double o_jerk = (ddyaw - ref_ddyaw)/ref_T;
        if ( fabs ( o_jerk ) > ref_o_jerk_max ) {
            if( o_jerk > 0.0 ) o_jerk = ref_o_jerk_max;
            else o_jerk = -ref_o_jerk_max;
        }

        ddyaw = ref_ddyaw + o_jerk*ref_T;
        if( fabs( ddyaw ) > ref_o_acc_max ) {
            if ( ddyaw > 0.0 ) ref_ddyaw = ref_o_acc_max;
            else if( ddyaw < 0.0 ) ref_ddyaw = -ref_o_acc_max;
        }
        else 
            ref_ddyaw = ddyaw;
        
        dyaw = ref_dyaw + ref_ddyaw*ref_T;
        if( fabs( dyaw ) > ref_o_vel_max ) {
            if( dyaw > 0.0 ) dyaw = ref_o_vel_max;
            else dyaw = -ref_o_vel_max;
        }
        else 
            ref_dyaw = dyaw;

        _yaw_des += ref_dyaw*ref_T;
        
        r.sleep();
    }

}

void HL_CONTROLLER::publish_control() {

    ros::Rate r(_rate);
    geometry_msgs::PoseStamped p_ctrl;
    geometry_msgs::Twist vel_ctrl;


    while( !_first_w_mes ) usleep(0.1*1e6);
    boost::thread pfilter_t( &HL_CONTROLLER::pfilter, this);


    tf::TransformBroadcaster broadcaster;
	tf::Transform transform;

    while (ros::ok()) {
        p_ctrl.pose.position.x = _p_des(0);
        p_ctrl.pose.position.y = _p_des(1);
        p_ctrl.pose.position.z = _p_des(2);


        //cout << "_p_des: " << _p_des.transpose() << endl;

        tf::Quaternion q2e;
        q2e.setRPY(0, 0, _yaw_des);
        q2e = q2e.normalize();

        p_ctrl.pose.orientation.w = q2e.w();
        p_ctrl.pose.orientation.x = q2e.x();
        p_ctrl.pose.orientation.y = q2e.y();
        p_ctrl.pose.orientation.z = q2e.z();

        _local_pos_pub.publish( p_ctrl );

        //Send tf
        transform.setOrigin(tf::Vector3(p_ctrl.pose.position.x, p_ctrl.pose.position.y, p_ctrl.pose.position.z));
        tf::Quaternion q(p_ctrl.pose.orientation.x, p_ctrl.pose.orientation.y, p_ctrl.pose.orientation.z, p_ctrl.pose.orientation.w);
        transform.setRotation(q);
        tf::StampedTransform stamp_transform(transform, ros::Time::now(), "odom", "desired_pose");
        broadcaster.sendTransform(stamp_transform);

    
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
    _p_cmd = _w_p;

    test_landing();


//    cout << "entrato \n";
//    explore();
/*
    ros::Rate rate(200);
    while(ros::ok()){
        
        rate.sleep();
        ros::spinOnce();
    }
    */
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
