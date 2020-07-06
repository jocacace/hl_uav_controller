#include "controller.h"




void HL_CONTROLLER::get_tfs () {

    ros::Rate r(10);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while( ros::ok() ) {


        try{
            ros::Time now = ros::Time(0);
            listener.waitForTransform(_fixed_frame, _camera_frame[0], now, ros::Duration(0.1));    
            listener.lookupTransform(_fixed_frame, _camera_frame[0] , now, transform);

            _sensors_tf[0].t = ros::Time::now();
            _sensors_tf[0].frame = _camera_frame[0];
            _sensors_tf[0].ready = true;
            _sensors_tf[0].Hws.block(0,0,3,3) = utilities::QuatToMat( Vector4d( transform.getRotation().w(),  transform.getRotation().x(),  transform.getRotation().y(),  transform.getRotation().z() ) );
            
            _sensors_tf[0].Hws(0,3) = transform.getOrigin().x();
            _sensors_tf[0].Hws(1,3) = transform.getOrigin().y();
            _sensors_tf[0].Hws(2,3) = transform.getOrigin().z();
                    
        }
        catch (tf::TransformException ex){
            //    ROS_ERROR("%s",ex.what());
            //    ros::Duration(1.0).sleep();
        }
        r.sleep();
    }

}

//Input: List of qrcodes in camera frame
void HL_CONTROLLER::qr_d_cb( lvision::codes c ) {

    /*
    header: 
    seq: 168
    stamp: 
        secs: 220
        nsecs:  68000000
    frame_id: "rgbd_camera_optical_frame"
    c: 
    - 
        code: 
        data: "Algeria\tAMEL 300-3"
        center: 
        x: -0.419630795717
        y: 0.030616838485
        z: 2.05524468422
    */

    if ( !_sensors_tf[0].ready ) return;


    for( int i=0; i<c.c.size(); i++) {

        //camera frame 2 world (odom) frame
        //Vector4d p_w = H_ws.inverse()*( Vector4d( c.c[i].center.x, c.c[i].center.y, c.c[i].center.z, 1.0 ));
        //cout << i << ": " << Vector3d( p_w(0), p_w(1), p_w(2) ).transpose() << endl;

    }

    
      //std::pair<std::string,double> myshopping ("baking powder",0.3);

}

void HL_CONTROLLER::qr_f1_cb( lvision::codes c ) {

}

void HL_CONTROLLER::qr_f2_cb( lvision::codes c ) {

}

void HL_CONTROLLER::test_landing() {

    if(!_in_flight){
        takeoff(2.5);
    }
}