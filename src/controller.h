#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "boost/thread.hpp"
//---mavros_msgs
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
//---
#include "tf/tf.h"
#include "utils.h"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;


enum CTRL_TYPE {velocity, position};

class HL_CONTROLLER {
  public:
    HL_CONTROLLER();
    void run();
    void mavros_state_cb( mavros_msgs::State mstate);
    void ctrl_loop();
    void state_machine();
    void load_param();
    void localization_cb ( geometry_msgs::PoseStampedConstPtr msg );
    void publish_control();

    void takeoff( double );
    void manual_land();
    void velocity_controller();
    void position_controller();
    void rotate( double angle );

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _mavros_state_sub;
    ros::Subscriber _localization_sub;
    ros::ServiceClient _arming_client;
    ros::ServiceClient _set_mode_client;
    ros::ServiceClient _land_client;
    ros::Publisher _local_pos_pub;
    ros::Publisher _local_vel_pub;
    mavros_msgs::State _mstate;

    Eigen::Vector3d _w_p;
    Eigen::Vector4d _w_q;

    int _rate;

    Eigen::Vector3d _p_des;
    Eigen::Vector4d _q_des;
    Eigen::Vector3d _dp_des;
    CTRL_TYPE _ctrl_mode;   
    bool _first_w_mes;

    //geometry_msgs::Pose _w_pose;
};
