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
#include "utils.h"
#include "Eigen/Dense"
#include "tf/tf.h"
#include "motion_planner/generate_plan.h"

#define FIELD_VOLUME 600 //m^3
#define ELEMENTS 4 //elements number of the static agenda
#define ALPHA 0.9 //percentual of the max volume accepted

using namespace Eigen;
using namespace std;

enum CTRL_TYPE {velocity, position};

struct Agenda{
  string type;
  int number;
  geometry_msgs::Pose target_pose;
  geometry_msgs::Pose robot_pose;
  bool found;
};

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

    //--- Actions ---
    //TO DO: rendere bool le funzioni di movimento, in modo che il controllo sappia quando è finito il movimento
    void takeoff( const double );
    void look_around(); 
    void move_drone(const double, const double, const double, const double);
    void manual_land();

    //--- Controllers ---
    void velocity_controller();
    void position_controller();

    //--- Exploration ---
    bool search_QR();
    bool next_point();
    bool measured_volume();
    void check_agenda();
    void explore();
    double evaluate_volume();

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

    motion_planner::generate_plan _plan;

    Eigen::Vector3d _w_p;
    Eigen::Vector4d _w_q;

    int _rate;

    Eigen::Vector3d _p_des;
    Eigen::Vector4d _q_des;
    Eigen::Vector3d _dp_des;
    CTRL_TYPE _ctrl_mode; 

    Agenda _objective_list[ELEMENTS];
    double _measured_volume;
    bool _first_iter;
    bool _first_w_mes;

    bool _finish;
    bool _in_flight;
    bool _new_plan;
    bool _moved;

    //geometry_msgs::Pose _w_pose;
};
