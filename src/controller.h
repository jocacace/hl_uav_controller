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
#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <unordered_map>
#include "lvision/codes.h"
#include "lvision/code.h"

#define FIELD_VOLUME 600 //m^3
#define ELEMENTS 1 //elements number of the static agenda
#define ALPHA 0.9 //percentual of the max volume accepted
#define ERROR_THRESHOLD_POS 0.05
#define ERROR_THRESHOLD_YAW 0.3

using namespace Eigen;
using namespace std;

enum CTRL_TYPE {velocity, position};


typedef struct S_tf {

  ros::Time t;
  string frame;
  Matrix4d Hws;
  bool ready;

  S_tf () {
    ready = false;
    Hws = Matrix4d::Identity();
  }

}S_tf;

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
    void pfilter();
    
    //--- Actions ---
    //TO DO: rendere bool le funzioni di movimento, in modo che il controllo sappia quando è finito il movimento
    void takeoff( const double );
    void look_around(const double); 
    void move_drone(const double, const double, const double, const double);
    void manual_land();

    //--- Controllers ---
    void velocity_controller();
    void position_controller();

    //--- QR landing --
    void test_landing();
    void get_tfs();


    //--- Exploration ---
    bool search_QR();
    void search_next_point();
    void evaluate_volume();
    void check_agenda();
    void explore();
    void load_targets();
    void print_nodes();
    void routine_on_point();
    bool finished_pos();
    bool finished_yaw();


    void qr_d_cb( lvision::codes c );
    void qr_f1_cb( lvision::codes c );
    void qr_f2_cb( lvision::codes c );


    octomap::OcTree* _tree;

   
  private:
    ros::NodeHandle _nh;
    ros::Subscriber _mavros_state_sub;
    ros::Subscriber _localization_sub;
    ros::Subscriber _qr_d_cbs[3];
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
    double _yaw_des;
    Eigen::Vector3d _dp_des;
    Eigen::Vector3d _p_cmd;
    double _yaw_cmd;
    CTRL_TYPE _ctrl_mode; 

    Agenda _objective_list[ELEMENTS];
    double _vol_occ;
    double _vol_free;
    double _vol_unk;
    double _vol_tot;
    bool _first_iter;
    bool _first_w_mes;

    bool _finish;
    bool _in_flight;
    bool _moved;
    std::unordered_map<std::string, Vector3d> _qrs[3];

    string _fixed_frame;
    string _camera_frame[3];

    S_tf _sensors_tf[3];


    //geometry_msgs::Pose _w_pose;
};
