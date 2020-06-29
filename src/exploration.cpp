#include "controller.h"

bool HL_CONTROLLER::search_QR(){
  //Verificare se c'è un QR-code nel volume misurato
  /*Se è stato trovato un QR_code:
  1) verificare se non è già presente in agenda
  2) se non è presente in agenda aggiungerlo e lanciare agenda_check
  */
  ROS_INFO("QR code found");
  _objective_list[0].found = true;
  _objective_list[0].number = 15;
  _objective_list[0].target_pose.position.x= _w_p(0);
  _objective_list[0].target_pose.position.y= _w_p(1);
  _objective_list[0].target_pose.position.z= _w_p(2);

  _objective_list[0].target_pose.orientation.w = _w_q(0);
  _objective_list[0].target_pose.orientation.x = _w_q(1);
  _objective_list[0].target_pose.orientation.y = _w_q(2);
  _objective_list[0].target_pose.orientation.z = _w_q(3);
  return true;
}

void HL_CONTROLLER::search_next_point(){
  /*Identificare il prossimo punto da esplorare e inviare la posizione al drone
  Il prossimo punto da esplorare segue la priorità:
  1)un oggetto in agenda (è possibile inserire una eventuale priorità fra gli oggetti)
    e poi lanciare agenda check
  2)un cubo frontiera left > front > right > back
  */
  //search_QR();
  ROS_INFO("Moving to target");
  _p_des(0) = 0.0;
  _p_des(1) = -2.3;

}

void HL_CONTROLLER::evaluate_volume(){

  for(octomap::OcTree::leaf_iterator it = _tree->begin_leafs(), end = _tree->end_leafs(); it != end; ++it){
    double side_length = it.getSize();
    if (_tree->isNodeOccupied(*it)){ // occupied leaf node
      _vol_occ += pow(side_length,3);
    }
    else { // free leaf node
      _vol_free += pow(side_length,3);
    }

  }

  _vol_unk = FIELD_VOLUME - _vol_occ - _vol_free;
  _vol_tot = _vol_occ + _vol_free;

  cout << "Field volume: " << FIELD_VOLUME << endl;
  cout << "Occupied: " << _vol_occ << endl;
  cout << "Free: " << _vol_free << endl;
  cout << "Unknown: " << _vol_unk << endl;
      
}

void HL_CONTROLLER::check_agenda(){
  int count = 0;
  for(int i=0; i< ELEMENTS; i++){
    if(_objective_list[i].found)
      count ++;
  }

  if(count == ELEMENTS){
    _finish = true;
    ROS_INFO("All targets found");
  }
  else{
    _finish = false;
    ROS_INFO("Missing %d targets", int(ELEMENTS)-count);
  }
  
}

void HL_CONTROLLER::explore(){
  ros::Rate rate(100);
  int count = 0;
  bool explore = true;
  ROS_INFO("Exploring");

  if(!_in_flight){
    takeoff(2.5);
    _in_flight = true;
    sleep(3.0);
  }

  while(explore){
    if(!_finish){
      if(_in_flight){
        routine_on_point();
      }
    }
    else
    {
      if(_vol_tot >= ALPHA*FIELD_VOLUME){
        explore = false;
        //To Do: far funzionare la funzione di landing
        //manual_land();
        _p_des(2) = 0.0;
        ROS_INFO("Landing");
        
      }
      else
      {
        //scegliere il modo di agire
      }
        
    }

    count ++;
    rate.sleep();
    ros::spinOnce();
  }
  
}

void HL_CONTROLLER::routine_on_point(){
  look_around(3.14);
  while(!finished_yaw()) {}
  look_around(0.0);
  while(!finished_yaw()) {}


}

void HL_CONTROLLER::print_nodes(){
  for(octomap::OcTree::leaf_iterator it = _tree->begin_leafs(), end = _tree->end_leafs(); it != end; ++it){
        // Fetching the coordinates in octomap-space
        cout << "  x = " << it.getX() << endl;
        cout << "  y = " << it.getY() << endl;
        cout << "  z = " << it.getZ() << endl;
        cout << "  size = " << it.getSize() << endl;
        cout << "  depth = " << it.getDepth() << endl;
    }
}

bool HL_CONTROLLER::finished_pos(){
  if( (_p_des(0) - _w_p(0)) <= ERROR_THRESHOLD_POS &&
      (_p_des(1) - _w_p(1)) <= ERROR_THRESHOLD_POS &&
      (_p_des(2) - _w_p(2)) <= ERROR_THRESHOLD_POS  )
    return true;
  else
    return false;
}

bool HL_CONTROLLER::finished_yaw(){
  Vector3d rpy= utilities::R2XYZ ( utilities::QuatToMat ( Vector4d(_w_q(0), _w_q(1), _w_q(2), _w_q(3)) ) );
  if( (_yaw_des - rpy(2)) <= ERROR_THRESHOLD_YAW )
    return true;
  else
    return false;
}