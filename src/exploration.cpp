#include "controller.h"

bool HL_CONTROLLER::search_QR(){
  //Verificare se c'è un QR-code nel volume misurato
  /*Se è stato trovato un QR_code:
  1) verificare se non è già presente in agenda
  2) se non è presente in agenda aggiungerlo e lanciare agenda_check
  */
  cout << "Qr found \n";
  usleep(10000);
  return true;
}

bool HL_CONTROLLER::next_point(){
  /*Identificare il prossimo punto da esplorare e inviare la posizione al drone
  Il prossimo punto da esplorare segue la priorità:
  1)un oggetto in agenda (è possibile inserire una eventuale priorità fra gli oggetti)
    e poi lanciare agenda check
  2)un cubo frontiera left > front > right > back
  */
  //search_QR();
  ROS_INFO("Moving to target");
  move_drone(2.0, 2.0, 3.5, 0.0);
  return true;
}

bool HL_CONTROLLER::measured_volume(){

  _measured_volume = evaluate_volume();
  if(_measured_volume >= ALPHA*FIELD_VOLUME)
      return true;
  else
      return false;
      
}

void HL_CONTROLLER::check_agenda(){
  int count = 0;
  for(int i=0; i<ELEMENTS; i++){
    if(_objective_list[i].found)
      count ++;
  }

  if(count == ELEMENTS)
    _finish = true;
  else
    _finish = false;
  
}

double HL_CONTROLLER::evaluate_volume(){
    //Calcolare il volume della mappa misurata
    return 580.0;
}

void HL_CONTROLLER::explore(){
  ros::Rate rate(100);
  int count = 0;
  bool explore = true;
  ROS_INFO("Exploring");

  if(!_in_flight){
    takeoff(3.5);
    _in_flight = true;
  }
  
  while(explore){
    if(!_finish){
      if(_in_flight){
        if(count ==1000)
          next_point();
        else if(count ==2000)
          look_around();
        else if(count == 3000)
          _finish = true;
      }
    }
    else
    {
      if(measured_volume()){
        explore = false;
        //manual_land();
        move_drone(_w_p(0), _w_p(1), 0.0, 0.0);
        //To Do: rifare la funzione di landing
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
