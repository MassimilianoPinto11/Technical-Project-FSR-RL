#include "ros/ros.h"
#include "ros/package.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "gazebo_msgs/LinkStates.h"
#include "dogbot_ctrl/dyntree.h"
#include "dogbot_ctrl/opt.h"

#include <Eigen/Core>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include "alglib/optimization.h"
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <std_msgs/Float64.h>
#include <chrono>  

/*include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>*/

#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ContactsState.h"
//#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>

using namespace std;


class DOGBOT_TOPIC {

    public:
        DOGBOT_TOPIC();
        void joint_states_cb(const sensor_msgs::JointState::ConstPtr& js);
        void base_states_cb(const gazebo_msgs::ModelStates & msg);
        void contact_fl_cb(gazebo_msgs::ContactsStateConstPtr);
        void contact_fr_cb(gazebo_msgs::ContactsStateConstPtr);
        void contact_bl_cb(gazebo_msgs::ContactsStateConstPtr);
        void contact_br_cb(gazebo_msgs::ContactsStateConstPtr);
        void run();
        void loop();
        void estimator();
        void trajectory();
        void control(ros::Rate loop_rate);
        
        towr::NlpFormulation dogplanner(Eigen::Vector3d &desired_pos, double flag);
        
        ros::ServiceClient _set_model_state_srv;
        ros::ServiceClient _set_model_config_srv;
        ros::ServiceClient _pauseGazebo;
        ros::ServiceClient _unpauseGazebo;

        bool endswing = false;
        bool takeoff = false;
        bool endswing2 = false;
        bool takeoff2 = false;
        
  
        DYNTREE* dyn;
        CONTROL* ctrl;
        

        iDynTree::Vector6 rc_ref;
        iDynTree::Vector6 d_rc_ref;
        iDynTree::Vector6 dd_rc_ref;
        Eigen::Matrix<double,12,1> fext;
        Eigen::Matrix<double,12,1> _tau_ref;
        Eigen::Matrix<double,12,1> _fgr_ref;

        towr::SplineHolder solution; 
        bool _js_available = false, _bs_available = false, _ready=false, _ready_ref=false, _ready_est=false;
         float _a=2;
        Eigen::Matrix<double,12,1> jointPos, jointVel;
        Eigen::Matrix<double,6,1> basePos, baseVel;
        Eigen::Matrix4d world_H_base;
        Eigen::Vector3d gravity;
        ros::Time begin;
        ros::Time begin2;

        double _duration_stand;
        double _duration_stand3;
        double _duration_swing_1;
  
        double _duration_swing_2;

  
        Eigen::Matrix<double,12,1> _q_dot;
        Eigen::Matrix<double,12,1> _q_dd;

        gazebo::msgs::WorldControl _step; 
        gazebo::transport::PublisherPtr _pub;

        Eigen::Matrix<double,6,1> compos;
        Eigen::Matrix<double,6,1> comvel;

        double _flag_=1;
        towr::NlpFormulation trj ;

        int i = 0;
        bool stand_2 = false;


        

        
    private:
       
        ros::NodeHandle _nh;
        ros::Subscriber _js_sub, _bs_sub, _cntbl_sub, _cntfl_sub,  _cntbr_sub,  _cntfr_sub;
        ros::Publisher _cmd_pub[12];
        ros::Publisher  _data_pub;

        std_msgs::Float32MultiArray _data;

        bool _contact_FL = true;      
        bool _contact_FR = true;
        bool _contact_BL = true;
        bool _contact_BR = true;
        Eigen::Matrix<double,3,1> _force_FLfoot;
        Eigen::Matrix<double,3,1> _force_FRfoot;
        Eigen::Matrix<double,3,1> _force_BLfoot;
        Eigen::Matrix<double,3,1> _force_BRfoot;

        bool _already_used = false;


        Eigen::MatrixXd integration(Eigen::Matrix<double,12,1> &element,Eigen::Matrix<double,12,1> &int_prev);
       
        Eigen::Vector3d nextpos ;

        Eigen::MatrixXd _gamma;
        Eigen::MatrixXd  _integral_old;
        Eigen::MatrixXd  _fgr_sensor;
        bool first_time = true;

        Eigen::Matrix<double,12,1> integral_dot ;
        Eigen::Matrix<double,12,1> integral;
        Eigen::Matrix<double,12,1> rho ;

        int stand_phase(ros::Rate loop_rate);
        int swing_phase1(ros::Rate loop_rate);
        int swing_phase2(ros::Rate loop_rate);

        string _model_name;


        bool flag= true;
        int flag_trj = 0;

        bool negative_yaw = false;
        bool positive_yaw = false;

        double base_r=0, base_p=0, base_yaw=0;

        
            

};

DOGBOT_TOPIC::DOGBOT_TOPIC(){

    _set_model_config_srv = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    _set_model_state_srv = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    _js_sub = _nh.subscribe("/dogbot/joint_states", 1, &DOGBOT_TOPIC::joint_states_cb,this);
    _bs_sub = _nh.subscribe("/gazebo/model_states", 1, &DOGBOT_TOPIC::base_states_cb, this);
    _cntfl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",1, &DOGBOT_TOPIC::contact_fl_cb,this);
    _cntfr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",1,&DOGBOT_TOPIC::contact_fr_cb,this);
    _cntbl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",1,  &DOGBOT_TOPIC::contact_bl_cb,this);
    _cntbr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",1, &DOGBOT_TOPIC::contact_br_cb,this);
    
    _data_pub=_nh.advertise< std_msgs::Float32MultiArray >("/dogbot/data", 1);
    _data.data.resize(4);

    _pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    gravity[0]= 0;
    gravity[1]= 0;
    gravity[2]= -9.8;

    nextpos.setZero();


    _cmd_pub[0] = _nh.advertise<std_msgs::Float64>("/dogbot/front_right_roll_effort_controller/command", 1);
    _cmd_pub[1] = _nh.advertise<std_msgs::Float64>("/dogbot/front_right_pitch_effort_controller/command", 1);
    _cmd_pub[2] = _nh.advertise<std_msgs::Float64>("/dogbot/front_right_knee_effort_controller/command", 1);
    _cmd_pub[3] = _nh.advertise<std_msgs::Float64>("/dogbot/front_left_roll_effort_controller/command", 1);
    _cmd_pub[4] = _nh.advertise<std_msgs::Float64>("/dogbot/front_left_pitch_effort_controller/command", 1);
    _cmd_pub[5] = _nh.advertise<std_msgs::Float64>("/dogbot/front_left_knee_effort_controller/command", 1);
    _cmd_pub[6] = _nh.advertise<std_msgs::Float64>("/dogbot/back_right_roll_effort_controller/command", 1);
    _cmd_pub[7] = _nh.advertise<std_msgs::Float64>("/dogbot/back_right_pitch_effort_controller/command",1);
    _cmd_pub[8] = _nh.advertise<std_msgs::Float64>("/dogbot/back_right_knee_effort_controller/command", 1);
    _cmd_pub[9] = _nh.advertise<std_msgs::Float64>("/dogbot/back_left_roll_effort_controller/command", 1);
    _cmd_pub[10] = _nh.advertise<std_msgs::Float64>("/dogbot/back_left_pitch_effort_controller/command", 1);
    _cmd_pub[11] = _nh.advertise<std_msgs::Float64>("/dogbot/back_left_knee_effort_controller/command", 1);


    std::string dogbot_path = ros::package::getPath("dogbot_description");
    string modelFile= dogbot_path + "/urdf/dogbot.urdf";

    _model_name = "dogbot";

    dyn = new DYNTREE(modelFile);
    ctrl = new CONTROL(*dyn);
    

   nextpos.setZero();
   _gamma = Eigen::MatrixXd::Zero(12,1);
   _integral_old = Eigen::MatrixXd::Zero(12,1);
   fext = Eigen::MatrixXd::Zero(12,1);
   _fgr_sensor = Eigen::MatrixXd::Zero(12,1);
    integral_dot = Eigen::MatrixXd::Zero(12,1);
      integral = Eigen::MatrixXd::Zero(12,1);
      rho = Eigen::MatrixXd::Zero(12,1);
}


void DOGBOT_TOPIC::joint_states_cb(const sensor_msgs::JointState::ConstPtr& js){ 

_js_available = true;

  
        jointPos(0,0) =js->position[11];//FR_roll
        jointPos(1,0) =js->position[8];//FL_roll
        jointPos(2,0) =js->position[5];//BR_roll
        jointPos(3,0) =js->position[2];//BL_roll
        jointPos(4,0) =js->position[1];//BL_p
        jointPos(5,0) =js->position[0];//BL_k
        jointPos(6,0) =js->position[4];//BR_p
        jointPos(7,0) =js->position[3];//BR_K
        jointPos(8,0) =js->position[7];//FL_p
        jointPos(9,0) =js->position[6];//FL_K
        jointPos(10,0) =js->position[10];//FR_P
        jointPos(11,0) =js->position[9];//FR_K
        
        //if(js->velocity.size()>0){
            jointVel(0,0) =js->velocity[11];//FR_roll
            jointVel(1,0) =js->velocity[8];//FL_roll
            jointVel(2,0) =js->velocity[5];//BR_roll
            jointVel(3,0) =js->velocity[2];//BL_roll
            jointVel(4,0) =js->velocity[1];//BL_p
            jointVel(5,0) =js->velocity[0];//BL_k
            jointVel(6,0) =js->velocity[4];//BR_p
            jointVel(7,0) =js->velocity[3];//BR_K
            jointVel(8,0) =js->velocity[7];//FL_p
            jointVel(9,0) =js->velocity[6];//FL_K
            jointVel(10,0)=js->velocity[10];//FR_P
            jointVel(11,0)=js->velocity[9];//FR_K
    //   }
    

     
}

void DOGBOT_TOPIC::base_states_cb( const gazebo_msgs::ModelStates & msg ) {

    

    bool found = false;
    int index = 0;
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }
    
if(found){
    world_H_base.setIdentity();

    basePos(0,0)=msg.pose[index].position.x;
    basePos(1,0)=msg.pose[index].position.y;
    basePos(2,0)=msg.pose[index].position.z;

    Eigen::Vector4d oriention_base;
     
    tf::Quaternion q(
    msg.pose[index].orientation.x,
    msg.pose[index].orientation.y,
    msg.pose[index].orientation.z,
    msg.pose[index].orientation.w);
    q.normalize();
    Eigen::Matrix<double,3,3> rot;
    tf::matrixTFToEigen(tf::Matrix3x3(q),rot);   /**< quaternion -> rotation Matrix */

    

    if(base_yaw< 0.0){

      negative_yaw = true;


    }


        if(base_yaw > 0.0){

      positive_yaw = true;


    }

    tf::Matrix3x3(q).getRPY(base_r, base_p, base_yaw);

    

    if(base_yaw > 0.0 &&  negative_yaw){

      base_yaw = base_yaw -2*3.14159;

    }


    if(base_yaw < 0.0 &&  positive_yaw){

      base_yaw = base_yaw + 2*3.14159;

    }

    basePos(3,0)= base_r;
    basePos(4,0)= base_p;
    basePos(5,0) = base_yaw;






    world_H_base.block(0,0,3,3)= rot;
    world_H_base.block(0,3,3,1)= basePos.block(0,0,3,1);

    baseVel(0,0)=msg.twist[index].linear.x;
    baseVel(1,0)=msg.twist[index].linear.y;
    baseVel(2,0)=msg.twist[index].linear.z;
    baseVel(3,0)=msg.twist[index].angular.x;
    baseVel(4,0)=msg.twist[index].angular.y;
    baseVel(5,0)=msg.twist[index].angular.z;

    _bs_available = true;
  }
}

void DOGBOT_TOPIC::contact_fl_cb(gazebo_msgs::ContactsStateConstPtr eeFL){

	if(eeFL->states.empty()){ 
    _contact_FL= false;
	}
	else{
		_contact_FL= true;
    _force_FLfoot<<eeFL->states[0].total_wrench.force.x, eeFL->states[0].total_wrench.force.y, eeFL->states[0].total_wrench.force.z;
	
  }

}

void DOGBOT_TOPIC::contact_fr_cb(gazebo_msgs::ContactsStateConstPtr eeFR){

	if(eeFR->states.empty()){ 
    _contact_FR= false;
	}
	else{
		_contact_FR= true;
    _force_FRfoot<<eeFR->states[0].total_wrench.force.x, eeFR->states[0].total_wrench.force.y, eeFR->states[0].total_wrench.force.z;
	
  }
  
}

void DOGBOT_TOPIC::contact_bl_cb(gazebo_msgs::ContactsStateConstPtr eeBL){

  if(eeBL->states.empty()){ 
    _contact_BL= false;
	}
	else{
		_contact_BL= true;
    _force_BLfoot<<eeBL->states[0].total_wrench.force.x, eeBL->states[0].total_wrench.force.y, eeBL->states[0].total_wrench.force.z;
	
  }
  
}

void DOGBOT_TOPIC::contact_br_cb(gazebo_msgs::ContactsStateConstPtr eeBR){

  if(eeBR->states.empty()){ 
    _contact_BR= false;
	}
	else{
		_contact_BR= true;
    _force_BRfoot<<eeBR->states[0].total_wrench.force.x, eeBR->states[0].total_wrench.force.y, eeBR->states[0].total_wrench.force.z;
	
  }
  

}




void DOGBOT_TOPIC::run(){
    boost::thread loop_t ( &DOGBOT_TOPIC::loop, this);
    ros::spin();	
}




Eigen::MatrixXd DOGBOT_TOPIC::integration( Eigen::Matrix<double,12,1> &element,Eigen::Matrix<double,12,1> &int_prev){
    
    float dt=0.001;
    
    Eigen::MatrixXd integrated = Eigen::MatrixXd::Zero(12,1);
    integrated = int_prev + (element*dt);

    return integrated;
}


void DOGBOT_TOPIC::trajectory(){
      
  Eigen::Matrix<double,6,1> pos_com =dyn->getCOM_pos();

  if(flag_trj == 0){
        if(compos[1]>-2.0){
            nextpos[1]= nextpos[1] -0.04;
        }
        else{
          nextpos[1]= -2.0;
          flag_trj=1;
        }

  }


  if(flag_trj == 1){
    
    if(i <9 && flag_trj == 1 ){

        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= nextpos[2] - 0.1745;
        i++;


    }
    else{
        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= - 1.5708;
        flag_trj = 2;
        i = 0;
        

      }

  

  }

  if(flag_trj == 2){
  if(compos[0]>-2.0){
      nextpos[0]= nextpos[0]- 0.04;
        nextpos[1]= nextpos[1];
        nextpos[2]=  - 1.5708;}
  else{

        nextpos[0]= - 2;
        nextpos[1]= nextpos[1];
        nextpos[2]=  - 1.5708;
        flag_trj = 3;

  }
  }

  if(flag_trj == 3){

    if(i <9 && flag_trj == 3 ){

        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= nextpos[2] + 0.1745;
        i++;


    }

    else{
        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= 0;
        flag_trj = 4;
        i = 0;
        

      }
  }

  if (flag_trj == 4){

  if(compos[1]>-4.9){
      nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1]- 0.04;
        nextpos[2]=  0;}
  else{

        nextpos[0]= nextpos[0];
        nextpos[1]= - 5.0;
        nextpos[2]=  0;
        flag_trj = 5;

  }


  }

  if (flag_trj == 5){

      if(i <18 && flag_trj == 5 ){

        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= nextpos[2] + 0.1745;
        i++;


    }

    else{
        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= 3.14;
        flag_trj = 6;
        i = 0;
        

      }

  }



  if (flag_trj == 6){

  if(compos[1]<-4.0){
      nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1]+ 0.04;
        nextpos[2]=  3.14;}
  else{

        nextpos[0]= nextpos[0];
        nextpos[1]= - 4.0;
        nextpos[2]=  3.14;
        flag_trj = 7;

  }
  }

  if (flag_trj == 7){

      if(i <9 && flag_trj == 7 ){

        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= nextpos[2] + 0.1745;
        i++;


    }

    else{
        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= 4.71;
        flag_trj = 8;
        i = 0;
        

      }

  }

  if (flag_trj == 8){

  if(compos[0]>-4.0){
      nextpos[0]= nextpos[0]-0.04;
        nextpos[1]= nextpos[1];
        nextpos[2]= 4.71;}
  else{

        nextpos[0]= - 4.0;
        nextpos[1]= nextpos[1];
        nextpos[2]= 4.71;
        flag_trj = 9;

  }
  }

  if (flag_trj == 9){

      if(i <9 && flag_trj == 9 ){

        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= nextpos[2] - 0.1745;
        i++;


    }

    else{
        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= 3.14;
        flag_trj = 10;
        i = 0;
        

      }

  }

  if (flag_trj == 10){

  if(compos[1]<-1){
      nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1]+0.04;
        nextpos[2]= 3.14;}
  else{

        nextpos[0]= nextpos[0];
        nextpos[1]= -1;
        nextpos[2]= 3.14;
        flag_trj = 11;

  }
  }

  if (flag_trj == 11){

      if(i <9 && flag_trj == 11 ){

        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= nextpos[2] - 0.1745;
        i++;


    }

    else{
        nextpos[0]= nextpos[0];
        nextpos[1]= nextpos[1];
        nextpos[2]= 1.57;
        flag_trj = 12;
        i = 0;
        

      }

  }


  if (flag_trj == 12){

  if(compos[0]<3.9){
      nextpos[0]= nextpos[0]+0.04;
        nextpos[1]= nextpos[1];
        nextpos[2]= 1.57;}
  else{

        nextpos[0]= 4;
        nextpos[1]= nextpos[1];
        nextpos[2]= 1.57;
        flag_trj = 13;

    }
  }

  if (flag_trj == 13){

  if(compos[0]>0){
      nextpos[0]= nextpos[0]-0.04;
        nextpos[1]= nextpos[1];
        nextpos[2]= 1.57;}
  else{

        nextpos[0]= 0;
        nextpos[1]= nextpos[1];
        nextpos[2]= 1.57;
        flag_trj = 13;

    }
  }      
  

}


towr::NlpFormulation DOGBOT_TOPIC::dogplanner(Eigen::Vector3d &desired_pos, double flag){

    towr::NlpFormulation formulation;

    
    auto gategen = towr::GaitGenerator::MakeGaitGenerator(4);
    Eigen::Matrix<double,3,1> bl, br, fl, fr;

    formulation.terrain_ = std::make_shared<towr::FlatGround>(0.0); 
    formulation.model_ = towr::RobotModel(towr::RobotModel::Dogbot);

    auto stance = formulation.model_.kinematic_model_->GetNominalStanceInBase(); 
    formulation.initial_ee_W_ = stance;
    Eigen::Matrix<double,3,1> com_pos_lin = dyn->getCOM_pos().block(0,0,3,1);

    formulation.initial_base_.lin.at(towr::kPos) << com_pos_lin;
    formulation.initial_base_.ang.at(towr::kPos) << dyn->getCOM_pos().block(3,0,3,1);
    formulation.initial_base_.lin.at(towr::kVel) << dyn->getCOM_vel().block(0,0,3,1);
    formulation.initial_base_.ang.at(towr::kVel) << dyn->getCOM_vel().block(3,0,3,1);




    Eigen::Vector3d pos_ee;
    for (int i=0; i<4;i++ ){
      switch(i){
        case 0: pos_ee= dyn->getBLpos();
        break;
        case 1: pos_ee=dyn->getBRpos();
        break;
        case 2: pos_ee=dyn->getFLpos();
        break;
        case 3: pos_ee=dyn->getFRpos();
        break;
     }

    formulation.initial_ee_W_.at(i)[0] = pos_ee[0];
    formulation.initial_ee_W_.at(i)[1] = pos_ee[1];

    }

    std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){ p.z() = 0.0; }); 




   formulation.final_base_.lin.at(towr::kPos) << desired_pos[0], desired_pos[1], 0.36; 
   formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, desired_pos[2];




if(flag==1){
    auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
    gategen->SetCombo(gait_type);
}

if(flag == 2){

      auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);
    gategen->SetCombo(gait_type);
}

    formulation.params_.ee_phase_durations_.clear();
    for(int i=0;i<4;++i){
	    formulation.params_.ee_phase_durations_.push_back(gategen->GetPhaseDurations(0.5,i)); 
  	    formulation.params_.ee_in_contact_at_start_.push_back(gategen->IsInContactAtStart(i));
    }

    ifopt::Problem nlp;
    

    for(auto c: formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for(auto c: formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for(auto c: formulation.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver -> SetOption("jacobian_approximation","exact");
    solver -> SetOption("max_cpu_time",20.0);
    solver->SetOption("print_level", 5);
    solver -> Solve(nlp);
    

    return formulation; //POTREI ANCHE FARE CHE MI DA SOLUTION
}


void DOGBOT_TOPIC::control(ros::Rate loop_rate){



}


int DOGBOT_TOPIC::stand_phase(ros::Rate loop_rate){ 
   while((ros::Time::now()-begin).toSec()< _duration_stand )
   {
    
        //cout<<"STAND_PHASE "<<endl;
        double t = (ros::Time::now()-begin).toSec();
        //cout<<"tempo: "<<t<<endl;
        double tempo= 0.0;
       
        _tau_ref=Eigen::MatrixXd::Zero(12,1);
    
       
        iDynTree::Vector6 rc_ref, d_rc_ref, dd_rc_ref;
    

        iDynTree::toEigen(rc_ref)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
        iDynTree::toEigen(d_rc_ref)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
        iDynTree::toEigen(dd_rc_ref) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();


        //UPDATE
        dyn->dogbot_idyntree(world_H_base,jointPos,jointVel,baseVel,gravity);
      
        
        _tau_ref= ctrl->getTau_stand(rc_ref,d_rc_ref,dd_rc_ref,fext); 


        std_msgs::Float64 cmd_[12];


        cmd_[0].data =_tau_ref(0,0); 
        cmd_[1].data =_tau_ref(10,0);
        cmd_[2].data =_tau_ref(11,0);
        cmd_[3].data =_tau_ref(1,0);
        cmd_[4].data =_tau_ref(8,0);
        cmd_[5].data =_tau_ref(9,0);
        cmd_[6].data =_tau_ref(2,0);
        cmd_[7].data =_tau_ref(6,0);
        cmd_[8].data =_tau_ref(7,0);
        cmd_[9].data =_tau_ref(3,0);
        cmd_[10].data=_tau_ref(4,0);
        cmd_[11].data=_tau_ref(5,0);
     
      for(int i=0;i<12;i++){
           _cmd_pub[i].publish( cmd_[i] );
            }


        //_pub->Publish(_step);
        // loop_rate.sleep();


   }

   return 0;
}

  

//SWING_1 

int DOGBOT_TOPIC::swing_phase1(ros::Rate loop_rate){

   
    while((ros::Time::now()-begin).toSec()< _duration_swing_1 && !endswing){ 
        
        //cout<<"SWING1_PHASE "<<endl;
        //double tempo= 0.0;

        _tau_ref=Eigen::MatrixXd::Zero(12,1);
    
        double t = (ros::Time::now()-begin).toSec();
        //cout<<"tempo: "<<t<<endl;
        
        
        Eigen::Matrix<double,12,1> fext;
        iDynTree::Vector6 rc_refsw1, d_rc_refsw1, dd_rc_refsw1;


        iDynTree::toEigen(rc_refsw1)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
        iDynTree::toEigen(d_rc_refsw1)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
        iDynTree::toEigen(dd_rc_refsw1) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
        
        
        Eigen::Matrix<double,6,1> dd_xsw_ref;
        dd_xsw_ref<< solution.ee_motion_.at(1)->GetPoint(t).a(),
                        solution.ee_motion_.at(2)->GetPoint(t).a();

        Eigen::Matrix<double,6,1> d_xsw_ref;
        d_xsw_ref<< solution.ee_motion_.at(1)->GetPoint(t).v(),
                    solution.ee_motion_.at(2)->GetPoint(t).v();
        

        Eigen::Matrix<double,6,1> xsw_ref;
        xsw_ref<< solution.ee_motion_.at(1)->GetPoint(t).p(),
                    solution.ee_motion_.at(2)->GetPoint(t).p();


        //UPDATE
        dyn->dogbot_idyntree(world_H_base,jointPos,jointVel,baseVel,gravity);

        _tau_ref= ctrl->getTau_swing(0,rc_refsw1,d_rc_refsw1,dd_rc_refsw1,dd_xsw_ref,d_xsw_ref,xsw_ref,fext); 



        std_msgs::Float64 cmd_[12];


        cmd_[0].data =_tau_ref(0,0); 
        cmd_[1].data =_tau_ref(10,0);
        cmd_[2].data =_tau_ref(11,0);
        cmd_[3].data =_tau_ref(1,0);
        cmd_[4].data =_tau_ref(8,0);
        cmd_[5].data =_tau_ref(9,0);
        cmd_[6].data =_tau_ref(2,0);
        cmd_[7].data =_tau_ref(6,0);
        cmd_[8].data =_tau_ref(7,0);
        cmd_[9].data =_tau_ref(3,0);
        cmd_[10].data=_tau_ref(4,0);
        cmd_[11].data=_tau_ref(5,0);
     
     
        for(int i=0;i<12;i++){
           _cmd_pub[i].publish( cmd_[i] );
   
            }


        if( !_contact_BR && !_contact_FL)  takeoff=true;
        if(takeoff && _contact_BR && _contact_FL) endswing=true;


        // _pub->Publish(_step);
        //loop_rate.sleep();

  }
    return 0;
}


  

//SWING_2 
  
int DOGBOT_TOPIC::swing_phase2(ros::Rate loop_rate){
 while((ros::Time::now()-begin).toSec()< _duration_swing_2 && !endswing2 ){ //
     
      //   cout<<" SWING2 "<<endl;
      double t = (ros::Time::now()-begin).toSec();
      //   cout<<"tempo: "<<t<<endl;
      //   cout<<endl;
      double tempo= 0.0;

      _tau_ref=Eigen::MatrixXd::Zero(12,1);
    
      Eigen::Matrix<double,12,1> fext;
      iDynTree::Vector6 rc_ref, d_rc_ref, dd_rc_ref;

      iDynTree::toEigen(rc_ref)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
      iDynTree::toEigen(d_rc_ref)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
      iDynTree::toEigen(dd_rc_ref) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
      
      Eigen::Matrix<double,6,1> dd_xsw_ref2;
      dd_xsw_ref2<< solution.ee_motion_.at(0)->GetPoint(t).a(),
                    solution.ee_motion_.at(3)->GetPoint(t).a();

      Eigen::Matrix<double,6,1> d_xsw_ref2;
      d_xsw_ref2<< solution.ee_motion_.at(0)->GetPoint(t).v(),
                  solution.ee_motion_.at(3)->GetPoint(t).v();
      
      Eigen::Matrix<double,6,1> xsw_ref2;
      xsw_ref2<< solution.ee_motion_.at(0)->GetPoint(t).p(),
                solution.ee_motion_.at(3)->GetPoint(t).p();


      //UPDATE
      dyn->dogbot_idyntree(world_H_base,jointPos,jointVel,baseVel,gravity );   


      _tau_ref= ctrl->getTau_swing(1,rc_ref,d_rc_ref,dd_rc_ref,dd_xsw_ref2,d_xsw_ref2,xsw_ref2,fext);       


      std_msgs::Float64 cmd_[12];


      cmd_[0].data =_tau_ref(0,0); 
      cmd_[1].data =_tau_ref(10,0);
      cmd_[2].data =_tau_ref(11,0);
      cmd_[3].data =_tau_ref(1,0);
      cmd_[4].data =_tau_ref(8,0);
      cmd_[5].data =_tau_ref(9,0);
      cmd_[6].data =_tau_ref(2,0);
      cmd_[7].data =_tau_ref(6,0);
      cmd_[8].data =_tau_ref(7,0);
      cmd_[9].data =_tau_ref(3,0);
      cmd_[10].data=_tau_ref(4,0);
      cmd_[11].data=_tau_ref(5,0);
      
     
      for(int i=0;i<12;i++){
           _cmd_pub[i].publish( cmd_[i] );

            }


      if( !_contact_BL && !_contact_FR)  takeoff2 = true;
      if(takeoff2 && _contact_BL && _contact_FR){endswing2 = true;} 

      
      //_pub->Publish(_step); 

      //loop_rate.sleep();
     
 }
   return 0;
}



void DOGBOT_TOPIC::estimator(){

  /*ros::Rate rate_est(1000);

  while( !_ready_est) {cout<<"SONO NEL WHILE NOT READY ESTIMATOR "<<endl;}

  while(ros::ok()){


    Eigen::Matrix<double,3,3> Tbl= dyn->getBLworldtransform();
    Eigen::Matrix<double,3,3> Tbr= dyn->getBRworldtransform();
    Eigen::Matrix<double,3,3> Tfl= dyn->getFLworldtransform();
    Eigen::Matrix<double,3,3> Tfr= dyn->getFRworldtransform();
    Eigen::Matrix<double,12,1> _fgr_sensor;
    _fgr_sensor<< Tbl*_force_BLfoot, Tbr* _force_BRfoot, Tfl* _force_FLfoot ,Tfr* _force_FRfoot;

    double h = 0.001;
    Eigen::Matrix<double,12,12> k_1=10*Eigen::MatrixXd::Identity(12,12);
    Eigen::Matrix<double,12,1> q_dot= dyn->get_vc().block(6,0,12,1);
    Eigen::Matrix<double,12,12> jst= dyn->getJlegslin().block(0,6,12,12);

    rho= dyn->getMDAG().block(6,6,12,12) * q_dot;
    
    integral_dot = _gamma + dyn->getBiasCOM().block(6,0,12,1)+ _tau_ref + jst.transpose()*_fgr_sensor;
    
    integral = _integral_old + h*integral_dot;
    
    _gamma = k_1*rho -k_1*integral;
    
    _integral_old = integral;

    fext = jst.transpose().inverse()*_gamma;


    rate_est.sleep();

  }*/
  
}


void DOGBOT_TOPIC::loop (){


  ros::Rate loop_rate(1000);
  std_srvs::Empty pauseSrv;
  std_srvs::Empty unpauseSrv;


  gazebo_msgs::SetModelState robot_init_state;
  robot_init_state.request.model_state.model_name = "dogbot";
  robot_init_state.request.model_state.reference_frame = "world";
  robot_init_state.request.model_state.pose.position.x=-0.00;
  robot_init_state.request.model_state.pose.position.y= -0.034102251365;
  robot_init_state.request.model_state.pose.position.z=0.430159040502;
  robot_init_state.request.model_state.pose.orientation.x=0.0;
  robot_init_state.request.model_state.pose.orientation.y=0.0;
  robot_init_state.request.model_state.pose.orientation.z=0.0;
  robot_init_state.request.model_state.pose.orientation.w=1;

  if(_set_model_state_srv.call(robot_init_state))
    ROS_INFO("Robot state set.");
  else
    ROS_INFO("Failed to set robot state.");

  
  gazebo_msgs::SetModelConfiguration robot_init_config;
  robot_init_config.request.model_name = "dogbot";
  robot_init_config.request.urdf_param_name = "robot_description";
  robot_init_config.request.joint_names.push_back("back_left_roll_joint");
  robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
  robot_init_config.request.joint_names.push_back("back_left_knee_joint");
  robot_init_config.request.joint_names.push_back("back_right_roll_joint");
  robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
  robot_init_config.request.joint_names.push_back("back_right_knee_joint");
  robot_init_config.request.joint_names.push_back("front_left_roll_joint");
  robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
  robot_init_config.request.joint_names.push_back("front_left_knee_joint");
  robot_init_config.request.joint_names.push_back("front_right_roll_joint");
  robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
  robot_init_config.request.joint_names.push_back("front_right_knee_joint");
  robot_init_config.request.joint_positions.push_back( 0.0004875394147498824);
  robot_init_config.request.joint_positions.push_back( -0.884249947977489);
  robot_init_config.request.joint_positions.push_back(-1.6039026405138666);
  robot_init_config.request.joint_positions.push_back( 0.0006243098169198547);
  robot_init_config.request.joint_positions.push_back(0.8861978063639038);
  robot_init_config.request.joint_positions.push_back(1.6032646991719783);
  robot_init_config.request.joint_positions.push_back(-3.197670677312914e-05);
  robot_init_config.request.joint_positions.push_back(-0.8848124990461947);
  robot_init_config.request.joint_positions.push_back(-1.6039627256817717);
  robot_init_config.request.joint_positions.push_back(-0.0005127385581351618);
  robot_init_config.request.joint_positions.push_back(0.886353788084274);
  robot_init_config.request.joint_positions.push_back( 1.60361055049274);

  if(_set_model_config_srv.call(robot_init_config))
    ROS_INFO("Robot configuration set.");
  else
    ROS_INFO("Failed to set robot configuration.");

  ros::spinOnce();

  while( !_js_available)
    usleep(0.1*1e6);

  while( !_bs_available )
    usleep(0.1*1e6);


  _pauseGazebo.call(pauseSrv);

  if(_pauseGazebo.call(pauseSrv))
    ROS_INFO("Simulation paused.");
  else
    ROS_INFO("Failed to pause simulation.");


  dyn->dogbot_idyntree(world_H_base,jointPos,jointVel,baseVel,gravity );
  

 
   
  while(ros::ok){
        
    cout<<"SONO PRIMA DELL'UPDATE"<<endl;

      
    compos =dyn->getCOM_pos();
    cout<<"COMPOS: "<<endl;
    cout<<compos<<endl;
    cout<<endl;

    comvel = dyn->get_vc();
    cout<<"COMVEL: "<<endl;
    cout<<comvel<<endl;
    cout<<endl;

          

  

    _pauseGazebo.call(pauseSrv);

    if(_pauseGazebo.call(pauseSrv))
       ROS_INFO("Simulation paused.");
    else
       ROS_INFO("Failed to pause simulation.");

  
    trajectory();
    trj =dogplanner(nextpos,1);




    dyn->dogbot_idyntree(world_H_base,jointPos,jointVel,baseVel,gravity );
    Eigen::MatrixXd compos2 =dyn->getCOM_pos();
    cout<<"COMPOS DOPO TOWR: "<<endl;
    cout<<compos2<<endl;
    cout<<endl;

    endswing = false;
    takeoff = false;
    endswing2 = false;
    takeoff2 = false;
        

    _unpauseGazebo.call(unpauseSrv);



    _tau_ref=Eigen::MatrixXd::Zero(12,1);
    _fgr_ref=Eigen::MatrixXd::Zero(12,1);


//STAND 1
    _duration_stand = trj.params_.ee_phase_durations_.at(1)[0];
    begin = ros::Time::now(); 

    stand_phase(loop_rate);


  

//SWING 1

    _duration_swing_1 = trj.params_.ee_phase_durations_.at(1)[0] + trj.params_.ee_phase_durations_.at(1)[1];

    swing_phase1(loop_rate);


  

//STAND 2

    _duration_stand = trj.params_.ee_phase_durations_.at(0)[0];
    _duration_stand3 = trj.params_.ee_phase_durations_.at(0)[0];

    stand_phase(loop_rate);

    stand_2 = true;



    _pauseGazebo.call(pauseSrv);

    if(_pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused. Replanning...");
    else
        ROS_INFO("Failed to pause simulation.");  

  
    dyn->dogbot_idyntree(world_H_base,jointPos,jointVel,baseVel,gravity );
    Eigen::MatrixXd compos3 =dyn->getCOM_pos();
    cout<<"COMPOS PRIMA TOWR: "<<endl;
    cout<<compos3<<endl;
    cout<<endl;


    trj =dogplanner(nextpos,2);


    endswing = false;
    takeoff = false;
    endswing2 = false;
    takeoff2 = false;
          

    _unpauseGazebo.call(unpauseSrv);

    //STAND 3

    _duration_stand = trj.params_.ee_phase_durations_.at(0)[0] ;

    begin = ros::Time::now();

    stand_phase(loop_rate);

    //SWING2

    _duration_swing_2 = trj.params_.ee_phase_durations_.at(0)[0] + trj.params_.ee_phase_durations_.at(0)[1];

    swing_phase2(loop_rate);



    //STAND 4

    _duration_stand = trj.params_.ee_phase_durations_.at(1)[0];

    stand_phase(loop_rate);



    _pauseGazebo.call(pauseSrv);
  

     }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "dogbot_ctrl");
    
    DOGBOT_TOPIC topic;

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::client::setup(argc,argv);

    topic._pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    topic._pub->WaitForConnection();

    topic._step.set_step(1);

    topic.run();

	
    return 0;          
}


