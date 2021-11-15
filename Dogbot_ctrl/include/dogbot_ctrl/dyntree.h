#ifndef DYNTREE_H
#define DYNTREE_H

#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"

//#include <ifopt/ipopt_solver.h>
//#include <towr/initialization/gait_generator.h>
//#include <towr/terrain/examples/height_map_examples.h>
//#include <towr/nlp_formulation.h>

#include <cstdlib>
#include <Eigen/Core>
#include <tf/tf.h>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

using namespace std;

class DYNTREE{
    public:
        DYNTREE(std::string modelFile);

       
        void dogbot_idyntree(Eigen::Matrix4d &World_H_base,Eigen::Matrix<double,12,1> &_JointPos, Eigen::Matrix<double,12,1> &_JointVel ,Eigen::Matrix<double,6,1> &Basevel,Eigen::Vector3d &Gravity );
        void computeT();
        void computeJ();
        void computeJdot_qdot();
        double getMass();
        
        Eigen::MatrixXd getCOM_pos();
        Eigen::MatrixXd getCOM_vel();
        Eigen::MatrixXd getCOM_poslin();

        Eigen::MatrixXd getJlegs();
        Eigen::MatrixXd getJlegslin();
        Eigen::MatrixXd getJdotqd_lin();
       
        Eigen::MatrixXd getMassMatrixCOM_com();


        Eigen::MatrixXd getmatrixC();
        Eigen::MatrixXd getCCoM();
        Eigen::MatrixXd get_vc();
        Eigen::MatrixXd get_vb();
        Eigen::VectorXd get_qb();
        
      
        Eigen::MatrixXd getMDAG();//PER DEBUG
        Eigen::MatrixXd getMassMatrix();

        Eigen::MatrixXd getJfoot();

        //Foot position
        Eigen::MatrixXd getBLpos();
        Eigen::MatrixXd getBRpos();
        Eigen::MatrixXd getFLpos();
        Eigen::MatrixXd getFRpos();

        //Foot velocity
        Eigen::MatrixXd getBLvel();
        Eigen::MatrixXd getBRvel();
        Eigen::MatrixXd getFLvel();
        Eigen::MatrixXd getFRvel();

        Eigen::MatrixXd _C;
        Eigen::MatrixXd getBiasCOM();
   
        Eigen::MatrixXd computeDYN();

        Eigen::Matrix<double,3,3> getBLworldtransform();
        Eigen::Matrix<double,3,3> getBRworldtransform();
        Eigen::Matrix<double,3,3> getFLworldtransform();
        Eigen::Matrix<double,3,3> getFRworldtransform();
        
        Eigen::Matrix<double,3,3> get_rotmatrix();
     

        void stampaDEB();


    iDynTree::Model _model; //per debug ma può essere private
    iDynTree::ModelLoader _mdlLoader;	
    private:

    
      //Idyntree variables
      iDynTree::Transform _world_H_base;
      iDynTree::VectorDynSize _jointPos;
      iDynTree::Twist _baseVel;
      iDynTree::VectorDynSize _jointVel;
      iDynTree::Vector3 _gravity; 
      iDynTree::KinDynComputations _kinDynComp;
      
      int _nDOFs;
      double _dogbot_mass;

      
      
      //Com variables
      Eigen::MatrixXd _comPos ;
      Eigen::MatrixXd _comPos_lin ;
      Eigen::MatrixXd _comVel ;
      
      //COM+joints vectors
      Eigen::VectorXd _qc;
      Eigen::MatrixXd _vc;
      Eigen::VectorXd _d_vc;//
      
      //Base+joints vectors
      Eigen::VectorXd _qb;
      Eigen::MatrixXd _vb;
      Eigen::VectorXd _d_vb;//

      //Jacobian
      Eigen::MatrixXd _Jlegs ;
      Eigen::MatrixXd _Jlegs_dag ;
      Eigen::MatrixXd _Jlegs_lin ; //_jlegs_dag lin 12X18

      Eigen::MatrixXd _Jcom ; // non so a cosa serve CenterOfMassJacobian

      //M Matrix
      Eigen::MatrixXd _MassMatrix; //FloatingMassMatrix
      Eigen::MatrixXd _MassMatrixCOM;

      //Transformation matrix
      Eigen::MatrixXd _T; 
      Eigen::MatrixXd _T_inv_dot;
      
      // J_dot * q_dot
      Eigen::MatrixXd _Jd_qd; 
      Eigen::MatrixXd _Jd_qd_dag; //In Com representation
      Eigen::MatrixXd _Jd_qd_lin; //_Jd_qd_dag lin 12X1

      //Coriolis + gravitational terms (Bias)
      Eigen::VectorXd _h;
      Eigen::VectorXd _g;
      
      Eigen::MatrixXd _C_dag;
      Eigen::MatrixXd _BiasCOM;
      
      double index_vel=0;
      
      

 	Eigen::MatrixXd _Rot_Matrix;
 	
 	double _ang_prev =0.0;
 	bool _ang_prev_negative = false;
 	bool _ang_prev_positive = false;

     


      


      

      

    
  

float _a =2; //Per debug


};

#endif 
