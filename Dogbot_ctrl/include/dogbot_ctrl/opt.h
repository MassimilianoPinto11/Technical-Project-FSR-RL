#ifndef OPT_H
#define OPT_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "alglib/optimization.h"

#include "dogbot_ctrl/dyntree.h"
#include <Eigen/Core>




using namespace alglib;

class CONTROL{
    public:

        CONTROL();
        CONTROL(DYNTREE &dog);
        Eigen::MatrixXd getTau_stand(iDynTree::Vector6 &rc_ref, iDynTree::Vector6 &d_rc_ref, iDynTree::Vector6 &dd_rc_ref, Eigen::Matrix<double,12,1> &fext);
        Eigen::MatrixXd getTau_swing(int swinglegs, iDynTree::Vector6 &rc_ref, iDynTree::Vector6 &d_rc_ref, iDynTree::Vector6 &dd_rc_ref,Eigen::Matrix<double,6,1> &dd_xsw_ref,Eigen::Matrix<double,6,1> &d_xsw_ref, Eigen::Matrix<double,6,1> &xsw_ref, Eigen::Matrix<double,12,1> &fext);
       
        Eigen::MatrixXd getWcom_des();
        Eigen::MatrixXd getfgr();
   
        Eigen::MatrixXd getprova();
        Eigen::MatrixXd getprova2();
        Eigen::MatrixXd getprova3();
        Eigen::MatrixXd getprova4();
        Eigen::MatrixXd getprova5();
        Eigen::MatrixXd getelin();


        

        
    private:
    
        DYNTREE*  _dogbot;
        iDynTree::Vector6 _rc_ref ;
        iDynTree::Vector6 _d_rc_ref ;
        iDynTree::Vector6 _dd_rc_ref ;
        float a=6;
        Eigen::MatrixXd _fgr;
        Eigen::MatrixXd _vec;
        Eigen::MatrixXd _vec2;
        Eigen::MatrixXd _vec3;
        Eigen::MatrixXd _vec4;
        Eigen::MatrixXd _vec5;
        Eigen::MatrixXd _elin;
        Eigen::MatrixXd _tau_star;
        
        Eigen::Matrix<double,12,1> _qmin,_qmax;

        Eigen::Matrix<double,16,12> Dfr ;
        Eigen::Matrix<double,4,3> Di ;
        Eigen::Matrix<double,5,3> Di_sw ;
        Eigen::Matrix<double,10,6> Dfr_sw ;
        
        

        
};

#endif
