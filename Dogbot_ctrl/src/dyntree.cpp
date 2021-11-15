#include "dogbot_ctrl/dyntree.h"


DYNTREE::DYNTREE(std::string modelFile){
   




    bool ok = _mdlLoader.loadModelFromFile(modelFile);
    
    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << _mdlLoader.model().toString() << std::endl; //Print of all model
    }
    
    ok = _kinDynComp.loadRobotModel(_mdlLoader.model());

    if( !ok )
    {
        cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << endl
                  << _mdlLoader.model().toString() << std::endl;
       // return EXIT_FAILURE;
    }   


    _model = _kinDynComp.model();

    _kinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    _nDOFs = _model.getNrOfDOFs();

    _dogbot_mass = _model.getTotalMass();

    


    //Joints variables
    _jointPos = iDynTree::VectorDynSize(_nDOFs);
    _baseVel = iDynTree::Twist();
    _jointVel = iDynTree::VectorDynSize(_nDOFs);


    //Com variables
    _comPos = Eigen::MatrixXd (6,1);
    _comPos_lin = Eigen::MatrixXd (3,1);
    _comVel =Eigen::MatrixXd (6,1);

    //Base+Joints vector
    _qb.resize(6+_nDOFs);
    _vb =Eigen::MatrixXd (18,1);

    //COM+joints variables
    _qc.resize(6+_nDOFs);
    _vc=Eigen::MatrixXd (18,1);


    //Transformation matrix
    _T =  Eigen::MatrixXd (6+_nDOFs, 6+_nDOFs);
    _T_inv_dot= Eigen::MatrixXd (6+_nDOFs, 6+_nDOFs);

    // J_dot * q_dot
    _Jd_qd =  Eigen::MatrixXd (6*4,1);
    _Jd_qd_dag = Eigen::MatrixXd (6*4,1);//In Com representation
    _Jd_qd_lin =Eigen::MatrixXd (12,1); //_Jd_qd_dag lin 12X1

   //Jacobian
   _Jlegs = Eigen::MatrixXd::Zero(6*4,6+_nDOFs);
   _Jlegs_dag = Eigen::MatrixXd (6*4,6+_nDOFs);
   _Jlegs_lin = Eigen::MatrixXd (12,6+_nDOFs); //_jlegs_dag lin 12X18

    //M Matrix
    _MassMatrix = Eigen::MatrixXd (6+_nDOFs, 6+_nDOFs);
    _MassMatrixCOM = Eigen::MatrixXd  (6+_nDOFs, 6+_nDOFs);

    //Bias coriolis + gravitational terms

    _h = Eigen::VectorXd (6+_nDOFs);
    _g = Eigen::VectorXd (6+_nDOFs);
    _C = Eigen::MatrixXd(6+_nDOFs,6+_nDOFs) ;
    _C_dag = Eigen::MatrixXd(6+_nDOFs,6+_nDOFs) ;
    

    _Jcom = Eigen::MatrixXd (3,6+_nDOFs);


	_BiasCOM = Eigen::MatrixXd (6+_nDOFs,1);



    }

void DYNTREE::stampaDEB(){


    cout<<_T<<endl;
   /*
    cout<<"ecco:com "<<endl;
    cout<<_comVel<<endl;
    cout<<"ecco2: base"<<endl;
    cout<<_vb.block(0,0,6,1)<<endl;
*/


}

 Eigen::MatrixXd DYNTREE::computeDYN(){

    iDynTree::FreeFloatingGeneralizedTorques invDynTrqs(_model);
    iDynTree::LinkNetExternalWrenches extForces(_model);
    extForces.zero();
     
     Eigen::VectorXd baseAcc =Eigen::VectorXd::Zero(6);

     Eigen::VectorXd jointAcc =Eigen::VectorXd::Zero(12);


    _kinDynComp.inverseDynamics(iDynTree::make_span(baseAcc),
                               iDynTree::make_span(jointAcc),
                               extForces,
                               invDynTrqs);

    Eigen::Matrix<double,18,1> torques ;
    torques << iDynTree::toEigen(invDynTrqs.baseWrench()),iDynTree::toEigen(invDynTrqs.jointTorques());

    return torques;
 }




void DYNTREE::dogbot_idyntree(Eigen::Matrix4d &World_H_base,Eigen::Matrix<double,12,1> &JointPos, Eigen::Matrix<double,12,1> &JointVel ,Eigen::Matrix<double,6,1> &BaseVel,Eigen::Vector3d &Gravity ){



    fromEigen(_world_H_base,World_H_base);
    fromEigen(_baseVel,BaseVel);
    toEigen(_jointPos) = JointPos;
    toEigen(_jointVel) = JointVel;
    toEigen(_gravity)  = Gravity;


    Eigen::Vector3d worldeigen=toEigen(_world_H_base.getPosition());

	
    while (worldeigen==Eigen::Vector3d::Zero()){
        iDynTree::fromEigen(_world_H_base,World_H_base);
        worldeigen=toEigen(_world_H_base.getPosition());
    }

    Eigen::Vector3d worldeigen2=toEigen(_kinDynComp.getCenterOfMassPosition());

    while (worldeigen2==Eigen::Vector3d::Zero()){
        
        worldeigen2=toEigen(_kinDynComp.getCenterOfMassPosition());
    }

    if(index_vel>=3){
      Eigen::Vector3d  worldeigen3=toEigen(_kinDynComp.getCenterOfMassVelocity());

        while (worldeigen3==Eigen::Vector3d::Zero()){
            
            worldeigen3=toEigen(_kinDynComp.getCenterOfMassVelocity());
        }


    }

    index_vel++;

    

    _kinDynComp.setRobotState(_world_H_base,_jointPos,
                            _baseVel,_jointVel,_gravity);


//Compute com vector

    _Rot_Matrix = World_H_base.block(0,0,3,3);

    if(_ang_prev < 0.0 && std::abs(_ang_prev)>3.0){
        _ang_prev_negative = true;
    }

    if(_ang_prev > 0.0 && std::abs(_ang_prev)>3.0){
        _ang_prev_positive = true;
    }

    Eigen::Vector3d com_pos = iDynTree::toEigen(_kinDynComp.getCenterOfMassPosition()); //3x1
    iDynTree::Vector3 com_orient=_world_H_base.getRotation().asRPY(); //3x1


    _ang_prev = toEigen(com_orient)[2];

    if( toEigen(com_orient)[2] > 0.0 && _ang_prev_negative){
        
        toEigen(com_orient)[2] = toEigen(com_orient)[2] - 2* 3.14159;
    }

    if( toEigen(com_orient)[2] < 0.0 && _ang_prev_positive){
        
        toEigen(com_orient)[2] = toEigen(com_orient)[2] + 2* 3.14159;
    }


    _comPos << com_pos, toEigen(com_orient); //6x1

//Compute com_vel vector

    Eigen::Vector3d com_vel = iDynTree::toEigen(_kinDynComp.getCenterOfMassVelocity());
    Eigen::Vector3d com_vel_ang =  BaseVel.block(3,0,3,1);


    _comVel << com_vel, com_vel_ang; //6x1

//Compute Base+Joints vectors
    _qb<< iDynTree::toEigen(_world_H_base.getPosition()), iDynTree::toEigen(com_orient),JointPos; //Base orientation= Com orientation
    _vb.block(0,0,6,1) = BaseVel;
    _vb.block(6,0,12,1)= JointVel;


//Compute COM+joints vectors

    _qc<< _comPos,JointPos; //(6+12)x1
    _vc.block(0,0,6,1) = _comVel;//(6+12)x1
    _vc.block(6,0,12,1)= JointVel;


_kinDynComp.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(_MassMatrix));


//// USEFUL MATRICES ///

    computeT(); //To pass to CoM representation

//Compute M in CoM representation

    _MassMatrixCOM = _T.transpose().inverse()*_MassMatrix*_T.inverse();
    _MassMatrixCOM(0,0) = _MassMatrixCOM(0,0) - _MassMatrixCOM(0,0)*0.1;
    _MassMatrixCOM(1,1) = _MassMatrixCOM(1,1) - _MassMatrixCOM(1,1)*0.1;
    _MassMatrixCOM(2,2) = _MassMatrixCOM(2,2) - _MassMatrixCOM(2,2)*0.1;


//Compute J

    computeJ();//J_legs
    _Jlegs_dag= _Jlegs*_T.inverse(); // In CoM representation


//Compute J lin, 12X18 matrix
   Eigen::Matrix<double,12,24> B;
    B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

    _Jlegs_lin= B*_Jlegs_dag;  


//Compute Jdot*q_dot

    computeJdot_qdot();
    _Jd_qd_dag =_Jd_qd + _Jlegs * _T_inv_dot * _vc; //In Com representation

//Compute Jdot*q_dot lin,  12X1

    _Jd_qd_lin = B * _Jd_qd_dag;

//Compute Bias

    iDynTree::FreeFloatingGeneralizedTorques bias_force(_model);
    _kinDynComp.generalizedBiasForces(bias_force);

    _h << iDynTree::toEigen(bias_force.baseWrench()),
          iDynTree::toEigen(bias_force.jointTorques());

    iDynTree::FreeFloatingGeneralizedTorques gravity_force(_model); 
    _kinDynComp.generalizedGravityForces(gravity_force);
    
    _g << iDynTree::toEigen(gravity_force.baseWrench()),
          iDynTree::toEigen(gravity_force.jointTorques());
    
    Eigen::Matrix<double,1,18> _v_pinv;
    
    _v_pinv = _vb.completeOrthogonalDecomposition().pseudoInverse();


    _C =( _h - _g ) * _v_pinv;
    
   // _C_dag=_T.transpose().inverse()*_C*_T.inverse()+_T.transpose().inverse()*_MassMatrix*_T_inv_dot; //Bias in CoM representation
  
      Eigen::Matrix<double,18,1> bias ;
    bias << iDynTree::toEigen(bias_force.baseWrench()),
          iDynTree::toEigen(bias_force.jointTorques());

    _BiasCOM = _T.transpose().inverse()*bias + _T.transpose().inverse()*_MassMatrix*_T_inv_dot*_vc;

}




void DYNTREE::computeT(){


    Eigen::Vector3d com_pos = toEigen(_kinDynComp.getCenterOfMassPosition());
    Eigen::Vector3d base_pos =toEigen(_world_H_base.getPosition());

    Eigen::Vector3d P_bc = com_pos - base_pos;

    Eigen::MatrixXd S_Pbc(3,3);
    S_Pbc << 0, -P_bc[2], P_bc[1],
            P_bc[2], 0, -P_bc[0],
            -P_bc[1], P_bc[0], 0; 

    Eigen::MatrixXd X(6,6);
    X << Eigen::MatrixXd::Identity(3,3), S_Pbc.transpose(),
        Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);


    Eigen::MatrixXd M_11(6,6);
    M_11= _MassMatrix.block(0,0,6,6);

    Eigen::MatrixXd M_12(6,12);
    M_12= _MassMatrix.block(0,6,6,12);

    Eigen::MatrixXd J_s(6,12);
    J_s= X*(M_11.inverse()*M_12);
    // Eigen::MatrixXd Mb_Mj = M_11.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(M_12);
    

    _T.block(0,0,6,6)= X;
    _T.block(0,6,6,12) = J_s;
    _T.block(6,0,12,6)=   Eigen::MatrixXd::Zero(12, 6);
    _T.block(6,6,12,12)=   Eigen::MatrixXd::Identity(12, 12);   
    
    //Compute time derivative of T^-1 

    Eigen::Vector3d com_vel = toEigen(_kinDynComp.getCenterOfMassVelocity());
    Eigen::Vector3d base_vel =toEigen(_baseVel.getLinearVec3());

    Eigen::Vector3d p_bc_dot = com_vel- base_vel;

    Eigen::MatrixXd S_pbc_dot(3,3);
    S_pbc_dot << 0, -p_bc_dot[2], p_bc_dot[1],
            p_bc_dot[2], 0, -p_bc_dot[0],
            -p_bc_dot[1], p_bc_dot[0], 0; 

    Eigen::MatrixXd X_dot = Eigen::MatrixXd::Zero(6,6); 
    X_dot.block(0,3,3,3) = S_pbc_dot;


    Eigen::VectorXd  mdr=(_dogbot_mass- _dogbot_mass*0.1) * p_bc_dot; //

    Eigen::MatrixXd S_mdr(3,3);
    S_mdr << 0.0, -mdr[2], mdr[1],
            mdr[2], 0.0, -mdr[0],
            -mdr[1], mdr[0], 0.0; 

    Eigen::Matrix<double,6,6> Mb_dot = Eigen::MatrixXd::Zero(6,6); 
    Mb_dot.block(0,3,3,3) = S_mdr.transpose();
    Mb_dot.block(3,3,3,3) = S_mdr;

    Eigen::Matrix<double,6,6> Mb1_dot;
    Mb1_dot = (M_11.transpose().inverse() * Mb_dot.transpose()).transpose();
   // Mb1_dot = M_11.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Mb_dot.transpose()).transpose();

    Eigen::Matrix<double,6,6> Mb2_dot = -(M_11.inverse() * Mb1_dot);
   // Mb2_dot = - (M_11.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( Mb1_dot));



    Eigen::Matrix<double,6,12> J_s_dot;
    J_s_dot = X_dot * (M_11.inverse()*M_12) + X * Mb2_dot * M_12;
    //J_s_dot = X_dot * Mb_Mj + X * Mb2_dot * M_12;

    Eigen::Matrix<double,3,12> J_s1_dot = J_s_dot.block(0,0,3,12);

    _T_inv_dot = Eigen::MatrixXd::Zero(18,18);
    _T_inv_dot.block(0,3,3,3)= S_pbc_dot;
    _T_inv_dot.block(0,6,3,12) = -J_s1_dot;


}

void DYNTREE::computeJ(){

 
    string back_left = "back_left_foot";
    string back_right = "back_right_foot";
    string front_left = "front_left_foot";
    string front_right = "front_right_foot";


    iDynTree::FrameIndex FrameIndex_bl = _model.getFrameIndex(back_left);
    iDynTree::FrameIndex FrameIndex_br = _model.getFrameIndex(back_right);
    iDynTree::FrameIndex FrameIndex_fl = _model.getFrameIndex(front_left);
    iDynTree::FrameIndex FrameIndex_fr = _model.getFrameIndex(front_right);

    iDynTree::MatrixDynSize J1(6,6+_nDOFs);
    iDynTree::MatrixDynSize J2(6,6+_nDOFs);
    iDynTree::MatrixDynSize J3(6,6+_nDOFs);
    iDynTree::MatrixDynSize J4(6,6+_nDOFs);


    _kinDynComp.getFrameFreeFloatingJacobian(FrameIndex_bl, J1);
    _kinDynComp.getFrameFreeFloatingJacobian(FrameIndex_br, J2);
    _kinDynComp.getFrameFreeFloatingJacobian(FrameIndex_fl, J3);
    _kinDynComp.getFrameFreeFloatingJacobian(FrameIndex_fr, J4);



    _Jlegs<< toEigen(J1), toEigen(J2), toEigen(J3), toEigen(J4);  

}




void DYNTREE::computeJdot_qdot(){

    string back_left = "back_left_foot";
    string back_right = "back_right_foot";
    string front_left = "front_left_foot";
    string front_right = "front_right_foot";


    iDynTree::FrameIndex FrameIndex_bl = _model.getFrameIndex(back_left);
    iDynTree::FrameIndex FrameIndex_br = _model.getFrameIndex(back_right);
    iDynTree::FrameIndex FrameIndex_fl = _model.getFrameIndex(front_left);
    iDynTree::FrameIndex FrameIndex_fr = _model.getFrameIndex(front_right);

    iDynTree::Vector6 Bias1=_kinDynComp.getFrameBiasAcc(FrameIndex_bl);
    iDynTree::Vector6 Bias2=_kinDynComp.getFrameBiasAcc(FrameIndex_br);
    iDynTree::Vector6 Bias3=_kinDynComp.getFrameBiasAcc(FrameIndex_fl);
    iDynTree::Vector6 Bias4=_kinDynComp.getFrameBiasAcc(FrameIndex_fr);

    _Jd_qd<< toEigen(Bias1), toEigen(Bias2), toEigen(Bias3), toEigen(Bias4);



}  

Eigen::Matrix<double,3,3> DYNTREE::getBLworldtransform()
{    
	
	iDynTree::Transform  World_br;
    World_br=_kinDynComp.getWorldTransform(8);
    return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> DYNTREE::getBRworldtransform()
{   iDynTree::Transform  World_br;
    World_br=_kinDynComp.getWorldTransform(11);

   return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> DYNTREE::getFLworldtransform()
{   iDynTree::Transform  World_br;
    World_br=_kinDynComp.getWorldTransform(14);

 return toEigen(World_br.getRotation());
}

Eigen::Matrix<double,3,3> DYNTREE::getFRworldtransform()
{   iDynTree::Transform  World_br;
    World_br=_kinDynComp.getWorldTransform(17);

   return toEigen(World_br.getRotation());
}







Eigen::MatrixXd DYNTREE::getCOM_poslin(){

    _comPos_lin = iDynTree::toEigen(_kinDynComp.getCenterOfMassPosition());

    return _comPos_lin;
}


Eigen::MatrixXd DYNTREE::getCOM_pos(){

    return _comPos;
}


Eigen::MatrixXd DYNTREE::getCOM_vel(){

    return _comVel;
}

Eigen::MatrixXd DYNTREE::getMDAG(){

    return _MassMatrixCOM;

}

Eigen::MatrixXd DYNTREE::getMassMatrix(){

    return _MassMatrix;

}


Eigen::MatrixXd DYNTREE::getBiasCOM(){

    return _BiasCOM ;

}

Eigen::MatrixXd DYNTREE::getMassMatrixCOM_com()
{

    return _MassMatrixCOM.block(0,0,6,6);

}

double DYNTREE::getMass(){
    
    return _dogbot_mass;
}

Eigen::MatrixXd DYNTREE::getJlegs(){

return _Jlegs; }

Eigen::MatrixXd DYNTREE::getJlegslin(){

    return _Jlegs_lin; 
}

Eigen::MatrixXd DYNTREE::getJdotqd_lin(){

    return _Jd_qd_lin;

}

Eigen::MatrixXd DYNTREE::getCCoM(){
	
	return _C_dag;

}

Eigen::MatrixXd DYNTREE::getmatrixC(){
	
	return _C; 

}


Eigen::MatrixXd DYNTREE::get_vc(){
	
	return _vc; 

}

Eigen::MatrixXd DYNTREE::get_vb(){
	
	return _vb; 

}
Eigen::VectorXd DYNTREE::get_qb(){
    
    return _qb;

}


Eigen::MatrixXd DYNTREE::getJfoot(){

    string back_left = "back_left_foot";
    iDynTree::FrameIndex FrameIndex_ = _model.getFrameIndex(back_left);
    iDynTree::MatrixDynSize J(6,6+_nDOFs);


    _kinDynComp.getFrameFreeFloatingJacobian(8, J);

    return iDynTree::toEigen(J);
 }

    //Foot position
Eigen::MatrixXd DYNTREE::getBLpos(){

    iDynTree::Transform  W_BL;
    W_BL= _kinDynComp.getWorldTransform(8);
    
    return toEigen(W_BL.getPosition());
}

Eigen::MatrixXd DYNTREE::getBRpos(){

    iDynTree::Transform  W_BR;
    W_BR= _kinDynComp.getWorldTransform(11);
    
    return toEigen(W_BR.getPosition());

}

Eigen::MatrixXd DYNTREE::getFLpos(){

    iDynTree::Transform  W_FL;
    W_FL= _kinDynComp.getWorldTransform(14);
    
    return toEigen(W_FL.getPosition());

}

Eigen::MatrixXd DYNTREE::getFRpos(){

    iDynTree::Transform  W_FR;
    W_FR= _kinDynComp.getWorldTransform(17);
    
    return toEigen(W_FR.getPosition());

}

//Foot velocity
Eigen::MatrixXd DYNTREE::getBLvel(){

    iDynTree::Twist BLvel;
    BLvel=_kinDynComp.getFrameVel(8);

    return toEigen(BLvel.getLinearVec3());
}

Eigen::MatrixXd DYNTREE::getBRvel(){

    iDynTree::Twist BRvel;
    BRvel=_kinDynComp.getFrameVel(11);

    return toEigen(BRvel.getLinearVec3());

}

Eigen::MatrixXd DYNTREE::getFLvel(){

    iDynTree::Twist FLvel;
    FLvel=_kinDynComp.getFrameVel(14);

    return toEigen(FLvel.getLinearVec3());

}

Eigen::MatrixXd DYNTREE::getFRvel(){

    iDynTree::Twist FRvel;
    FRvel=_kinDynComp.getFrameVel(17);

    return toEigen(FRvel.getLinearVec3());

}

Eigen::Matrix<double,3,3> DYNTREE::get_rotmatrix(){


    return _Rot_Matrix;

}
