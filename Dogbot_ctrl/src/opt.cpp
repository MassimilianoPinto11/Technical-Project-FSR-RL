
#include "dogbot_ctrl/opt.h"
using namespace alglib;


CONTROL::CONTROL(){

}

CONTROL::CONTROL(DYNTREE &dog)

{
    _dogbot= &dog;
    _fgr= Eigen::MatrixXd::Zero(12,1);
    _tau_star = Eigen::MatrixXd::Zero(12,1);

    _qmin<< -1.75 , -1.75,-1.75,-1.75,-1.58, -2.62, -3.15, -0.02,  -1.58, -2.62, -3.15, -0.02;
    _qmax<< 1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62,  3.15, 0.02, 1.58, 2.62;

    Dfr =Eigen::Matrix<double,16,12>::Zero();
  
    Di =Eigen::Matrix<double,4,3>::Zero(); 

    double mu=0.4; 
    Eigen::Vector3d n_hat = Eigen::Vector3d::Zero();
    n_hat[2]=1;
    Eigen::Vector3d t1_hat = Eigen::Vector3d::Zero();
    t1_hat[0]=1;
    Eigen::Vector3d t2_hat = Eigen::Vector3d::Zero();
    t2_hat[1]=1;

    Di << (t1_hat -mu*n_hat).transpose(),
          (-t1_hat -mu*n_hat).transpose(),
          (t2_hat -mu*n_hat).transpose(), 
          (-t2_hat -mu*n_hat).transpose();
          
        

    for(int i=0; i<4; i++){

      Dfr.block(0+4*i,0+3*i,4,3)= Di; 
    }

    Di_sw =Eigen::Matrix<double,5,3>::Zero();

        Di << (t1_hat -mu*n_hat).transpose(),
          (-t1_hat -mu*n_hat).transpose(),
          (t2_hat -mu*n_hat).transpose(), 
          (-t2_hat -mu*n_hat).transpose(),
          -n_hat.transpose();

    Dfr_sw =Eigen::Matrix<double,10,6>::Zero();
  

    for(int i=0; i<2; i++){

      Dfr_sw.block(0+5*i,0+3*i,4,3)= Di; 

    }

    

}



Eigen::MatrixXd CONTROL::getWcom_des(){
  

  Eigen::MatrixXd Kp_w, Kd_w;
  Kp_w=1000*Eigen::MatrixXd::Identity(6,6); //DA aggiustare 
  Kd_w=80*Eigen::MatrixXd::Identity(6,6); //Da aggiustare 
  
  
  Eigen::Matrix<double,6,1> e ;
  Eigen::Matrix<double,3,1> e_lin= toEigen(_rc_ref).block(0,0,3,1)- _dogbot->getCOM_pos().block(0,0,3,1);
  Eigen::Matrix<double,3,3> rot =  _dogbot->get_rotmatrix();
  Eigen::Matrix<double,3,1> e_ang = rot*(toEigen(_rc_ref).block(3,0,3,1)- _dogbot->getCOM_pos().block(3,0,3,1));
  
  e << e_lin, e_ang;
  _elin = e_lin;

  Eigen::Matrix<double,6,1> d_e= toEigen(_d_rc_ref)- _dogbot->getCOM_vel();
  

  double mass_dogbot = _dogbot->getMass();
  mass_dogbot = mass_dogbot; //- mass_dogbot*0.1;



  Eigen::MatrixXd g = Eigen::MatrixXd::Zero(6,1);
  g(2,0)=9.8;

  Eigen::MatrixXd M_com = _dogbot->getMassMatrixCOM_com();

  Eigen::Matrix<double,6,1> Wcom_des;
  Wcom_des= Kp_w*e + Kd_w*d_e +  mass_dogbot*g + M_com*toEigen(_dd_rc_ref);  //  //



  return Wcom_des;
}

Eigen::MatrixXd CONTROL::getTau_stand(iDynTree::Vector6 &rc_ref, iDynTree::Vector6 &d_rc_ref, iDynTree::Vector6 &dd_rc_ref, Eigen::Matrix<double,12,1> &fgr_sensor){
  
  _rc_ref= rc_ref;
  _d_rc_ref =d_rc_ref;
  _dd_rc_ref= dd_rc_ref ;

  Eigen::Matrix<double,6,1> Wcom_des_stand = getWcom_des();

 _vec = Wcom_des_stand;
  real_2d_array Q, L; 
  real_1d_array c; 
  integer_1d_array Lt;
  real_1d_array x;
  minqpreport rep;
  minqpstate state;

  int n=30;
  Q.setlength(n,n);
  c.setlength(n);
  L.setlength(82,n+1);
  Lt.setlength(82);

//Compute Q and c
  Eigen::Matrix<double, 12, 18> Jst= _dogbot->getJlegslin();
  
    
  Eigen::Matrix<double,12,6> Jst_com= Jst.block(0,0,12,6);
  Eigen::Matrix<double, 12, 12> Jst_j= Jst.block(0,6,12,12);

  Eigen::Matrix<double,12,30> Sigma= Eigen::Matrix<double,12,30>::Zero();
  Sigma.block(0,18,12,12) = Eigen::Matrix<double,12,12>::Identity();

  Eigen::Matrix<double,6,30>  T = Jst_com.transpose()*Sigma;

  Eigen::Matrix<double,6,6> W =50*Eigen::Matrix<double,6,6>::Identity(); //DA SETTARE
  Eigen::Matrix<double,30,30> eigQ= Eigen::Matrix<double,30,30>::Zero();
  eigQ=T.transpose()* W * T;
 
  

  Eigen::Matrix<double,6,1> G =  Wcom_des_stand; //- Jst_com.transpose() * _fext ;
 
  Eigen::Matrix<double,30,1> eigc=Eigen::Matrix<double,30,1>::Zero();
  eigc= -T.transpose()* W.transpose() *G; 

  Eigen::Matrix<double,30,30> R= Eigen::Matrix<double,30,30>::Identity(); 

  eigQ=eigQ+R;
 

  for ( int i = 0; i < 30; i++ ){
      for ( int j = 0; j < 30; j++ ){
           Q(i,j) = eigQ(i,j); }   
          
  } 

  for ( int i = 0; i < 30; i++ ){
    
          c(i) = eigc(i,0); }
        
  


//QP optimizer

  minqpcreate(n,state);
	minqpsetquadraticterm(state, Q);
  minqpsetlinearterm(state,c);

//Equality constraints




  //Matrix A
  Eigen::Matrix<double,18, 30> A= Eigen::Matrix<double,18,30>::Zero(); 
  Eigen::Matrix<double,6,6> MassMatrixCom = _dogbot->getMassMatrixCOM_com();
  A.block(0,0,6,6)= MassMatrixCom;
  A.block(0,18,6,12)= -Jst_com.transpose(); 
  A.block(6,0,12,6)= Jst_com;
  A.block(6,6,12,12)= Jst_j;

  for(int i = 0 ; i <18 ;i++){
    for(int j=0;j<30;j++){
      L(i,j)= A(i,j);
  }
  }
  
  //Vector b
  Eigen::Matrix<double,18, 1> b= Eigen::Matrix<double,18,1>::Zero();
  Eigen::Matrix<double,6,1> gravity;
	gravity<<0,0,9.8,0,0,0; 
  double mass= MassMatrixCom(0,0);
  Eigen::Matrix<double,18,1> Bias = _dogbot->getBiasCOM();
  b.block(0,0,6,1)=-Bias.block(0,0,6,1);
  b.block(6,0,12,1)=-(_dogbot->getJdotqd_lin());

   for(int i = 0 ; i <18 ;i++){
    
      L(i,30)= b(i,0);
  }
  
  

///Inequality Constraints

 //Matrix D
  Eigen::Matrix<double,64,30> D = Eigen::Matrix<double,64,30>::Zero(); 

  D.block(0,18,16,12)= Dfr;

  Eigen::Matrix<double,12,12> M_22=_dogbot->getMDAG().block(6,6,12,12);
  D.block(16,6,12,12)= M_22;
  D.block(16,18,12,12)= -Jst_j.transpose();
  D.block(28,6,12,12)= -M_22;
  D.block(28,18,12,12)= Jst_j.transpose();
  D.block(40,6,12,12)= Eigen::Matrix<double,12,12>::Identity();
  D.block(52,6,12,12)= -Eigen::Matrix<double,12,12>::Identity();


  for(int i = 0 ; i <64 ;i++){
    for(int j=0;j<30;j++){
      L(i+18,j)= D(i,j);
  }
  }


  ///Vector c
  Eigen::Matrix<double,64, 1> ci= Eigen::Matrix<double,64,1>::Zero(); 


  Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
  Eigen::Matrix<double,12,1> tau_min= - tau_max;

  ci.block(16,0,12,1)= tau_max - Bias.block(6,0,12,1);
  ci.block(28,0,12,1)= -tau_min + Bias.block(6,0,12,1);

  double deltat=0.01;
  Eigen::Matrix<double,12, 1> eigenq=_dogbot->get_qb().block(6,0,12,1);
  Eigen::Matrix<double,12, 1> eigendq=_dogbot->get_vb().block(6,0,12,1);
  Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(_qmin-eigenq-deltat*eigendq);
  Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(_qmax-eigenq-deltat*eigendq);


  ci.block(40,0,12,1)=ddqmax;
  ci.block(52,0,12,1)=-ddqmin;



  for(int i = 0 ; i <64 ;i++){
    
      L(i+18,30)= ci(i,0);
  }
  

  for(int i =0;i<18;i++){
    Lt(i)=0.0;
  }

   for(int i =18;i<82;i++){
    Lt(i)=-1.0;
  }

  
  minqpsetlc(state, L, Lt);
  minqpsetscaleautodiag(state);
  minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5);
  minqpoptimize(state);
  minqpresults(state, x, rep);

  Eigen::VectorXd x_eig= Eigen::VectorXd::Zero(30);
  	for ( int i = 0; i < x_eig.size(); i++ )
             x_eig(i)=x(i);

  Eigen::MatrixXd dd_q_opt= x_eig.block(6,0,12,1);
  Eigen::MatrixXd fgr_opt= x_eig.block(18,0,12,1);

  _fgr= Eigen::MatrixXd::Zero(12,1);
  _fgr = fgr_opt;
  _vec2 = dd_q_opt;
  _vec3 = x_eig.block(0,0,6,1);
 
  Eigen::Matrix<double,12,1> tau= Eigen::MatrixXd::Zero(12,1);
  tau= M_22*dd_q_opt- Jst_j.transpose()*fgr_opt + Bias.block(6,0,12,1);
  _tau_star = tau;
  return tau;
  
}


Eigen::MatrixXd CONTROL::getTau_swing(int swinglegs, iDynTree::Vector6 &rc_ref, iDynTree::Vector6 &d_rc_ref, iDynTree::Vector6 &dd_rc_ref, Eigen::Matrix<double,6,1> &dd_xsw_ref, Eigen::Matrix<double,6,1> &d_xsw_ref,Eigen::Matrix<double,6,1> &xsw_ref, Eigen::Matrix<double,12,1> &fgr_sensor){

  int sw_leg1;
  int sw_leg2;
  int st_leg1;
  int st_leg2;

  _rc_ref= rc_ref;
  _d_rc_ref =d_rc_ref;
  _dd_rc_ref= dd_rc_ref ;

  Eigen::Matrix<double,6,1> Wcom_des_swing = getWcom_des();
   _vec = Wcom_des_swing;

  if(swinglegs == 0){
    sw_leg1 = 3;//BR
    sw_leg2 = 6;//FL 
    st_leg1 = 0;
    st_leg2 = 9;
  }
  else{
    sw_leg1 = 0;//BL
    sw_leg2 = 9;//FR
    st_leg1 = 3;
    st_leg2 = 6;
  }
  

  real_2d_array Q, L; 
  real_1d_array c; 
  integer_1d_array Lt;
  real_1d_array x_sw;
  minqpreport rep;
  minqpstate state_sw;

  int n=30;
  Q.setlength(n,n);
  c.setlength(n);
  L.setlength(82,n+1);
  Lt.setlength(82);


//Compute Q and c
  Eigen::Matrix<double, 12, 18> J= _dogbot->getJlegslin();
  Eigen::Matrix<double,6,6> Jst_com = Eigen::Matrix<double,6,6>::Zero();
  Jst_com.block(0,0,3,6)= J.block(st_leg1,0,3,6);
  Jst_com.block(3,0,3,6)= J.block(st_leg2,0,3,6);
  
  Eigen::Matrix<double, 6, 12> Jst_j = Eigen::Matrix<double,6,12>::Zero();
  Jst_j.block(0,0,3,12) = J.block(st_leg1,6,3,12);
  Jst_j.block(3,0,3,12) = J.block(st_leg2,6,3,12);

  Eigen::Matrix<double, 6, 18> Jst;
  Jst << Jst_com, Jst_j;


  Eigen::Matrix<double, 6, 6> Jsw_com= Eigen::Matrix<double,6,6>::Zero();
  Jsw_com.block(0,0,3,6) = J.block(sw_leg1,0,3,6);
  Jsw_com.block(3,0,3,6) = J.block(sw_leg2,0,3,6);

  Eigen::Matrix<double, 6, 12> Jsw_j=  Eigen::Matrix<double,6,12>::Zero();
  Jsw_j.block(0,0,3,12) = J.block(sw_leg1,6,3,12);
  Jsw_j.block(3,0,3,12) = J.block(sw_leg2,6,3,12);

  Eigen::Matrix<double,6,30> Sigma= Eigen::Matrix<double,6,30>::Zero();
  Sigma.block(0,18,6,6) = Eigen::Matrix<double,6,6>::Identity();

  Eigen::Matrix<double,6,30>  T = Jst_com.transpose()*Sigma;

  Eigen::Matrix<double,6,6> W = 50* Eigen::Matrix<double,6,6>::Identity(); 
  Eigen::Matrix<double,30,30> eigQ= Eigen::Matrix<double,30,30>::Zero(); 
  eigQ= T.transpose()* W * T;


  Eigen::Matrix<double,30,30> R= Eigen::Matrix<double,30,30>::Identity(); 
  R.block(24,24,6,6)=8*Eigen::Matrix<double,6,6>::Identity(); 
  
  eigQ=(eigQ+R);

  

  Eigen::Matrix<double,6,1> G =  Wcom_des_swing; //- J.block(0,0,12,6).transpose() * _fext ;

  Eigen::Matrix<double,30,1> eigc=  Eigen::Matrix<double,30,1>::Zero();
  eigc= -T.transpose()* W.transpose() * G; 
 
   for ( int i = 0; i < eigQ.rows(); i++ ){
        for ( int j = 0; j < eigQ.cols(); j++ ){
             Q(i,j) = eigQ(i,j);
    } }
   
    for ( int i = 0; i < eigc.rows(); i++ ){
       for ( int j = 0; j < eigc.cols(); j++ ){
             c(i) = eigc(i,j);
    }}

  minqpcreate(n,state_sw);
	minqpsetquadraticterm(state_sw, Q);
  minqpsetlinearterm(state_sw,c);

  //Equality constraints

  //Matrix A
  Eigen::Matrix<double,12, 30> A= Eigen::Matrix<double,12,30>::Zero(); 
  Eigen::Matrix<double,6,6> MassMatrixCom = _dogbot->getMassMatrixCOM_com();
  A.block(0,0,6,6)= MassMatrixCom;
  A.block(0,18,6,6)= -Jst_com.transpose(); 
  A.block(6,0,6,6)= Jst_com;
  A.block(6,6,6,12)= Jst_j;


  for(int i = 0 ; i <12 ;i++){
    for(int j=0;j<30;j++){
      L(i,j)= A(i,j);
  }
  }
  
  //Vector b
  Eigen::Matrix<double,12, 1> b= Eigen::Matrix<double,12,1>::Zero();
  Eigen::Matrix<double,6,1> gravity;
	gravity<<0,0,9.8,0,0,0; 
  double mass= MassMatrixCom(0,0);
  Eigen::Matrix<double,18,1> Bias = _dogbot->getBiasCOM();
  b.block(0,0,6,1)=-Bias.block(0,0,6,1);
 
  Eigen::Matrix<double,12, 1> Jst_qd_dot=_dogbot->getJdotqd_lin();
  b.block(6,0,3,1)=-Jst_qd_dot.block(st_leg1,0,3,1);
  b.block(9,0,3,1)=-Jst_qd_dot.block(st_leg2,0,3,1);

   for(int i = 0 ; i <12 ;i++){
    
      L(i,30)= b(i,0);
  }
  

///Inequality Constraints

 //Matrix D
  Eigen::Matrix<double,70,30> D = Eigen::Matrix<double,70,30>::Zero(); 

  D.block(0,18,10,6)= Dfr_sw;

  Eigen::Matrix<double,12,12> M_22=_dogbot->getMDAG().block(6,6,12,12);
  D.block(10,6,12,12)= M_22;
  D.block(10,18,12,6)= -Jst_j.transpose();
  D.block(22,6,12,12)= -M_22;
  D.block(22,18,12,6)= Jst_j.transpose();
  D.block(34,0,6,6) = Jsw_com;  
  D.block(34,6,6,12) = Jsw_j;
  D.block(34,24,6,6)= Eigen::Matrix<double,6,6>::Identity();
  D.block(40,0,6,6) = -Jsw_com; 
  D.block(40,6,6,12) =-Jsw_j;
  D.block(40,24,6,6)= -Eigen::Matrix<double,6,6>::Identity();

  D.block(46,6,12,12)=Eigen::Matrix<double,12,12>::Identity();
  D.block(58,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();

  for(int i = 0 ; i <70 ;i++){
    for(int j=0;j<30;j++){
      L(i+12,j)= D(i,j);
  }
  }
  

  ///Vector c
  Eigen::Matrix<double,70, 1> ci= Eigen::Matrix<double,70,1>::Zero(); 

  Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
  Eigen::Matrix<double,12,1> tau_min= - tau_max;

  ci.block(10,0,12,1)= tau_max -  Bias.block(6,0,12,1);
  ci.block(22,0,12,1)= -(tau_min - Bias.block(6,0,12,1));


  Eigen::MatrixXd Kp = 250*Eigen::MatrixXd::Identity(6,6); 
  Eigen::MatrixXd Kd =50*Eigen::MatrixXd::Identity(6,6);

  Eigen::Matrix<double,6,1> e_sw ;

  if(swinglegs == 0){
    e_sw << xsw_ref.block(0,0,3,1)- _dogbot->getBRpos(), xsw_ref.block(3,0,3,1)- _dogbot->getFLpos();}
  if(swinglegs == 1) {
    e_sw << xsw_ref.block(0,0,3,1)- _dogbot->getBLpos(), xsw_ref.block(3,0,3,1)- _dogbot->getFRpos();
 
  }

  Eigen::Matrix<double,6,1> d_e_sw ;

  if(swinglegs == 0){
    d_e_sw << d_xsw_ref.block(0,0,3,1)- _dogbot->getBRvel(), d_xsw_ref.block(3,0,3,1)- _dogbot->getFLvel();}
  if(swinglegs == 1){
    d_e_sw << d_xsw_ref.block(0,0,3,1)- _dogbot->getBLvel(), d_xsw_ref.block(3,0,3,1)- _dogbot->getFRvel();
  }




  Eigen::Matrix<double,6,1> dd_xsw_cmd = dd_xsw_ref+ Kd*d_e_sw + Kp*e_sw;
  _vec4 =dd_xsw_cmd;
  _vec5=  Kp*e_sw;

  Eigen::Matrix<double,12,1> Jdot_qd_lin =_dogbot->getJdotqd_lin();
  Eigen::Matrix<double,6,1> Jswdot_qd_lin= Eigen::Matrix<double,6,1>::Zero();

  Jswdot_qd_lin << Jdot_qd_lin.block(sw_leg1,0,3,1),Jdot_qd_lin.block(sw_leg2,0,3,1);
  /*
  Eigen::Matrix<double,6,1> fext_lambda= Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,18,18> M_com = _dogbot->getMDAG();
  Eigen::Matrix<double,18,18> Si;
  Si<<Eigen::Matrix<double,6,18>::Zero(),
      Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();
  
  Eigen::Matrix<double,18,18> P=Eigen::Matrix<double,18,18>::Identity();
  P= P -Jst.transpose()*(Jst*M_com.inverse()* Jst.transpose()).inverse()* Jst*M_com.inverse();

  fext_lambda<<J.block(sw_leg1,0,3,18)*M_com.inverse()*Si.transpose()*P*J.transpose()*_fext,
	            J.block(sw_leg2,0,3,18)*M_com.inverse()*Si.transpose()*P*J.transpose()*_fext;
*/
  ci.block(34,0,6,1)= dd_xsw_cmd - Jswdot_qd_lin; //- fext_lambda; 
  ci.block(40,0,6,1)= -dd_xsw_cmd + Jswdot_qd_lin; // + fext_lambda;

  double deltat=0.01;
  Eigen::Matrix<double,12, 1> eigenq=_dogbot->get_qb().block(6,0,12,1);
  Eigen::Matrix<double,12, 1> eigendq=_dogbot->get_vb().block(6,0,12,1);
  Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(_qmin-eigenq-deltat*eigendq);
  Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(_qmax-eigenq-deltat*eigendq);


  ci.block(46,0,12,1)=ddqmax;
	ci.block(58,0,12,1)=-ddqmin;

  for(int i = 0 ; i <70 ;i++){
    
      L(i+12,30)= ci(i,0);
  }

  for(int i =0;i<12;i++){
    Lt(i)=0.0;
  }

   for(int i =12;i<82;i++){
    Lt(i)=-1.0;
  }


  minqpsetlc(state_sw, L, Lt);
  minqpsetscaleautodiag(state_sw);
  minqpsetalgodenseaul(state_sw, 1.0e-2, 1.0e+4, 5);
  minqpoptimize(state_sw);
  minqpresults(state_sw, x_sw, rep);

    Eigen::VectorXd x_eig= Eigen::VectorXd::Zero(30);
  	for ( int i = 0; i < x_eig.size(); i++ )
             x_eig(i)=x_sw(i);

  Eigen::MatrixXd dd_q_opt= x_eig.block(6,0,12,1);
  Eigen::MatrixXd fgr_opt= x_eig.block(18,0,6,1);

  _fgr= Eigen::MatrixXd::Zero(12,1);
  _fgr.block(st_leg1,0,3,1) =  fgr_opt.block(0,0,3,1);
  _fgr.block(st_leg2,0,3,1) =  fgr_opt.block(3,0,3,1);

  

_vec2 = dd_q_opt;
  
  Eigen::Matrix<double,12,1> tau= Eigen::MatrixXd::Zero(12,1);
  tau= M_22*dd_q_opt- J.block(0,6,12,12).transpose()*_fgr + Bias.block(6,0,12,1);
  
  _tau_star = tau;

  return tau;
}




Eigen::MatrixXd CONTROL::getfgr(){


        return _fgr;
}


Eigen::MatrixXd CONTROL::getprova(){

return _vec;

}

Eigen::MatrixXd CONTROL::getprova2(){

return _vec2;

}

Eigen::MatrixXd CONTROL::getprova3(){

return _vec3;

}

Eigen::MatrixXd CONTROL::getprova4(){

return _vec4;

}

Eigen::MatrixXd CONTROL::getprova5(){

return _vec5;

}

Eigen::MatrixXd CONTROL::getelin(){

return _elin;

}
