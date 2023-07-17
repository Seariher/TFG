#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "pinocchio/spatial/explog.hpp"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <xpp_msgs/RobotStateCartesian.h>

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include <vector>
#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace std;


void EnforceLimits(double& val,  int art){
  
  const static double haa_min = -50/180.0*M_PI;
  const static double haa_max =  50/180.0*M_PI;

  const static double hfe_min = -40/180.0*M_PI;
  const static double hfe_max =  258/180.0*M_PI;

  const static double kfe_min = -161.5/180.0*M_PI;
  const static double kfe_max =  -50/180.0*M_PI;

  switch (art) {
        case 0:
          val = val>haa_max? haa_max : val;

          val = val<haa_min? haa_min : val;
          break;
        case 1:
          val = val>hfe_max? hfe_max : val;

          val = val<hfe_min? hfe_min : val;
          break;
        case 2:
          val = val>kfe_max? kfe_max : val;
          val = val<kfe_min? kfe_min : val;
          break;
        default: // joint angles for this foot do not exist
          break;
      }
}

Eigen::Vector3d inverse_kinematics(const Eigen::Vector3d& ee_pos_B, const Eigen::Vector3d& hfe_to_haa_z, double lu, double ll)
{

  double q_HAA_bf, q_HFE_bf, q_KFE_bf; 

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

  xr = ee_pos_B;

  q_HAA_bf = -atan2(xr[1],-xr[2]);

  R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA_bf), -sin(q_HAA_bf), 0.0, sin(q_HAA_bf), cos(q_HAA_bf);

  xr = (R * xr).eval();

  xr += hfe_to_haa_z; 

  double tmp1 = pow(xr[0],2)+pow(xr[2],2);

  double alpha = atan2(-xr[2],xr[0]) - 0.5*M_PI; 

  double some_random_value_for_beta = (pow(lu,2)+tmp1-pow(ll,2))/(2.*lu*sqrt(tmp1));
  if (some_random_value_for_beta > 1) {
    some_random_value_for_beta = 1;
  }
  if (some_random_value_for_beta < -1) {
    some_random_value_for_beta = -1;
  }
  double beta = acos(some_random_value_for_beta);

  q_HFE_bf = alpha + beta;

  double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp1)/(2.*ll*lu);
 
  if (some_random_value_for_gamma > 1) {
    some_random_value_for_gamma = 1;
  }
  if (some_random_value_for_gamma < -1) {
    some_random_value_for_gamma = -1;
  }
  double gamma  = acos(some_random_value_for_gamma);

  q_KFE_bf = gamma - M_PI;


  EnforceLimits(q_HAA_bf, 0);
  EnforceLimits(q_HFE_bf, 1);
  EnforceLimits(q_KFE_bf, 2);
  
  return Eigen::Vector3d(q_HAA_bf, q_HFE_bf, q_KFE_bf);
}

int main(int argc, char *argv[])
{
  // INICIAMOS EL NODO
  ros::init(argc, argv, "bag_publisher_node");
  
  // SE CREA EL NODO Y VARIABLES PUBLISHER DE CADA ARTICULACION, SE DETERMINA EL RATIO DE PUBLICACION
  ros::NodeHandle n;
  ros::Rate rate(700);
  ros::Publisher servo_pub[12];

  servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_hip_controller/command", 1);
  servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_thigh_controller/command", 1);
  servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_calf_controller/command", 1);
  servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_hip_controller/command", 1);
  servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_thigh_controller/command", 1);
  servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_calf_controller/command", 1);
  servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_hip_controller/command", 1);
  servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_thigh_controller/command", 1);
  servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_calf_controller/command", 1);
  servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_hip_controller/command", 1);
  servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_thigh_controller/command", 1);
  servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_calf_controller/command", 1);
  
  // MENSAJE QUE SE PUBLICA
  unitree_legged_msgs::LowCmd low_cmd_ros;
  
  // SE CARGA EL MODELO URDF Y SE OBTIENE EL DATA CON PINOCCHIO
  const std::string urdf_filename = "/home/seariher/TFG_ws/src/xpp/robots/go1_description/urdf/go1.urdf";
  const int JOINT_ID = 3;

  pinocchio::Model model_complete;
  pinocchio::urdf::buildModel(urdf_filename,model_complete);

  // SE CREAN LOS MODELOS REDUCIDOS
  std::vector<std::string> articulaciones_bloqueadas_FL_name{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  std::vector<std::string> articulaciones_bloqueadas_FR_name{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  std::vector<std::string> articulaciones_bloqueadas_RL_name{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  std::vector<std::string> articulaciones_bloqueadas_RR_name{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"};
  
  std::vector<pinocchio::JointIndex> articulaciones_bloqueadas_FL_id;
  std::vector<pinocchio::JointIndex> articulaciones_bloqueadas_FR_id;
  std::vector<pinocchio::JointIndex> articulaciones_bloqueadas_RL_id;
  std::vector<pinocchio::JointIndex> articulaciones_bloqueadas_RR_id;

  articulaciones_bloqueadas_FL_id.reserve(articulaciones_bloqueadas_FL_name.size());
  articulaciones_bloqueadas_FR_id.reserve(articulaciones_bloqueadas_FR_name.size());
  articulaciones_bloqueadas_RL_id.reserve(articulaciones_bloqueadas_RL_name.size());
  articulaciones_bloqueadas_RR_id.reserve(articulaciones_bloqueadas_RR_name.size());

  for (const auto& str : articulaciones_bloqueadas_FL_name){
    articulaciones_bloqueadas_FL_id.emplace_back(model_complete.getJointId(str));
  }

  for (const auto& str : articulaciones_bloqueadas_FR_name){
    articulaciones_bloqueadas_FR_id.emplace_back(model_complete.getJointId(str));
  }

  for (const auto& str : articulaciones_bloqueadas_RL_name){
    articulaciones_bloqueadas_RL_id.emplace_back(model_complete.getJointId(str));
  }

  for (const auto& str : articulaciones_bloqueadas_RR_name){
    articulaciones_bloqueadas_RR_id.emplace_back(model_complete.getJointId(str));
  }

  auto qc = pinocchio::neutral(model_complete);
  pinocchio::Model model_FL = pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_FL_id, qc);
  pinocchio::Model model_FR = pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_FR_id, qc);
  pinocchio::Model model_RL = pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_RL_id, qc);
  pinocchio::Model model_RR = pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_RR_id, qc);

  // SE OBTIENEN LOS DATA DE CADA MODELO Y SE CREAN LAS MATRICES PARA LAS JACOBIANAS
  pinocchio::Data data_FL(model_FL);
  pinocchio::Data data_FR(model_FR);
  pinocchio::Data data_RL(model_RL);
  pinocchio::Data data_RR(model_RR);

  pinocchio::Data::Matrix6x J_FL(6, model_FL.nv);
  pinocchio::Data::Matrix6x J_FR(6, model_FR.nv);
  pinocchio::Data::Matrix6x J_RL(6, model_RL.nv);
  pinocchio::Data::Matrix6x J_RR(6, model_RR.nv);
  
  J_FL.setZero();
  J_FR.setZero();
  J_RL.setZero();
  J_RR.setZero();

  Eigen::VectorXd q_FL(3), q_FR(3), q_RL(3), q_RR(3), q_FL_des(3), q_FR_des(3), q_RL_des(3), q_RR_des(3), q(12);
  q_FL << 0.0, 0.67, -1.3; q_FR << -0.0, 0.67, -1.3; q_RL << 0.0, 0.67, -1.3; q_RR << -0.0, 0.67, -1.3;

  Eigen::VectorXd f_FL(6), f_FR(6), f_RL(6), f_RR(6);
  for(int i = 0; i < 6; i++){
    f_FL[i] = 0; f_FR[i] = 0; f_RL[i] = 0; f_RR[i] = 0; 
  }

  Eigen::VectorXd tau_FL(6), tau_FR(6), tau_RL(6), tau_RR(6), tau(12);
  for(int i = 0; i < 6; i++){
    tau_FL[i] = 0; tau_FR[i] = 0; tau_RL[i] = 0; tau_RR[i] = 0; 
  }

  for(int i = 0; i < 12; i++){
    tau[i] = 0; 
    q[i] = 0;
  }

  // COSAS PARA REALIZAR LA CINEMATICA INVERSA

  Eigen::Vector3d base2hip_FL_(0.18, 0.125, 0.0);
  Eigen::Vector3d base2hip_FR_(0.18, -0.125, 0.0);
  Eigen::Vector3d base2hip_RL_(-0.18, 0.125, 0.0);
  Eigen::Vector3d base2hip_RR_(-0.18, -0.125, 0.0);
  
  Eigen::Vector3d hfe_to_haa_z(0.0, 0.0, 0.048); 
  double length_thigh = 0.190;
  double length_shank = 0.198;

  // LEVANTAR EL ROBOT

  string opcion;
  cout << "Selecciona trayectoria: ";
  cin >> opcion;
  cout << "Se ha seleccionado la trayectoria " << opcion << endl;
  double percent;
  double duration = 4000;

  double pos_final[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
  double pos_inicial[12] = {-0.36, 1.243, -2.81, 0.36, 1.243, -2.81, -0.36, 1.243, -2.81, 0.36, 1.243, -2.81};

  for(int i=1; i<=duration; i++){
      percent = (double)i/duration;
      for(int j=0; j<12; j++){
          low_cmd_ros.motorCmd[j].mode = 10;
          low_cmd_ros.motorCmd[j].q = pos_inicial[j]*(1-percent) + pos_final[j]*percent;
          low_cmd_ros.motorCmd[j].Kp = 180;
          low_cmd_ros.motorCmd[j].dq = 0.0;
          low_cmd_ros.motorCmd[j].Kd = 5;
          low_cmd_ros.motorCmd[j].tau = 0;
      }
      for(int m=0; m<12; m++){
        servo_pub[m].publish(low_cmd_ros.motorCmd[m]);
      }
      usleep(1000);
  }

  int tiempo = 0;
  // ESTABLECIMIENTO DE TIEMPO
  if(opcion == "2" || opcion == "3"){
    tiempo = 2400;
  }
  else if(opcion == "5" || opcion == "6"){
    tiempo = 3600;
  }
  else if(opcion == "4" || opcion == "7" || opcion == "8" || opcion == "9"){
    tiempo = 2400;
  }
  
  // COSAS DEL BAG
  rosbag::Bag bag;
  bag.open("/home/seariher/Escritorio/opcion" + opcion + ".bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  //topics.push_back(std::string("/xpp/joint_go1_des"));
  topics.push_back(std::string("/xpp/state_des"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  int iteration = 0;
  foreach(rosbag::MessageInstance const m, view){

    xpp_msgs::RobotStateCartesian::ConstPtr force = m.instantiate<xpp_msgs::RobotStateCartesian>();

    if(force != NULL && iteration <= tiempo){
      iteration++;
      q_FL_des[0] = force->ee_motion[0].pos.x - force->base.pose.position.x;
      q_FL_des[1] = force->ee_motion[0].pos.y - force->base.pose.position.y;
      q_FL_des[2] = force->ee_motion[0].pos.z - force->base.pose.position.z;

      q_FR_des[0] = force->ee_motion[1].pos.x - force->base.pose.position.x;
      q_FR_des[1] = force->ee_motion[1].pos.y - force->base.pose.position.y;
      q_FR_des[2] = force->ee_motion[1].pos.z - force->base.pose.position.z;

      q_RL_des[0] = force->ee_motion[2].pos.x - force->base.pose.position.x;
      q_RL_des[1] = force->ee_motion[2].pos.y - force->base.pose.position.y;
      q_RL_des[2] = force->ee_motion[2].pos.z - force->base.pose.position.z;

      q_RR_des[0] = force->ee_motion[3].pos.x - force->base.pose.position.x;
      q_RR_des[1] = force->ee_motion[3].pos.y - force->base.pose.position.y;
      q_RR_des[2] = force->ee_motion[3].pos.z - force->base.pose.position.z;

      Eigen::Matrix3d R;

      q_FL_des -= base2hip_FL_;
      q_FR_des -= base2hip_FR_;
      q_RL_des -= base2hip_RL_;
      q_RR_des -= base2hip_RR_;

      q_FL = inverse_kinematics(q_FL_des, hfe_to_haa_z, length_thigh, length_shank);
      q_FR = inverse_kinematics(q_FR_des, hfe_to_haa_z, length_thigh, length_shank);
      q_RL = inverse_kinematics(q_RL_des, hfe_to_haa_z, length_thigh, length_shank);
      q_RR = inverse_kinematics(q_RR_des, hfe_to_haa_z, length_thigh, length_shank);

      pinocchio::computeJointJacobian(model_FL, data_FL, q_FL, JOINT_ID, J_FL);
      pinocchio::computeJointJacobian(model_FR, data_FR, q_FR, JOINT_ID, J_FR);
      pinocchio::computeJointJacobian(model_RL, data_RL, q_RL, JOINT_ID, J_RL);
      pinocchio::computeJointJacobian(model_RR, data_RR, q_RR, JOINT_ID, J_RR);

      f_FL[0] = force->ee_forces[0].x;
      f_FL[1] = force->ee_forces[0].y;
      f_FL[2] = force->ee_forces[0].z;

      f_FR[0] = force->ee_forces[1].x;
      f_FR[1] = force->ee_forces[1].y;
      f_FR[2] = force->ee_forces[1].z;

      f_RL[0] = force->ee_forces[2].x;
      f_RL[1] = force->ee_forces[2].y;
      f_RL[2] = force->ee_forces[2].z;

      f_RR[0] = force->ee_forces[3].x;
      f_RR[1] = force->ee_forces[3].y;
      f_RR[2] = force->ee_forces[3].z;

      tau_FL = J_FL.transpose()*f_FL;
      tau_FR = J_FR.transpose()*f_FR;
      tau_RL = J_RL.transpose()*f_RL;
      tau_RR = J_RR.transpose()*f_RR;

      int z = 0;
      int a = 0;
      for(int l = 0; l < 12; l++){
        if (z == 3){
          z = 0;
          a++;
        }
        
        if(a == 0){
          q[l] = q_FR[z];
          tau[l] = tau_FR[z];
        }
        else if(a == 1){
          q[l] = q_FL[z];
          tau[l] = tau_FL[z];
        }
        else if(a == 2){
          q[l] = q_RR[z];
          tau[l] = tau_RR[z];
        }
        else if(a == 3){
          q[l] = q_RL[z];
          tau[l] = tau_RL[z];
        }
        z++;
      }

      if(iteration == 1){
        for(int i=1; i<=duration; i++){
          percent = (double)i/duration;
          for(int j=0; j<12; j++){
              low_cmd_ros.motorCmd[j].mode = 10;
              low_cmd_ros.motorCmd[j].q = pos_final[j]*(1-percent) + q[j]*percent;
              low_cmd_ros.motorCmd[j].Kp = 180;
              low_cmd_ros.motorCmd[j].dq = 0.0;
              low_cmd_ros.motorCmd[j].Kd = 5;
              low_cmd_ros.motorCmd[j].tau = 0;
          }
          for(int m=0; m<12; m++){
            servo_pub[m].publish(low_cmd_ros.motorCmd[m]);
          }
          usleep(1000);
        }
      }
      else if(iteration == tiempo){
        for(int i=1; i<=duration; i++){
          percent = (double)i/duration;
          for(int j=0; j<12; j++){
              low_cmd_ros.motorCmd[j].mode = 10;
              low_cmd_ros.motorCmd[j].q = q[j]*(1-percent) + pos_inicial[j]*percent;
              low_cmd_ros.motorCmd[j].Kp = 180;
              low_cmd_ros.motorCmd[j].dq = 0.0;
              low_cmd_ros.motorCmd[j].Kd = 5;
              low_cmd_ros.motorCmd[j].tau = 0;
          }
          for(int m=0; m<12; m++){
            servo_pub[m].publish(low_cmd_ros.motorCmd[m]);
          }
          usleep(1000);
        }
      }
      else if(iteration < tiempo){

        for (int i = 0; i < 12; i++){
          low_cmd_ros.motorCmd[i].mode = 10;
          low_cmd_ros.motorCmd[i].q = q[i];
          low_cmd_ros.motorCmd[i].Kp = 180;
          low_cmd_ros.motorCmd[i].dq = 0.0;
          low_cmd_ros.motorCmd[i].Kd = 5;
          low_cmd_ros.motorCmd[i].tau = tau[i];
        }

        for(int m=0; m<12; m++){
          servo_pub[m].publish(low_cmd_ros.motorCmd[m]);
        }

        rate.sleep();
      }
    }
  }

  bag.close();

  return 0;
}