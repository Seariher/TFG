#include <go1_description/go1leg_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>


namespace xpp {


Go1legInverseKinematics::Vector3d
Go1legInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B) const
{
  double q_HAA, q_HFE, q_KFE; 

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

  xr = ee_pos_B;

  // compute the HAA angle
  q_HAA = -atan2(xr[Y],-xr[Z]);

  // rotate into the HFE coordinate system (rot around X)
  R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA), -sin(q_HAA), 0.0, sin(q_HAA), cos(q_HAA);

  xr = (R * xr).eval();

  // translate into the HFE coordinate system (along Z axis)
  xr += hfe_to_haa_z;  //distance of HFE to HAA in z direction

  // compute square of length from HFE to foot
  double tmp = pow(xr[X],2)+pow(xr[Z],2);

  // compute temporary angles (with reachability check)
  double lu = length_thigh;  // length of upper leg
  double ll = length_shank;  // length of lower leg
  double alpha = atan2(-xr[Z],xr[X]) - 0.5*M_PI;  //  flip and rotate to match Go1 joint definition

  double some_random_value_for_beta = (pow(lu,2)+tmp-pow(ll,2))/(2.*lu*sqrt(tmp)); // this must be between -1 and 1
  if (some_random_value_for_beta > 1) {
    some_random_value_for_beta = 1;
  }
  if (some_random_value_for_beta < -1) {
    some_random_value_for_beta = -1;
  }
  double beta = acos(some_random_value_for_beta);

  // compute Hip FE angle
  q_HFE = alpha + beta;

  double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp)/(2.*ll*lu);
  // law of cosines give the knee angle
  if (some_random_value_for_gamma > 1) {
    some_random_value_for_gamma = 1;
  }
  if (some_random_value_for_gamma < -1) {
    some_random_value_for_gamma = -1;
  }
  double gamma  = acos(some_random_value_for_gamma);

  q_KFE = gamma - M_PI;

  // forward knee bend
  EnforceLimits(q_HAA, HAA);
  EnforceLimits(q_HFE, HFE);
  EnforceLimits(q_KFE, KFE);
  
  return Vector3d(q_HAA, q_HFE, q_KFE);
}

void
Go1legInverseKinematics::EnforceLimits (double& val, Go1JointID joint) const
{
  // totally exaggerated joint angle limits
  const static double haa_min = -50;
  const static double haa_max =  50;

  const static double hfe_min = -40;
  const static double hfe_max =  258;

  const static double kfe_min = -161.5;
  const static double kfe_max =  -50;

  // reduced joint angles for optimization
  static const std::map<Go1JointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<Go1JointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

} /* namespace xpp */
