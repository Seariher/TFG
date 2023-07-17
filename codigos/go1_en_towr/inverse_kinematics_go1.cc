#include <go1_description/inverse_kinematics_go1.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

namespace xpp {

Joints
InverseKinematicsGo1::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = x_B.ToImpl();
  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {

    using namespace quad;
    ee_pos_H = pos_B.at(ee);
    switch (ee) {
      case LF:
        ee_pos_H -= base2hip_LF_;
        break;
      case RF:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
	      ee_pos_H -= base2hip_LF_;
        break;
      case LH:
        ee_pos_H -= base2hip_LH_;
        break;
      case RH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
	      ee_pos_H -= base2hip_LH_;
        break;
      default: // joint angles for this foot do not exist
        break;
    }

    q_vec.push_back(leg.GetJointAngles(ee_pos_H));
  }

  return Joints(q_vec);
}

} /* namespace xpp */
