#ifndef TESSERACT_KINEMATICS_ABB_IRB2400_IKFAST_KINEMATICS_H
#define TESSERACT_KINEMATICS_ABB_IRB2400_IKFAST_KINEMATICS_H

#include <Eigen/Geometry>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <vector>

namespace tesseract_kinematics
{
class AbbIRB2400Kinematics : public IKFastInvKin
{
public:
  AbbIRB2400Kinematics() = default;
  AbbIRB2400Kinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr& fwd_kin);
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_ABB_IRB2400_IKFAST_KINEMATICS_H
