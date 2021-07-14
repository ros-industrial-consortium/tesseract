/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include "abb_irb2400_ikfast_kinematics.h"
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_kinematics
{
AbbIRB2400Kinematics::AbbIRB2400Kinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr& fwd_kin)
  : IKFastInvKin()
{
  init(fwd_kin->getName(),
       fwd_kin->getBaseLinkName(),
       fwd_kin->getTipLinkName(),
       fwd_kin->getJointNames(),
       fwd_kin->getLinkNames(),
       fwd_kin->getActiveLinkNames(),
       fwd_kin->getLimits(),
       fwd_kin->getRedundancyCapableJointIndices());
}

}  // namespace tesseract_kinematics
