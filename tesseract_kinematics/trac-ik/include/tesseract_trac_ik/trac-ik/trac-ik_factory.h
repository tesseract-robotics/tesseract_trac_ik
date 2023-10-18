/**
 * @file trac-ik_factory.h
 * @brief Tesseract Trac-IK Factory.
 *
 * @author Roelof Oomen
 * @date July 25, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_KINEMATICS_TRACIK_FACTORY_H
#define TESSERACT_KINEMATICS_TRACIK_FACTORY_H

#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

namespace tesseract_kinematics
{

class TracIKInvKinChainFactory : public InvKinFactory
{
  InverseKinematics::UPtr create(const std::string& solver_name,
                                 const tesseract_scene_graph::SceneGraph& scene_graph,
                                 const tesseract_scene_graph::SceneState& scene_state,
                                 const KinematicsPluginFactory& plugin_factory,
                                 const YAML::Node& config) const override final;
};

TESSERACT_PLUGIN_ANCHOR_DECL(TracIKFactoryAnchor)

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_TRACIK_FACTORY_H
