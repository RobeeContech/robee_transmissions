// Copyright 2022 RobeeContech LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robee_transmissions/piston_transmission_loader.hpp"

#include <memory>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robee_transmissions/piston_transmission.hpp"

namespace robee_transmission_interface
{

  double get_param(const std::unordered_map<std::string, std::string>& parameters, const std::string&& key) 
  {
    auto res = parameters.find(key);
    if (res!= parameters.end())
      return std::stod(res->second);
    else 
      return 0;
  }

  std::shared_ptr<transmission_interface::Transmission> PistonTransmissionLoader::load(const hardware_interface::TransmissionInfo & transmission_info)
  {
    try
    {
      const auto jnt_r = get_param(transmission_info.parameters, "r");
      const auto jnt_b = get_param(transmission_info.parameters, "b");
      const auto jnt_l = get_param(transmission_info.parameters, "l");
      const auto jnt_offset = transmission_info.joints.at(0).offset;
      const auto jnt_mechanical_reduction = transmission_info.joints.at(0).mechanical_reduction;

      std::shared_ptr<transmission_interface::Transmission> transmission(new PistonTransmission(jnt_r,jnt_b,jnt_l, jnt_offset,
                                                                                                jnt_mechanical_reduction));
      return transmission;
    }
    catch (const std::exception & ex)
    { 
      RCLCPP_ERROR(
        rclcpp::get_logger("Piston_transmission_loader"),
        "Failed to construct transmission '%s'", ex.what());
      return std::shared_ptr<transmission_interface::Transmission>();
    }
  }

}  // namespace robee_transmission_interface

PLUGINLIB_EXPORT_CLASS(
  robee_transmission_interface::PistonTransmissionLoader,
  transmission_interface::TransmissionLoader)
