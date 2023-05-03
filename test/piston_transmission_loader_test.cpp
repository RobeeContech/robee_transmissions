// Copyright 2022 RobeeRobotics LTD
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

#include <gmock/gmock.h>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "robee_transmissions/piston_transmission.hpp"
#include "robee_transmissions/piston_transmission_loader.hpp"
#include "transmission_interface/transmission_loader.hpp"

using testing::SizeIs;

class TransmissionPluginLoader
{
public:
  std::shared_ptr<transmission_interface::TransmissionLoader> create(const std::string & type)
  {
    try
    {
      return class_loader_.createUniqueInstance(type);
    }
    catch (std::exception & ex)
    {
      std::cerr << ex.what() << std::endl;
      return std::shared_ptr<transmission_interface::TransmissionLoader>();
    }
  }

private:
  // must keep it alive because instance destroyers need it
  pluginlib::ClassLoader<transmission_interface::TransmissionLoader> class_loader_ = {
    "transmission_interface", "transmission_interface::TransmissionLoader"};
};

TEST(PistonTransmissionLoaderTest, FullSpec)
{
  // Parse transmission info
  std::string urdf_to_test = R"(
    <?xml version="1.0"?>
    <robot name="robot" xmlns="http://www.ros.org">
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <transmission name="transmission1">
          <plugin>robee_transmission_interface/PistonTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>0.5</offset>
            <mechanical_reduction>200.0</mechanical_reduction>
          </joint>
          <param name="r">0.1</param>
          <param name="b">0.1</param>
          <param name="l">0.1</param>
          <param name="actuator_offset">1000.0/param>
        </transmission>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  robee_transmission_interface::PistonTransmission * piston_transmission =
    dynamic_cast<robee_transmission_interface::PistonTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != piston_transmission);

  // const std::vector<double> & joint_reduction = piston_transmission->get_joint_reduction();
  // EXPECT_EQ(2.0, joint_reduction[0]);
  // EXPECT_EQ(-2.0, joint_reduction[1]);

  // const std::vector<double> & joint_offset = piston_transmission->get_joint_offset();
  // EXPECT_EQ(0.5, joint_offset[0]);
  // EXPECT_EQ(-0.5, joint_offset[1]);
}
