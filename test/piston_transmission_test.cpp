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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "random_generator_utils.hpp"
#include "robee_transmissions/piston_transmission.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using testing::DoubleNear;
using transmission_interface::ActuatorHandle;
using robee_transmission_interface::PistonTransmission;
using transmission_interface::Exception;
using transmission_interface::JointHandle;
// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PistonTest, TestLegal1)
{
  EXPECT_NO_THROW(PistonTransmission(0.1,0.1,0.1, 0.1,1000.0,100));
}



