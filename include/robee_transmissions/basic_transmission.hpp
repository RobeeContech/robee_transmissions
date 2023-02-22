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

#ifndef TRANSMISSION_INTERFACE__BASIC_TRANSMISSION_HPP_
#define TRANSMISSION_INTERFACE__BASIC_TRANSMISSION_HPP_

#include <cassert>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/accessor.hpp"
#include "transmission_interface/exception.hpp"
#include "transmission_interface/transmission.hpp"


#include "rclcpp/rclcpp.hpp"

namespace robee_transmission_interface
{
/// Implementation of a Basic reducer transmission.
/**
 * This transmission relates <b>one actuator</b> and <b>one joint</b> through a reducer (or amplifier).
 * Timing belts and gears are examples of this transmission type, and are illustrated below.
 * \image html simple_transmission.png
 *
 * <CENTER>
 * <table>
 * <tr><th></th><th><CENTER>Effort</CENTER></th><th><CENTER>Velocity</CENTER></th><th><CENTER>Position</CENTER></th></tr>
 * <tr><td>
 * <b> Actuator to joint </b>
 * </td>
 * <td>
 * \f[ \tau_j = n \tau_a \f]
 * </td>
 * <td>
 * \f[ \dot{x}_j = \dot{x}_a / n \f]
 * </td>
 * <td>
 * \f[ x_j = x_a / n + x_{off} \f]
 * </td>
 * </tr>
 * <tr><td>
 * <b> Joint to actuator </b>
 * </td>
 * <td>
 * \f[ \tau_a = \tau_j / n\f]
 * </td>
 * <td>
 * \f[ \dot{x}_a = n \dot{x}_j \f]
 * </td>
 * <td>
 * \f[ x_a = n (x_j - x_{off}) \f]
 * </td></tr></table>
 * </CENTER>
 *
 * where:
 * - \f$ x \f$, \f$ \dot{x} \f$ and \f$ \tau \f$ are position, velocity and effort variables, respectively.
 * - Subindices \f$ _a \f$ and \f$ _j \f$ are used to represent actuator-space and joint-space variables, respectively.
 * - \f$ x_{off}\f$ represents the offset between motor and joint zeros, expressed in joint position coordinates.
 * - \f$ n \f$ is the transmission ratio, and can be computed as the ratio between the output and input pulley
 *   radii for the timing belt; or the ratio between output and input teeth for the gear system.
 *   The transmission ratio can take any real value \e except zero. In particular:
 *     - If its absolute value is greater than one, it's a velocity reducer / effort amplifier, while if its absolute
 *       value lies in \f$ (0, 1) \f$ it's a velocity amplifier / effort reducer.
 *     - Negative values represent a direction flip, ie. actuator and joint move in opposite directions. For example,
 *       in timing belts actuator and joint move in the same direction, while in single-stage gear systems actuator and
 *       joint move in opposite directtransmission_interface::ions.
 *
 * \ingroup transmission_types
 */
class BasicTransmission : public transmission_interface::Transmission
{
public:
  /**
   * \param[in] joint_reduction    Reduction ratio of joints.
   * \param[in] joint_offset       Offset of joints.
   * \pre Nonzero  joint reduction values.
   */
  BasicTransmission(const double & joint_reduction, const double& joint_offset = 0);
  
  /**
   * \param[in] joint_handles     Handles of joint values.
   * \param[in] actuator_handles  Handles of actuator values.
   * \pre Handles are valid and matching in size
   */
  void configure(
    const std::vector<transmission_interface::JointHandle> & joint_handles,
    const std::vector<transmission_interface::ActuatorHandle> & actuator_handles) override;

  /// Transform variables from actuator to joint space.
  /**
   * \pre Actuator and joint vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuator_to_joint() override;

  /// Transform variables from joint to actuator space.
  /**
   * \pre Actuator and joint vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void joint_to_actuator() override;

  std::size_t num_actuators() const override { return 1; }
  std::size_t num_joints() const override { return 1; }

  const double & get_joint_reduction() const { return joint_reduction_; }
  const double & get_joint_offset() const { return joint_offset_; }
  /// Get human-friendly report of handles
  std::string get_handles_info() const;

protected:
  double joint_reduction_;
  double  joint_offset_;

  transmission_interface::JointHandle joint_position_= {"", "", nullptr};
  transmission_interface::JointHandle joint_velocity_= {"", "", nullptr};
  transmission_interface::JointHandle joint_effort_= {"", "", nullptr};

  transmission_interface::ActuatorHandle actuator_position_= {"", "", nullptr};
  transmission_interface::ActuatorHandle actuator_velocity_= {"", "", nullptr};
  transmission_interface::ActuatorHandle actuator_effort_= {"", "", nullptr};
};


 
inline BasicTransmission::BasicTransmission(const double & joint_reduction, const double & joint_offset)
  : joint_reduction_(joint_reduction), joint_offset_(joint_offset)
{
  if (joint_reduction == 0.0)
  {
    throw transmission_interface::Exception("Transmission reduction ratio cannot be zero.");
  }
}


template <class HandleType>
HandleType get_by_interface(
  const std::vector<HandleType> & handles, const std::string & interface_name)
{
  const auto result = std::find_if(
    handles.cbegin(), handles.cend(),
    [&interface_name](const auto handle) { return handle.get_interface_name() == interface_name; });
  if (result == handles.cend())
  {
    return HandleType(handles.cbegin()->get_prefix_name(), interface_name, nullptr);
  }
  return *result;
}

template <class T>
bool are_names_identical(const std::vector<T> & handles)
{
  std::vector<std::string> names;
  std::transform(
    handles.cbegin(), handles.cend(), std::back_inserter(names),
    [](const auto & handle) { return handle.get_prefix_name(); });
  return std::equal(names.cbegin() + 1, names.cend(), names.cbegin());
}

void BasicTransmission::configure(
  const std::vector<transmission_interface::JointHandle> & joint_handles,
  const std::vector<transmission_interface::ActuatorHandle> & actuator_handles)
{
   if (joint_handles.empty())
  {
    throw  transmission_interface::Exception("No joint handles were passed in");
  }

  if (actuator_handles.empty())
  {
    throw  transmission_interface::Exception("No actuator handles were passed in");
  }

  if (!are_names_identical(joint_handles))
  {
    throw  transmission_interface::Exception("Joint names given to transmissions should be identical");
  }

  if (!are_names_identical(actuator_handles))
  {
    throw  transmission_interface::Exception("Actuator names given to transmissions should be identical");
  }

  joint_position_ = get_by_interface(joint_handles, hardware_interface::HW_IF_POSITION);
  joint_velocity_ = get_by_interface(joint_handles, hardware_interface::HW_IF_VELOCITY);
  joint_effort_ = get_by_interface(joint_handles, hardware_interface::HW_IF_EFFORT);

  if (!joint_position_ && !joint_velocity_ && !joint_effort_)
  {
    throw  transmission_interface::Exception("None of the provided joint handles are valid or from the required interfaces");
  }

  actuator_position_ = get_by_interface(actuator_handles, hardware_interface::HW_IF_POSITION);
  actuator_velocity_ = get_by_interface(actuator_handles, hardware_interface::HW_IF_VELOCITY);
  actuator_effort_ = get_by_interface(actuator_handles, hardware_interface::HW_IF_EFFORT);

  if (!actuator_position_ && !actuator_velocity_ && !actuator_effort_)
  {
    throw  transmission_interface::Exception("None of the provided joint handles are valid or from the required interfaces");
  }
}

inline void BasicTransmission::actuator_to_joint()
{
  
  if (joint_effort_ && actuator_effort_)
  {
    joint_effort_.set_value(actuator_effort_.get_value() * joint_reduction_);
  }

  if (joint_velocity_ && actuator_velocity_)
  {
    joint_velocity_.set_value(actuator_velocity_.get_value() / joint_reduction_);
  }

  if (joint_position_ && actuator_position_)
  {
    joint_position_.set_value(actuator_position_.get_value() / joint_reduction_ + joint_offset_);
  }
 

}

inline void BasicTransmission::joint_to_actuator()
{
  
  if (joint_position_ && actuator_position_)
  {
       actuator_position_.set_value((joint_position_.get_value() - joint_offset_) * joint_reduction_);
  }
	
  if (joint_velocity_ && actuator_velocity_)
  {
      actuator_velocity_.set_value(joint_velocity_.get_value() * joint_reduction_);
  }

  if (joint_effort_ && actuator_effort_)
  {
    //todo 
    actuator_effort_.set_value(joint_effort_.get_value() / joint_reduction_);
  }

}


}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__BASIC_TRANSMISSION_HPP_