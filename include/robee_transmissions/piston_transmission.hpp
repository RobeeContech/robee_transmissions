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

#ifndef TRANSMISSION_INTERFACE__PISTON_TRANSMISSION_HPP_
#define TRANSMISSION_INTERFACE__PISTON_TRANSMISSION_HPP_

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
/// Implementation of a piston reducer transmission.
/**
 * This transmission 
 *  actuator_pos(joint_pos) =  (( r^2 + b^2 - 2 * r * b * cos(joint_offset + joint_pos)) / l ) * joint_mechanical_reduction_
 */
class PistonTransmission : public transmission_interface::Transmission
{
public:
  /**
   * \param[in] r      length of
   * \param[in] b      length of
   * \param[in] l      length of
   * \param[in] joint_offset       Offset of joints.
   * \param[in] actuator_offset       Offset of joints.
   * \param[in] joint_mechanical_reduction       linear conversation ratio between piston opening in meters to piston encoder ticks .
   * \pre Nonzero  joint reduction values.
   */
  PistonTransmission(const double & r,const double & b,const double & l, 
                    const double & joint_offset, const double & actuator_offset, const double & joint_mechanical_reduction);
  
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

  // const double & get_joint_reduction() const { return joint_reduction_; }
  // const double & get_joint_offset() const { return joint_offset_; }
  /// Get human-friendly report of handles
  std::string get_handles_info() const;

protected:
  double  r_, b_, l_;
  double  joint_offset_, actuator_offset_;
  double  joint_mechanical_reduction_;

  transmission_interface::JointHandle joint_position_= {"", "", nullptr};
  transmission_interface::JointHandle joint_velocity_= {"", "", nullptr};
  transmission_interface::JointHandle joint_effort_= {"", "", nullptr};

  transmission_interface::ActuatorHandle actuator_position_= {"", "", nullptr};
  transmission_interface::ActuatorHandle actuator_velocity_= {"", "", nullptr};
  transmission_interface::ActuatorHandle actuator_effort_= {"", "", nullptr};
};


 
inline PistonTransmission::PistonTransmission(const double & r,const double & b,const double & l, 
                                              const double & joint_offset, const double & actuator_offset,
                                              const double & joint_mechanical_reduction)
  : r_(r),b_(b), l_(l), joint_offset_(joint_offset), actuator_offset_(actuator_offset),
    joint_mechanical_reduction_(joint_mechanical_reduction)
{
  if (r == 0.0) throw transmission_interface::Exception("Piston Transmission r param  cannot be zero.");
  if (b == 0.0) throw transmission_interface::Exception("Piston Transmission b param  cannot be zero.");
  if (l == 0.0) throw transmission_interface::Exception("Piston Transmission l param  cannot be zero.");
  if (joint_mechanical_reduction == 0.0) throw transmission_interface::Exception("Piston Transmission joint_mechanical_reduction param  cannot be zero.");
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

void PistonTransmission::configure(
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

/**
 *  joint_pos(actuator_pos) =   ((((actuator_pos + actuator_offset_) / joint_mechanical_reduction_) + l - r^2 - b^2) / (-2*r*b)) - joint_offset
*/
inline void PistonTransmission::actuator_to_joint()
{
  if (joint_effort_ && actuator_effort_)
  {
    joint_effort_.set_value(actuator_effort_.get_value() * joint_mechanical_reduction_);
  }

  if (joint_velocity_ && actuator_velocity_)
  {
    joint_velocity_.set_value(actuator_velocity_.get_value() / joint_mechanical_reduction_);
  }

  if (joint_position_ && actuator_position_)
  {
    joint_position_.set_value(((((actuator_position_.get_value() + actuator_offset_) / joint_mechanical_reduction_) + l_ - r_*r_ - b_*b_) / (-2*r_*b_)) - joint_offset_);
  }
}

/**
 * actuator_pos(joint_pos) =  ((( r^2 + b^2 - 2 * r * b * cos(joint_offset + joint_pos)) / l ) * joint_mechanical_reduction_ ) - actuator_offset_
*/
inline void PistonTransmission::joint_to_actuator()
{
  
  if (joint_position_ && actuator_position_)
  {
       actuator_position_.set_value(((( r_*r_ + b_*b_ - 2 * r_ * b_ * cos(joint_offset_ + joint_position_.get_value())) / l_ ) * joint_mechanical_reduction_) - actuator_offset_);
  }
	
  if (joint_velocity_ && actuator_velocity_)
  {
      actuator_velocity_.set_value(joint_velocity_.get_value() * joint_mechanical_reduction_);
  }

  if (joint_effort_ && actuator_effort_)
  {
    //todo 
    actuator_effort_.set_value(joint_effort_.get_value() / joint_mechanical_reduction_);
  }

}


}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__PISTON_TRANSMISSION_HPP_