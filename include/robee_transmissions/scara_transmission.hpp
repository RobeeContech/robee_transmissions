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

#ifndef TRANSMISSION_INTERFACE__SCARA_TRANSMISSION_HPP_
#define TRANSMISSION_INTERFACE__SCARA_TRANSMISSION_HPP_

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
/// Implementation of a scara transmission.
/**
 *
 * This transmission relates <b>two actuators</b> and <b>two joints</b> through a scara crew mechanism, as illustrated
 * below.
 * \image html scara_transmission.png
 *
 * <CENTER>
 * <table>
 * <tr><th></th><th><CENTER>Effort</CENTER></th><th><CENTER>Velocity</CENTER></th><th><CENTER>Position</CENTER></th></tr>
 * <tr><td>
 * <b> Actuator to joint </b>
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \tau_{j_1} & = ?
 * \tau_{j_2} & = ?
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \dot{x}_{j_1} & = & ( \dot{x}_{a_2} - \dot{x}_{a_2} ) / tr_{j_1}
 * \dot{x}_{j_2} & = & \dot{x}_{a_2} / tr_{j_2}
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * x_{j_1} & = (& x_{a_2}  - x_{a_2}) / tr_{j_1}
 * x_{j_2} & =  & x_{a_2} / tr_{j_2}
 * \f}
 * </td>
 * </tr>
 * <tr><td>
 * <b> Joint to actuator </b>
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \tau_{a_1} & = ?
 * \tau_{a_2} & = ?
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \dot{x}_{a_1} & = & \dot{x}_{j_1} * tr_{j_1} + \dot{x}_{j_2} * tr_{j_2}
 * \dot{x}_{a_2} & = & \dot{x}_{j_2} * tr_{j_2}
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * x_{a_1} & = & x_{j_1} * tr_{j_1} + x_{j_2} * tr_{j_2}
 * x_{a_2} & = & x_{j_2} * tr_{j_2}
 * \f}
 * </td></tr></table>
 * </CENTER>
 *
 * where:
 * - \f$ x \f$, \f$ \dot{x} \f$ and \f$ \tau \f$ are position, velocity and effort variables, respectively.
 * - Subindices \f$ _a \f$ and \f$ _j \f$ are used to represent actuator-space and joint-space variables, respectively.
 * - \f$ j_1 is prismatic along an axis, and  \f$ j_2 is rotation round that same axis, with actuator \f$ a_1 \f$ a_2 respectively.
 * - \f$ tr \f$ represents a joint transmission ratio. Reducers/amplifiers are allowed 
 *   (depicted as timing belts in the figure).
 *  A transmission ratio can take any real value \e except zero. In particular:
 *     - If its absolute value is greater than one, it's a velocity reducer / effort amplifier, while if its absolute
 *       value lies in \f$ (0, 1) \f$ it's a velocity amplifier / effort reducer.
 *     - Negative values represent a direction flip, ie. input and output move in opposite directions.
 *     - <b>Important:</b> Use transmission ratio signs to match this class' convention of positive actuator/joint
 *       directions with a given mechanical design, as they will in general not match.
 *
 * \note This implementation currently assumes a specific layout for location of the actuators and joint axes which is
 * common in robotic mechanisms. Please file an enhancement ticket if your use case does not adhere to this layout.
 *
 * \ingroup transmission_types
 */
class ScaraTransmission : public transmission_interface::Transmission
{
public:
  /**
   * \param[in] joint_reduction    Reduction ratio of joints.
   * \param[in] joint_offset       Offset of joints.
   * \pre Nonzero  joint reduction values.
   */
  ScaraTransmission(double screw_reduction, const std::vector<double> & joint_reduction,const std::vector<double> & joint_ppr, const std::vector<double> & joint_offset = {0.0, 0.0});
  
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

  std::size_t num_actuators() const override { return 2; }
  std::size_t num_joints() const override { return 2; }

  const std::vector<double> & get_joint_reduction() const { return joint_reduction_; }
  const std::vector<double> & get_joint_offset() const { return joint_offset_; }
  const std::vector<double> & get_joint_ppr() const { return joint_ppr_; }

  /// Get human-friendly report of handles
  std::string get_handles_info() const;

protected:
  double screw_reduction_;
  std::vector<double> joint_reduction_;
  std::vector<double> joint_ppr_;
  std::vector<double> joint_offset_;

  std::vector<double> joint_position_last_ ;

  

  std::vector<transmission_interface::JointHandle> joint_position_; 
  std::vector<transmission_interface::JointHandle> joint_velocity_;
  std::vector<transmission_interface::JointHandle> joint_effort_;

  std::vector<transmission_interface::ActuatorHandle> actuator_position_;
  std::vector<transmission_interface::ActuatorHandle> actuator_velocity_;
  std::vector<transmission_interface::ActuatorHandle> actuator_effort_;

  int debug = 0;
};


 
inline ScaraTransmission::ScaraTransmission(double screw_reduction,
                                            const std::vector<double> & joint_reduction, 
                                            const std::vector<double> & joint_ppr,
                                            const std::vector<double> & joint_offset)
  : screw_reduction_(screw_reduction), joint_reduction_(joint_reduction), joint_ppr_(joint_ppr), joint_offset_(joint_offset)
{
  if (num_joints() != joint_reduction_.size() || num_joints() != joint_offset_.size() || num_joints() != joint_ppr_.size())
  {
    RCLCPP_INFO(rclcpp::get_logger("ScaraTransmission"),"Reduction, ppr and offset vectors must have size 2.");
    throw  transmission_interface::Exception("Reduction and offset vectors must have size 2.");
  }
  
  if ( 0.0 == joint_reduction_[0] || 0.0 == joint_reduction_[1])
  {
    RCLCPP_INFO(rclcpp::get_logger("ScaraTransmission"),"Transmission reduction ratios cannot be zero.");
    throw  transmission_interface::Exception("Transmission reduction ratios cannot be zero.");
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("ScaraTransmission"),"joint_ppr_[0] =" << joint_ppr_[0]);
  for(auto& x : joint_ppr_) { x/= 2*M_PI;}
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ScaraTransmission"),"joint_ppr_[0] =" << joint_ppr_[0]);
}

void ScaraTransmission::configure(
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

  const auto joint_names = get_names(joint_handles);
  const auto actuator_names = get_names(actuator_handles);

  joint_position_last_.push_back(0.0);
  joint_position_last_.push_back(0.0);
  joint_position_ =
    get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_POSITION);
  joint_velocity_ =
    get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_VELOCITY);
  joint_effort_ = get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_EFFORT);

  if (joint_position_.size() == 0 && joint_velocity_.size() == 0 && joint_effort_.size() == 0)
  {
    throw  transmission_interface::Exception("Not enough valid or required joint handles were presented.");
  }

  if (joint_position_.size() > 0 && joint_position_.size() != 2)  throw  transmission_interface::Exception(" illegal joint_position_.size() bigger then zero but not 2");
  if (joint_velocity_.size() > 0 && joint_velocity_.size() != 2)  throw  transmission_interface::Exception(" illegal joint_velocity_.size() bigger then zero but not 2");
  if (joint_effort_.size() > 0 && joint_effort_.size() != 2)  throw  transmission_interface::Exception(" illegal joint_effort_.size() bigger then zero but not 2");

  actuator_position_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_POSITION);
  actuator_velocity_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_VELOCITY);
  actuator_effort_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_EFFORT);

  if (actuator_position_.size() == 0 && actuator_velocity_.size() == 0 &&actuator_effort_.size() == 0)
  {
    throw  transmission_interface::Exception(
      "Not enough valid or required actuator handles were presented. \n" + get_handles_info());
  }

  if (actuator_position_.size() > 0 && actuator_position_.size() != 2)  throw  transmission_interface::Exception(" illegal actuator_position_.size() bigger then zero but not 2");
  if (actuator_velocity_.size() > 0 && actuator_velocity_.size() != 2)  throw  transmission_interface::Exception(" illegal actuator_velocity_.size() bigger then zero but not 2");
  if (actuator_effort_.size() > 0 && actuator_effort_.size() != 2)  throw  transmission_interface::Exception(" illegal actuator_effort_.size() bigger then zero but not 2");


  if (
    joint_position_.size() != actuator_position_.size() &&
    joint_velocity_.size() != actuator_velocity_.size() &&
    joint_effort_.size() != actuator_effort_.size())
  {
    throw  transmission_interface::Exception("Pair-wise mismatch on interfaces. \n" + get_handles_info());
  }
}

inline void ScaraTransmission::actuator_to_joint()
{
  if (joint_position_.size() == num_joints())
  {
    auto ap0 = actuator_position_[0].get_value() / (joint_ppr_[0] * joint_reduction_[0]);
    auto ap1 = actuator_position_[1].get_value() / (joint_ppr_[1] * joint_reduction_[1]);
    volatile auto jp0 = joint_offset_[0] +(ap0- ap1) / screw_reduction_ ;
    volatile auto jp1 = joint_offset_[1] + ap1;
    joint_position_[0].set_value(jp0);
    joint_position_[1].set_value(jp1);
  }
	    
  if (joint_velocity_.size() == num_joints())
  {
    auto av0 = actuator_velocity_[0].get_value() / (joint_ppr_[0] * joint_reduction_[0]);
    auto av1 = actuator_velocity_[1].get_value() / (joint_ppr_[1] * joint_reduction_[1]);

    volatile auto jv0 = (av0 - av1)  /   screw_reduction_;
    volatile auto jv1 = av1; 
    joint_velocity_[0].set_value(jv0);   
    joint_velocity_[1].set_value(jv1);
  }

  if (joint_effort_.size() == num_joints())
  {
	  //todo 
	  joint_effort_[0].set_value(actuator_effort_[0].get_value());
	  joint_effort_[1].set_value(actuator_effort_[1].get_value());
  }

  debug++;
  
  if (debug > 50  || ((!std::isnan(joint_position_[0].get_value())) &&  (!std::isnan(joint_position_[1].get_value())) &&
    (joint_position_last_[0]!= joint_position_[0].get_value()  || joint_position_last_[1]  !=  joint_position_[1].get_value()) ))
  {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ScaraTransmission"), "pos:\n"<< 
      "j[0] = " <<  joint_position_[0].get_value() <<  ", a[0] = " <<  actuator_position_[0].get_value() << "\n"<<
      "j[1] = " <<  joint_position_[1].get_value() <<  ", a[1] = " <<  actuator_position_[1].get_value());

      joint_position_last_[0] = joint_position_[0].get_value() ;
      joint_position_last_[1] = joint_position_[1].get_value() ;
      debug = 0;
  }
}


inline void ScaraTransmission::joint_to_actuator()
{
  if (actuator_position_.size() == num_joints())
  {
      double joints_offset_applied[2] = {joint_position_[0].get_value() - joint_offset_[0], joint_position_[1].get_value() - joint_offset_[1]};
      actuator_position_[0].set_value( (joints_offset_applied[0] * screw_reduction_  + joints_offset_applied[1]) * joint_reduction_[0]* joint_ppr_[0]);
      actuator_position_[1].set_value( joints_offset_applied[1] * joint_reduction_[1] *  joint_ppr_[1]);
  }
	
  if (actuator_velocity_.size() == num_joints())
  {
      actuator_velocity_[1].set_value(joint_velocity_[1].get_value() * joint_reduction_[1] *  joint_ppr_[1]);
      actuator_velocity_[0].set_value((joint_velocity_[0].get_value() * screw_reduction_  + actuator_velocity_[1].get_value())* joint_reduction_[0] * joint_ppr_[0]);
  }

  if (actuator_effort_.size() == num_joints())
  {
    //todo 
    actuator_effort_[0].set_value(joint_effort_[0].get_value());
    actuator_effort_[1].set_value(joint_effort_[1].get_value());
  }

  debug++;

  if (debug > 50  || ((!std::isnan(joint_position_[0].get_value())) &&  (!std::isnan(joint_position_[1].get_value())) &&
    (joint_position_last_[0]!= joint_position_[0].get_value()  || joint_position_last_[1]  !=  joint_position_[1].get_value()) ))
  {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ScaraTransmission"), "cmd:\n"<< 
      "j[0] = " <<  joint_position_[0].get_value() <<  ", a[0] = " <<  actuator_position_[0].get_value() << "\n"<<
      "j[1] = " <<  joint_position_[1].get_value() <<  ", a[1] = " <<  actuator_position_[1].get_value());

      joint_position_last_[0] = joint_position_[0].get_value() ;
      joint_position_last_[1] = joint_position_[1].get_value() ;
      debug = 0;
  }
}

std::string ScaraTransmission::get_handles_info() const
{
  return std::string("Got the following handles:\n") +
         "Joint position: " + transmission_interface::to_string(get_names(joint_position_)) +
         ", Actuator position: " + transmission_interface::to_string(get_names(actuator_position_)) + "\n" +
         "Joint velocity: " + transmission_interface::to_string(get_names(joint_velocity_)) +
         ", Actuator velocity: " + transmission_interface::to_string(get_names(actuator_velocity_)) + "\n" +
         "Joint effort: " + transmission_interface::to_string(get_names(joint_effort_)) +
         ", Actuator effort: " + transmission_interface::to_string(get_names(actuator_effort_));
}

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__SCARA_TRANSMISSION_HPP_