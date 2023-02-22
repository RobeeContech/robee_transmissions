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
#include "robee_transmissions/scara_transmission.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using testing::DoubleNear;
using transmission_interface::ActuatorHandle;
using robee_transmission_interface::ScaraTransmission;
using transmission_interface::Exception;
using transmission_interface::JointHandle;
// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  const std::vector<double> reduction_good = {1.0, 1.0};
  const std::vector<double> reduction_bad1 = {0.0, 0.0};
  const std::vector<double> reduction_bad2 = {1.0, 0.0};
  const std::vector<double> reduction_bad3 = {0.0, 1.0};
  const std::vector<double> offset_good = {1.0, 1.0};
  const std::vector<double> zero_offset = {0, 0};

  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(ScaraTransmission(reduction_bad1,zero_offset), Exception);
  EXPECT_THROW(ScaraTransmission(reduction_bad2,zero_offset), Exception);
  EXPECT_THROW(ScaraTransmission(reduction_bad3,zero_offset), Exception);


  EXPECT_THROW(ScaraTransmission(reduction_bad1, offset_good), Exception);
  EXPECT_THROW(ScaraTransmission(reduction_bad2, offset_good), Exception);
  EXPECT_THROW(ScaraTransmission(reduction_bad3, offset_good), Exception);

  // Invalid instance creation: Wrong parameter sizes
  const std::vector<double> reduction_bad_size = {1.0};
  const std::vector<double> & offset_bad_size = reduction_bad_size;
  EXPECT_THROW(ScaraTransmission(reduction_bad_size,zero_offset), Exception);
  EXPECT_THROW(ScaraTransmission(reduction_good, offset_bad_size), Exception);

  // Valid instance creation
  EXPECT_NO_THROW(ScaraTransmission(reduction_good,zero_offset));
  EXPECT_NO_THROW(ScaraTransmission(reduction_good, offset_good));
}




TEST(PreconditionsTest, AccessorValidation)
{
  std::vector<double> jnt_reduction = {4.0, -4.0};
  std::vector<double> jnt_offset = {1.0, -1.0};

  ScaraTransmission trans(jnt_reduction, jnt_offset);

  EXPECT_EQ(2u, trans.num_actuators());
  EXPECT_EQ(2u, trans.num_joints());
  EXPECT_EQ(4.0, trans.get_joint_reduction()[0]);
  EXPECT_EQ(-4.0, trans.get_joint_reduction()[1]);
  EXPECT_EQ(1.0, trans.get_joint_offset()[0]);
  EXPECT_EQ(-1.0, trans.get_joint_offset()[1]);
}

void testConfigureWithBadHandles(std::string interface_name)
{
  ScaraTransmission trans({1.0, 1.0}, {0.0, 0.0});
  double dummy;

  auto a1_handle = ActuatorHandle("act1", interface_name, &dummy);
  auto a2_handle = ActuatorHandle("act2", interface_name, &dummy);
  auto a3_handle = ActuatorHandle("act3", interface_name, &dummy);
  auto j1_handle = JointHandle("joint1", interface_name, &dummy);
  auto j2_handle = JointHandle("joint2", interface_name, &dummy);
  auto j3_handle = JointHandle("joint3", interface_name, &dummy);
  auto invalid_a1_handle = ActuatorHandle("act1", interface_name, nullptr);
  auto invalid_j1_handle = JointHandle("joint1", interface_name, nullptr);

  EXPECT_THROW(trans.configure({}, {}), Exception);
  EXPECT_THROW(trans.configure({j1_handle}, {}), Exception);
  EXPECT_THROW(trans.configure({j1_handle}, {a1_handle}), Exception);
  EXPECT_THROW(trans.configure({}, {a1_handle}), Exception);
  EXPECT_THROW(trans.configure({j1_handle, j2_handle}, {a1_handle}), Exception);
  EXPECT_THROW(trans.configure({j1_handle}, {a1_handle, a2_handle}), Exception);
  EXPECT_THROW(
    trans.configure({j1_handle, j2_handle, j3_handle}, {a1_handle, a2_handle}), Exception);
  EXPECT_THROW(
    trans.configure({j1_handle, j2_handle}, {a1_handle, a2_handle, a3_handle}), Exception);
  EXPECT_THROW(
    trans.configure({j1_handle, j2_handle, j3_handle}, {a1_handle, a2_handle, a3_handle}),
    Exception);
  EXPECT_THROW(trans.configure({j1_handle, j2_handle}, {invalid_a1_handle, a2_handle}), Exception);
  EXPECT_THROW(trans.configure({invalid_j1_handle, j2_handle}, {a1_handle, a2_handle}), Exception);
  EXPECT_THROW(
    trans.configure({invalid_j1_handle, j2_handle}, {invalid_a1_handle, a2_handle}), Exception);
}

TEST(ConfigureTest, FailsWithBadHandles)
{
  testConfigureWithBadHandles(HW_IF_POSITION);
  testConfigureWithBadHandles(HW_IF_VELOCITY);
  testConfigureWithBadHandles(HW_IF_EFFORT);
}

class TransmissionSetup : public ::testing::Test
{
protected:
  // Input/output transmission data
  double a_val[2];
  double j_val[2];
  std::vector<double *> a_vec = {&a_val[0], &a_val[1]};
  std::vector<double *> j_vec = {&j_val[0], &j_val[1]};
};


class SelfTransmissionSetup : public ::testing::Test
{
  protected:
    // Input/output transmission data
    double val[2];
    std::vector<double *> a_vec = {&val[0], &val[1]};
    std::vector<double *> j_vec = {&val[0], &val[1]};
};

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup
{
protected:


 void init_ScaraTransmission(ScaraTransmission & trans,const std::string & interface_name){
    auto a1_handle = ActuatorHandle("act1", interface_name, a_vec[0]);
    auto a2_handle = ActuatorHandle("act2", interface_name, a_vec[1]);
    auto joint1_handle = JointHandle("joint1", interface_name, j_vec[0]);
    auto joint2_handle = JointHandle("joint2", interface_name, j_vec[1]);
    trans.configure({joint1_handle, joint2_handle}, {a1_handle, a2_handle});
 }

  const double EXTREMAL_VALUE = 1337.1337;
  /// \param trans Transmission instance.
  /// \param ref_val Reference value that will be transformed with the respective forward
  /// and inverse transmission transformations.
  /// \param interface_name The name of the interface to test, position, velocity, etc.
  void testIdentityMap(
    ScaraTransmission & trans, const std::vector<double> & ref_val,
    const std::string & interface_name)
  {
    // set actuator values to reference
    a_val[0] = ref_val[0];
    a_val[1] = ref_val[1];
    // create handles and configure
    init_ScaraTransmission(trans,interface_name);

    // actuator->joint->actuator roundtrip
    // but we also set actuator values to an extremal value
    // to ensure joint_to_actuator is not a no-op
    trans.actuator_to_joint(); 
    a_val[0] = a_val[1] = EXTREMAL_VALUE;
    trans.joint_to_actuator();
    EXPECT_THAT(ref_val[0], DoubleNear(a_val[0], EPS));
    EXPECT_THAT(ref_val[1], DoubleNear(a_val[1], EPS));
  }

   /// \param trans Transmission instance.
  /// \param ref_val Reference value that will be transformed with the respective inverse
  /// and forward transmission transformations.
  /// \param interface_name The name of the interface to test, position, velocity, etc.
  void testReverseIdentityMap(
    ScaraTransmission & trans, const std::vector<double> & ref_val,
    const std::string & interface_name)
  {
    // set joint values to reference
    j_val[0] = ref_val[0];
    j_val[1] = ref_val[1];
    // create handles and configure
    init_ScaraTransmission(trans,interface_name);

    // joint->actuator->joint roundtrip
    // but we also set joint values to an extremal value
    // to ensure joint_to_actuator is not a no-op
    trans.joint_to_actuator();
    j_val[0] = j_val[1] = EXTREMAL_VALUE;
    trans.actuator_to_joint();
    EXPECT_THAT(ref_val[0], DoubleNear(j_val[0], EPS));
    EXPECT_THAT(ref_val[1], DoubleNear(j_val[1], EPS));
  }

  void test_IdentityMap_Case(ScaraTransmission& trans, vector<double> i)
  {
      testIdentityMap(trans, i, HW_IF_POSITION);
      testIdentityMap(trans, i, HW_IF_VELOCITY);
    //   testIdentityMap(trans, i, HW_IF_EFFORT);
       testReverseIdentityMap(trans, i, HW_IF_POSITION);
       testReverseIdentityMap(trans, i, HW_IF_VELOCITY);
    //   testReverseIdentityMap(trans, i, HW_IF_EFFORT);
  }

  void test_IdentityMap_Case(vector<double> r,vector<double> o, vector<double> i)
  {
      ScaraTransmission trans(r, o);
      test_IdentityMap_Case(trans,i);
  }

  void test_joint_to_actuator(vector<double> reduction,vector<double> off, vector<double> j, vector<double> ac){
    ScaraTransmission trans(reduction, off);
    init_ScaraTransmission(trans,HW_IF_POSITION);
    a_val[0] = a_val[1] = EXTREMAL_VALUE;
    j_val[0] = j[0];
    j_val[1] = j[1];
    trans.joint_to_actuator();
    EXPECT_THAT(ac[0], DoubleNear(a_val[0], EPS));
    EXPECT_THAT(ac[1], DoubleNear(a_val[1], EPS));
  }


    void test_actuator_to_joint(vector<double> reduction,vector<double> off, vector<double> j, vector<double> ac){
    ScaraTransmission trans(reduction, off);
    init_ScaraTransmission(trans,HW_IF_POSITION);
    j_val[0] = j_val[1] = EXTREMAL_VALUE;
    a_val[0] = ac[0];
    a_val[1] = ac[1];
    trans.actuator_to_joint(); 
    EXPECT_THAT(j[0], DoubleNear(j_val[0], EPS));
    EXPECT_THAT(j[1], DoubleNear(j_val[1], EPS));
  }



  // Generate a set of transmission instances
  // with random combinations of actuator/joint reduction and joint offset.
  static std::vector<ScaraTransmission> createTestInstances(
    const vector<ScaraTransmission>::size_type size)
  {
    std::vector<ScaraTransmission> out;
    out.reserve(size);
    // NOTE: Magic value
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);

    while (out.size() < size)
    {
      try
      {
        ScaraTransmission trans(
          randomVector(2, rand_gen), randomVector(2, rand_gen));
        out.push_back(trans);
      }
      catch (const Exception &)
      {
        // NOTE: If by chance a perfect zero is produced by the random number generator,
        // construction will fail
        // We swallow the exception and move on to prevent a test crash.
      }
    }
    return out;
  }
};



/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxSelfTransferTest : public SelfTransmissionSetup
{
protected:


 void init_ScaraTransmission(ScaraTransmission & trans,const std::string & interface_name){
    auto a1_handle = ActuatorHandle("act1", interface_name, a_vec[0]);
    auto a2_handle = ActuatorHandle("act2", interface_name, a_vec[1]);
    auto joint1_handle = JointHandle("joint1", interface_name, j_vec[0]);
    auto joint2_handle = JointHandle("joint2", interface_name, j_vec[1]);
    trans.configure({joint1_handle, joint2_handle}, {a1_handle, a2_handle});
 }

  const double EXTREMAL_VALUE = 1337.1337;
  /// \param trans Transmission instance.
  /// \param ref_val Reference value that will be transformed with the respective forward
  /// and inverse transmission transformations.
  /// \param interface_name The name of the interface to test, position, velocity, etc.
  void testIdentityMap(
    ScaraTransmission & trans, const std::vector<double> & ref_val,
    const std::string & interface_name)
  {
    // set actuator values to reference
    val[0] = ref_val[0];
    val[1] = ref_val[1];
    // create handles and configure
    init_ScaraTransmission(trans,interface_name);

    // actuator->joint->actuator roundtrip
    trans.actuator_to_joint(); 
    trans.joint_to_actuator();
    EXPECT_THAT(ref_val[0], DoubleNear(val[0], EPS));
    EXPECT_THAT(ref_val[1], DoubleNear(val[1], EPS));
  }

   /// \param trans Transmission instance.
  /// \param ref_val Reference value that will be transformed with the respective inverse
  /// and forward transmission transformations.
  /// \param interface_name The name of the interface to test, position, velocity, etc.
  void testReverseIdentityMap(
    ScaraTransmission & trans, const std::vector<double> & ref_val,
    const std::string & interface_name)
  {
    // set joint values to reference
    val[0] = ref_val[0];
    val[1] = ref_val[1];
    // create handles and configure
    init_ScaraTransmission(trans,interface_name);

    // joint->actuator->joint roundtrip
    trans.joint_to_actuator();
    trans.actuator_to_joint();
    EXPECT_THAT(ref_val[0], DoubleNear(val[0], EPS));
    EXPECT_THAT(ref_val[1], DoubleNear(val[1], EPS));
  }

  void test_IdentityMap_Case(ScaraTransmission& trans, vector<double> i)
  {
      testIdentityMap(trans, i, HW_IF_POSITION);
      testIdentityMap(trans, i, HW_IF_VELOCITY);
    //   testIdentityMap(trans, i, HW_IF_EFFORT);
       testReverseIdentityMap(trans, i, HW_IF_POSITION);
       testReverseIdentityMap(trans, i, HW_IF_VELOCITY);
    //   testReverseIdentityMap(trans, i, HW_IF_EFFORT);
  }

  void test_IdentityMap_Case(vector<double> r,vector<double> o, vector<double> i)
  {
      ScaraTransmission trans(r, o);
      test_IdentityMap_Case(trans,i);
  }

  void test_joint_to_actuator(vector<double> reduction,vector<double> off, vector<double> j, vector<double> ac){
    ScaraTransmission trans(reduction, off);
    init_ScaraTransmission(trans,HW_IF_POSITION);
    val[0] = j[0];
    val[1] = j[1];
    trans.joint_to_actuator();
    EXPECT_THAT(ac[0], DoubleNear(val[0], EPS));
    EXPECT_THAT(ac[1], DoubleNear(val[1], EPS));
  }


    void test_actuator_to_joint(vector<double> reduction,vector<double> off, vector<double> j, vector<double> ac){
    ScaraTransmission trans(reduction, off);
    init_ScaraTransmission(trans,HW_IF_POSITION);
    val[0] = ac[0];
    val[1] = ac[1];
    trans.actuator_to_joint(); 
    EXPECT_THAT(j[0], DoubleNear(val[0], EPS)) << "j0 = " << j[0] << ", val[0] = " << val[0] ;
    EXPECT_THAT(j[1], DoubleNear(val[1], EPS));
  }



  // Generate a set of transmission instances
  // with random combinations of actuator/joint reduction and joint offset.
  static std::vector<ScaraTransmission> createTestInstances(
    const vector<ScaraTransmission>::size_type size)
  {
    std::vector<ScaraTransmission> out;
    out.reserve(size);
    // NOTE: Magic value
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);

    while (out.size() < size)
    {
      try
      {
        ScaraTransmission trans(
          randomVector(2, rand_gen), randomVector(2, rand_gen));
        out.push_back(trans);
      }
      catch (const Exception &)
      {
        // NOTE: If by chance a perfect zero is produced by the random number generator,
        // construction will fail
        // We swallow the exception and move on to prevent a test crash.
      }
    }
    return out;
  }
};



TEST_F(BlackBoxTest, ReverseIdentityMap_Case1){
    test_IdentityMap_Case({1,1}, {0,0},{3,5});
}


TEST_F(BlackBoxTest, Robee_test_joint_to_actuator){
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {0,0}, {0,0});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {-12.5,0}, {-M_PI,0}); 
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {12.5,0}, {M_PI,0});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {0,M_PI}, {M_PI,M_PI});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {0,-M_PI}, {-M_PI,-M_PI});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {25,-M_PI}, {M_PI,-M_PI});
}

TEST_F(BlackBoxTest, Robee_test_actuator_to_joint){
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {0,0}, {0,0});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {-12.5,0}, {-M_PI,0}); 
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {12.5,0}, {M_PI,0});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {0,M_PI}, {M_PI,M_PI});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {0,-M_PI}, {-M_PI,-M_PI});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {25,-M_PI}, {M_PI,-M_PI});
}

TEST_F(BlackBoxTest, IdentityMap)
{
  // Transmission instances
  // NOTE: Magic value
  auto transmission_test_instances = createTestInstances(100);

  // Test different transmission configurations...
  for (auto && transmission : transmission_test_instances)
  {
    // ...and for each transmission, different input values
    // NOTE: Magic value
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);
    // NOTE: Magic value
    const unsigned int input_value_trials = 100;
    for (unsigned int i = 0; i < input_value_trials; ++i)
    {
      
      vector<double> input_value = randomVector(2, rand_gen);
      auto off = transmission.get_joint_offset();
      auto  rd = transmission.get_joint_reduction();
      test_IdentityMap_Case(transmission, input_value);
    }
  }
}




TEST_F(BlackBoxSelfTransferTest, ReverseIdentityMap_Case1){
    test_IdentityMap_Case({1,1}, {0,0},{3,5});
}


TEST_F(BlackBoxSelfTransferTest, Robee_test_joint_to_actuator){
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {0,0}, {0,0});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {-12.5,0}, {-M_PI,0}); 
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {12.5,0}, {M_PI,0});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {0,M_PI}, {M_PI,M_PI});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {0,-M_PI}, {-M_PI,-M_PI});
    test_joint_to_actuator({M_PI/12.5,1},{0,0}, {25,-M_PI}, {M_PI,-M_PI});
}

TEST_F(BlackBoxSelfTransferTest, Robee_test_actuator_to_joint){
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {0,0}, {0,0});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {-12.5,0}, {-M_PI,0}); 
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {12.5,0}, {M_PI,0});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {0,M_PI}, {M_PI,M_PI});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {0,-M_PI}, {-M_PI,-M_PI});
    test_actuator_to_joint({M_PI/12.5,1},{0,0}, {25,-M_PI}, {M_PI,-M_PI});
}

//values in deg and mm  Az,Arz = (0,16)   =>  Jz,Jrz = (0,16)
TEST_F(BlackBoxSelfTransferTest, Robee_test_actuator_to_joint_homeing_pos) {
    test_actuator_to_joint({M_PI/0.0125,1},{0.2793 / (M_PI/0.0125),0}, {0,0.2793}, {0,0.2793}); //values in rad and m
}

//values in deg and mm Az,Arz = (16,32)   =>  Jz,Jrz = (0,32)
TEST_F(BlackBoxSelfTransferTest, Robee_test_actuator_to_joint_z_rot_only) {
    test_actuator_to_joint({M_PI/0.0125,1},{0.2793 / (M_PI/0.0125),0},{0,0.5585}, {0.2793,0.5585}); //values in rad and m
}

//values in deg and mm Az,Arz = ((1/25)*360,16)   =>  Jz,Jrz = (1,16)
TEST_F(BlackBoxSelfTransferTest, Robee_test_actuator_to_joint_z_only) {
    test_actuator_to_joint({M_PI/0.0125,1},{0.2793 / (M_PI/0.0125),0}, {0.001,0.2793}, {0.2513,0.2793});//values in rad and m
}

//values in deg and mm Az,Arz = ((1/25)*360+16,32)   =>  Jz,Jrz = (1,32)
TEST_F(BlackBoxSelfTransferTest, Robee_test_actuator_to_joint_z_and_z_rot) {
    test_actuator_to_joint({M_PI/0.0125,1},{0.2793 / (M_PI/0.0125),0}, {0.001,0.5585}, {0.5306,0.5585});//values in rad and m
}


TEST_F(BlackBoxSelfTransferTest, IdentityMap)
{
  // Transmission instances
  // NOTE: Magic value
  auto transmission_test_instances = createTestInstances(100);

  // Test different transmission configurations...
  for (auto && transmission : transmission_test_instances)
  {
    // ...and for each transmission, different input values
    // NOTE: Magic value
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);
    // NOTE: Magic value
    const unsigned int input_value_trials = 100;
    for (unsigned int i = 0; i < input_value_trials; ++i)
    {
      
      vector<double> input_value = randomVector(2, rand_gen);
      auto off = transmission.get_joint_offset();
      auto  rd = transmission.get_joint_reduction();
      test_IdentityMap_Case(transmission, input_value);
    }
  }
}
