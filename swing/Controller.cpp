/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.h"
#include <iostream>
#include <math.h>

Controller::Controller(dart::dynamics::SkeletonPtr _skel, dart::constraint::ConstraintSolver* _constrSolver, double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;

  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the initial pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

  // Using SPD results in simple spring coefficients
  for (int i = 0; i < nDof; i++)
    mKp(i, i) = 400.0;
  for (int i = 0; i < nDof; i++)
    mKd(i, i) = 40.0;

  // Global dofs don't have PD control
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }

  // Make shoulders and elbows loose
  std::vector<int> dofIndex;
  dofIndex.push_back((mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_left")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_right")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 20.0;
    mKd(index, index) = 2.0;
  }

  // Make wrists even looser
  dofIndex.clear();
  dofIndex.push_back((mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_left_2")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_2")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 1.0;
    mKd(index, index) = 0.1;
  }

  for (int i = 0; i < nDof; i++)
    mSkel->getDof(i)->setDampingCoefficient(0.01);
  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  mTimer = 300;
  mState = "STAND";

  mVision = NULL;
  mVisionProcessor = new Controller::Vision(640, 480);
  lowestHeelPosition = 0.0;
}

Controller::~Controller() {
}

Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

double Controller::getTorque(int _index) {
  return mTorques[_index];
}

void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

void Controller::computeTorques(int _currentFrame) {
  // Process the vision sensor
  mVisionProcessor->processImage(mVision);

  // for (int i = 0; i < mSkel->getNumDofs(); i++) {
  //   dart::dynamics::DegreeOfFreedom *dof = mSkel->getDof(i);
  //   std::cout << dof->getName() << " = " << dof->getPosition() << std::endl;
  // }
  // std::cout << std::endl;
/*
  std::cout << "j_thigh_left_x" << " = " << mSkel->getDof("j_thigh_left_x")->getPosition() << std::endl;
  std::cout << "j_thigh_left_y" << " = " << mSkel->getDof("j_thigh_left_y")->getPosition() << std::endl;
  std::cout << "j_thigh_left_z" << " = " << mSkel->getDof("j_thigh_left_z")->getPosition() << std::endl;

  std::cout << "j_thigh_right_x" << " = " << mSkel->getDof("j_thigh_right_x")->getPosition() << std::endl;
  std::cout << "j_thigh_right_y" << " = " << mSkel->getDof("j_thigh_right_y")->getPosition() << std::endl;
  std::cout << "j_thigh_right_z" << " = " << mSkel->getDof("j_thigh_right_z")->getPosition() << std::endl;

  std::cout << "j_shin_left" << " = " << mSkel->getDof("j_shin_left")->getPosition() << std::endl;
  std::cout << "j_shin_right" << " = " << mSkel->getDof("j_shin_right")->getPosition() << std::endl;

  std::cout << "j_abdomen_1" << " = " << mSkel->getDof("j_abdomen_1")->getPosition() << std::endl;
  std::cout << "j_abdomen_2" << " = " << mSkel->getDof("j_abdomen_2")->getPosition() << std::endl;
  
  std::cout << "j_forearm_left" << " = " << mSkel->getDof("j_forearm_left")->getPosition() << std::endl;
  std::cout << "j_forearm_right" << " = " << mSkel->getDof("j_forearm_right")->getPosition() << std::endl;  

  std::cout << "j_bicep_left_x" << " = " << mSkel->getDof("j_bicep_left_x")->getPosition() << std::endl;
  std::cout << "j_bicep_left_y" << " = " << mSkel->getDof("j_bicep_left_y")->getPosition() << std::endl;
  std::cout << "j_bicep_left_z" << " = " << mSkel->getDof("j_bicep_left_z")->getPosition() << std::endl;

  std::cout << "j_bicep_right_x" << " = " << mSkel->getDof("j_bicep_right_x")->getPosition() << std::endl;
  std::cout << "j_bicep_right_y" << " = " << mSkel->getDof("j_bicep_right_y")->getPosition() << std::endl;
  std::cout << "j_bicep_right_z" << " = " << mSkel->getDof("j_bicep_right_z")->getPosition() << std::endl;
  
  std::cout << std::endl;
*/

  //std::cout << "Height of ground: " << mWorld->
  //std::cout << "Height of h_heel_left: " << mSkel->getBodyNode("h_heel_left")->getCOM()[1] << std::endl;
  mCurrentFrame = _currentFrame;
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB") {
    grab();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used. Do not remove this.
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

void Controller::checkContactState() {
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNodePtr body1 = cd->getContact(i).bodyNode1.lock().get();
    dart::dynamics::BodyNodePtr body2 = cd->getContact(i).bodyNode2.lock().get();
  
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_heel_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body2;
    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_heel_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body1;
  }
}

void Controller::stand() {
  // Change to default standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;

  // Switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "STAND -> CROUCH" << std::endl;
  }
}

void Controller::crouch() {

  if (mSkel->getBodyNode("h_heel_left")->getCOM()[1] < lowestHeelPosition) {
    lowestHeelPosition = mSkel->getBodyNode("h_heel_left")->getCOM()[1];
    std::cout << "Lowest heel position: " << lowestHeelPosition << std::endl;
  }

  // Change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;

  // After a while, lean forward
  if (mTimer < 200) {
    mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 1.0;
    mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 1.0;
  }

  stablePD();
  ankleStrategy();
  mTimer--;

  if (mTimer == 0) {
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "CROUCH -> JUMP" << std::endl;

  }
}

void Controller::jump() {

  // Change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -1.0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 1.0;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.5;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.5;
  
  stablePD();

  // Use Jacobian transpose to compute pushing torques
  Eigen::Vector3d vf(-1100.0, -2600, 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);
  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (mFootContact == NULL) {
    mState = "REACH";
    std::cout << mCurrentFrame << ": " << "JUMP -> REACH" << std::endl;
  }
}

void Controller::reach() {
  // Change to reaching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();

  checkContactState();
  if (mFootContact) { // If feet are in contact again, go back to JUMP and continue to push
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "REACH -> JUMP" << std::endl;
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "REACH -> GRAB" << std::endl;
  } else {
    mState = "REACH";
  }
}

void Controller::grab() {
  leftHandGrab();
  rightHandGrab();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mState = "SWING";
    std::cout << mCurrentFrame << ": " << "GRAB -> SWING" << std::endl;
  }
}  

void Controller::swing() {
  // TODO: Need a better controller to increase the speed
  // and land at the right moment
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  
  stablePD();


  // Compute the landing position and how time to land from release
  double y_0, v, theta, g, d, x_land, t;
  Eigen::Vector3d v_com;

  y_0 = mSkel->getBodyNode("h_pelvis")->getCOM()[1] + lowestHeelPosition;
  //y_0 = mSkel->getCOM()[1] + lowestHeelPosition;
  v_com = mSkel->getCOMLinearVelocity();
  v = sqrt(pow(v_com[0],2) + pow(v_com[1],2));
  theta = atan(v_com[1] / v_com[0]);
  std::cout << "theta: " << theta << std::endl;
  g = mSkel->getGravity()[1];
  d = (v * cos(theta) / g ) * (v * sin(theta) + sqrt(pow(v * sin(theta), 2) + 2 * g * y_0));
  x_land = mSkel->getCOM()[0] + d;
  t = x_land / (v * cos(theta));
  std::cout << "x_land: " << x_land << std::endl;
  std::cout << "t: " << t << std::endl;
  

  // TODO: Figure out the condition to release the bar
  if (false) {
    mState = "RELEASE";
    std::cout << mCurrentFrame << ": " << "SWING -> RELEASE" << std::endl;
  }
}

void Controller::release() {
  leftHandRelease();
  rightHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -1.5;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.5;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.5;
  stablePD();
}
  
void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
   if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }  
}

void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getLinearJacobian(_bodyNode, _offset);
  mTorques += jacobian.transpose() * _force;
}

void Controller::leftHandGrab() {  
  if (mLeftHandHold != NULL)
    return;
  checkContactState();
  if (mLeftHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftHandHold = hold;
}

void Controller::leftHandRelease() {
  if (mLeftHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftHandHold);
  mSkel->getBodyNode("h_hand_left")->setCollidable(true);
  mLeftHandHold = NULL;
}

void Controller::rightHandGrab() {  
  if (mRightHandHold != NULL)
    return;

  checkContactState();
  if (mRightHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightHandHold = hold;
}

void Controller::rightHandRelease() {
  if (mRightHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightHandHold);
  mSkel->getBodyNode("h_hand_right")->setCollidable(true);
  mRightHandHold = NULL;
}

void Controller::setState(std::string _state) {
  mState = _state;
}

dart::dynamics::SkeletonPtr Controller::getSkel() {
  return mSkel;
}

Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

Eigen::MatrixXd Controller::getKd() {
  return mKd;
}

void Controller::Vision::processImage(std::vector<unsigned char>* input) {
  if (input == NULL)
    return;

  // std::cout << "Total number of pixels: " << input->size() << std::endl;
  // std::cout << "Dimensions " << mWidth << " x " << mHeight << std::endl;

  int red = 92;
  int green = 82;
  int blue = 92;

  for (int y = mHeight - 1; y >= 0; y--) {
      int x = mWidth / 2;

      int index = (y*mWidth*4) + x*4;
      int r = input->at(index);
      int g = input->at(index + 1);
      int b = input->at(index + 2);
      int a = input->at(index + 3);

      if (red == r && green == g && blue == b) {
        int prev = mHighestRow;
        mHighestRow = (mHeight - y);
        break;
      }
  }

  // Add the position to the queue
  if (mLastPositions.size() == 100) {
    mLastPositions.pop_front();
  }
  mLastPositions.push_back(mHighestRow);

  // Compute the distance travelled in the last 100 frames
  int total_distance = 0;
  for (int i = 0; i < mLastPositions.size() - 1; i++) {
    total_distance += abs(mLastPositions[i+1] - mLastPositions[i]);
  }
  mPixelsPerFrame = total_distance / 99.0;
  mVelocitySign = (mLastPositions[9] - mLastPositions[8]) < 0 ? -1 : 1;

  std::cout << "Platform speed (pixels per frame): " << mPixelsPerFrame << std::endl;
}
