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
#include <stdlib.h>

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
  
  mMinX = 0.835;
  mMaxX = 0.845;
  mPrevSwingState = 1;
  mSwingState = 2;
  mXPos = 0.0;
  mPrevXPos = 0.0;
  mXThreshold = 0.01;
  xLandMax = 0.0;
  mCounter = 0;
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
  } else if (mState == "PRESWING") {
    preSwing();
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
    mState = "PRESWING";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "GRAB -> PRESWING" << std::endl;
    preSwing();
  }
}  


void Controller::preSwing() {
  // Need to get enough momentum to being swinging
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 2;
  
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mState = "SWING";
    std::cout << mCurrentFrame << ": " << "PRESWING -> SWING" << std::endl;
    preSwing();
  }


}

void Controller::swing() {
  // TODO: Need a better controller to increase the speed
  // and land at the right moment 
  mCounter++; 
  checkSwingState();
  setSwingPose();
  stablePD();

  /*
  // The default configuration
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 1;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  
  //mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 2;
  //mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 2;
  
  stablePD();
  */
  
  // double x_land, t;
  // computeLandingData(&x_land, &t);
  // if (x_land > xLandMax && t < 2) {
  //   xLandMax = x_land;
  //   std::cout << mCounter << " xLandMax: " << xLandMax << " t: " << t << std::endl;
  // }
  // //std::cout << "mSwingState: " << mSwingState << std::endl;
  // std::cout << "x_land: " << x_land << " t: " << t << std::endl;
  // //std::cout << "pelvis_pos: " << mSkel->getBodyNode("h_pelvis")->getCOM()[0] << std::endl;  
  // //std::cout << "mMinX: " << mMinX << " mMaxX: " << mMaxX << " mSwingState: " << mSwingState << std::endl;

  // double LA_x = mSkel->getCOMLinearAcceleration()[0];
  // double LA_y = mSkel->getCOMLinearAcceleration()[1];
  // double LA = sqrt(pow(LA_x,2) + pow(LA_y,2)); 
  // //std::cout << LA_y << " " << LA_x << " LA: " << LA << std::endl; 

  // int forward = mVisionProcessor->positionAtTime(t);
  // std::cout << "platform at t: " << forward << std::endl;

  dart::dynamics::BodyNode* hand = mSkel->getBodyNode("h_hand_right");
  dart::dynamics::BodyNode* ab = mSkel->getBodyNode("h_abdomen");
  Eigen::Vector3d handPos = hand->getCOM();
  Eigen::Vector3d abPos = ab->getCOM();
  double angle = atan2(abPos[1] - handPos[1], abPos[0] - handPos[0]) * 180 / 3.14159;
  std::cout << "angle: " << angle << std::endl;

  Eigen::Vector3d skelVelocity = mSkel->getCOMLinearVelocity();

  // TODO: Figure out the condition to release the bar
  // if (mVisionProcessor->mVelocitySign == 1 && (forward > 110 && forward < 120) && (x_land > 0.7 && x_land < 0.8)) {
  // if (x_land > 1.2 && t < 1.5) {
  if ((angle > -80 && angle < -70) && (skelVelocity[1] > 0) && (mVisionProcessor->mVelocitySign == 1 && mVisionProcessor->mPercentage > 0.5 && mVisionProcessor->mPercentage < 0.8)) {
    mState = "RELEASE";
    std::cout << mCurrentFrame << ": " << "SWING -> RELEASE" << std::endl;
  }
}

void Controller::release() {
  leftHandRelease();
  rightHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 2.5;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 2.5;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -2.5;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -2.5;
  //mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -1.5;
  //mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.5;
  //mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.5;
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


void Controller::checkSwingState() {
    
  //mPrevXPos = mXPos;
  mXPos = mSkel->getBodyNode("h_toe_left")->getCOM()[0];
  //mXVel = mXPos - mPrevXPos;
  
  switch (mSwingState) {
  //* // Basic swing
  case 1: // Slowing down on the right side
    if (mSkel->getCOMLinearAcceleration()[1] < 0 && mXPos < 0.835)
      mSwingState = 2;
    break;
  case 2: // Slowing down on the left side
    if (mSkel->getCOMLinearAcceleration()[1] < 0 && mXPos > 0.835)
      mSwingState = 1;
    break;
  //*/
  /* // Tap swing (hollow, arch, hollow through back)
  case 1: // Hollow to half way down
    if (mSkel->getCOMLinearAcceleration()[1] > 0 && mSkel->getCOMLinearVelocity()[0] > 0 && mXPos < 0.835)
      mSwingState = 2;
    break;
  case 2: // Arch the second half down
    if (mXPos > 0.835)
      mSwingState = 1;
    break;
  //*/
  default:
    std::cout << "Invalid state" << std::endl;
    break;
  }
}

void Controller::setSwingPose() {

  switch (mSwingState) {
  //* // Basic swing
  case 1:
    mDesiredDofs = mDefaultPose;
    mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 1;
    mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
    mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 1;
    mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
    mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
    mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
    mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0;
    mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0;
    mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.6;
    mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.6;
    break;
  case 2:
    mDesiredDofs = mDefaultPose;
    mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = -5.5;
    mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
    mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = -5.5;
    mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
    mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = -0.9;
    mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = -0.9;
    mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 1.3;
    mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 1.3;
    mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = 0.1;
    mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = 0.1;
    break;
  //*/
  /* // Tap swing
  case 1:
    mDesiredDofs = mDefaultPose;
    mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 1;
    mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
    mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 1;
    mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
    mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = -0.1;
    mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = -0.1;
    mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.6;
    mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.6;
    mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = 0.3;
    mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = 0.3;
    
    //mDesiredDofs[mSkel->getDof("j_scapula_left")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_scapula_right")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()] = 0;
    break;
  case 2:
    mDesiredDofs = mDefaultPose;
    mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 8;
    mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
    mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 8;
    mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
    mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 2.9;
    mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 2.9;
    mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = -0.6;
    mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = -0.6;
    mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.6;
    mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.6;
    
    //mDesiredDofs[mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_scapula_left")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_scapula_right")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()] = 0;
    //mDesiredDofs[mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()] = 0;
    break;
  //*/
  default:
    std::cout << "Invalid state" << std::endl;
  }
}


void Controller::computeLandingData(double* x_land, double* t) {
  // Compute the landing position and how time to land from release
  double y_0, v, theta, g, d;
  Eigen::Vector3d v_com;

  y_0 = mSkel->getBodyNode("h_pelvis")->getCOM()[1] + lowestHeelPosition;
  v_com = mSkel->getCOMLinearVelocity();
  v = sqrt(pow(v_com[0],2) + pow(v_com[1],2));
  theta = atan(v_com[1] / v_com[0]);
  std::cout << "v: " << v << " - theta: " << (theta*180/3.141592876) << std::endl;
  g = mSkel->getGravity()[1];
  d = (v * cos(theta) / g ) * (v * sin(theta) + sqrt(pow(v * sin(theta), 2) + 2 * g * y_0));
  *x_land = mSkel->getCOM()[0] + d;
  *t = *x_land / (v * cos(theta));
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

  int prev;

  for (int y = mHeight - 1; y >= 0; y--) {
      int x = mWidth / 2;

      int index = (y*mWidth*4) + x*4;
      int r = input->at(index);
      int g = input->at(index + 1);
      int b = input->at(index + 2);
      int a = input->at(index + 3);

      if (red == r && green == g && blue == b) {
        prev = mHighestRow;
        mHighestRow = (mHeight - y);

        if (mCloserBoundary == -1 || mHighestRow > mCloserBoundary)
          mCloserBoundary = mHighestRow;

        if (mFurtherBoundary == -1 || mHighestRow < mFurtherBoundary) 
          mFurtherBoundary = mHighestRow;

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
  mPixelsPerSecond = mPixelsPerFrame * 1000;
  
  int frameDifference = mLastPositions[99] - mLastPositions[98];
  if (frameDifference != 0)
    mVelocitySign = (frameDifference > 0) ? 1 : -1;

  // Add the position to the queue
  if (mLastSpeeds.size() == 1000) {
    mLastSpeeds.pop_front();
  }
  mLastSpeeds.push_back(mPixelsPerSecond);

  double speedAverages = 0;
  for (int i = 0; i < mLastSpeeds.size(); i++) {
    speedAverages += mLastSpeeds[i];
  }
  speedAverages /= mLastSpeeds.size();

  // Compute the position of the platform along this path
  double range = (mCloserBoundary - mFurtherBoundary);
  if (range != 0)
    mPercentage = (mHighestRow - mFurtherBoundary) / range;

  // std::cout << "Platform speed (pixels per frame): " << mPixelsPerFrame << std::endl;
  // std::cout << "Platform speed (pixels per second): " << mPixelsPerSecond << std::endl;
  // std::cout << "Average platform speed (pixels per second): " << speedAverages << std::endl;
  // std::cout << "Position: " << mHighestRow << std::endl;
  // std::cout << "Closest boundary: " << mCloserBoundary << std::endl;
  // std::cout << "Furthest boundary: " << mFurtherBoundary << std::endl;
}

int Controller::Vision::positionAtTime(int time) {
  int position = mHighestRow + (mVelocitySign * mPixelsPerSecond * time);

  while (position > mCloserBoundary || position < mFurtherBoundary) {
    // Moving closer
    if (position > mCloserBoundary) {
      int diff = position - mCloserBoundary;
      position -= 2*diff;
    }
    // Moving away
    else if (position < mFurtherBoundary) {
      int diff = mFurtherBoundary - position;
      position += 2*diff;
    }
  }

  return position;
}
