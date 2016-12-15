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

#ifndef APPS_SWING_CONTROLLER_H_
#define APPS_SWING_CONTROLLER_H_

#include <vector>
#include <deque>

#include <Eigen/Dense>

#include "dart/dart.h"


class Controller {
private:
  class Vision;

public:
  Controller(dart::dynamics::SkeletonPtr _skel, dart::constraint::ConstraintSolver* _constrSolver,
             double _t);
  virtual ~Controller();

  Eigen::VectorXd getTorques();
  double getTorque(int _index);
  void setDesiredDof(int _index, double _val);
  void computeTorques(int _currentFrame);
  void setState(std::string _state);
  dart::dynamics::SkeletonPtr getSkel();
  Eigen::VectorXd getDesiredDofs();
  Eigen::MatrixXd getKp();
  Eigen::MatrixXd getKd();
  
  std::vector<unsigned char>* mVision;
  Vision *mVisionProcessor;
  double lowestHeelPosition;

protected:
  void stand();
  void crouch();
  void jump();
  void reach();
  void grab();
  void preSwing();
  void swing();
  void release();
  // Basic control building blocks
  void stablePD();
  void ankleStrategy();
  void virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset);
  // Contact states
  void checkContactState();
  void leftHandGrab();
  void rightHandGrab();
  void leftHandRelease();
  void rightHandRelease();

  void checkSwingState();
  void setSwingPose();
  void computeLandingData(double* x_land, double* t);

  dart::dynamics::SkeletonPtr mSkel;
  dart::constraint::ConstraintSolver* mConstraintSolver;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDefaultPose;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  double mTimestep;
  double mPreOffset;
  int mTimer;
  std::string mState;
  dart::constraint::JointConstraint* mLeftHandHold;
  dart::constraint::JointConstraint* mRightHandHold;
  dart::dynamics::BodyNode* mFootContact;
  dart::dynamics::BodyNodePtr mLeftHandContact;
  dart::dynamics::BodyNodePtr mRightHandContact;
  int mCurrentFrame;

  double mMinX;
  double mMaxX;
  double mNumStates;
  unsigned int mPrevSwingState;
  unsigned int mSwingState;
  double mXPos;
  double mPrevXPos;
  double mXThreshold;
  double xLandMax;
  unsigned int mCounter;

private:
  class Vision {
    public:
      Vision(int width, int height)
        : mWidth(width)
        , mHeight(height)
        , mHighestRow(0)
        , mPixelsPerFrame(0) 
        , mLastPositions(100)
        , mLastSpeeds(100)
        , mCloserBoundary(-1)
        , mFurtherBoundary(-1)
      {

      }

      void processImage(std::vector<unsigned char>* vision);
      int positionAtTime(int time);

    
      int mHighestRow;
      double mPixelsPerFrame;
      double mPixelsPerSecond;
      std::deque<double> mLastSpeeds;
      int mVelocitySign;
      std::deque<int> mLastPositions;

      int mCloserBoundary;
      int mFurtherBoundary;
      double mPercentage;

      int mWidth;
      int mHeight;
  };
};

#endif  // APPS_SWING_CONTROLLER_H_
