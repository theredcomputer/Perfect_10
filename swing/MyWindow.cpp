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

#include "MyWindow.h"

#define RIGHT_BOUNDARY 0.6
#define LEFT_BOUNDARY -0.6
#define PLATFORM_SPEED 0.75

MyWindow::MyWindow(): SimWindow() {
  mController = NULL;
 
  mTrans[0] = -1500.f;
  mZoom = 0.25f;
  mPersp = 60.0;
  mShowMarkers = false;
  mSpeed = PLATFORM_SPEED;
  mDumpImages = false;
  mTotalEffort = 0;
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  // Compute control force
  mController->computeTorques(mWorld->getSimFrames());
  // Apply control force to skeleton
  Eigen::VectorXd appliedTorque = mController->getTorques();
  mTotalEffort += appliedTorque.norm() * 0.001;
  mWorld->getSkeleton("fullbody1")->setForces(appliedTorque);
  // Update the moving platform
  movePlatforms();
  // Integrate forward for one time step
  mWorld->step();
  // Update the sensor at 60Hz; pixel values (RGBA) are stored in mInputSensor
  if (mWorld->getSimFrames() % 17 == 0)
    updateSensor();
}

void MyWindow::drawSkels() {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
    mWorld->getSkeleton(i)->draw(mRI);
  
  // display the frame count in 2D text
  char buff[64];
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%.4f", mTotalEffort);
#else
  std::snprintf(buff, sizeof(buff), "%.4f", mTotalEffort);
#endif
  std::string effort(buff);
  glColor3f(0.0, 0.0, 0.0);
  dart::gui::drawStringOnScreen(0.02f, 0.05f, effort);
  glEnable(GL_LIGHTING);
  
#ifdef _WIN32
  _snprintf(buff, sizeof(buff), "/ %d",
            mWorld->getRecording()->getNumFrames());
#else
  std::snprintf(buff, sizeof(buff), "/ %d",
                mWorld->getRecording()->getNumFrames());
#endif
  std::string elapsedTime(buff);
  glColor3f(0.0, 0.0, 0.0);
  dart::gui::drawStringOnScreen(0.15f, 0.02f, elapsedTime);
  glEnable(GL_LIGHTING);
  
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating) {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay) {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating) {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'd': // dump sensor images or not
      mDumpImages = !mDumpImages;
      break;
    case 'm':  // release
      mController->setState("RELEASE");
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void MyWindow::setController(Controller* _controller) {
  mController = _controller;
}

void MyWindow::movePlatforms() {
  dart::dynamics::SkeletonPtr platform = mWorld->getSkeleton("landing1");
  int index = platform->getDof("joint_pos_x")->getIndexInSkeleton();
  platform->setCommand(index, mSpeed);
  if (platform->getDof("joint_pos_x")->getPosition() > RIGHT_BOUNDARY)
    mSpeed = -PLATFORM_SPEED;
  if (platform->getDof("joint_pos_x")->getPosition() < LEFT_BOUNDARY)
    mSpeed = PLATFORM_SPEED;
}

void MyWindow::updateSensor() {
  if (mInputSensor.size() == 0)
    mInputSensor.resize(4 * mWinWidth * mWinHeight);
    
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(mPersp,
                 static_cast<double>(mWinWidth)/static_cast<double>(mWinHeight),
                 0.1, 10.0);
  gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  initGL();
  
  Eigen::Quaterniond q(0.6426, 0.3047, 0.6307, 0.3103);
  
  Eigen::Transform<double, 3, Eigen::Affine> t(q);
  glMultMatrixd(t.data());
  
  
  glScalef(0.51, 0.51, 0.51);
  glTranslatef(-1.68962, 0.00920964, -0.00350334);
  
  initLights();
  draw();
  
  glReadPixels(0, 0, mWinWidth, mWinHeight,
               GL_RGBA, GL_UNSIGNED_BYTE, &mInputSensor[0]);

  if (mDumpImages)
    dumpImages();
}

bool MyWindow::dumpImages() {
  if (mScreenshotTemp.size() == 0)
    mScreenshotTemp.resize(4 * mWinWidth * mWinHeight);

  static int count = 0;
  char fileBase[32] = "Image";
  char fileName[64];
  // png
#ifdef _WIN32
  _snprintf(fileName, sizeof(fileName), "%s%.4d.png", fileBase, count++);
#else
  std::snprintf(fileName, sizeof(fileName), "%s%.4d.png", fileBase, count++);
#endif
  
  for (int row = 0; row < mWinHeight; row++) {
    memcpy(&mScreenshotTemp[row * mWinWidth * 4],
           &mInputSensor[(mWinHeight - row - 1) * mWinWidth * 4], mWinWidth * 4);
  }
  
  unsigned result = lodepng::encode(fileName,
                                    mScreenshotTemp, mWinWidth, mWinHeight);
  
  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
    << lodepng_error_text(result) << std::endl;
    return false;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return true;
  }
}
