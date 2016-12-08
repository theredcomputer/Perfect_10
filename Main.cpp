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

#include <iostream>
#include <vector>

#include "dart/dart.h"
#include "MyWindow.h"

int main(int argc, char* argv[]) {
  // Create and initialize the world


  dart::simulation::WorldPtr myWorld
      = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/swing.skel");
  assert(myWorld != NULL);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  dart::constraint::ConstraintSolver* cs = myWorld->getConstraintSolver();
  cs->setCollisionDetector(new dart::collision::DARTCollisionDetector());

  // Set initial pose
  dart::dynamics::SkeletonPtr biped = myWorld->getSkeleton("fullbody1");
  biped->getDof("j_thigh_left_z")->setPosition(  0.15);
  biped->getDof("j_shin_left")->setPosition(    -0.40);
  biped->getDof("j_heel_left_1")->setPosition(   0.25);
  biped->getDof("j_thigh_right_z")->setPosition( 0.15);
  biped->getDof("j_shin_right")->setPosition(   -0.40);
  biped->getDof("j_heel_right_1")->setPosition(  0.25);
  
  // Set the moving platform is kinematically controlled;
  // its movement is not affected by external forces
  dart::dynamics::SkeletonPtr platform = myWorld->getSkeleton("landing1");
  platform->getJoint("joint")->setActuatorType(dart::dynamics::Joint::VELOCITY);

  // Create controller
  Controller* myController = new Controller(biped,
                                            myWorld->getConstraintSolver(),
                                            myWorld->getTimeStep());

  // Create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);
  window.setController(myController);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'d': dump sensor images into files on/off" << std::endl;
  std::cout << "'m': command the character to dismount" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Swing");
  glutMainLoop();

  return 0;
}

/* List of dof names for the character
 
j_pelvis_rot_x
j_pelvis_rot_y
j_pelvis_rot_z
j_pelvis_pos_x
j_pelvis_pos_y
j_pelvis_pos_z
j_thigh_left_z
j_thigh_left_y
j_thigh_left_x
j_shin_left
j_heel_left_1
j_heel_left_2
j_toe_left
j_thigh_right_z
j_thigh_right_y
j_thigh_right_x
j_shin_right
j_heel_right_1
j_heel_right_2
j_toe_right
j_abdomen_1
j_abdomen_2
j_spine
j_head_1
j_head_2
j_scapula_left
j_bicep_left_z
j_bicep_left_y
j_bicep_left_x
j_forearm_left
j_hand_left_1
j_hand_left_2
j_scapula_right
j_bicep_right_z
j_bicep_right_y
j_bicep_right_x
j_forearm_right
j_hand_right_1
j_hand_right_2
*/