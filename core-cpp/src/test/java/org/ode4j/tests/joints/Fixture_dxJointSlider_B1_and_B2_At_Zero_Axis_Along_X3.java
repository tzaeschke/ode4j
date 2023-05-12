/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zäschke      *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests.joints;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddForce;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetSliderPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetSliderPositionRate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.Test;

//This file create unit test for some of the functions found in:
//ode/src/joinst/slider.cpp
public class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X3
  extends Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X {
  // Apply force on 1st body in the X direction also the Axis direction
  //
  //  X------->       X---------> Axis -->
  //  B1          =>  B1
  //  B2 F->             B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on 1st body in the inverse X direction
  //
  //  X------->           X---------> Axis -->
  //  B1           =>     B1
  //  B2  <-F          B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_of_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }