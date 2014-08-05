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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateSlider;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetSliderPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetSliderAxis;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointSlider;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

// Only body 2
// The body are positionned at (0, 0, 0), with no rotation
// The joint is a Slider Joint
// Axis is in the oppsite X axis
// Anchor at (0, 0, 0)
public class Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X extends TestSuperClass 
  {
    public Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X()
      {
        wId = dWorldCreate();

        bId2 = dBodyCreate (wId);
        dBodySetPosition (bId2, 0, 0, 0);

        jId   = dJointCreateSlider (wId, null);
        joint = (DxJointSlider) jId;


        dJointAttach (jId, null, bId2);

        dJointSetSliderAxis(jId, axis.get0(), axis.get1(), axis.get2());
      }

    //~Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

    DBody bId2;

    DSliderJoint jId;
    DxJointSlider joint;

    static final DVector3 axis = new DVector3(-1, 0, 0);

    static final double offset = 3.1;
  //};
//  final dVector3 Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X.axis = {-1, 0, 0};
//  final double    Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X.offset = (3.1);

  // Move 2nd body offset unit in the X direction
  //
  //  X------->       X--------->  <--- Axis
  //  B2          =>     B2
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void  test_dJointSetSliderAxisOffset_B2_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 2nd body offset unit in the opposite X direction
  //
  //  X------->          X--------->   <--- Axis
  //  B2          =>  B2
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderAxisOffset_B2_Minus_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, -offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }
  }