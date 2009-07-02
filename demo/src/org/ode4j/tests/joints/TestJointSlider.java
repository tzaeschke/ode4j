/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests.joints;


import org.junit.AfterClass;
import org.junit.Test;
import org.junit.experimental.runners.Enclosed;
import org.junit.runner.RunWith;

import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.ode4j.ode.internal.joints.DxJointSlider;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;


//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/slider.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

//#include <UnitTest++.h>
//#include <ode/ode.h>
//
//#include "../../ode/src/joints/slider.h"

//SUITE (TestdxJointSlider)
//{

@RunWith(Enclosed.class)
public class TestJointSlider {
	
    public static class dxJointSlider_Fixture_1
    {
        public dxJointSlider_Fixture_1()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, -1, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 1, 0);

            jId   = dJointCreateSlider (wId, null);
            joint = (DxJointSlider) jId;


            dJointAttach (jId, bId1, bId2);
        }

    	@AfterClass
    	public static void DESTRUCTOR() {
    		dWorldDestroy(wId);
    	}

    	static DWorld wId;

        DBody bId1;
        DBody bId2;


        DSliderJoint jId;
        DxJointSlider joint;
//    };

    //TEST_FIXTURE (dxJointSlider_Fixture_1, 
        @Test public void test_dJointSetSlider()
    {
        // the 2 bodies are align
        dJointSetSliderAxis (jId, 1, 0, 0);
        CHECK_CLOSE (joint.qrel.get0(), 1.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get1(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get2(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get3(), 0.0, 1e-4);

        DMatrix3 R = new DMatrix3();
        // Rotate 2nd body 90deg around X
        dBodySetPosition (bId2, 0, 0, 1);
        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId2, R);

        dJointSetSliderAxis (jId, 1, 0 ,0);
        CHECK_CLOSE (joint.qrel.get0(), 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint.qrel.get1(), 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint.qrel.get2(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get3(), 0.0, 1e-4);


        // Rotate 2nd body -90deg around X
        dBodySetPosition (bId2, 0, 0, -1);
        dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
        dBodySetRotation (bId2, R);

        dJointSetSliderAxis (jId, 1, 0 ,0);
        CHECK_CLOSE (joint.qrel.get0(), 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint.qrel.get1(), -0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint.qrel.get2(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get3(), 0.0, 1e-4);


        // Rotate 2nd body 90deg around Z
        dBodySetPosition (bId2, 0, 1, 0);
        dRFromAxisAndAngle (R, 0, 0, 1, M_PI/2.0);
        dBodySetRotation (bId2, R);

        dJointSetSliderAxis (jId, 1, 0 ,0);
        CHECK_CLOSE (joint.qrel.get0(), 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint.qrel.get1(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get2(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get3(), 0.70710678118654757, 1e-4);


        // Rotate 2nd body 45deg around Y
        dBodySetPosition (bId2, 0, 1, 0);
        dRFromAxisAndAngle (R, 0, 1, 0, M_PI/4.0);
        dBodySetRotation (bId2, R);

        dJointSetSliderAxis (jId, 1, 0 ,0);
        CHECK_CLOSE (joint.qrel.get0(), 0.92387953251128674, 1e-4);
        CHECK_CLOSE (joint.qrel.get1(), 0.0, 1e-4);
        CHECK_CLOSE (joint.qrel.get2(), 0.38268343236508984, 1e-4);
        CHECK_CLOSE (joint.qrel.get3(), 0.0, 1e-4);

        // Rotate in a strange manner
        // Both bodies at origin
        dRFromEulerAngles (R, (0.23), (3.1), (-0.73));
        dBodySetPosition (bId1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        dRFromEulerAngles (R, (-0.57), (1.49), (0.81));
        dBodySetPosition (bId2, 0, 0, 0);
        dBodySetRotation (bId2, R);

        dJointSetSliderAxis (jId, 1, 0 ,0);
        CHECK_CLOSE (joint.qrel.get0(), -0.25526036263124319, 1e-4);
        CHECK_CLOSE (joint.qrel.get1(),  0.28434861188441968, 1e-4);
        CHECK_CLOSE (joint.qrel.get2(), -0.65308047160141625, 1e-4);
        CHECK_CLOSE (joint.qrel.get3(),  0.65381489108282143, 1e-4);
    }
        }


  // The 2 bodies are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X
  {
    public Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X()
      {
        wId = dWorldCreate();

        bId1 = dBodyCreate (wId);
        dBodySetPosition (bId1, 0, 0, 0);

        bId2 = dBodyCreate (wId);
        dBodySetPosition (bId2, 0, 0, 0);

        jId   = dJointCreateSlider (wId, null);
        joint = (DxJointSlider) jId;


        dJointAttach (jId, bId1, bId2);

        dJointSetSliderAxis(jId, axis.get0(), axis.get1(), axis.get2());
      }

    //~Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

    DBody bId1;
    DBody bId2;


    DSliderJoint jId;
    DxJointSlider joint;

    static final DVector3 axis = new DVector3(1, 0, 0);

    static final double offset = 3.1;
//  };
//  final dVector3 Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X.axis = {1, 0, 0};
//  final double    Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X.offset = (3.1);

  // Move 1st body offset unit in the X direction
  //
  //  X------->       X---------> Axis -->
  //  B1          =>     B1
  //  B2              B2
  //
  // Start with a Offset of offset unit
  //
  //  X------->       X---------> Axis -->
  //     B1       =>  B1
  //  B2              B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
                @Test public void test_dJointSetSliderAxisOffset_B1_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 1st body offset unit in the opposite X direction
  //
  //  X------->          X---------> Axis -->
  //  B1          =>  B1
  //  B2                 B2
  //
  // Start with a Offset of -offset unit
  //
  //      X------->      X---------> Axis -->
  //  B1            =>   B1
  //      B2             B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
                @Test public void  test_dJointSetSliderAxisOffset_B1_Minus_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, -offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 2nd body offset unit in the X direction
  //
  //  X------->       X---------> Axis -->
  //  B1          =>  B1
  //  B2                 B2
  //
  // Start with a Offset of offset unit
  //
  //  X------->       X---------> Axis -->
  //  B1          =>  B1
  //     B2           B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
                @Test public void   test_dJointSetSliderAxisOffset_B2_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 2nd body offset unit in the opposite X direction
  //
  //  X------->          X---------> Axis -->
  //  B1          =>     B1
  //  B2              B2
  //
  // Start with a Offset of -offset unit
  //
  //     X------->    X---------> Axis -->
  //     B1       =>  B1
  //  B2              B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
  @Test public void  test_dJointSetSliderAxisOffset_B2_Minus_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, -offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }
  }


  // The 2 bodies are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is the opposite of the X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X
  {
    public Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X()
      {
        wId = dWorldCreate();

        bId1 = dBodyCreate (wId);
        dBodySetPosition (bId1, 0, 0, 0);

        bId2 = dBodyCreate (wId);
        dBodySetPosition (bId2, 0, 0, 0);

        jId   = dJointCreateSlider (wId, null);
        joint = (DxJointSlider) jId;


        dJointAttach (jId, bId1, bId2);


        dJointSetSliderAxis(jId, axis.get0(), axis.get1(), axis.get2());
      }

    //~Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

    DBody bId1;
    DBody bId2;


    DSliderJoint jId;
    DxJointSlider joint;

    static final DVector3 axis = new DVector3(-1, 0, 0);
    static final double offset = 3.1;
//  };
//  final dVector3 Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X.axis = {-1, 0, 0};
//  final double    Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X.offset = (3.1);

  // Move 1st body offset unit in the X direction
  //
  //  X------->       X--------->   <-- Axis
  //  B1          =>     B1
  //  B2              B2
  //
  // Start with a Offset of offset unit
  //
  //  X------->       X--------->  <-- Axis
  //     B1       =>  B1
  //  B2              B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderAxisOffset_B1_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 1st body offset unit in the opposite X direction
  //
  //  X------->          X--------->  <-- Axis
  //  B1          =>  B1
  //  B2                 B2
  //
  // Start with a Offset of offset unit
  //
  //     X------->  X--------->      <-- Axis
  //  B1       =>   B1
  //     B2         B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		  @Test public void test_dJointSetSliderAxisOffset_B1_Minus_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, -offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 2nd body offset unit in the X direction
  //
  //  X------->       X--------->  <-- Axis
  //  B1          =>  B1
  //  B2                 B2
  //
  // Start with a Offset of offset unit
  //
  //  X------->       X--------->  <-- Axis
  //  B1          =>  B1
  //     B2           B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		  @Test public void test_dJointSetSliderAxisOffset_B2_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 2nd body offset unit in the opposite X direction
  //
  //  X------->          X--------->  <-- Axis
  //  B1          =>     B1
  //  B2              B2
  //
  // Start with a Offset of -offset unit
  //
  //     X------->    X--------->     <-- Axis
  //     B1       =>  B1
  //  B2              B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		  @Test public void test_dJointSetSliderAxisOffset_B2_Minus_3Unit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, -offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }
  }

  // Only body 1
  // The body are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X
  {
    public Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X()
      {
        wId = dWorldCreate();

        bId1 = dBodyCreate (wId);
        dBodySetPosition (bId1, 0, 0, 0);

        jId   = dJointCreateSlider (wId, null);
        joint = (DxJointSlider) jId;


        dJointAttach (jId, bId1, null);

        dJointSetSliderAxis(jId, axis.get0(), axis.get1(), axis.get2());
      }

    //~Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

    DBody bId1;

    DSliderJoint jId;
    DxJointSlider joint;

    static final DVector3 axis = new DVector3(1, 0, 0);

    static final double offset = 3.1;
//  };
//  final dVector3 Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X.axis = {1, 0, 0};
//  final double    Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X.offset = (3.1);

  // Move 1st body offset unit in the X direction
  //
  //  X------->       X---------> Axis -->
  //  B1          =>     B1
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderAxisOffset_B1_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 1st body offset unit in the opposite X direction
  //
  //  X------->          X---------> Axis -->
  //  B1          =>  B1
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderAxisOffset_B1_Minus_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, -offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }
  }
  
  // Only body 1
  // The body are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is in the oppsite X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X
  {
    public Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X()
      {
        wId = dWorldCreate();

        bId1 = dBodyCreate (wId);
        dBodySetPosition (bId1, 0, 0, 0);

        jId   = dJointCreateSlider (wId, null);
        joint = (DxJointSlider) jId;


        dJointAttach (jId, bId1, null);

        dJointSetSliderAxis(jId, axis.get0(), axis.get1(), axis.get2());
      }

    //~Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

    DBody bId1;

    DSliderJoint jId;
    DxJointSlider joint;

    static final DVector3 axis = new DVector3(-1, 0, 0);

    static final double offset = 3.1;
//  };
//  final dVector3 Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X.axis = {-1, 0, 0};
//  final double    Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X.offset = (3.1);

  // Move 1st body offset unit in the X direction
  //
  //  X------->       X--------->  <--- Axis
  //  B1          =>     B1
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X,
  @Test public void  test_dJointSetSliderAxisOffset_B1_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 1st body offset unit in the opposite X direction
  //
  //  X------->          X--------->   <--- Axis
  //  B1          =>  B1
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderAxisOffset_B1_Minus_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId1, -offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }
  }








  // Only body 2
  // The body are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X
  {
    public Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X()
      {
        wId = dWorldCreate();

        bId2 = dBodyCreate (wId);
        dBodySetPosition (bId2, 0, 0, 0);

        jId   = dJointCreateSlider (wId, null);
        joint = (DxJointSlider) jId;


        dJointAttach (jId, null, bId2);

        dJointSetSliderAxis(jId, axis.get0(), axis.get1(), axis.get2());
      }

    //~Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

    DBody bId2;

    DSliderJoint jId;
    DxJointSlider joint;

    static final DVector3 axis = new DVector3(1, 0, 0);

    static final double offset = 3.1;
//  };           Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X
//    final dVector3 Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X.axis = {1, 0, 0};
//    final double    Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X.offset = (3.1);

  // Move 2nd body offset unit in the X direction
  //
  //  X------->       X---------> Axis -->
  //  B2          =>     B2
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X,
  @Test public void  test_dJointSetSliderAxisOffset_B2_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, offset, 0, 0);

      CHECK_CLOSE (-offset, dJointGetSliderPosition(jId), 1e-4);
    }

  // Move 2nd body offset unit in the opposite X direction
  //
  //  X------->          X---------> Axis -->
  //  B2          =>  B2
  //
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderAxisOffset_B2_Minus_OffsetUnit()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);

      dBodySetPosition(bId2, -offset, 0, 0);

      CHECK_CLOSE (offset, dJointGetSliderPosition(jId), 1e-4);
    }
    }

  // Only body 2
  // The body are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is in the oppsite X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X
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
  
  // ==========================================================================
  // Test Position Rate
  // ==========================================================================

  public static class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X2
  extends Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X {
  // Apply force on 1st body in the X direction also the Axis direction
  //
  //  X------->       X---------> Axis -->
  //  B1  F->      =>     B1
  //  B2              B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  
  // Apply force on 1st body in the inverse X direction
  //
  //  X------->           X---------> Axis -->
  //  B1  <-F      => B1
  //  B2                  B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_of_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }

  public static class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X2
  extends Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X {

  // Apply force on 1st body in the X direction also the Axis direction
  //
  //  X------->       X---------> <-- Axis
  //  B1  F->      =>     B1
  //  B2              B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on 1st body in the inverse X direction
  //
  //  X------->           X---------> <-- Axis
  //  B1  <-F      => B1
  //  B2                  B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_of_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }
  
  
  public static class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Along_X3
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

  public static class Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X3
  extends Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X {
  // Apply force on 1st body in the X direction also the Axis direction
  //
  //  X------->       X---------> <-- Axis
  //  B1          =>  B1
  //  B2 F->             B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on 1st body in the inverse X direction
  //
  //  X------->           X---------> <-- Axis
  //  B1          =>      B1
  //  B2 <-F           B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_and_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_of_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }

  
  public static class Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X2
  extends Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X {
  // Apply force on 1st body in the X direction also the Axis direction
  //
  //  X------->       X---------> Axis -->
  //  B1  F->      =>     B1
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on 1st body in the inverse X direction
  //
  //  X------->           X---------> Axis -->
  //  B1  <-F      => B1
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_of_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }
  

  public static class Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X2
  extends Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X {
  // Apply force on 1st body in the X direction also the Axis direction
  //
  //  X------->       X---------> <-- Axis
  //  B1  F->      =>     B1
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on 1st body in the inverse X direction
  //
  //  X------->           X---------> <-- Axis
  //  B1  <-F      => B1
  //TEST_FIXTURE (Fixture_dxJointSlider_B1_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_of_Axis_on_B1()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId1, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }
  

  public static class Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X2
  extends Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X {
  // Apply force on body 2 in the X direction also the Axis direction
  //
  //  X------->       X---------> Axis -->
  //  B2 F->             B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on body 2 in the inverse X direction
  //
  //  X------->           X---------> Axis -->
  //  B2  <-F          B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Along_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_of_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }

  public static class Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X2
  extends Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X {
  // Apply force on body 2 in the X direction also the Axis direction
  //
  //  X------->       X---------> <-- Axis
  //  B2 F->             B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Inverse_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, 1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (1, dJointGetSliderPositionRate(jId), 1e-4);
    }

  // Apply force on body 2 in the inverse X direction
  //
  //  X------->           X---------> <-- Axis
  //  B2 <-F           B2
  //TEST_FIXTURE (Fixture_dxJointSlider_B2_At_Zero_Axis_Inverse_of_X,
  @Test public void test_dJointSetSliderPositionRate_Force_Along_of_Axis_on_B2()
    {
      CHECK_CLOSE (0.0, dJointGetSliderPosition(jId), 1e-4);
      CHECK_CLOSE (0.0, dJointGetSliderPositionRate(jId), 1e-4);

      dBodyAddForce(bId2, -1.0, 0, 0);
      dWorldQuickStep (wId, 1.0);

      CHECK_CLOSE (-1, dJointGetSliderPositionRate(jId), 1e-4);
    }
  }





  // Create 2 bodies attached by a Slider joint
  // Axis is along the X axis (Default value
  // Anchor at (0, 0, 0)      (Default value)
  //
  //       ^Y
  //       |
  //       |
  //       |
  //       |
  // Body1 |     Body2
  // *     Z-----*->x
  public static class dxJointSlider_Test_Initialization
  {
      public dxJointSlider_Test_Initialization()
      {
          wId = dWorldCreate();

          // Remove gravity to have the only force be the force of the joint
          dWorldSetGravity(wId, 0,0,0);

          for (int j=0; j<2; ++j)
          {
              bId[j][0] = dBodyCreate (wId);
              dBodySetPosition (bId[j][0], -1, -2, -3);

              bId[j][1] = dBodyCreate (wId);
              dBodySetPosition (bId[j][1], 11, 22, 33);


              DMatrix3 R = new DMatrix3();
              DVector3 axis = new DVector3(); // Random axis

              axis.set(	(0.53),
            		   -(0.71),
            		    (0.43));
              dNormalize3(axis);
              dRFromAxisAndAngle (R, axis,
                                  (0.47123)); // 27deg
              dBodySetRotation (bId[j][0], R);


              axis.set(	(1.2),
            		    (0.87),
            		   -(0.33));
              dNormalize3(axis);
              dRFromAxisAndAngle (R, axis,
                                  (0.47123)); // 27deg
              dBodySetRotation (bId[j][1], R);


              jId[j] = dJointCreateSlider (wId, null);
              dJointAttach (jId[j], bId[j][0], bId[j][1]);
          }
      }

//      ~dxJointSlider_Test_Initialization()
		@AfterClass
		public static void DESTRUCTOR() {
          dWorldDestroy (wId);
      }

      static DWorld wId;

      DBody[][] bId = new DBody[2][2];


      DSliderJoint[] jId = new DSliderJoint[2];

//  };


  // Test if setting a Slider joint with its default values
  // will behave the same as a default Slider joint
  //TEST_FIXTURE (dxJointSlider_Test_Initialization,
    @Test public void test_Slider_Initialization()
  {
//      using namespace std;

      DVector3 axis = new DVector3();
      dJointGetSliderAxis(jId[1], axis);
      dJointSetSliderAxis(jId[1], axis.get0(), axis.get1(), axis.get2());


      CHECK_CLOSE (dJointGetSliderPosition(jId[0]),
                   dJointGetSliderPosition(jId[1]), 1e-6);


      for (int b=0; b<2; ++b)
      {
          // Compare body b of the first joint with its equivalent on the
          // second joint
          DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
          DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
          CHECK_CLOSE (qA.get0(), qB.get0(), 1e-6);
          CHECK_CLOSE (qA.get1(), qB.get1(), 1e-6);
          CHECK_CLOSE (qA.get2(), qB.get2(), 1e-6);
          CHECK_CLOSE (qA.get3(), qB.get3(), 1e-6);
      }

      dWorldStep (wId,0.5);
      dWorldStep (wId,0.5);
      dWorldStep (wId,0.5);
      dWorldStep (wId,0.5);

      for (int b=0; b<2; ++b)
      {
          // Compare body b of the first joint with its equivalent on the
          // second joint
    	  DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
    	  DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
          CHECK_CLOSE (qA.get0(), qB.get0(), 1e-6);
          CHECK_CLOSE (qA.get1(), qB.get1(), 1e-6);
          CHECK_CLOSE (qA.get2(), qB.get2(), 1e-6);
          CHECK_CLOSE (qA.get3(), qB.get3(), 1e-6);


          DVector3C posA = dBodyGetPosition(bId[0][b]);
          DVector3C posB = dBodyGetPosition(bId[1][b]);
          CHECK_CLOSE (posA.get0(), posB.get0(), 1e-6);
          CHECK_CLOSE (posA.get1(), posB.get1(), 1e-6);
          CHECK_CLOSE (posA.get2(), posB.get2(), 1e-6);
          //CHECK_CLOSE (posA.get3(), posB.get3(), 1e-6);
      }
  }
}


  // Compare Only body 1 to 2 bodies with one fixed.
  //
  // The body are positionned at (0, 0, 0), with no rotation
  // The joint is a Slider Joint
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  public static class Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X
  {
      public Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X()
      {
          wId = dWorldCreate();

          bId1_12 = dBodyCreate (wId);
          dBodySetPosition (bId1_12, 0, 0, 0);

          bId2_12 = dBodyCreate (wId);
          dBodySetPosition (bId2_12, 0, 0, 0);
          // The force will be added in the function since it is not
          // always on the same body

          jId_12 = dJointCreateSlider (wId, null);
          dJointAttach(jId_12, bId1_12, bId2_12);

          fixed = dJointCreateFixed (wId, null);



          bId = dBodyCreate (wId);
          dBodySetPosition (bId, 0, 0, 0);

          dBodyAddForce (bId, 4, 0, 0);

          jId = dJointCreateSlider (wId, null);
      }

//      ~Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
          dWorldDestroy (wId);
      }

      static DWorld wId;

      DBody bId1_12;
      DBody bId2_12;

      DSliderJoint jId_12; // Joint with 2 bodies

      DFixedJoint fixed;



      DBody  bId;
      DSliderJoint jId;    // Joint with one body
//  };

  // This test compare the result of a slider with 2 bodies where body body 2 is
  // fixed to the world to a slider with only one body at position 1.
  //
  // Test the limits [-1, 0.25] when only one body at is attached to the joint
  // using dJointAttache(jId, bId, 0);
  //
  //TEST_FIXTURE(Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X,
  @Test public void test_Limit_minus1_025_One_Body_on_left()
  {
      dBodyAddForce (bId1_12, 4, 0, 0);

      dJointAttach(jId_12, bId1_12, bId2_12);
      dJointSetSliderParam(jId_12, dParamLoStop, -1);
      dJointSetSliderParam(jId_12, dParamHiStop, 0.25);

      dJointAttach(fixed, null, bId2_12);
      dJointSetFixed(fixed);

      dJointAttach(jId, bId, null);
      dJointSetSliderParam(jId, dParamLoStop, -1);
      dJointSetSliderParam(jId, dParamHiStop, 0.25);


      for (int i=0; i<50; ++i)
          dWorldStep(wId, 1.0);

      DVector3C pos1_12 = dBodyGetPosition(bId1_12);

      DVector3C pos = dBodyGetPosition(bId);


      CHECK_CLOSE (pos.get0(), pos1_12.get0(), 1e-2);
      CHECK_CLOSE (pos.get1(), pos1_12.get1(), 1e-2);
      CHECK_CLOSE (pos.get2(), pos1_12.get2(), 1e-2);
  }



  // This test compare the result of a slider with 2 bodies where body body 1 is
  // fixed to the world to a slider with only one body at position 2.
  //
  // Test the limits [-1, 0.25] when only one body at is attached to the joint
  // using dJointAttache(jId, 0, bId);
  //
  //TEST_FIXTURE(Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X,
  @Test public void test_Limit_minus1_025_One_Body_on_right()
  {
      dBodyAddForce (bId2_12, 4, 0, 0);

      dJointAttach(jId_12, bId1_12, bId2_12);
      dJointSetSliderParam(jId_12, dParamLoStop, -1);
      dJointSetSliderParam(jId_12, dParamHiStop, 0.25);

      dJointAttach(fixed, bId1_12, null);
      dJointSetFixed(fixed);


      dJointAttach(jId, null, bId);
      dJointSetSliderParam(jId, dParamLoStop, -1);
      dJointSetSliderParam(jId, dParamHiStop, 0.25);

      for (int i=0; i<50; ++i)
      {
          dWorldStep(wId, 1.0);
      }

      DVector3C pos2_12 = dBodyGetPosition(bId2_12);

      DVector3C pos = dBodyGetPosition(bId);


      CHECK_CLOSE (pos.get0(), pos2_12.get0(), 1e-2);
      CHECK_CLOSE (pos.get1(), pos2_12.get1(), 1e-2);
      CHECK_CLOSE (pos.get2(), pos2_12.get2(), 1e-2);
  }



  // This test compare the result of a slider with 2 bodies where body body 2 is
  // fixed to the world to a slider with only one body at position 1.
  //
  // Test the limits [0, 0] when only one body at is attached to the joint
  // using dJointAttache(jId, bId, 0);
  //
  // The body should not move since their is no room between the two limits
  //
  //TEST_FIXTURE(Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X,
  @Test public void test_Limit_0_0_One_Body_on_left()
  {
      dBodyAddForce (bId1_12, 4, 0, 0);

      dJointAttach(jId_12, bId1_12, bId2_12);
      dJointSetSliderParam(jId_12, dParamLoStop, 0);
      dJointSetSliderParam(jId_12, dParamHiStop, 0);

      dJointAttach(fixed, null, bId2_12);
      dJointSetFixed(fixed);


      dJointAttach(jId, bId, null);
      dJointSetSliderParam(jId, dParamLoStop, 0);
      dJointSetSliderParam(jId, dParamHiStop, 0);

      for (int i=0; i<500; ++i)
          dWorldStep(wId, 1.0);

      DVector3C pos1_12 = dBodyGetPosition(bId1_12);

      DVector3C pos = dBodyGetPosition(bId);


      CHECK_CLOSE (pos.get0(), pos1_12.get0(), 1e-4);
      CHECK_CLOSE (pos.get1(), pos1_12.get1(), 1e-4);
      CHECK_CLOSE (pos.get2(), pos1_12.get2(), 1e-4);

      CHECK_CLOSE (pos.get0(), 0, 1e-4);
      CHECK_CLOSE (pos.get1(), 0, 1e-4);
      CHECK_CLOSE (pos.get2(), 0, 1e-4);
  }


  // This test compare the result of a slider with 2 bodies where body body 1 is
  // fixed to the world to a slider with only one body at position 2.
  //
  // Test the limits [0, 0] when only one body at is attached to the joint
  // using dJointAttache(jId, 0, bId);
  //
  // The body should not move since their is no room between the two limits
  //
  //TEST_FIXTURE(Fixture_dxJointSlider_Compare_Body_At_Zero_Axis_Along_X,
  @Test public void test_Limit_0_0_One_Body_on_right()
  {
      dBodyAddForce (bId2_12, 4, 0, 0);

      dJointAttach(jId_12, bId1_12, bId2_12);
      dJointSetSliderParam(jId_12, dParamLoStop, 0);
      dJointSetSliderParam(jId_12, dParamHiStop, 0);

      dJointAttach(fixed, bId1_12, null);
      dJointSetFixed(fixed);


      dJointAttach(jId, null, bId);
      dJointSetSliderParam(jId, dParamLoStop, 0);
      dJointSetSliderParam(jId, dParamHiStop, 0);

      for (int i=0; i<500; ++i)
          dWorldStep(wId, 1.0);

      DVector3C pos2_12 = dBodyGetPosition(bId2_12);

      DVector3C pos = dBodyGetPosition(bId);


      CHECK_CLOSE (pos.get0(), pos2_12.get0(), 1e-4);
      CHECK_CLOSE (pos.get1(), pos2_12.get1(), 1e-4);
      CHECK_CLOSE (pos.get2(), pos2_12.get2(), 1e-4);

      CHECK_CLOSE (pos.get0(), 0, 1e-4);
      CHECK_CLOSE (pos.get1(), 0, 1e-4);
      CHECK_CLOSE (pos.get2(), 0, 1e-4);
  }

}


} // End of SUITE TestdxJointSlider

