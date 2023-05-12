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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePU;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAnchorOffset;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAxisP;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointPU;

// Only one body
// The body are positionned at (0, 0, 0), with no rotation
// The joint is a PU Joint
// Axis is in the oppsite X axis
// Anchor at (0, 0, 0)
// N.B. By default the body is attached at position 1 on the joint
//      dJointAttach (jId, bId, 0);
public class Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X
    {
        public Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId = dBodyCreate (wId);
            dBodySetPosition (bId, 0, 0, 0);

            jId   = dJointCreatePU (wId, null);
            joint = (DxJointPU) jId;


            dJointAttach (jId, bId, null);

            dJointSetPUAxisP (jId, axis.get0(), axis.get1(), axis.get2());
        }

//        ~Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
            dWorldDestroy (wId);
        }

        static DWorld wId;

        DBody bId;

        DPUJoint jId;
        DxJointPU joint;

        static final DVector3C axis = new DVector3(-1, 0, 0);

        static final double offset = 3.1;
//    };
//    const dVector3 Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X::axis =
//    {
//        -1, 0, 0
//    };
//    const dReal    Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);


    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B1       =>  B1
    //TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
    @Test public void test_dJointSetPUAxisOffset_B1_At_Position_1_OffsetUnit()
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->   <--- Axis
    //  B1          =>  B1
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X--------->   <--- Axis
    //  B1            =>   B1
    //TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
    @Test public void test_dJointSetPUAxisOffset_B1_Minus_OffsetUnit()
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }


    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B2       =>  B2
    //TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
    @Test public void test_dJointSetPUAxisOffset_B2_OffsetUnit()
    {
        // By default it is attached to position 1
        // Now attach the body at positiojn 2
        dJointAttach(jId, null, bId);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis.get0(), offset*axis.get1(), offset*axis.get2());
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->   <--- Axis
    //  B2          =>  B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X--------->   <--- Axis
    //  B2            =>   B2
    //TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
    @Test public void test_dJointSetPUAxisOffset_B2_Minus_OffsetUnit()
    {
        // By default it is attached to position 1
        // Now attach the body at positiojn 2
        dJointAttach(jId, null, bId);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis.get0(), -offset*axis.get1(), -offset*axis.get2());
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }
    }