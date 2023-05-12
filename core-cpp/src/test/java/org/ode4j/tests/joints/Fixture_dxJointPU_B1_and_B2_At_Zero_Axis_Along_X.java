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
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePU;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUAxis3;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAxis3;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointPU;

// The 2 bodies are positionned at (0, 0, 0),  and (0, 0, 0)
// The second body has a rotation of 27deg around X axis.
// The joint is a PU Joint
// Axis is along the X axis
// Anchor at (0, 0, 0)
public class Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X
    {
        public Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            DMatrix3 R = new DMatrix3();

            dRFromAxisAndAngle (R, 1, 0, 0, (0.47123)); // 27deg
            dBodySetRotation (bId2, R);

            jId   = dJointCreatePU (wId, null);
            joint = (DxJointPU) jId;


            dJointAttach (jId, bId1, bId2);
        }

//        ~Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
            dWorldDestroy (wId);
        }

        static DWorld wId;

        DBody bId1;
        DBody bId2;


        DPUJoint jId;
        DxJointPU joint;
//    };

    // Test is dJointSetPUAxis and dJointGetPUAxis return same value
    //TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X,
    @Test public void test_dJointSetGetPUAxis()
    {
        DVector3 axisOrig = new DVector3(), axis = new DVector3();


        dJointGetPUAxis1 (jId, axisOrig);
        dJointGetPUAxis1 (jId, axis);
        dJointSetPUAxis1 (jId, axis.get0(), axis.get1(), axis.get2());
        dJointGetPUAxis1 (jId, axis);
        CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
        CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
        CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);


        dJointGetPUAxis2 (jId, axisOrig);
        dJointGetPUAxis2(jId, axis);
        dJointSetPUAxis2 (jId, axis.get0(), axis.get1(), axis.get2());
        dJointGetPUAxis2 (jId, axis);
        CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
        CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
        CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);


        dJointGetPUAxis3 (jId, axisOrig);
        dJointGetPUAxis3(jId, axis);
        dJointSetPUAxis3 (jId, axis.get0(), axis.get1(), axis.get2());
        dJointGetPUAxis3 (jId, axis);
        CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
        CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
        CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);
    }
    }