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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePR;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAxis2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointPR;

// The 2 bodies are positionned at (0, 0, 0),  and (0, 0, 0)
// The bodis have rotation of 27deg around some axis.
// The joint is a PR Joint
// Axis is along the X axis
// Anchor at (0, 0, 0)
public class Fixture_dxJointPR_B1_and_B2_Random_Orientation_At_Zero_Axis_Along_X
    {
        public Fixture_dxJointPR_B1_and_B2_Random_Orientation_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            DMatrix3 R = new DMatrix3();

            DVector3 axis = new DVector3(); // Random axis

            axis.set( (0.53),
            		-(0.71),
            		(0.43));
            dNormalize3(axis);
            dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
                                (0.47123)); // 27deg
            dBodySetRotation (bId1, R);


            axis.set(	(1.2),
            			(0.87),
            			-(0.33));
            dNormalize3(axis);
            dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
                                (0.47123)); // 27deg
            dBodySetRotation (bId2, R);

            jId   = dJointCreatePR (wId, null);
            joint = (DxJointPR) jId;


            dJointAttach (jId, bId1, bId2);
        }

//        ~Fixture_dxJointPR_B1_and_B2_Random_Orientation_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
            dWorldDestroy (wId);
        }

        static DWorld wId;

        DBody bId1;
        DBody bId2;


        DPRJoint jId;
        DxJointPR joint;
//    };

    // Test is dJointSetPRAxis and dJointGetPRAxis return same value
    //TEST_FIXTURE (Fixture_dxJointPR_B1_and_B2_Random_Orientation_At_Zero_Axis_Along_X,
    @Test public void test_dJointSetGetPRAxis()
    {
        DVector3 axisOrig = new DVector3(), axis = new DVector3();


        dJointGetPRAxis1 (jId, axisOrig);
        dJointGetPRAxis1 (jId, axis);
        dJointSetPRAxis1 (jId, axis.get0(), axis.get1(), axis.get2());
        dJointGetPRAxis1 (jId, axis);
        CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
        CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
        CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);


        dJointGetPRAxis2 (jId, axisOrig);
        dJointGetPRAxis2(jId, axis);
        dJointSetPRAxis2 (jId, axis.get0(), axis.get1(), axis.get2());
        dJointGetPRAxis2 (jId, axis);
        CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
        CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
        CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);
    }
    }