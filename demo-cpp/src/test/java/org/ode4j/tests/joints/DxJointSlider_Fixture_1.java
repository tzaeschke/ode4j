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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateSlider;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetSliderAxis;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dRFromEulerAngles;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointSlider;

//This file create unit test for some of the functions found in:
//ode/src/joinst/slider.cpp
public class DxJointSlider_Fixture_1
    {
        public DxJointSlider_Fixture_1()
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