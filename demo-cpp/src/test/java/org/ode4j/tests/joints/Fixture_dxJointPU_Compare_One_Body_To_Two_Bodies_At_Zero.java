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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateFixed;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePU;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUAxisP;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUPositionRate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetFixed;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAxisP;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop3;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop3;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

// Compare only one body to 2 bodies with one fixed.
//
// The body are positionned at (0, 0, 0), with no rotation
// The joint is a PU Joint with default values
public class Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero extends TestSuperClass
    {
	@Before
        public void Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero_init()
        {
            wId = dWorldCreate();

            bId1_12 = dBodyCreate (wId);
            dBodySetPosition (bId1_12, 0, 0, 0);

            bId2_12 = dBodyCreate (wId);
            dBodySetPosition (bId2_12, 0, 0, 0);
            // The force will be added in the function since it is not
            // always on the same body

            jId_12 = dJointCreatePU (wId, null);
            dJointAttach(jId_12, bId1_12, bId2_12);

            fixed = dJointCreateFixed (wId, null);



            jId = dJointCreatePU (wId, null);

            bId = dBodyCreate (wId);
            dBodySetPosition (bId, 0, 0, 0);

            // Linear velocity along the prismatic axis;
            DVector3 axis = new DVector3();
            dJointGetPUAxisP(jId_12, axis);
            dJointSetPUAxisP(jId, axis.get0(), axis.get1(), axis.get2());
            dBodySetLinearVel (bId, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());
        }

//        ~Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero()
		@After
		public void DESTRUCTOR() {
            dWorldDestroy (wId);
        }

		private static DWorld wId;

		private DBody bId1_12;
		private DBody bId2_12;

		private DPUJoint jId_12; // Joint with 2 bodies

		private DFixedJoint fixed;



		private DBody  bId;
		private DPUJoint jId;    // Joint with one body

		private static final double magnitude = 4.27;
//    };
//    const dReal Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero::magnitude = REAL (4.27);


    //TEST_FIXTURE (Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
    @Test public void test_dJointSetPUPositionRate_Only_B1()
    {
        // Linear velocity along the prismatic axis;
        DVector3 axis = new DVector3();
        dJointGetPUAxisP(jId_12, axis);
        dBodySetLinearVel (bId1_12, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());

        dJointAttach(jId_12, bId1_12, bId2_12);

        dJointAttach(fixed, null, bId2_12);
        dJointSetFixed(fixed);

        dJointAttach(jId, bId, null);

        CHECK_CLOSE(dJointGetPUPositionRate(jId_12), dJointGetPUPositionRate(jId), 1e-2);
    }


    //TEST_FIXTURE (Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
    @Test public void test_dJointSetPUPositionRate_Only_B2()
    {
        // Linear velocity along the prismatic axis;
        DVector3 axis = new DVector3();
        dJointGetPUAxisP(jId_12, axis);
        dBodySetLinearVel (bId2_12, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());

        dJointAttach(jId_12, bId1_12, bId2_12);

        dJointAttach(fixed, bId1_12, null);
        dJointSetFixed(fixed);

        dJointAttach(jId, null, bId);

        CHECK_CLOSE(dJointGetPUPositionRate(jId_12), dJointGetPUPositionRate(jId), 1e-2);
    }








    // This test compare the result of a pu joint with 2 bodies where body body 2 is
    // fixed to the world to a pu joint with only one body at position 1.
    //
    // Test the limits [-1, 0.25] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    //TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
    @Test public void test_Limit_minus1_025_One_Body_on_left()
    {
        DVector3 axis = new DVector3();
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis.get0(), axis.get1(), axis.get2());
        dBodySetLinearVel (bId1_12, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, -1);
        dJointSetPUParam(jId_12, dParamHiStop3, 0.25);

        dJointAttach(fixed, null, bId2_12);
        dJointSetFixed(fixed);

        dJointAttach(jId, bId, null);
        dJointSetPUParam(jId, dParamLoStop3, -1);
        dJointSetPUParam(jId, dParamHiStop3, 0.25);


        for (int i=0; i<50; ++i)
            dWorldStep(wId, 1.0);


        DVector3C pos1_12 = dBodyGetPosition(bId1_12);
        DVector3C pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos1_12.get0(), pos.get0(), 1e-2);
        CHECK_CLOSE (pos1_12.get1(), pos.get1(), 1e-2);
        CHECK_CLOSE (pos1_12.get2(), pos.get2(), 1e-2);

        DQuaternionC q1_12 = dBodyGetQuaternion(bId1_12);
        DQuaternionC q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q1_12.get0(), q.get0(), 1e-4);
        CHECK_CLOSE (q1_12.get1(), q.get1(), 1e-4);
        CHECK_CLOSE (q1_12.get2(), q.get2(), 1e-4);
        CHECK_CLOSE (q1_12.get3(), q.get3(), 1e-4);

        // Should be different than zero
        CHECK( dJointGetPUPosition(jId_12)!=0 );
        CHECK( dJointGetPUPosition(jId)!=0 );

        CHECK( dJointGetPUPositionRate(jId_12)!=0 );
        CHECK( dJointGetPUPositionRate(jId)!=0 );
    }



    // This test compare the result of a pu joint with 2 bodies where body body 1 is
    // fixed to the world to a pu joint with only one body at position 2.
    //
    // Test the limits [-1, 0.25] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    //TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
    @Test public void test_Limit_minus1_025_One_Body_on_right()
    {
        DVector3 axis = new DVector3();
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis.get0(), axis.get1(), axis.get2());
        dBodySetLinearVel (bId2_12, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, -1);
        dJointSetPUParam(jId_12, dParamHiStop3, 0.25);

        dJointAttach(fixed, bId1_12, null);
        dJointSetFixed(fixed);


        dJointAttach(jId, null, bId);
        dJointSetPUParam(jId, dParamLoStop3, -1);
        dJointSetPUParam(jId, dParamHiStop3, 0.25);

        for (int i=0; i<50; ++i)
            dWorldStep(wId, 1.0);


        DVector3C pos2_12 = dBodyGetPosition(bId2_12);
        DVector3C pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos2_12.get0(), pos.get0(), 1e-2);
        CHECK_CLOSE (pos2_12.get1(), pos.get1(), 1e-2);
        CHECK_CLOSE (pos2_12.get2(), pos.get2(), 1e-2);


        DQuaternionC q2_12 = dBodyGetQuaternion(bId2_12);
        DQuaternionC q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q2_12.get0(), q.get0(), 1e-4);
        CHECK_CLOSE (q2_12.get1(), q.get1(), 1e-4);
        CHECK_CLOSE (q2_12.get2(), q.get2(), 1e-4);
        CHECK_CLOSE (q2_12.get3(), q.get3(), 1e-4);

        // Should be different than zero
        CHECK( dJointGetPUPosition(jId_12) != 0 );
        CHECK( dJointGetPUPosition(jId) != 0 );

        CHECK( dJointGetPUPositionRate(jId_12) != 0 );
        CHECK( dJointGetPUPositionRate(jId) != 0 );
    }



    // This test compare the result of a pu joint with 2 bodies where body 2 is
    // fixed to the world to a pu joint with only one body at position 1.
    //
    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    // The body should not move since their is no room between the two limits
    //
    //TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
    @Test public void test_Limit_0_0_One_Body_on_left()
    {
    	DVector3 axis = new DVector3();
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis.get0(), axis.get1(), axis.get2());
        dBodySetLinearVel (bId1_12, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, 0);
        dJointSetPUParam(jId_12, dParamHiStop3, 0);

        dJointAttach(fixed, null, bId2_12);
        dJointSetFixed(fixed);


        dJointAttach(jId, bId, null);
        dJointSetPUParam(jId, dParamLoStop3, 0);
        dJointSetPUParam(jId, dParamHiStop3, 0);

        for (int i=0; i<500; ++i)
            dWorldStep(wId, 1.0);


        DVector3C pos1_12 = dBodyGetPosition(bId1_12);
        DVector3C pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos1_12.get0(), pos.get0(), 1e-4);
        CHECK_CLOSE (pos1_12.get1(), pos.get1(), 1e-4);
        CHECK_CLOSE (pos1_12.get2(), pos.get2(), 1e-4);

        CHECK_CLOSE (0, pos.get0(), 1e-4);
        CHECK_CLOSE (0, pos.get1(), 1e-4);
        CHECK_CLOSE (0, pos.get2(), 1e-4);


        DQuaternionC q1_12 = dBodyGetQuaternion(bId1_12);
        DQuaternionC q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q1_12.get0(), q.get0(), 1e-4);
        CHECK_CLOSE (q1_12.get1(), q.get1(), 1e-4);
        CHECK_CLOSE (q1_12.get2(), q.get2(), 1e-4);
        CHECK_CLOSE (q1_12.get3(), q.get3(), 1e-4);
    }


    // This test compare the result of a pu joint with 2 bodies where body body 1 is
    // fixed to the world to a pu joint with only one body at position 2.
    //
    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    // The body should not move since their is no room between the two limits
    //
    //TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
    @Test public void test_Limit_0_0_One_Body_on_right()
    {
        DVector3 axis = new DVector3();
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis.get0(), axis.get1(), axis.get2());
        dBodySetLinearVel (bId2_12, magnitude*axis.get0(), magnitude*axis.get1(), magnitude*axis.get2());

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, 0);
        dJointSetPUParam(jId_12, dParamHiStop3, 0);

        dJointAttach(fixed, bId1_12, null);
        dJointSetFixed(fixed);


        dJointAttach(jId, null, bId);
        dJointSetPUParam(jId, dParamLoStop3, 0);
        dJointSetPUParam(jId, dParamHiStop3, 0);

        for (int i=0; i<500; ++i)
            dWorldStep(wId, 1.0);

        DVector3C pos2_12 = dBodyGetPosition(bId2_12);
        DVector3C pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos2_12.get0(), pos.get0(), 1e-4);
        CHECK_CLOSE (pos2_12.get1(), pos.get1(), 1e-4);
        CHECK_CLOSE (pos2_12.get2(), pos.get2(), 1e-4);

        CHECK_CLOSE (0, pos.get0(), 1e-4);
        CHECK_CLOSE (0, pos.get1(), 1e-4);
        CHECK_CLOSE (0, pos.get2(), 1e-4);


        DQuaternionC q2_12 = dBodyGetQuaternion(bId2_12);
        DQuaternionC q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q2_12.get0(), q.get0(), 1e-4);
        CHECK_CLOSE (q2_12.get1(), q.get1(), 1e-4);
        CHECK_CLOSE (q2_12.get2(), q.get2(), 1e-4);
        CHECK_CLOSE (q2_12.get3(), q.get3(), 1e-4);
    }
    }