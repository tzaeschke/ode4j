/*
 * Created on May 13, 2012
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Generation - Code and Comments
 */
package org.ode4j.tests.joints;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateHinge2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetHinge2Anchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetHinge2Axis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetHinge2Axis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHinge2Anchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHinge2Axis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHinge2Axis2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DWorld;

/**
 * The 2 bodies are positionned at (-1, -2, -3),  and (11, 22, 33)
 * The bodis have rotation of 27deg around some axis.
 * The joint is a Hinge2 Joint
 * Axis is along the X axis
 * Anchor at (0, 0, 0)
 */
public class DxJointHinge2_Fixture_B1_and_B2_At_Zero_Axis_Along_X {
	public DxJointHinge2_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
	{
		wId = dWorldCreate();

		for (int j=0; j<2; ++j) {
			bId[j][0] = dBodyCreate (wId);
			dBodySetPosition (bId[j][0], -1, -2, -3);

			bId[j][1] = dBodyCreate (wId);
			dBodySetPosition (bId[j][1], 11, 22, 33);


			DMatrix3 R = new DMatrix3();
			DVector3 axis = new DVector3(); // Random axis

			axis.set(  (0.53),
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

					jId[j]   = dJointCreateHinge2 (wId, null);
					dJointAttach (jId[j], bId[j][0], bId[j][1]);
		}
	}

	//    ~dxJointHinge2_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy (wId);
	}

	static DWorld wId;

	DBody[][] bId = new DBody[2][2];


	DHinge2Joint[] jId = new DHinge2Joint[2];
	//  };

	// Rotate 2nd body 90deg around X then back to original position
	//
	//   ^  ^       ^
	//   |  |  =>   |  <---
	//   |  |       |
	//  B1  B2     B1   B2
	//
	// Start with a Delta of 90deg
	//   ^           ^   ^
	//   | <---  =>  |   |
	//   |           |   |
	//  B1  B2      B1   B2
	//TEST_FIXTURE (dxJointHinge2_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
	@Test public void test_dJointSetHinge2AxisOffset_B2_90deg() {

		DVector3 anchor = new DVector3();
		dJointGetHinge2Anchor(jId[1], anchor);
		dJointSetHinge2Anchor(jId[1], anchor.get0(), anchor.get1(), anchor.get2());

		DVector3 axis = new DVector3();
		dJointGetHinge2Axis1(jId[1], axis);
		dJointSetHinge2Axis1(jId[1], axis.get0(), axis.get1(), axis.get2());

		dJointGetHinge2Axis2(jId[1], axis);
		dJointSetHinge2Axis2(jId[1], axis.get0(), axis.get1(), axis.get2());


		for (int b=0; b<2; ++b) {
			// Compare body b of the first joint with its equivalent on the
			// second joint
			DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
			DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
			CHECK_CLOSE (qA.get0(), qB.get0(), 1e-4);
			CHECK_CLOSE (qA.get1(), qB.get1(), 1e-4);
			CHECK_CLOSE (qA.get2(), qB.get2(), 1e-4);
			CHECK_CLOSE (qA.get3(), qB.get3(), 1e-4);
		}

		dWorldStep (wId,0.5);
		dWorldStep (wId,0.5);
		dWorldStep (wId,0.5);
		dWorldStep (wId,0.5);

		for (int b=0; b<2; ++b) {
			// Compare body b of the first joint with its equivalent on the
			// second joint
			DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
			DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
			CHECK_CLOSE (qA.get0(), qB.get0(), 1e-4);
			CHECK_CLOSE (qA.get1(), qB.get1(), 1e-4);
			CHECK_CLOSE (qA.get2(), qB.get2(), 1e-4);
			CHECK_CLOSE (qA.get3(), qB.get3(), 1e-4);


			DVector3C posA = dBodyGetPosition(bId[0][b]);
			DVector3C posB = dBodyGetPosition(bId[1][b]);
			CHECK_CLOSE (posA.get0(), posB.get0(), 1e-4);
			CHECK_CLOSE (posA.get1(), posB.get1(), 1e-4);
			CHECK_CLOSE (posA.get2(), posB.get2(), 1e-4);
			//CHECK_CLOSE (posA[3], posB[3], 1e-4);
		}
	}
}