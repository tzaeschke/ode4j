/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
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
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddForce;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddTorque;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.*;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetCFM;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetERP;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.drawstuff.DrawStuff.dsStop;
import static org.ode4j.ode.DMisc.dMaxDifference;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dQFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.OdeMath.dMultiply1_331;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.ErrorHandler.dError;
import static org.ode4j.ode.internal.cpp4j.Cmath.cos;
import static org.ode4j.ode.internal.cpp4j.Cmath.fabs;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.cpp4j.Ctype;


/**
 * perform tests on all the joint types.
 * this should be done using the double precision version of the library.
 * 
 * usage:
 *   test_joints [-nXXX] [-g] [-i] [-e] [path_to_textures]
 * 
 * if a test number is given then that specific test is performed, otherwise
 * all the tests are performed. the tests are numbered `xxyy', where xx
 * corresponds to the joint type and yy is the sub-test number. not every
 * number maps to an actual test.
 * 
 * flags:
 *   i: the test is interactive.
 *   g: turn off graphical display (can't use this with `i').
 *   e: turn on occasional error perturbations
 *   n: perform test XYZ
 * some tests compute and display error values. these values are scaled so
 * less than 1 is good and greater than 1 is bad. other tests just show graphical results which
 * you must verify visually.
 */
public class DemoJoints extends dsFunctions {

	//// some constants
	//#define NUM_JOINTS 10	// number of joints to test (the `xx' value)
	//#define SIDE (0.5f)	// side length of a box - don't change this
	//#define MASS (1.0)	// mass of a box
	//#define STEPSIZE 0.05
	//some constants

	private final DVector3 xunit = new DVector3( 1, 0, 0 ), zunit = new DVector3( 0, 0, 1 );

	private static final int NUM_JOINTS = 10;	// number of joints to test (the `xx' value)
	private static final float SIDE = 0.5f;	// side length of a box - don't change this
	private static final float MASS = 1.0f;	// mass of a box
	private static final float STEPSIZE = 0.05f;


	// dynamics objects
	private static DWorld world;
	private static DBody[] body= new DBody[2];
	private static DAMotorJoint jointAM;
	private static DFixedJoint jointF;
	private static DHingeJoint jointH;
	private static DHinge2Joint jointH2;
	private static DPRJoint jointPR;
	private static DSliderJoint jointS;
	private static DUniversalJoint jointU;


	// data from the command line arguments
	private static int cmd_test_num = -1;
	//static int cmd_interactive = 0;
	//static int cmd_graphics = 1;
	private static boolean cmd_interactive = false;
	private static boolean cmd_graphics = true;
//	private static String cmd_path_to_textures = null;
	private static int cmd_occasional_error = 0;	// perturb occasionally


	// info about the current test
	//struct TestInfo;
	private static int test_num = 0;		// number of the current test
	private static int iteration = 0;
	private static int max_iterations = 0;
	private static float max_error = 0;

	//****************************************************************************
	// utility stuff

	private static double length (DVector3 a)
	{
		return a.length();
		//return dSqrt (a.v[0]*a.v[0] + a.v[1]*a.v[1] + a.v[2]*a.v[2]);
	}


	// get the max difference between a 3x3 matrix and the identity

	private double cmpIdentity (final DMatrix3C A)
	{
		DMatrix3 I = new DMatrix3();
		I.setZero();//dSetZero (I,12);
//		I.v[0] = 1;
//		I.v[5] = 1;
//		I.v[10] = 1;
		I.eqIdentity();
		return dMaxDifference (A,I);
	}

	//****************************************************************************
	// test world construction and utilities

	private void constructWorldForTest (double gravity, int bodycount,
			/* body 1 pos */           double pos1x, double pos1y, double pos1z,
			/* body 2 pos */           double pos2x, double pos2y, double pos2z,
			/* body 1 rotation axis */ double ax1x, double ax1y, double ax1z,
			/* body 1 rotation axis */ double ax2x, double ax2y, double ax2z,
			/* rotation angles */      double a1, double a2)
	{
		// create world
		world = dWorldCreate();
		dWorldSetERP (world,0.2);
		dWorldSetCFM (world,1e-6);
		dWorldSetGravity (world,0,0,gravity);

		DMass m = dMassCreate();
		dMassSetBox (m,1,SIDE,SIDE,SIDE);
		dMassAdjust (m,MASS);

		body[0] = dBodyCreate (world);
		dBodySetMass (body[0],m);
		dBodySetPosition (body[0], pos1x, pos1y, pos1z);
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q,ax1x,ax1y,ax1z,a1);
		dBodySetQuaternion (body[0],q);

		if (bodycount==2) {
			body[1] = dBodyCreate (world);
			dBodySetMass (body[1],m);
			dBodySetPosition (body[1], pos2x, pos2y, pos2z);
			dQFromAxisAndAngle (q,ax2x,ax2y,ax2z,a2);
			dBodySetQuaternion (body[1],q);
		}
		else body[1] = null;
	}


	// add an oscillating torque to body 0

	private static double a_1=0;
	private void addOscillatingTorque (double tscale)
	{
		//TZ  static double a=0;
		dBodyAddTorque (body[0],tscale*cos(2*a_1),tscale*cos(2.7183*a_1),
				tscale*cos(1.5708*a_1));
		a_1 += 0.01;
	}


	private static double a_2=0;
	private void addOscillatingTorqueAbout(double tscale, double x, double y, double z)
	{
		//TZ  static double a=0;
		dBodyAddTorque (body[0], tscale*cos(a_2) * x, tscale*cos(a_2) * y,
				tscale * cos(a_2) * z);
		a_2 += 0.02;
	}
	private void addOscillatingTorqueAbout(double tscale, DVector3 v)
	{
		//TZ  static double a=0;
		dBodyAddTorque (body[0], tscale*cos(a_2) * v.get0(), tscale*cos(a_2) * v.get1(),
				tscale * cos(a_2) * v.get2());
		a_2 += 0.02;
	}


	// damp the rotational motion of body 0 a bit

	private void dampRotationalMotion (double kd)
	{
		final DVector3C w = dBodyGetAngularVel (body[0]);
		dBodyAddTorque (body[0],-kd*w.get0(),-kd*w.get1(),-kd*w.get2());
	}


	// add a spring force to keep the bodies together, otherwise they may fly
	// apart with some joints.

	private void addSpringForce (double ks)
	{
		DVector3C p1 = dBodyGetPosition (body[0]);
		DVector3C p2 = dBodyGetPosition (body[1]);
		DVector3 tmp = new DVector3();
		//dBodyAddForce (body[0],ks*(p2.v[0]-p1.v[0]),ks*(p2.v[1]-p1.v[1]),ks*(p2.v[2]-p1.v[2]));
		tmp.eqDiff(p2, p1).scale(ks);
		body[0].addForce(tmp);
		//dBodyAddForce (body[1],ks*(p1.v[0]-p2.v[0]),ks*(p1.v[1]-p2.v[1]),ks*(p1.v[2]-p2.v[2]));
		tmp.eqDiff(p1, p2).scale(ks);
		body[1].addForce(tmp);
	}


	// add an oscillating Force to body 0

	private static double a_3=0;
	private void addOscillatingForce (double fscale)
	{
		//TZ static double a=0;
		dBodyAddForce (body[0],fscale*cos(2*a_3),fscale*cos(2.7183*a_3),
				fscale*cos(1.5708*a_3));
		a_3 += 0.01;
	}

	//****************************************************************************
	// stuff specific to the tests
	//
	//   0xx : fixed
	//   1xx : ball and socket
	//   2xx : hinge
	//   3xx : slider
	//   4xx : hinge 2
	//   5xx : contact
	//   6xx : amotor
	//   7xx : universal joint
	//   8xx : PR joint (Prismatic and Rotoide)

	// setup for the given test. return 0 if there is no such test

	private int setupTest (int n)
	{
		switch (n) {

		// ********** fixed joint

		case 0: {			// 2 body
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,1,0, 1,1,0,
					0.25*M_PI,0.25*M_PI);
			jointF = dJointCreateFixed (world,null);
			dJointAttach (jointF,body[0],body[1]);
			dJointSetFixed (jointF);
			return 1;
		}

		case 1: {			// 1 body to static env
			constructWorldForTest (0,1,
					0.5*SIDE,0.5*SIDE,1, 0,0,0,
					1,0,0, 1,0,0,
					0,0);
			jointF = dJointCreateFixed (world,null);
			dJointAttach (jointF,body[0],null);
			dJointSetFixed (jointF);
			return 1;
		}

		case 2: {			// 2 body with relative rotation
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,1,0, 1,1,0,
					0.25*M_PI,-0.25*M_PI);
			jointF = dJointCreateFixed (world,null);
			dJointAttach (jointF,body[0],body[1]);
			dJointSetFixed (jointF);
			return 1;
		}

		case 3: {			// 1 body to static env with relative rotation
			constructWorldForTest (0,1,
					0.5*SIDE,0.5*SIDE,1, 0,0,0,
					1,0,0, 1,0,0,
					0.25*M_PI,0);
			jointF = dJointCreateFixed (world,null);
			dJointAttach (jointF,body[0],null);
			dJointSetFixed (jointF);
			return 1;
		}

		// ********** hinge joint

		case 200:			// 2 body
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,1,0, 1,1,0, 0.25*M_PI,0.25*M_PI);
			jointH = dJointCreateHinge (world,null);
			dJointAttach (jointH,body[0],body[1]);
			dJointSetHingeAnchor (jointH,0,0,1);
			dJointSetHingeAxis (jointH,1,-1,1.41421356);
			return 1;

		case 220:			// hinge angle polarity test
		case 221:			// hinge angle rate test
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,0,0, 1,0,0, 0,0);
			jointH = dJointCreateHinge (world,null);
			dJointAttach (jointH,body[0],body[1]);
			dJointSetHingeAnchor (jointH,0,0,1);
			dJointSetHingeAxis (jointH,0,0,1);
			max_iterations = 50;
			return 1;

		case 230:			// hinge motor rate (and polarity) test
		case 231:			// ...with stops
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,0,0, 1,0,0, 0,0);
			jointH = dJointCreateHinge (world,null);
			dJointAttach (jointH,body[0],body[1]);
			dJointSetHingeAnchor (jointH,0,0,1);
			dJointSetHingeAxis (jointH,0,0,1);
			dJointSetHingeParam (jointH,dParamFMax,1);
			if (n==231) {
				dJointSetHingeParam (jointH,dParamLoStop,-0.5);
				dJointSetHingeParam (jointH,dParamHiStop,0.5);
			}
			return 1;

		case 250:			// limit bounce test (gravity down)
		case 251: {			// ...gravity up
			constructWorldForTest ((n==251) ? 0.1 : -0.1, 2,
					0.5*SIDE,0,1+0.5*SIDE, -0.5*SIDE,0,1-0.5*SIDE,
					1,0,0, 1,0,0, 0,0);
			jointH = dJointCreateHinge (world,null);
			dJointAttach (jointH,body[0],body[1]);
			dJointSetHingeAnchor (jointH,0,0,1);
			dJointSetHingeAxis (jointH,0,1,0);
			dJointSetHingeParam (jointH,dParamLoStop,-0.9);
			dJointSetHingeParam (jointH,dParamHiStop,0.7854);
			dJointSetHingeParam (jointH,dParamBounce,0.5);
			// anchor 2nd body with a fixed joint
			DFixedJoint j = dJointCreateFixed (world,null);
			dJointAttach (j,body[1],null);
			dJointSetFixed (j);
			return 1;
		}

		// ********** slider

		case 300:			// 2 body
			constructWorldForTest (0,2,
					0,0,1, 0.2,0.2,1.2,
					0,0,1, -1,1,0, 0,0.25*M_PI);
			jointS = dJointCreateSlider (world,null);
			dJointAttach (jointS,body[0],body[1]);
			dJointSetSliderAxis (jointS,1,1,1);
			return 1;

		case 320:			// slider angle polarity test
		case 321:			// slider angle rate test
			constructWorldForTest (0,2,
					0,0,1, 0,0,1.2,
					1,0,0, 1,0,0, 0,0);
			jointS = dJointCreateSlider (world,null);
			dJointAttach (jointS,body[0],body[1]);
			dJointSetSliderAxis (jointS,0,0,1);
			max_iterations = 50;
			return 1;

		case 330:			// slider motor rate (and polarity) test
		case 331:			// ...with stops
			constructWorldForTest (0, 2,
					0,0,1, 0,0,1.2,
					1,0,0, 1,0,0, 0,0);
			jointS = dJointCreateSlider (world,null);
			dJointAttach (jointS,body[0],body[1]);
			dJointSetSliderAxis (jointS,0,0,1);
			dJointSetSliderParam (jointS,dParamFMax,100);
			if (n==331) {
				dJointSetSliderParam (jointS,dParamLoStop,-0.4);
				dJointSetSliderParam (jointS,dParamHiStop,0.4);
			}
			return 1;

		case 350:			// limit bounce tests
		case 351: {
			constructWorldForTest ((n==351) ? 0.1 : -0.1, 2,
					0,0,1, 0,0,1.2,
					1,0,0, 1,0,0, 0,0);
			jointS = dJointCreateSlider (world,null);
			dJointAttach (jointS,body[0],body[1]);
			dJointSetSliderAxis (jointS,0,0,1);
			dJointSetSliderParam (jointS,dParamLoStop,-0.5);
			dJointSetSliderParam (jointS,dParamHiStop,0.5);
			dJointSetSliderParam (jointS,dParamBounce,0.5);
			// anchor 2nd body with a fixed joint
			DFixedJoint j = dJointCreateFixed (world,null);
			dJointAttach (j,body[1],null);
			dJointSetFixed (j);
			return 1;
		}

		// ********** hinge-2 joint

		case 420:			// hinge-2 steering angle polarity test
		case 421:			// hinge-2 steering angle rate test
			constructWorldForTest (0,2,
					0.5*SIDE,0,1, -0.5*SIDE,0,1,
					1,0,0, 1,0,0, 0,0);
			jointH2 = dJointCreateHinge2 (world,null);
			dJointAttach (jointH2,body[0],body[1]);
			dJointSetHinge2Anchor (jointH2,-0.5*SIDE,0,1);
			dJointSetHinge2Axes (jointH2,xunit, zunit);
			max_iterations = 50;
			return 1;

		case 430:			// hinge 2 steering motor rate (+polarity) test
		case 431:			// ...with stops
		case 432:			// hinge 2 wheel motor rate (+polarity) test
			constructWorldForTest (0,2,
					0.5*SIDE,0,1, -0.5*SIDE,0,1,
					1,0,0, 1,0,0, 0,0);
			jointH2 = dJointCreateHinge2 (world,null);
			dJointAttach (jointH2,body[0],body[1]);
			dJointSetHinge2Anchor (jointH2,-0.5*SIDE,0,1);
			dJointSetHinge2Axes (jointH2,xunit, zunit);
			dJointSetHinge2Param (jointH2,dParamFMax,1);
			dJointSetHinge2Param (jointH2,dParamFMax2,1);
			if (n==431) {
				dJointSetHinge2Param (jointH2,dParamLoStop,-0.5);
				dJointSetHinge2Param (jointH2,dParamHiStop,0.5);
			}
			return 1;

			// ********** angular motor joint

		case 600:			// test euler angle calculations
			constructWorldForTest (0,2,
					-SIDE*0.5,0,1, SIDE*0.5,0,1,
					0,0,1, 0,0,1, 0,0);
			jointAM = dJointCreateAMotor (world,null);
			dJointAttach (jointAM,body[0],body[1]);

			dJointSetAMotorNumAxes (jointAM,3);
			dJointSetAMotorAxis (jointAM,0,1, 0,0,1);
			dJointSetAMotorAxis (jointAM,2,2, 1,0,0);
			dJointSetAMotorMode (jointAM,dAMotorEuler);
			max_iterations = 200;
			return 1;

			// ********** universal joint

		case 700:			// 2 body
		case 701:
		case 702:
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,1,0, 1,1,0, 0.25*M_PI,0.25*M_PI);
			jointU = dJointCreateUniversal (world,null);
			dJointAttach (jointU,body[0],body[1]);
			dJointSetUniversalAnchor (jointU,0,0,1);
			dJointSetUniversalAxis1 (jointU, 1, -1, 1.41421356);
			dJointSetUniversalAxis2 (jointU, 1, -1, -1.41421356);
			return 1;

		case 720:		// universal transmit torque test
		case 721:
		case 722:
		case 730:		// universal torque about axis 1
		case 731:
		case 732:
		case 740:		// universal torque about axis 2
		case 741:
		case 742:
			constructWorldForTest (0,2,
					0.5*SIDE,0.5*SIDE,1, -0.5*SIDE,-0.5*SIDE,1,
					1,0,0, 1,0,0, 0,0);
			jointU = dJointCreateUniversal (world,null);
			dJointAttach (jointU,body[0],body[1]);
			dJointSetUniversalAnchor (jointU,0,0,1);
			dJointSetUniversalAxis1 (jointU,0,0,1);
			dJointSetUniversalAxis2 (jointU, 1, -1,0);
			max_iterations = 100;
			return 1;

			// Joint PR (Prismatic and Rotoide)
		case 800:     // 2 body
		case 801:     // 2 bodies with spring force and prismatic fixed
		case 802:     // 2 bodies with torque on body1 and prismatic fixed
			constructWorldForTest (0, 2,
					-1.0, 0.0, 1.0,
					1.0, 0.0, 1.0,
					1,0,0, 1,0,0,
					0, 0);
			jointPR = dJointCreatePR (world, null);
			dJointAttach (jointPR, body[0], body[1]);
			dJointSetPRAnchor (jointPR,-0.5, 0.0, 1.0);
			dJointSetPRAxis1 (jointPR, 0, 1, 0);
			dJointSetPRAxis2 (jointPR, 1, 0, 0);
			dJointSetPRParam (jointPR,dParamLoStop,-0.5);
			dJointSetPRParam (jointPR,dParamHiStop,0.5);
			dJointSetPRParam (jointPR,dParamLoStop2,0);
			dJointSetPRParam (jointPR,dParamHiStop2,0);
			return 1;
		case 803:   // 2 bodies with spring force and prismatic NOT fixed
		case 804:   // 2 bodies with torque force and prismatic NOT fixed
		case 805:   // 2 bodies with force only on first body
			constructWorldForTest (0, 2,
					-1.0, 0.0, 1.0,
					1.0, 0.0, 1.0,
					1,0,0, 1,0,0,
					0, 0);
			jointPR = dJointCreatePR (world, null);
			dJointAttach (jointPR, body[0], body[1]);
			dJointSetPRAnchor (jointPR,-0.5, 0.0, 1.0);
			dJointSetPRAxis1 (jointPR, 0, 1, 0);
			dJointSetPRAxis2 (jointPR, 1, 0, 0);
			dJointSetPRParam (jointPR,dParamLoStop,-0.5);
			dJointSetPRParam (jointPR,dParamHiStop,0.5);
			dJointSetPRParam (jointPR,dParamLoStop2,-0.5);
			dJointSetPRParam (jointPR,dParamHiStop2,0.5);
			return 1;
		}
		return 0;
	}

	private static double last_angle_221 = 0;
	private static double a_231 = 0;
	private static double last_pos_321 = 0;
	private static double a_331 = 0;
	private static double last_angle_421 = 0;
	private static double a_431 = 0;
	private static double a_432 = 0;
	private static double a1_600,a2_600,a3_600;
	private static double last_angle_701 = 0;
	private static double last_angle_702 = 0;
	private static double last_angle_721 = 0;
	private static double last_angle_722 = 0;
	private static double last_angle_731 = 0;
	private static double last_angle_732 = 0;
	private static double last_angle_741 = 0;
	private static double last_angle_742 = 0;
	private static double a_804 = 0;

	// do stuff specific to this test each iteration. you can check some
	// invariants for the test -- the return value is some scaled error measurement
	// that must be less than 1.
	// return a dInfinity if error is not measured for this n.

	private double doStuffAndGetError (int n)
	{
		switch (n) {

		// ********** fixed joint

		case 0: {			// 2 body
			addOscillatingTorque (0.1);
			dampRotationalMotion (0.1);
			// check the orientations are the same
			DMatrix3C R1 = dBodyGetRotation (body[0]);
			DMatrix3C R2 = dBodyGetRotation (body[1]);
			double err1 = dMaxDifference (R1,R2);
			// check the body offset is correct
			DVector3 p = new DVector3(),pp = new DVector3();
			DVector3C p1 = dBodyGetPosition (body[0]);
			DVector3C p2 = dBodyGetPosition (body[1]);
			//    for (int i=0; i<3; i++) p.v[i] = p2.v[i] - p1.v[i];
			p.eqDiff(p2, p1);
			OdeMath.dMultiply1_331 (pp,R1,p);
			//    pp.v[0] += 0.5;
			//    pp.v[1] += 0.5;
			pp.add(0.5, 0.5, 0);
			return (err1 + length (pp)) * 300;
		}

		case 1: {			// 1 body to static env
			addOscillatingTorque (0.1);

			// check the orientation is the identity
			double err1 = cmpIdentity (dBodyGetRotation (body[0]));

			// check the body offset is correct
			DVector3 p = new DVector3();
			DVector3C p1 = dBodyGetPosition (body[0]);
			//    for (int i=0; i<3; i++) p.v[i] = p1.v[i];
			//    p.v[0] -= 0.25;
			//    p.v[1] -= 0.25;
			//    p.v[2] -= 1;
			p.set(p1).sub(0.25, 0.25, 1);
			return (err1 + length (p)) * 1e6;
		}

		case 2: {			// 2 body
			addOscillatingTorque (0.1);
			dampRotationalMotion (0.1);
			// check the body offset is correct
			// Should really check body rotation too.  Oh well.
			DMatrix3C R1 = dBodyGetRotation (body[0]);
			DVector3 p = new DVector3(),pp = new DVector3();
			DVector3C p1 = dBodyGetPosition (body[0]);
			DVector3C p2 = dBodyGetPosition (body[1]);
			//    for (int i=0; i<3; i++) p.v[i] = p2.v[i] - p1.v[i];
			p.eqDiff(p2, p1);
			dMultiply1_331 (pp,R1,p);
			//    pp.v[0] += 0.5;
			//    pp.v[1] += 0.5;
			pp.add(0.5, 0.5, 0);
			return length(pp) * 300;
		}

		case 3: {			// 1 body to static env with relative rotation
			addOscillatingTorque (0.1);

			// check the body offset is correct
			DVector3 p = new DVector3();
			DVector3C p1 = dBodyGetPosition (body[0]);
			//    for (int i=0; i<3; i++) p[i] = p1[i];
			//    p[0] -= 0.25;
			//    p[1] -= 0.25;
			//    p[2] -= 1;
			p.set(p1).sub(0.25, 0.25, 1);
			return  length (p) * 1e6;
		}


		// ********** hinge joint

		case 200:			// 2 body
			addOscillatingTorque (0.1);
			dampRotationalMotion (0.1);
			return dInfinity;

		case 220:			// hinge angle polarity test
			dBodyAddTorque (body[0],0,0,0.01);
			dBodyAddTorque (body[1],0,0,-0.01);
			if (iteration == 40) {
				double a = dJointGetHingeAngle (jointH);
				if (a > 0.5 && a < 1) return 0; else return 10;
			}
			return 0;

		case 221: {			// hinge angle rate test
			//TZ static double last_angle = 0;
			dBodyAddTorque (body[0],0,0,0.01);
			dBodyAddTorque (body[1],0,0,-0.01);
			double a = dJointGetHingeAngle (jointH);
			double r = dJointGetHingeAngleRate (jointH);
			double er = (a-last_angle_221)/STEPSIZE;		// estimated rate
			last_angle_221 = a;
			return fabs(r-er) * 4e4;
		}

		case 230:			// hinge motor rate (and polarity) test
		case 231: {			// ...with stops
			//TZ static double a = 0;
			double r = dJointGetHingeAngleRate (jointH);
			double err = fabs (cos(a_231) - r);
			if (a_231==0) err = 0;
			a_231 += 0.03;
			dJointSetHingeParam (jointH,dParamVel,cos(a_231));
			if (n==231) return dInfinity;
			return err * 1e6;
		}

		// ********** slider joint

		case 300:			// 2 body
			addOscillatingTorque (0.05);
			dampRotationalMotion (0.1);
			addSpringForce (0.5);
			return dInfinity;

		case 320:			// slider angle polarity test
			dBodyAddForce (body[0],0,0,0.1);
			dBodyAddForce (body[1],0,0,-0.1);
			if (iteration == 40) {
				double a = dJointGetSliderPosition (jointS);
				if (a > 0.2 && a < 0.5) 
					return 0; 
				else 
					return 10;
			}
			return 0;

		case 321: {			// slider angle rate test
			//TZ static double last_pos = 0;
			dBodyAddForce (body[0],0,0,0.1);
			dBodyAddForce (body[1],0,0,-0.1);
			double p = dJointGetSliderPosition (jointS);
			double r = dJointGetSliderPositionRate (jointS);
			double er = (p-last_pos_321)/STEPSIZE;	// estimated rate (almost exact)
			last_pos_321 = p;
			return fabs(r-er) * 1e9;
		}

		case 330:			// slider motor rate (and polarity) test
		case 331: {			// ...with stops
			//TZ static double a = 0;
			double r = dJointGetSliderPositionRate (jointS);
			double err = fabs (0.7*cos(a_331) - r);
			if (a_331 < 0.04) err = 0;
			a_331 += 0.03;
			dJointSetSliderParam (jointS,dParamVel,0.7*cos(a_331));
			if (n==331) return dInfinity;
			return err * 1e6;
		}

		// ********** hinge-2 joint

		case 420:			// hinge-2 steering angle polarity test
			dBodyAddTorque (body[0],0,0,0.01);
			dBodyAddTorque (body[1],0,0,-0.01);
			if (iteration == 40) {
				double a = dJointGetHinge2Angle1 (jointH2);
				if (a > 0.5 && a < 0.6) return 0; else return 10;
			}
			return 0;

		case 421: {			// hinge-2 steering angle rate test
			//TZ static double last_angle = 0;
			dBodyAddTorque (body[0],0,0,0.01);
			dBodyAddTorque (body[1],0,0,-0.01);
			double a = dJointGetHinge2Angle1 (jointH2);
			double r = dJointGetHinge2Angle1Rate (jointH2);
			double er = (a-last_angle_421)/STEPSIZE;		// estimated rate
			last_angle_421 = a;
			return fabs(r-er)*2e4;
		}

		case 430:			// hinge 2 steering motor rate (+polarity) test
		case 431: {			// ...with stops
			//TZ static double a = 0;
			double r = dJointGetHinge2Angle1Rate (jointH2);
			double err = fabs (cos(a_431) - r);
			if (a_431==0) err = 0;
			a_431 += 0.03;
			dJointSetHinge2Param (jointH2,dParamVel,cos(a_431));
			if (n==431) return dInfinity;
			return err * 1e6;
		}

		case 432: {			// hinge 2 wheel motor rate (+polarity) test
			//TZ static double a = 0;
			double r = dJointGetHinge2Angle2Rate (jointH2);
			double err = fabs (cos(a_432) - r);
			if (a_432==0) err = 0;
			a_432 += 0.03;
			dJointSetHinge2Param (jointH2,dParamVel2,cos(a_432));
			return err * 1e6;
		}

		// ********** angular motor joint

		case 600: {			// test euler angle calculations
			// desired euler angles from last iteration
			//TZ static double a1,a2,a3;

			// find actual euler angles
			double aa1 = dJointGetAMotorAngle (jointAM,0);
			double aa2 = dJointGetAMotorAngle (jointAM,1);
			double aa3 = dJointGetAMotorAngle (jointAM,2);
			// printf ("actual  = %.4f %.4f %.4f\n\n",aa1,aa2,aa3);

			double err = dInfinity;
			if (iteration > 0) {
				err = dFabs(aa1-a1_600) + dFabs(aa2-a2_600) + dFabs(aa3-a3_600);
				err *= 1e10;
			}

			// get random base rotation for both bodies
			DMatrix3 Rbase = new DMatrix3();
			dRFromAxisAndAngle (Rbase, 3*(dRandReal()-0.5), 3*(dRandReal()-0.5),
					3*(dRandReal()-0.5), 3*(dRandReal()-0.5));
			dBodySetRotation (body[0],Rbase);

			// rotate body 2 by random euler angles w.r.t. body 1
			a1_600 = 3.14 * 2 * (dRandReal()-0.5);
			a2_600 = 1.57 * 2 * (dRandReal()-0.5);
			a3_600 = 3.14 * 2 * (dRandReal()-0.5);
			DMatrix3 R1 = new DMatrix3(),R2 = new DMatrix3(),R3 = new DMatrix3();
			DMatrix3 Rtmp1 = new DMatrix3(),Rtmp2 = new DMatrix3();
			dRFromAxisAndAngle (R1,0,0,1,-a1_600);
			dRFromAxisAndAngle (R2,0,1,0,a2_600);
			dRFromAxisAndAngle (R3,1,0,0,-a3_600);
			Rtmp1.eqMul(R2, R3);//dMultiply0 (Rtmp1,R2,R3,3,3,3);
			Rtmp2.eqMul(R1, Rtmp1);//dMultiply0 (Rtmp2,R1,Rtmp1,3,3,3);
			Rtmp1.eqMul(Rbase, Rtmp2);//dMultiply0 (Rtmp1,Rbase,Rtmp2,3,3,3);
			dBodySetRotation (body[1],Rtmp1);
			// printf ("desired = %.4f %.4f %.4f\n",a1,a2,a3);

			return err;
		}

		// ********** universal joint

		case 700: {		// 2 body: joint constraint
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();

			addOscillatingTorque (0.1);
			dampRotationalMotion (0.1);
			dJointGetUniversalAxis1(jointU, ax1);
			dJointGetUniversalAxis2(jointU, ax2);
			return fabs(10*OdeMath.dCalcVectorDot3(ax1, ax2));
		}

		case 701: {		// 2 body: angle 1 rate
			//TZ static double last_angle = 0;
			addOscillatingTorque (0.1);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle1(jointU);
			double r = dJointGetUniversalAngle1Rate(jointU);
			double diff = a - last_angle_701;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_701 = a;
			// I'm not sure why the error is so large here.
			return fabs(r - er) * 1e1;
		}

		case 702: {		// 2 body: angle 2 rate
			//TZ static double last_angle = 0;
			addOscillatingTorque (0.1);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle2(jointU);
			double r = dJointGetUniversalAngle2Rate(jointU);
			double diff = a - last_angle_702;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_702 = a;
			// I'm not sure why the error is so large here.
			return fabs(r - er) * 1e1;
		}

		case 720: {		// universal transmit torque test: constraint error
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			addOscillatingTorqueAbout (0.1, 1, 1, 0);
			dampRotationalMotion (0.1);
			dJointGetUniversalAxis1(jointU, ax1);
			dJointGetUniversalAxis2(jointU, ax2);
			return fabs(10*dCalcVectorDot3(ax1, ax2));
		}

		case 721: {		// universal transmit torque test: angle1 rate
			//TZ static double last_angle = 0;
			addOscillatingTorqueAbout (0.1, 1, 1, 0);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle1(jointU);
			double r = dJointGetUniversalAngle1Rate(jointU);
			double diff = a - last_angle_721;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_721 = a;
			return fabs(r - er) * 1e10;
		}

		case 722: {		// universal transmit torque test: angle2 rate
			//TZ static double last_angle = 0;
			addOscillatingTorqueAbout (0.1, 1, 1, 0);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle2(jointU);
			double r = dJointGetUniversalAngle2Rate(jointU);
			double diff = a - last_angle_722;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_722 = a;
			return fabs(r - er) * 1e10;
		}

		case 730:{
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			dJointGetUniversalAxis1(jointU, ax1);
			dJointGetUniversalAxis2(jointU, ax2);
			addOscillatingTorqueAbout (0.1, ax1);
			dampRotationalMotion (0.1);
			return fabs(10*dCalcVectorDot3(ax1, ax2));
		}

		case 731:{
			DVector3 ax1 = new DVector3();
			//TZ static double last_angle = 0;
			dJointGetUniversalAxis1(jointU, ax1);
			addOscillatingTorqueAbout (0.1, ax1);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle1(jointU);
			double r = dJointGetUniversalAngle1Rate(jointU);
			double diff = a - last_angle_731;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_731 = a;
			return fabs(r - er) * 2e3;
		}

		case 732:{
			DVector3 ax1 = new DVector3();
			//TZ static double last_angle = 0;
			dJointGetUniversalAxis1(jointU, ax1);
			addOscillatingTorqueAbout (0.1, ax1);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle2(jointU);
			double r = dJointGetUniversalAngle2Rate(jointU);
			double diff = a - last_angle_732;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_732 = a;
			return fabs(r - er) * 1e10;
		}

		case 740:{
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			dJointGetUniversalAxis1(jointU, ax1);
			dJointGetUniversalAxis2(jointU, ax2);
			addOscillatingTorqueAbout (0.1, ax2);
			dampRotationalMotion (0.1);
			return fabs(10*dCalcVectorDot3(ax1, ax2));
		}

		case 741:{
			DVector3 ax2 = new DVector3();
			//TZ static double last_angle = 0;
			dJointGetUniversalAxis2(jointU, ax2);
			addOscillatingTorqueAbout (0.1, ax2);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle1(jointU);
			double r = dJointGetUniversalAngle1Rate(jointU);
			double diff = a - last_angle_741;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_741 = a;
			return fabs(r - er) * 1e10;
		}

		case 742:{
			DVector3 ax2 = new DVector3();
			//TZ static double last_angle = 0;
			dJointGetUniversalAxis2(jointU, ax2);
			addOscillatingTorqueAbout (0.1, ax2);
			dampRotationalMotion (0.1);
			double a = dJointGetUniversalAngle2(jointU);
			double r = dJointGetUniversalAngle2Rate(jointU);
			double diff = a - last_angle_742;
			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			double er = diff / STEPSIZE;    // estimated rate
			last_angle_742 = a;
			return fabs(r - er) * 1e4;
		}

		// ********** slider joint
		case 801:
		case 803:
			addSpringForce (0.25);
			return dInfinity;

		case 802:
		case 804: {
			//TZ static double a = 0;
			dBodyAddTorque (body[0], 0, 0.01*cos(1.5708*a_804), 0);
			a_804 += 0.01;
			return dInfinity;
		}

		case 805:
			addOscillatingForce (0.1);
			return dInfinity;
		}


		return dInfinity;
	}

	//****************************************************************************
	// simulation stuff common to all the tests

	// start simulation - set viewpoint

	private static float[] xyz = {1.0382f,-1.0811f,1.4700f};
	private static float[] hpr = {135.0000f,-19.5000f,0.0000f};
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = {1.0382f,-1.0811f,1.4700f};
		//  static float hpr[3] = {135.0000f,-19.5000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
	}


	private static int count = 0;
	// simulation loop

	//static 
	private void simLoop (boolean pause)
	{
		// stop after a given number of iterations, as long as we are not in
		// interactive mode
		if (cmd_graphics && !cmd_interactive &&
				(iteration >= max_iterations)) {
			dsStop();
			return;
		}
		iteration++;

		if (!pause) {
			// do stuff for this test and check to see if the joint is behaving well
			double error = doStuffAndGetError (test_num);
			if (error > max_error) max_error = (float) error;
			if (cmd_interactive && error < dInfinity) {
				printf ("scaled error = %.4e\n",error);
			}

			// take a step
			dWorldStep (world,STEPSIZE);

			// occasionally re-orient the first body to create a deliberate error.
			if (cmd_occasional_error != 0) {
				//TZ      static int count = 0;
				if ((count % 20)==0) {
					// randomly adjust orientation of body[0]
					DMatrix3C R1;
					DMatrix3 R2 = new DMatrix3(),R3 = new DMatrix3();
					R1 = dBodyGetRotation (body[0]);
					dRFromAxisAndAngle (R2,dRandReal()-0.5,dRandReal()-0.5,
							dRandReal()-0.5,dRandReal()-0.5);
					R3.eqMul(R1, R2);//dMultiply0 (R3,R1,R2,3,3,3);
					dBodySetRotation (body[0],R3);

					// randomly adjust position of body[0]
					DVector3C pos = dBodyGetPosition (body[0]);
					dBodySetPosition (body[0],
							pos.get0()+0.2*(dRandReal()-0.5),
							pos.get1()+0.2*(dRandReal()-0.5),
							pos.get2()+0.2*(dRandReal()-0.5));
				}
				count++;
			}
		}

		if (cmd_graphics) {
			//    double sides1[3] = {SIDE,SIDE,SIDE};
			//    double sides2[3] = {SIDE*0.99f,SIDE*0.99f,SIDE*0.99f};
			DVector3 sides1 = new DVector3(SIDE,SIDE,SIDE);
			DVector3 sides2 = new DVector3(SIDE*0.99f,SIDE*0.99f,SIDE*0.99f);
			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
			dsSetColor (1,1,0);
			dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides1);
			if (body[1]!=null) {
				dsSetColor (0,1,1);
				dsDrawBox (dBodyGetPosition(body[1]),dBodyGetRotation(body[1]),sides2);
			}
		}
	}

	//****************************************************************************
	// conduct a specific test, and report the results

	private void doTest (String[] args, int n, int fatal_if_bad_n)
	{
		test_num = n;
		iteration = 0;
		max_iterations = 300;
		max_error = 0;

		if (setupTest (n) == 0) {
			if (fatal_if_bad_n!=0) dError (0,"bad test number");
			return;
		}

//		if (cmd_path_to_textures != null)
//			dsSetPathToTextures( cmd_path_to_textures );
//		else
//			fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// run simulation
		if (cmd_graphics) {
			dsSimulationLoop (args,640,480,this);
			//dsSimulationLoop (args,288,288,fn);
			//dsSimulationLoop (args,400,400,fn);
		}
		else {
			for (int i=0; i < max_iterations; i++) simLoop (false);
		}
		dWorldDestroy (world);
		body[0] = null;
		body[1] = null;
		jointAM = null;
		jointF = null;
		jointH = null;
		jointH2 = null;
		jointPR = null;
		jointS = null;
		jointU = null;

		// print results
		printf ("test %d: ",n);
		if (max_error == dInfinity) printf ("error not computed\n");
		else {
			printf ("max scaled error = %.4e",max_error);
			if (max_error < 1) printf (" - passed\n");
			else printf (" - FAILED\n");
		}
	}

	//****************************************************************************
	// main

	public static void main (String[] args)
	{
		int i;
		dInitODE2(0);

		// process the command line args. anything that starts with `-' is assumed
		// to be a drawstuff argument.
		for (i=0; i<args.length; i++) {
			//    if ( args[i][0]=='-' && args[i][1]=='i' && args[i][2]==0) cmd_interactive = true;
			//	  else if ( args[i][0]=='-' && args[i][1]=='g' && args[i][2]==0) cmd_graphics = false;
			//else if ( args[i][0]=='-' && args[i][1]=='e' && args[i][2]==0) cmd_graphics = false;
			//	    else if ( args[i][0]=='-' && args[i][1]=='n' && isdigit(args[i][2]) ) {
			//	        char *endptr;
			//	        long int n = strtol (&(argv[i][2]),&endptr,10);
			//	  			if (*endptr == 0) cmd_test_num = n;
			//	  		}
			if (args[i].equals("-i")) { cmd_interactive = true; args[i] = ""; }
			if (args[i].equals("-g")) { cmd_graphics = false; args[i] = ""; }
			if (args[i].equals("-e")) { cmd_graphics = false; args[i] = ""; }
			if (args[i].startsWith("-n") && Ctype.isdigit(args[i].charAt(2))) {
				cmd_test_num = Integer.parseInt(args[i].substring(2));
				 args[i] = ""; 
			}
//			else
//				cmd_path_to_textures = args[i];
		}

		DemoJoints demo = new DemoJoints();

		// do the tests
		if (cmd_test_num == -1) {
			for (i=0; i<NUM_JOINTS*100; i++) demo.doTest (args,i,0);
		}
		else {
			demo.doTest (args,cmd_test_num,1);
		}

		dCloseODE();
	}


	@Override
	public void command(char cmd) {
		//Nothing
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
		// Nothing
	}
}