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
package org.ode4j.ode;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import org.ode4j.math.*;
import org.ode4j.ode.DAMotorJoint.AMotorMode;

import static org.ode4j.ode.OdeMath.*;


/**
 * perform tests on all the joint types.
 * this should be done using the double precision version of the library.
 * <p>
 * usage:
 * test_joints [-nXXX] [-g] [-i] [-e] [path_to_textures]
 * <p>
 * if a test number is given then that specific test is performed, otherwise
 * all the tests are performed. the tests are numbered `xxyy', where xx
 * corresponds to the joint type and yy is the sub-test number. not every
 * number maps to an actual test.
 * <p>
 * flags:
 * i: the test is interactive.
 * g: turn off graphical display (can't use this with `i').
 * e: turn on occasional error perturbations
 * n: perform test XYZ
 * some tests compute and display error values. these values are scaled so
 * less than 1 is good and greater than 1 is bad. other tests just show graphical results which
 * you must verify visually.
 */
//@RunWith(Parameterized.class)
public class DemoJointsTest {

    private static final boolean PRINT = false;

    //// some constants
    //#define NUM_JOINTS 10	// number of joints to test (the `xx' value)
    //#define SIDE (0.5f)	// side length of a box - don't change this
    //#define MASS (1.0)	// mass of a box
    //#define STEPSIZE 0.05
    //some constants
    private static final int NUM_JOINTS = 10;    // number of joints to test (the `xx' value)
    private static final float SIDE = 0.5f;    // side length of a box - don't change this
    private static final float MASS = 1.0f;    // mass of a box
    private static final float STEPSIZE = 0.05f;

    private static final DVector3C xunit = new DVector3(1, 0, 0);
    private static final DVector3C zunit = new DVector3(0, 0, 1);


    // dynamics objects
    private static DWorld world;
    private static final DBody[] body = new DBody[2];
    //	private static DJoint joint;
    private static DAMotorJoint jointAMotor;
    private static DFixedJoint jointFixed;
    private static DHingeJoint jointHinge;
    private static DHinge2Joint jointHinge2;
    private static DPRJoint jointPR;
    private static DSliderJoint jointSlider;
    private static DUniversalJoint jointUniversal;


    private static int cmd_occasional_error = 0;    // perturb occasionally


    // info about the current test
    //struct TestInfo;
    private static int test_num = 0;        // number of the current test
    private static int iteration = 0;
    private static int max_iterations = 0;
    private static float max_error = 0;

    // static helpers
    static class PseudoStatic {
        double a_1 = 0;
        double a_2 = 0;
        double a_3 = 0;
        double last_angle_221 = 0;
        double a_231 = 0;
        double last_pos_321 = 0;
        double a_331 = 0;
        double last_angle_421 = 0;
        double a_431 = 0;
        double a_432 = 0;
        double a1_600, a2_600, a3_600;
        double last_angle_701 = 0;
        double last_angle_702 = 0;
        double last_angle_721 = 0;
        double last_angle_722 = 0;
        double last_angle_731 = 0;
        double last_angle_732 = 0;
        double last_angle_741 = 0;
        double last_angle_742 = 0;
        double a_804 = 0;

        int count = 0;
    }

    private PseudoStatic S = null;

    //****************************************************************************
    // utility stuff

    // get the max difference between a 3x3 matrix and the identity

    private double cmpIdentity(final DMatrix3C A) {
        DMatrix3 I = new DMatrix3();
        I.eqIdentity();
        return dMaxDifference(A, I);
    }

    private static void print(String s) {
        if (PRINT) {
            System.out.print(s);
        }
    }

    private static void println(String s) {
        if (PRINT) {
            System.out.println(s);
        }
    }

    //****************************************************************************
    // test world construction and utilities
    private void constructWorldForTest(double gravity, int bodycount,
            /* body 1 pos */           double pos1x, double pos1y, double pos1z,
            /* body 2 pos */           double pos2x, double pos2y, double pos2z,
            /* body 1 rotation axis */ double ax1x, double ax1y, double ax1z,
            /* body 1 rotation axis */ double ax2x, double ax2y, double ax2z,
            /* rotation angles */      double a1, double a2) {
        // create world
        world = OdeHelper.createWorld();
        world.setERP(0.2);
        world.setCFM(1e-6);
        world.setGravity(0, 0, gravity);

        DMass m = OdeHelper.createMass();
        m.setBox(1, SIDE, SIDE, SIDE);
        m.adjust(MASS);

        body[0] = OdeHelper.createBody(world);
        body[0].setMass(m);
        body[0].setPosition(pos1x, pos1y, pos1z);
        DQuaternion q = new DQuaternion();
        dQFromAxisAndAngle(q, ax1x, ax1y, ax1z, a1);
        body[0].setQuaternion(q);

        if (bodycount == 2) {
            body[1] = OdeHelper.createBody(world);
            body[1].setMass(m);
            body[1].setPosition(pos2x, pos2y, pos2z);
            dQFromAxisAndAngle(q, ax2x, ax2y, ax2z, a2);
            body[1].setQuaternion(q);
        } else body[1] = null;
    }


    // add an oscillating torque to body 0


    private void addOscillatingTorque(double tscale) {
        //TZ  static double a=0;
        body[0].addTorque(tscale * Math.cos(2 * S.a_1), tscale * Math.cos(2.7183 * S.a_1),
                tscale * Math.cos(1.5708 * S.a_1));
        S.a_1 += 0.01;
    }

    private void addOscillatingTorqueAbout(double tscale, double x, double y, double z) {
        //TZ  static double a=0;
        body[0].addTorque(tscale * Math.cos(S.a_2) * x, tscale * Math.cos(S.a_2) * y,
                tscale * Math.cos(S.a_2) * z);
        S.a_2 += 0.02;
    }

    private void addOscillatingTorqueAbout(double tscale, DVector3 v) {
        //TZ  static double a=0;
        body[0].addTorque(tscale * Math.cos(S.a_2) * v.get0(), tscale * Math.cos(S.a_2) * v.get1(),
                tscale * Math.cos(S.a_2) * v.get2());
        S.a_2 += 0.02;
    }


    // damp the rotational motion of body 0 a bit

    private void dampRotationalMotion(double kd) {
        DVector3C w = body[0].getAngularVel();
        body[0].addTorque(-kd * w.get0(), -kd * w.get1(), -kd * w.get2());
    }


    // add a spring force to keep the bodies together, otherwise they may fly
    // apart with some joints.

    private void addSpringForce(double ks) {
        DVector3C p1 = body[0].getPosition();
        DVector3C p2 = body[1].getPosition();
        DVector3 tmp = new DVector3();
        //dBodyAddForce (body[0],ks*(p2.v[0]-p1.v[0]),ks*(p2.v[1]-p1.v[1]),ks*(p2.v[2]-p1.v[2]));
        tmp.eqDiff(p2, p1).scale(ks);
        body[0].addForce(tmp);
        //dBodyAddForce (body[1],ks*(p1.v[0]-p2.v[0]),ks*(p1.v[1]-p2.v[1]),ks*(p1.v[2]-p2.v[2]));
        tmp.eqDiff(p1, p2).scale(ks);
        body[1].addForce(tmp);
    }


    // add an oscillating Force to body 0

    private void addOscillatingForce(double fscale) {
        //TZ static double a=0;
        body[0].addForce(fscale * Math.cos(2 * S.a_3), fscale * Math.cos(2.7183 * S.a_3),
                fscale * Math.cos(1.5708 * S.a_3));
        S.a_3 += 0.01;
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

    private int setupTest(int n) {
        switch (n) {

            // ********** fixed joint

            case 0: {            // 2 body
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 1, 0, 1, 1, 0,
                        0.25 * M_PI, 0.25 * M_PI);
                jointFixed = OdeHelper.createFixedJoint(world, null);
                jointFixed.attach(body[0], body[1]);
                jointFixed.setFixed();
                return 1;
            }

            case 1: {            // 1 body to static env
                constructWorldForTest(0, 1,
                        0.5 * SIDE, 0.5 * SIDE, 1, 0, 0, 0,
                        1, 0, 0, 1, 0, 0,
                        0, 0);
                jointFixed = OdeHelper.createFixedJoint(world, null);
                jointFixed.attach(body[0], null);
                jointFixed.setFixed();
                return 1;
            }

            case 2: {            // 2 body with relative rotation
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 1, 0, 1, 1, 0,
                        0.25 * M_PI, -0.25 * M_PI);
                jointFixed = OdeHelper.createFixedJoint(world, null);
                jointFixed.attach(body[0], body[1]);
                jointFixed.setFixed();
                return 1;
            }

            case 3: {            // 1 body to static env with relative rotation
                constructWorldForTest(0, 1,
                        0.5 * SIDE, 0.5 * SIDE, 1, 0, 0, 0,
                        1, 0, 0, 1, 0, 0,
                        0.25 * M_PI, 0);
                jointFixed = OdeHelper.createFixedJoint(world, null);
                jointFixed.attach(body[0], null);
                jointFixed.setFixed();
                return 1;
            }

            // ********** hinge joint

            case 200:            // 2 body
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 1, 0, 1, 1, 0, 0.25 * M_PI, 0.25 * M_PI);
                jointHinge = OdeHelper.createHingeJoint(world, null);
                jointHinge.attach(body[0], body[1]);
                jointHinge.setAnchor(0, 0, 1);
                jointHinge.setAxis(1, -1, 1.41421356);
                return 1;

            case 220:            // hinge angle polarity test
            case 221:            // hinge angle rate test
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointHinge = OdeHelper.createHingeJoint(world, null);
                jointHinge.attach(body[0], body[1]);
                jointHinge.setAnchor(0, 0, 1);
                jointHinge.setAxis(0, 0, 1);
                max_iterations = 50;
                return 1;

            case 230:            // hinge motor rate (and polarity) test
            case 231:            // ...with stops
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointHinge = OdeHelper.createHingeJoint(world, null);
                jointHinge.attach(body[0], body[1]);
                jointHinge.setAnchor(0, 0, 1);
                jointHinge.setAxis(0, 0, 1);
                jointHinge.setParamFMax(1);
                if (n == 231) {
                    jointHinge.setParamLoStop(-0.5);
                    jointHinge.setParamHiStop(0.5);
                }
                return 1;

            case 250:            // limit bounce test (gravity down)
            case 251: {            // ...gravity up
                constructWorldForTest((n == 251) ? 0.1 : -0.1, 2,
                        0.5 * SIDE, 0, 1 + 0.5 * SIDE, -0.5 * SIDE, 0, 1 - 0.5 * SIDE,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointHinge = OdeHelper.createHingeJoint(world, null);
                jointHinge.attach(body[0], body[1]);
                jointHinge.setAnchor(0, 0, 1);
                jointHinge.setAxis(0, 1, 0);
                jointHinge.setParamLoStop(-0.9);
                jointHinge.setParamHiStop(0.7854);
                jointHinge.setParamBounce(0.5);
                // anchor 2nd body with a fixed joint
                DFixedJoint j = OdeHelper.createFixedJoint(world, null);
                j.attach(body[1], null);
                j.setFixed();
                return 1;
            }

            // ********** slider

            case 300:            // 2 body
                constructWorldForTest(0, 2,
                        0, 0, 1, 0.2, 0.2, 1.2,
                        0, 0, 1, -1, 1, 0, 0, 0.25 * M_PI);
                jointSlider = OdeHelper.createSliderJoint(world, null);
                jointSlider.attach(body[0], body[1]);
                jointSlider.setAxis(1, 1, 1);
                return 1;

            case 320:            // slider angle polarity test
            case 321:            // slider angle rate test
                constructWorldForTest(0, 2,
                        0, 0, 1, 0, 0, 1.2,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointSlider = OdeHelper.createSliderJoint(world, null);
                jointSlider.attach(body[0], body[1]);
                jointSlider.setAxis(0, 0, 1);
                max_iterations = 50;
                return 1;

            case 330:            // slider motor rate (and polarity) test
            case 331:            // ...with stops
                constructWorldForTest(0, 2,
                        0, 0, 1, 0, 0, 1.2,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointSlider = OdeHelper.createSliderJoint(world, null);
                jointSlider.attach(body[0], body[1]);
                jointSlider.setAxis(0, 0, 1);
                jointSlider.setParamFMax(100);
                if (n == 331) {
                    jointSlider.setParamLoStop(-0.4);
                    jointSlider.setParamHiStop(0.4);
                }
                return 1;

            case 350:            // limit bounce tests
            case 351: {
                constructWorldForTest((n == 351) ? 0.1 : -0.1, 2,
                        0, 0, 1, 0, 0, 1.2,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointSlider = OdeHelper.createSliderJoint(world, null);
                jointSlider.attach(body[0], body[1]);
                jointSlider.setAxis(0, 0, 1);
                jointSlider.setParamLoStop(-0.5);
                jointSlider.setParamHiStop(0.5);
                jointSlider.setParamBounce(0.5);
                // anchor 2nd body with a fixed joint
                DFixedJoint j = OdeHelper.createFixedJoint(world, null);
                j.attach(body[1], null);
                j.setFixed();
                return 1;
            }

            // ********** hinge-2 joint

            case 420:            // hinge-2 steering angle polarity test
            case 421:            // hinge-2 steering angle rate test
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0, 1, -0.5 * SIDE, 0, 1,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointHinge2 = OdeHelper.createHinge2Joint(world, null);
                jointHinge2.attach(body[0], body[1]);
                jointHinge2.setAnchor(-0.5 * SIDE, 0, 1);
                jointHinge2.setAxes(zunit, xunit);
                max_iterations = 50;
                return 1;

            case 430:            // hinge 2 steering motor rate (+polarity) test
            case 431:            // ...with stops
            case 432:            // hinge 2 wheel motor rate (+polarity) test
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0, 1, -0.5 * SIDE, 0, 1,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointHinge2 = OdeHelper.createHinge2Joint(world, null);
                jointHinge2.attach(body[0], body[1]);
                jointHinge2.setAnchor(-0.5 * SIDE, 0, 1);
                jointHinge2.setAxes(zunit, xunit);
                jointHinge2.setParamFMax(1);
                jointHinge2.setParamFMax2(1);
                if (n == 431) {
                    jointHinge2.setParamLoStop(-0.5);
                    jointHinge2.setParamHiStop(0.5);
                }
                return 1;

            // ********** angular motor joint

            case 600:            // test euler angle calculations
                constructWorldForTest(0, 2,
                        -SIDE * 0.5, 0, 1, SIDE * 0.5, 0, 1,
                        0, 0, 1, 0, 0, 1, 0, 0);
                jointAMotor = OdeHelper.createAMotorJoint(world, null);
                jointAMotor.attach(body[0], body[1]);

                jointAMotor.setNumAxes(3);
                jointAMotor.setAxis(0, 1, 0, 0, 1);
                jointAMotor.setAxis(2, 2, 1, 0, 0);
                jointAMotor.setMode(AMotorMode.dAMotorEuler);
                max_iterations = 200;
                return 1;

            // ********** universal joint

            case 700:            // 2 body
            case 701:
            case 702:
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 1, 0, 1, 1, 0, 0.25 * M_PI, 0.25 * M_PI);
                jointUniversal = OdeHelper.createUniversalJoint(world, null);
                jointUniversal.attach(body[0], body[1]);
                jointUniversal.setAnchor(0, 0, 1);
                jointUniversal.setAxis1(1, -1, 1.41421356);
                jointUniversal.setAxis2(1, -1, -1.41421356);
                return 1;

            case 720:        // universal transmit torque test
            case 721:
            case 722:
            case 730:        // universal torque about axis 1
            case 731:
            case 732:
            case 740:        // universal torque about axis 2
            case 741:
            case 742:
                constructWorldForTest(0, 2,
                        0.5 * SIDE, 0.5 * SIDE, 1, -0.5 * SIDE, -0.5 * SIDE, 1,
                        1, 0, 0, 1, 0, 0, 0, 0);
                jointUniversal = OdeHelper.createUniversalJoint(world, null);
                jointUniversal.attach(body[0], body[1]);
                jointUniversal.setAnchor(0, 0, 1);
                jointUniversal.setAxis1(0, 0, 1);
                jointUniversal.setAxis2(1, -1, 0);
                max_iterations = 100;
                return 1;

            // Joint PR (Prismatic and Rotoide)
            case 800:     // 2 body
            case 801:     // 2 bodies with spring force and prismatic fixed
            case 802:     // 2 bodies with torque on body1 and prismatic fixed
                constructWorldForTest(0, 2,
                        -1.0, 0.0, 1.0,
                        1.0, 0.0, 1.0,
                        1, 0, 0, 1, 0, 0,
                        0, 0);
                jointPR = OdeHelper.createPRJoint(world, null);
                jointPR.attach(body[0], body[1]);
                jointPR.setAnchor(-0.5, 0.0, 1.0);
                jointPR.setAxis1(0, 1, 0);
                jointPR.setAxis2(1, 0, 0);
                jointPR.setParamLoStop(-0.5);
                jointPR.setParamHiStop(0.5);
                jointPR.setParamLoStop2(0);
                jointPR.setParamHiStop2(0);
                return 1;
            case 803:   // 2 bodies with spring force and prismatic NOT fixed
            case 804:   // 2 bodies with torque force and prismatic NOT fixed
            case 805:   // 2 bodies with force only on first body
                constructWorldForTest(0, 2,
                        -1.0, 0.0, 1.0,
                        1.0, 0.0, 1.0,
                        1, 0, 0, 1, 0, 0,
                        0, 0);
                jointPR = OdeHelper.createPRJoint(world, null);
                jointPR.attach(body[0], body[1]);
                jointPR.setAnchor(-0.5, 0.0, 1.0);
                jointPR.setAxis1(0, 1, 0);
                jointPR.setAxis2(1, 0, 0);
                jointPR.setParamLoStop(-0.5);
                jointPR.setParamHiStop(0.5);
                jointPR.setParamLoStop2(-0.5);
                jointPR.setParamHiStop2(0.5);
                return 1;
        }
        return 0;
    }


    // do stuff specific to this test each iteration. you can check some
    // invariants for the test -- the return value is some scaled error measurement
    // that must be less than 1.
    // return a dInfinity if error is not measured for this n.

    private double doStuffAndGetError(int n) {
        switch (n) {

            // ********** fixed joint

            case 0: {            // 2 body
                addOscillatingTorque(0.1);
                dampRotationalMotion(0.1);
                // check the orientations are the same
                DMatrix3C R1 = body[0].getRotation();
                DMatrix3C R2 = body[1].getRotation();
                double err1 = dMaxDifference(R1, R2);
                // check the body offset is correct
                DVector3 p = new DVector3(), pp = new DVector3();
                DVector3C p1 = body[0].getPosition();
                DVector3C p2 = body[1].getPosition();
                //    for (int i=0; i<3; i++) p.v[i] = p2.v[i] - p1.v[i];
                p.eqDiff(p2, p1);
                OdeMath.dMultiply1_331(pp, R1, p);
                //    pp.v[0] += 0.5;
                //    pp.v[1] += 0.5;
                pp.add(0.5, 0.5, 0);
                return (err1 + pp.length()) * 300;
            }

            case 1: {            // 1 body to static env
                addOscillatingTorque(0.1);

                // check the orientation is the identity
                double err1 = cmpIdentity(body[0].getRotation());

                // check the body offset is correct
                DVector3 p = new DVector3();
                DVector3C p1 = body[0].getPosition();
                //    for (int i=0; i<3; i++) p.v[i] = p1.v[i];
                //    p.v[0] -= 0.25;
                //    p.v[1] -= 0.25;
                //    p.v[2] -= 1;
                p.set(p1).sub(0.25, 0.25, 1);
                return (err1 + p.length()) * 1e6;
            }

            case 2: {            // 2 body
                addOscillatingTorque(0.1);
                dampRotationalMotion(0.1);
                // check the body offset is correct
                // Should really check body rotation too.  Oh well.
                DMatrix3C R1 = body[0].getRotation();
                DVector3 p = new DVector3(), pp = new DVector3();
                DVector3C p1 = body[0].getPosition();
                DVector3C p2 = body[1].getPosition();
                //    for (int i=0; i<3; i++) p.v[i] = p2.v[i] - p1.v[i];
                p.eqDiff(p2, p1);
                dMultiply1_331(pp, R1, p);
                //    pp.v[0] += 0.5;
                //    pp.v[1] += 0.5;
                pp.add(0.5, 0.5, 0);
                return pp.length() * 300;
            }

            case 3: {            // 1 body to static env with relative rotation
                addOscillatingTorque(0.1);

                // check the body offset is correct
                DVector3 p = new DVector3();
                DVector3C p1 = body[0].getPosition();
                //    for (int i=0; i<3; i++) p[i] = p1[i];
                //    p[0] -= 0.25;
                //    p[1] -= 0.25;
                //    p[2] -= 1;
                p.set(p1).sub(0.25, 0.25, 1);
                return p.length() * 1e6;
            }


            // ********** hinge joint

            case 200:            // 2 body
                addOscillatingTorque(0.1);
                dampRotationalMotion(0.1);
                return dInfinity;

            case 220:            // hinge angle polarity test
                body[0].addTorque(0, 0, 0.01);
                body[1].addTorque(0, 0, -0.01);
                if (iteration == 40) {
                    double a = jointHinge.getAngle();
                    if (a > 0.5 && a < 1) return 0;
                    else return 10;
                }
                return 0;

            case 221: {            // hinge angle rate test
                //TZ static double last_angle = 0;
                body[0].addTorque(0, 0, 0.01);
                body[1].addTorque(0, 0, -0.01);
                double a = jointHinge.getAngle();
                double r = jointHinge.getAngleRate();
                double er = (a - S.last_angle_221) / STEPSIZE;        // estimated rate
                S.last_angle_221 = a;
                return Math.abs(r - er) * 4e4;
            }

            case 230:            // hinge motor rate (and polarity) test
            case 231: {            // ...with stops
                //TZ static double a = 0;
                double r = jointHinge.getAngleRate();
                double err = Math.abs(Math.cos(S.a_231) - r);
                if (S.a_231 == 0) err = 0;
                S.a_231 += 0.03;
                jointHinge.setParamVel(Math.cos(S.a_231));
                if (n == 231) return dInfinity;
                return err * 1e6;
            }

            // ********** slider joint

            case 300:            // 2 body
                addOscillatingTorque(0.05);
                dampRotationalMotion(0.1);
                addSpringForce(0.5);
                return dInfinity;

            case 320:            // slider angle polarity test
                body[0].addForce(0, 0, 0.1);
                body[1].addForce(0, 0, -0.1);
                if (iteration == 40) {
                    double a = jointSlider.getPosition();
                    if (a > 0.2 && a < 0.5)
                        return 0;
                    else
                        return 10;
                }
                return 0;

            case 321: {            // slider angle rate test
                //TZ static double last_pos = 0;
                body[0].addForce(0, 0, 0.1);
                body[1].addForce(0, 0, -0.1);
                double p = jointSlider.getPosition();
                double r = jointSlider.getPositionRate();
                double er = (p - S.last_pos_321) / STEPSIZE;    // estimated rate (almost exact)
                S.last_pos_321 = p;
                return Math.abs(r - er) * 1e9;
            }

            case 330:            // slider motor rate (and polarity) test
            case 331: {            // ...with stops
                //TZ static double a = 0;
                double r = jointSlider.getPositionRate();
                double err = Math.abs(0.7 * Math.cos(S.a_331) - r);
                if (S.a_331 < 0.04) err = 0;
                S.a_331 += 0.03;
                jointSlider.setParamVel(0.7 * Math.cos(S.a_331));
                if (n == 331) return dInfinity;
                return err * 1e6;
            }

            // ********** hinge-2 joint

            case 420:            // hinge-2 steering angle polarity test
                body[0].addTorque(0, 0, 0.01);
                body[1].addTorque(0, 0, -0.01);
                if (iteration == 40) {
                    double a = jointHinge2.getAngle1();
                    if (a > 0.5 && a < 0.6) return 0;
                    else return 10;
                }
                return 0;

            case 421: {            // hinge-2 steering angle rate test
                //TZ static double last_angle = 0;
                body[0].addTorque(0, 0, 0.01);
                body[1].addTorque(0, 0, -0.01);
                double a = jointHinge2.getAngle1();
                double r = jointHinge2.getAngle1Rate();
                double er = (a - S.last_angle_421) / STEPSIZE;        // estimated rate
                S.last_angle_421 = a;
                return Math.abs(r - er) * 2e4;
            }

            case 430:            // hinge 2 steering motor rate (+polarity) test
            case 431: {            // ...with stops
                //TZ static double a = 0;
                double r = jointHinge2.getAngle1Rate();
                double err = Math.abs(Math.cos(S.a_431) - r);
                if (S.a_431 == 0) err = 0;
                S.a_431 += 0.03;
                jointHinge2.setParamVel(Math.cos(S.a_431));
                if (n == 431) return dInfinity;
                return err * 1e6;
            }

            case 432: {            // hinge 2 wheel motor rate (+polarity) test
                //TZ static double a = 0;
                double r = jointHinge2.getAngle2Rate();
                double err = Math.abs(Math.cos(S.a_432) - r);
                if (S.a_432 == 0) err = 0;
                S.a_432 += 0.03;
                jointHinge2.setParamVel2(Math.cos(S.a_432));
                return err * 1e6;
            }

            // ********** angular motor joint

            case 600: {            // test euler angle calculations
                // desired euler angles from last iteration
                //TZ static double a1,a2,a3;

                // find actual euler angles
                double aa1 = jointAMotor.getAngle(0);
                double aa2 = jointAMotor.getAngle(1);
                double aa3 = jointAMotor.getAngle(2);
                // printf ("actual  = %.4f %.4f %.4f\n\n",aa1,aa2,aa3);

                double err = dInfinity;
                if (iteration > 0) {
                    err = dFabs(aa1 - S.a1_600) + dFabs(aa2 - S.a2_600) + dFabs(aa3 - S.a3_600);
                    err *= 1e10;
                }

                // get random base rotation for both bodies
                DMatrix3 Rbase = new DMatrix3();
                dRFromAxisAndAngle(Rbase, 3 * (dRandReal() - 0.5), 3 * (dRandReal() - 0.5),
                        3 * (dRandReal() - 0.5), 3 * (dRandReal() - 0.5));
                body[0].setRotation(Rbase);

                // rotate body 2 by random euler angles w.r.t. body 1
                S.a1_600 = 3.14 * 2 * (dRandReal() - 0.5);
                S.a2_600 = 1.57 * 2 * (dRandReal() - 0.5);
                S.a3_600 = 3.14 * 2 * (dRandReal() - 0.5);
                DMatrix3 R1 = new DMatrix3(), R2 = new DMatrix3(), R3 = new DMatrix3();
                DMatrix3 Rtmp1 = new DMatrix3(), Rtmp2 = new DMatrix3();
                dRFromAxisAndAngle(R1, 0, 0, 1, -S.a1_600);
                dRFromAxisAndAngle(R2, 0, 1, 0, S.a2_600);
                dRFromAxisAndAngle(R3, 1, 0, 0, -S.a3_600);
                Rtmp1.eqMul(R2, R3);//dMultiply0 (Rtmp1,R2,R3,3,3,3);
                Rtmp2.eqMul(R1, Rtmp1);//dMultiply0 (Rtmp2,R1,Rtmp1,3,3,3);
                Rtmp1.eqMul(Rbase, Rtmp2);//dMultiply0 (Rtmp1,Rbase,Rtmp2,3,3,3);
                body[1].setRotation(Rtmp1);
                // printf ("desired = %.4f %.4f %.4f\n",a1,a2,a3);

                return err;
            }

            // ********** universal joint

            case 700: {        // 2 body: joint constraint
                DVector3 ax1 = new DVector3(), ax2 = new DVector3();

                addOscillatingTorque(0.1);
                dampRotationalMotion(0.1);
                jointUniversal.getAxis1(ax1);
                jointUniversal.getAxis2(ax2);
                return Math.abs(10 * ax1.dot(ax2));
            }

            case 701: {        // 2 body: angle 1 rate
                //TZ static double last_angle = 0;
                addOscillatingTorque(0.1);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle1();
                double r = jointUniversal.getAngle1Rate();
                double diff = a - S.last_angle_701;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_701 = a;
                // I'm not sure why the error is so large here.
                return Math.abs(r - er) * 1e1;
            }

            case 702: {        // 2 body: angle 2 rate
                //TZ static double last_angle = 0;
                addOscillatingTorque(0.1);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle2();
                double r = jointUniversal.getAngle2Rate();
                double diff = a - S.last_angle_702;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_702 = a;
                // I'm not sure why the error is so large here.
                return Math.abs(r - er) * 1e1;
            }

            case 720: {        // universal transmit torque test: constraint error
                DVector3 ax1 = new DVector3(), ax2 = new DVector3();
                addOscillatingTorqueAbout(0.1, 1, 1, 0);
                dampRotationalMotion(0.1);
                jointUniversal.getAxis1(ax1);
                jointUniversal.getAxis2(ax2);
                return Math.abs(10 * ax1.dot(ax2));
            }

            case 721: {        // universal transmit torque test: angle1 rate
                //TZ static double last_angle = 0;
                addOscillatingTorqueAbout(0.1, 1, 1, 0);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle1();
                double r = jointUniversal.getAngle1Rate();
                double diff = a - S.last_angle_721;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_721 = a;
                return Math.abs(r - er) * 1e10;
            }

            case 722: {        // universal transmit torque test: angle2 rate
                //TZ static double last_angle = 0;
                addOscillatingTorqueAbout(0.1, 1, 1, 0);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle2();
                double r = jointUniversal.getAngle2Rate();
                double diff = a - S.last_angle_722;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_722 = a;
                return Math.abs(r - er) * 1e10;
            }

            case 730: {
                DVector3 ax1 = new DVector3(), ax2 = new DVector3();
                jointUniversal.getAxis1(ax1);
                jointUniversal.getAxis2(ax2);
                addOscillatingTorqueAbout(0.1, ax1);
                dampRotationalMotion(0.1);
                return Math.abs(10 * ax1.dot(ax2));
            }

            case 731: {
                DVector3 ax1 = new DVector3();
                //TZ static double last_angle = 0;
                jointUniversal.getAxis1(ax1);
                addOscillatingTorqueAbout(0.1, ax1);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle1();
                double r = jointUniversal.getAngle1Rate();
                double diff = a - S.last_angle_731;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_731 = a;
                return Math.abs(r - er) * 2e3;
            }

            case 732: {
                DVector3 ax1 = new DVector3();
                //TZ static double last_angle = 0;
                jointUniversal.getAxis1(ax1);
                addOscillatingTorqueAbout(0.1, ax1);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle2();
                double r = jointUniversal.getAngle2Rate();
                double diff = a - S.last_angle_732;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_732 = a;
                return Math.abs(r - er) * 1e10;
            }

            case 740: {
                DVector3 ax1 = new DVector3(), ax2 = new DVector3();
                jointUniversal.getAxis1(ax1);
                jointUniversal.getAxis2(ax2);
                addOscillatingTorqueAbout(0.1, ax2);
                dampRotationalMotion(0.1);
                return Math.abs(10 * ax1.dot(ax2));
            }

            case 741: {
                DVector3 ax2 = new DVector3();
                //TZ static double last_angle = 0;
                jointUniversal.getAxis2(ax2);
                addOscillatingTorqueAbout(0.1, ax2);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle1();
                double r = jointUniversal.getAngle1Rate();
                double diff = a - S.last_angle_741;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_741 = a;
                return Math.abs(r - er) * 1e10;
            }

            case 742: {
                DVector3 ax2 = new DVector3();
                //TZ static double last_angle = 0;
                jointUniversal.getAxis2(ax2);
                addOscillatingTorqueAbout(0.1, ax2);
                dampRotationalMotion(0.1);
                double a = jointUniversal.getAngle2();
                double r = jointUniversal.getAngle2Rate();
                double diff = a - S.last_angle_742;
                if (diff > M_PI) diff -= 2 * M_PI;
                if (diff < -M_PI) diff += 2 * M_PI;
                double er = diff / STEPSIZE;    // estimated rate
                S.last_angle_742 = a;
                return Math.abs(r - er) * 1e4;
            }

            // ********** slider joint
            case 801:
            case 803:
                addSpringForce(0.25);
                return dInfinity;

            case 802:
            case 804: {
                //TZ static double a = 0;
                body[0].addTorque(0, 0.01 * Math.cos(1.5708 * S.a_804), 0);
                S.a_804 += 0.01;
                return dInfinity;
            }

            case 805:
                addOscillatingForce(0.1);
                return dInfinity;
        }


        return dInfinity;
    }

    //****************************************************************************
    // simulation stuff common to all the tests

    // simulation loop

    private void step() {
        iteration++;

        // do stuff for this test and check to see if the joint is behaving well
        double error = doStuffAndGetError(test_num);
        if (error > max_error) max_error = (float) error;
//			if (error < dInfinity) {  // TODO remove????
//				System.out.println ("scaled error = " + error);
//			}

        // take a step
        world.step(STEPSIZE);

        // occasionally re-orient the first body to create a deliberate error.
        if (cmd_occasional_error != 0) {
            //TZ      static int count = 0;
            if ((S.count % 20) == 0) {
                // randomly adjust orientation of body[0]
                DMatrix3C R1;
                DMatrix3 R2 = new DMatrix3(), R3 = new DMatrix3();
                R1 = body[0].getRotation();
                dRFromAxisAndAngle(R2, dRandReal() - 0.5, dRandReal() - 0.5,
                        dRandReal() - 0.5, dRandReal() - 0.5);
                R3.eqMul(R1, R2);//dMultiply0 (R3,R1,R2,3,3,3);
                body[0].setRotation(R3);

                // randomly adjust position of body[0]
                DVector3C pos = body[0].getPosition();
                body[0].setPosition(
                        pos.get0() + 0.2 * (dRandReal() - 0.5),
                        pos.get1() + 0.2 * (dRandReal() - 0.5),
                        pos.get2() + 0.2 * (dRandReal() - 0.5));
            }
            S.count++;
        }
    }

    //****************************************************************************
    // conduct a specific test, and report the results

    private void doTest(int n) {
        test_num = n;
        iteration = 0;
        max_iterations = 300;
        max_error = 0;
        S = new PseudoStatic();

        if (setupTest(n) == 0) {
            return;
        }

        // run simulation
        for (int i = 0; i < max_iterations; i++) {
            step();
        }

        world.destroy();
        world = null;
        body[0] = null;
        body[1] = null;
//		joint = null;
        jointAMotor = null;
        jointFixed = null;
        jointHinge = null;
        jointHinge2 = null;
        jointPR = null;
        jointSlider = null;
        jointUniversal = null;
        S = null;

        // print results
        print("test " + n + ": ");
        if (max_error == dInfinity) {
            println("error not computed");
        } else {
            print("max scaled error = " + max_error);
            if (max_error < 1) println(" - passed");
            else println(" - FAILED\n");
            Assert.assertTrue(max_error < 1);
        }
    }


    @BeforeClass
    public static void beforeClass() {
        OdeHelper.initODE2(0);
    }

    @AfterClass
    public static void afterClass() {
        OdeHelper.closeODE();
    }

//	@Parameterized.Parameters
//	public static Iterable<Object[]> data1() {
//		ArrayList<Object[]> l = new ArrayList<>();
//		DemoJointsTest dummy = new DemoJointsTest(-1);
//		for (int i=0; i<NUM_JOINTS*100; i++) {
//			if (dummy.setupTest(i) != 0) {
//				l.add(new Object[]{i});
//			}
//		}
//		return l;
//	}
//
//	private final int testId;
//
//	public DemoJointsTest(int testId) {
//		this.testId = testId;
//	}

    @Test
    public void doTest() {
        cmd_occasional_error = 0;
        for (int i = 0; i < NUM_JOINTS * 100; i++) {
            doTest(i);
        }
    }

//    @Test
//    public void doTestWithError() {
//        cmd_occasional_error = 1;
//        for (int i = 0; i < NUM_JOINTS * 100; i++) {
//            doTest(i);
//        }
//    }

}