/*************************************************************************
 *                                                                       *
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

import org.junit.Test;
import org.ode4j.math.DVector3;

/**
 *
 * @author Jing Huang
 */
public class TestIssue0018_NpeInQuickstep {

    private void simLoop(boolean pause) {
        if (!pause) {
            final double step = 0.005;
            //world.step(step);
            world.quickStep(step);
        }
    }

    private DWorld world;
    private DSpace space;
    private DBody body1;
    private DBody body2;
    private DDoubleHingeJoint joint1;
    private DDoubleHingeJoint joint2;
    private boolean applyForce = false;
    private DAMotorJoint _motor;
    //private static double[] xyz = {3.8966, -2.0614, 4.0300};
    //private static double[] hpr = {153.5, -16.5, 0};
    private DVector3 _axis0 = new DVector3(1, 0, 0);
    private DVector3 _axis1 = new DVector3(0, 1, 0);
    private DVector3 _axis2 = new DVector3(0, 0, 1);

    //@Override
    public void start() {
        world = OdeHelper.createWorld();
        world.setGravity(0, 0, 0);

        world.setDamping(1e-4, 1e-5);
        //	    dWorldSetERP(world, 1);

        space = OdeHelper.createSimpleSpace();

        body1 = OdeHelper.createBody(world);
        body2 = OdeHelper.createBody(world);

        body1.setPosition(0, 1, 0);
        body2.setPosition(0, 3, 0);

        DGeom g;
        DMass mass = OdeHelper.createMass();

        g = OdeHelper.createBox(space, 0.2, 1.9, 0.2);
        g.setBody(body1);
        mass.setBox(1, 0.2, 2, 0.2);
        body1.setMass(mass);

        g = OdeHelper.createBox(space, 0.2, 1.9, 0.2);
        g.setBody(body2);
        mass.setBox(1, 0.2, 2, 0.2);
        body2.setMass(mass);

        if (true) {
            joint1 = OdeHelper.createDHingeJoint(world);
            joint1.attach(body1, null);
            joint1.setAxis(0, 0, 1);
            joint1.setAnchor1(0, 0, 0);
            joint1.setAnchor2(0, 0, 0);

        }

        if (true) {
            joint2 = OdeHelper.createDHingeJoint(world);
            joint2.attach(body1, body2);
            joint2.setAxis(0, 1, 0);
            joint2.setAnchor1(0, 2, 0);
            joint2.setAnchor2(0, 2, 0);

            _motor = OdeHelper.createAMotorJoint(world);
            _motor.attach(body1, body2);
            _motor.setMode(DAMotorJoint.AMotorMode.dAMotorEuler);
            _motor.setNumAxes(3);
            _motor.setAxis(0, 1, _axis0);
            _motor.setAxis(2, 2, _axis2);

            // XXX: make these changeable, for example using the XML
            _motor.setParam(DJoint.PARAM_N.dParamFudgeFactor1, 0.8f);
            _motor.setParam(DJoint.PARAM_N.dParamFudgeFactor2, 0.8f);
            _motor.setParam(DJoint.PARAM_N.dParamFudgeFactor3, 0.8f);

            _motor.setParam(DJoint.PARAM_N.dParamStopCFM1, 0.2f);
            _motor.setParam(DJoint.PARAM_N.dParamStopCFM2, 0.2f);
            _motor.setParam(DJoint.PARAM_N.dParamStopCFM3, 0.2f);
        }
    }

    //@Override
    public void step(boolean pause) {
        simLoop(pause);
    }

    /**
     * See issue #18. This failed with NPE in quickstep() because of uninitialised DJointWithInfo.
     * The reason is that the 'nj' used for creation of the jointinfo[] is sometimes bigger than
     * the 'nj' used when initialising jointinfo[].
     */
    @Test
    public void test() {
        OdeHelper.initODE();

        start();
        for (int i = 0; i < 1000; i++) {
            step(false);
        }
    }
}