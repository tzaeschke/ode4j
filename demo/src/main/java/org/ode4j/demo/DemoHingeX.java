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
package org.ode4j.demo;

import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.dQFromAxisAndAngle;


/**
 * Hinge demo.
 */
public class DemoHingeX extends dsFunctions {
    private static final double SIDE = 5;    // side length of a box
    private static final double MASS = 125;    // mass of a box


    // dynamics and collision objects
    private DWorld world;
    private final DBody[] body = new DBody[3];
    private DFixedJoint fixed0;
    private DHingeJoint hinge01;
    private DHingeJoint hinge12;

    // start simulation - set viewpoint
    private static final float[] xyz = {5.0382f, -5.0811f, 18.4700f};
    private static final float[] hpr = {135.0000f, -19.5000f, 0.0000f};

    @Override
    public void start() {
        dsSetViewpoint(xyz, hpr);
        System.out.println("Press 'e' to start/stop occasional error.");
    }

    // called when a key pressed
    @Override
    public void command(char cmd) {
        if (cmd == 'e' || cmd == 'E') {
            //TZ		    occasional_error ^= 1;
        }
    }

    // simulation loop
    private static double a = 0;

    private void simLoop(boolean pause) {
        final double kd = -0.3;    // angular damping constant
        if (!pause) {
            // add an oscillating torque to body 0, and also damp its rotational motion
            final DVector3C w = body[0].getAngularVel();
            //body[0].addTorque ( kd*w.get0(), kd*w.get1()+10.1*Math.cos(a), kd*w.get2()+10.1*Math.sin(a));
            body[1].addTorque(0, 0, kd * w.get2() + 10 * Math.sin(a));
            world.step(0.05);
            a += 0.01;
        }

        DVector3 sides0 = new DVector3(SIDE, SIDE, SIDE);
        DVector3 sides1 = new DVector3(SIDE, SIDE, SIDE * 0.8f);
        DVector3 sides2 = new DVector3(SIDE, SIDE, SIDE);
        dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
        dsSetColor(1, 1, 0);
        dsDrawBox(body[0].getPosition(), body[0].getRotation(), sides0);
        dsSetColor(0, 1, 1);
        dsDrawBox(body[1].getPosition(), body[1].getRotation(), sides1);
        dsSetColor(1, 0, 1);
        dsDrawBox(body[2].getPosition(), body[2].getRotation(), sides2);
    }


    /**
     * @param args args
     */
    public static void main(String[] args) {
        new DemoHingeX().demo(args);
    }

    private void demo(String[] args) {
        // create world
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();

        DMass m = OdeHelper.createMass();
        m.setBox(1, SIDE, SIDE, SIDE);
        m.adjust(MASS);

        DQuaternion q = new DQuaternion();
        dQFromAxisAndAngle(q, 1, 1, 0, 0. * Math.PI);

        body[0] = OdeHelper.createBody(world);
        body[0].setMass(m);
        body[0].setPosition(0, 0, 3);
        //body[0].setQuaternion (q);

        fixed0 = OdeHelper.createFixedJoint(world, null);
        fixed0.attach(null, body[0]);
        fixed0.setFixed();

        body[1] = OdeHelper.createBody(world);
        body[1].setMass(m);
        body[1].setPosition(SIDE, SIDE, 3);
        //body[1].setQuaternion (q);

        hinge01 = OdeHelper.createHingeJoint(world, null);
        hinge01.attach(body[0], body[1]);
        hinge01.setAnchor(0.5 * SIDE, 0.5 * SIDE, 3);
        hinge01.setAxis(0, 0, 1);

        body[2] = OdeHelper.createBody(world);
        body[2].setMass(m);
        body[2].setPosition(2.0 * SIDE, 2.0 * SIDE, 3);
        //body[2].setQuaternion (q);

        hinge12 = OdeHelper.createHingeJoint(world, null);
        hinge12.attach(body[1], body[2]);
        hinge12.setAnchor(1.5 * SIDE, 1.5 * SIDE, 3);
        hinge12.setAxis(0, 0, 1);

        // run simulation
        dsSimulationLoop(args, 640, 480, this);

        world.destroy();
        OdeHelper.closeODE();
    }

    @Override
    public void step(boolean pause) {
        simLoop(pause);
    }

    @Override
    public void stop() {
        // Nothing to do
    }
}
