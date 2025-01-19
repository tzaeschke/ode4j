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

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 * Hinge demo.
 */
public class DemoHingeZ extends dsFunctions {
    private static final double SIDE = 5;    // side length of a box
    private static final double MASS = 125;    // mass of a box


    // dynamics and collision objects
    private DWorld world;
    private final DBody[] body = new DBody[3];
    private DSliderJoint slider2;
    private DHingeJoint hinge01;
    private DHingeJoint hinge12;
    private DJointGroup contactgroup;
    private DSpace space;

    // start simulation - set viewpoint
    private static final float[] xyz = {5.0382f, -5.0811f, 18.4700f};
    private static final float[] hpr = {135.0000f, -19.5000f, 0.0000f};

    private final DGeom.DNearCallback nearCallback = this::nearCallback;

    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        if (o1 instanceof DSpace || o2 instanceof DSpace) {
            // colliding a space with something
            OdeHelper.spaceCollide2(o1, o2, data, nearCallback);
            // Note we do not want to test intersections within a space,
            // only between spaces.
            return;
        }

        final int N = 32;
        DContactBuffer contacts = new DContactBuffer(N);
        int n = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                DContact contact = contacts.get(i);
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
                c.attach(contact.geom.g1.getBody(), contact.geom.g2.getBody());
            }
        }
    }


    @Override
    public void start() {
        dsSetViewpoint(xyz, hpr);
        System.out.println("Press 'e' to start/stop occasional error.");
    }

    // called when a key pressed
    @Override
    public void command(char cmd) {
        if (cmd == 'e' || cmd == 'E') {
            // Nothing
        }
    }

    // simulation loop
    private double a = 0;
    private int nn = 0;

    private void simLoop(boolean pause) {
        final double kd = -0.3;    // angular damping constant
        if (!pause) {
            // add an oscillating torque to body 0, and also damp its rotational motion
            final DVector3C w = body[0].getAngularVel();
            body[1].addTorque(0, kd * w.get2() + 2000 * Math.sin(a), 0);
            space.collide(null, nearCallback);
            world.quickStep(0.05);
            contactgroup.empty();
            a += 0.01;
            if (nn < 5 || nn % 50 == 0) {
                System.out.println("pos: " + slider2.getPosition());
            }
            nn++;
        }

        DVector3 sides0 = new DVector3(SIDE, SIDE, SIDE);
        DVector3 sides1 = new DVector3(SIDE, SIDE, SIDE);
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
        new DemoHingeZ().demo(args);
    }

    private void demo(String[] args) {
        // create world
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        world.setGravity(0, 0, -1);

        space = OdeHelper.createHashSpace(null);
        contactgroup = OdeHelper.createJointGroup();
        world.setGravity(0, 0, -0.5);
        OdeHelper.createPlane(space, 0, 0, 1, 0);

//        DBody bodyFloor = OdeHelper.createBody(world);
//        //bodyFloor.setMass(m);
//        bodyFloor.setPosition(0, 0, -5);
//        DBox boxFloor = OdeHelper.createBox(space, 0, 0, 10);
//        boxFloor.setBody(bodyFloor);
//        bodyFloor.setKinematic();
//        DFixedJoint fixed = OdeHelper.createFixedJoint(world);
//        //fixed.attach(bodyFloor);

        DMass m = OdeHelper.createMass();
        m.setBox(1, SIDE, SIDE, SIDE);
        m.adjust(MASS);

        body[0] = OdeHelper.createBody(world);
        body[0].setMass(m);
        body[0].setPosition(0, 0, SIDE);
        DBox box0 = OdeHelper.createBox(space, SIDE, SIDE, SIDE);
        box0.setBody(body[0]);

        body[1] = OdeHelper.createBody(world);
        body[1].setMass(m);
        body[1].setPosition(0, 0, 2 * SIDE);
        DBox box1 = OdeHelper.createBox(space, SIDE, SIDE, SIDE);
        box1.setBody(body[1]);

        body[2] = OdeHelper.createBody(world);
        body[2].setMass(m);
        // set to 0,0,0 initially to fix slider position to 0.
        body[2].setPosition(0, 0, 0);
        DQuaternion q2 = new DQuaternion( 0.9152532048256324, -1.6920811360625115E-4, 0.4028790636652719, 4.985194848135399E-5);
        body[2].setQuaternion(q2);
        DBox box2 = OdeHelper.createBox(space, SIDE, SIDE, SIDE);
        box2.setBody(body[2]);

        slider2 = OdeHelper.createSliderJoint(world, null);
        slider2.attach(null, body[2]);
        slider2.setAxis(0, 0, 1);
        body[2].setPosition(0, 0, 3 * SIDE);

        hinge01 = OdeHelper.createHingeJoint(world, null);
        hinge01.attach(body[0], body[1]);
        hinge01.setAnchor(0.5 * SIDE, 0, 1.5 * SIDE);
        hinge01.setAxis(0, 1, 0);

        hinge12 = OdeHelper.createHingeJoint(world, null);
        hinge12.attach(body[1], body[2]);
        hinge12.setAnchor(-0.5 * SIDE, 0, 2.5 * SIDE);
        hinge12.setAxis(0, 1, 0);


        // run simulation
        dsSimulationLoop(args, 640, 480, this);

        contactgroup.destroy();
        space.destroy();
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
