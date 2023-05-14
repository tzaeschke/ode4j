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
package org.ode4j.benchmarks;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.DSapSpace.AXES;

import java.util.function.Supplier;

import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;


public class DemoSpaceStressTest {
    // some constants

    private static int NUM = 2000;            // max number of objects
    private static final float DENSITY = 5.0f;        // density of all objects
    private static final int GPB = 3;            // maximum number of geometries per body
    private static final int MAX_CONTACTS = 4;        // maximum number of contact points per body
    private static final int WORLD_SIZE = 20;
    //private static final int WORLD_HEIGHT = 20;


    // dynamics and collision objects

    private static class MyObject {
        DBody body;            // the body
        DGeom[] geom = new DGeom[GPB];        // geometries representing this body
    }


    private static int num = 0;        // number of objects in simulation
    private static int nextobj = 0;        // next object to recycle if num==NUM
    private static DWorld world;
    private static DSpace space = null;
    private static final MyObject[] obj = new MyObject[NUM];
    private static DJointGroup contactgroup;
    private static int selected = -1;    // selected object
    private static boolean random_pos = true;    // drop objects from random position?


    // this is called by dSpaceCollide when two objects in space are
    // potentially colliding.
    private void nearCallback(DGeom o1, DGeom o2) {
        int i;
        // if (o1->body && o2->body) return;

        // exit without doing anything if the two bodies are connected by a joint
        DBody b1 = o1.getBody();
        DBody b2 = o2.getBody();
        if (b1 != null && b2 != null && areConnectedExcluding(b1, b2, DContactJoint.class)) return;

        DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
        for (i = 0; i < MAX_CONTACTS; i++) {
            DContact contact = contacts.get(i);
            contact.surface.mode = dContactBounce | dContactSoftCFM;
            contact.surface.mu = dInfinity;
            contact.surface.mu2 = 0;
            contact.surface.bounce = 0.1;
            contact.surface.bounce_vel = 0.1;
            contact.surface.soft_cfm = 0.01;
        }
        int numc = OdeHelper.collide(o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());
        if (numc != 0) {
            DMatrix3 RI = new DMatrix3();
            RI.setIdentity();
            for (i = 0; i < numc; i++) {
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contacts.get(i));
                c.attach(b1, b2);
            }
        }
    }

    /**
     * o to disable rendering.
     * b for box.
     * s for sphere.
     * c for capsule.
     * x for a composite object.
     * y for cylinder.
     */
    private void command(char cmd) {
        int i, j, k;
        double[] sides = new double[3];
        DMass m = OdeHelper.createMass();
        boolean setBody = false;

        cmd = Character.toLowerCase(cmd);
        if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y') {
            if (num < NUM) {
                // new object to be created
                i = num;
                num++;
            } else {
                // recycle existing object
                i = nextobj++;
                nextobj %= num; // wrap-around if needed

                // destroy the body and geoms for slot i
                obj[i].body.destroy();
                obj[i].body = null;

                for (k = 0; k < GPB; k++) {
                    if (obj[i].geom[k] != null) {
                        obj[i].geom[k].destroy();
                        obj[i].geom[k] = null;
                    }
                }
            }

            obj[i].body = OdeHelper.createBody(world);
            for (k = 0; k < 3; k++) {
                sides[k] = dRandReal() * 0.5 + 0.1;
            }

            DMatrix3 R = new DMatrix3();
            if (random_pos) {
                obj[i].body.setPosition(
                        dRandReal() * 2 - 1, dRandReal() * 2 - 1, dRandReal() + 2);
                dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
                        dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);
            } else {
                // higher than highest body position
                double maxheight = 0;
                for (k = 0; k < num; k++) {
                    DVector3C pos = obj[k].body.getPosition();
                    if (pos.get2() > maxheight) {
                        maxheight = pos.get2();
                    }
                }
                obj[i].body.setPosition(0, 0, maxheight + 1);
                R.setIdentity();
                //dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
            }
            obj[i].body.setRotation(R);

            if (cmd == 'b') {
                m.setBox(DENSITY, sides[0], sides[1], sides[2]);
                obj[i].geom[0] = OdeHelper.createBox(space, sides[0], sides[1], sides[2]);

            } else if (cmd == 'c') {
                sides[0] *= 0.5;
                m.setCapsule(DENSITY, 3, sides[0], sides[1]);
                obj[i].geom[0] = OdeHelper.createCapsule(space, sides[0], sides[1]);

            } else if (cmd == 'y') {
                sides[1] *= 0.5;
                m.setCylinder(DENSITY, 3, sides[0], sides[1]);
                obj[i].geom[0] = OdeHelper.createCylinder(space, sides[0], sides[1]);

            } else if (cmd == 's') {
                sides[0] *= 0.5;
                m.setSphere(DENSITY, sides[0]);
                obj[i].geom[0] = OdeHelper.createSphere(space, sides[0]);

            } else if (cmd == 'x') {
                setBody = true;

                // start accumulating masses for the encapsulated geometries
                DMass m2 = OdeHelper.createMass();
                m.setZero();

                DVector3[] dpos = DVector3.newArray(GPB);    // delta-positions for encapsulated geometries
                DMatrix3[] drot = DMatrix3.newArray(GPB);

                // set random delta positions
                for (j = 0; j < GPB; j++) {
                    for (k = 0; k < 3; k++) {
                        dpos[j].set(k, dRandReal() * 0.3 - 0.15);
                    }
                }

                for (k = 0; k < GPB; k++) {
                    if (k == 0) {
                        double radius = dRandReal() * 0.25 + 0.05;
                        obj[i].geom[k] = OdeHelper.createSphere(space, radius);
                        m2.setSphere(DENSITY, radius);
                    } else if (k == 1) {
                        obj[i].geom[k] = OdeHelper.createBox(space, sides[0], sides[1], sides[2]);
                        m2.setBox(DENSITY, sides[0], sides[1], sides[2]);
                    } else {
                        double radius = dRandReal() * 0.1 + 0.05;
                        double length = dRandReal() * 1.0 + 0.1;
                        obj[i].geom[k] = OdeHelper.createCapsule(space, radius, length);
                        m2.setCapsule(DENSITY, 3, radius, length);
                    }

                    dRFromAxisAndAngle(drot[k], dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
                            dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);

                    m2.rotate(drot[k]);

                    m2.translate(dpos[k]);

                    // add to the total mass
                    m.add(m2);
                }

                // move all encapsulated objects so that the center of mass is (0,0,0)
                DVector3C negC = m.getC().copy().scale(-1);
                for (k = 0; k < GPB; k++) {
                    obj[i].geom[k].setBody(obj[i].body);
//					dGeomSetPosition (g2[k],
//							dpos[k][0]-m.c[0],
//							dpos[k][1]-m.c[1],
//							dpos[k][2]-m.c[2]);
                    obj[i].geom[k].setOffsetPosition(dpos[k].reAdd(negC));
                    obj[i].geom[k].setOffsetRotation(drot[k]);
                }
                m.translate(negC);
                obj[i].body.setMass(m);
            }

            if (!setBody) {
                for (k = 0; k < GPB; k++) {
                    if (obj[i].geom[k] != null) {
                        obj[i].geom[k].setBody(obj[i].body);
                    }
                }

                obj[i].body.setMass(m);
            }
        }

        if (cmd == ' ') {
            selected++;
            if (selected >= num) selected = 0;
            if (selected < 0) selected = 0;
        } else if (cmd == 'd' && selected >= 0 && selected < num) {
            obj[selected].body.disable();
        } else if (cmd == 'e' && selected >= 0 && selected < num) {
            obj[selected].body.enable();
        } else if (cmd == 'r') {
            random_pos ^= true;
        }
    }

    // simulation loop

    private void simLoop() {
        OdeHelper.spaceCollide(space, 0, (data, o1, o2) -> nearCallback(o1, o2));
        // world.step (0.05);
        world.quickStep(0.05);

        // remove all contact joints
        contactgroup.empty();
    }


    private void before() {
        num = 0;
        nextobj = 0;
        // create world
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        contactgroup = OdeHelper.createJointGroup();
    }

    private void after() {
        contactgroup.destroy();
        space.destroy();
        space = null;
        world.destroy();
        world = null;
        OdeHelper.closeODE();
    }

    private void demoSpace(Supplier<DSpace> spaceFactory) {
        NUM = 500;
        before();
        space = spaceFactory.get();
        demo();
        after();

        NUM = 2000;
        before();
        space = spaceFactory.get();
        demo();
        after();
        System.out.println();
    }

    @Test
    public void demoQuadTree() {
        System.out.printf(":::: Using %-20s   ", "DQuadTreeSpace");
        DVector3 Center = new DVector3(0, 0, 0);
        DVector3 Extents = new DVector3(WORLD_SIZE * 0.55, WORLD_SIZE * 0.55, WORLD_SIZE * 0.55);
        demoSpace(() -> OdeHelper.createQuadTreeSpace(Center, Extents, 6));
    }

    @Test
    public void demoHash() {
        System.out.printf(":::: Using %-20s   ", "DHashSpace");
        demoSpace(OdeHelper::createHashSpace);
    }

    @Test
    public void demoSAP() {
        System.out.printf(":::: Using %-20s   ", "DSweepAndPruneSpace");
        demoSpace(() -> OdeHelper.createSapSpace(AXES.XYZ));
    }

    @Test
    public void demoSimple() {
        System.out.printf(":::: Using %-20s   ", "DSimpleSpace");
        demoSpace(OdeHelper::createSimpleSpace);
    }

    @Test
    public void demoBVH() {
        System.out.printf(":::: Using %-20s   ", "DBhvSpace");
        demoSpace(() -> OdeHelper.createBHVSpace(0));
    }

    private void demo() {
        world.setGravity(0, 0, -0.5);
        world.setCFM(1e-5);
        OdeHelper.createPlane(space, 0, 0, 1, 0);
        for (int i = 0; i < obj.length; i++) {
            obj[i] = new MyObject();
        }

        for (int i = 0; i < NUM; i++) {
            command('s');
        }

        long t0 = System.nanoTime();
        // run simulation
        for (int i = 0; i < 100; ++i) {
            simLoop();
        }
        long t1 = System.nanoTime();
        System.out.print("time: " + (t1 - t0) / 1000 / 1000 + "ms     ");
    }
}
