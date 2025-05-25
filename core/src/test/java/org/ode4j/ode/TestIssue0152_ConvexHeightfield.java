/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2025 Tilmann Zaeschke     *
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
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DHeightfield.DHeightfieldGetHeight;
import org.ode4j.ode.internal.DxTrimeshHeightfield;
import org.ode4j.ode.test.geoms.IcosahedronGeom;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;


public class TestIssue0152_ConvexHeightfield {

    private static final float DEGTORAD = 0.01745329251994329577f; //!< PI / 180.0, convert degrees to radians

    // Heightfield dimensions

    private static final int HFIELD_WSTEP = 15;            // Vertex count along edge >= 2
    private static final int HFIELD_DSTEP = 31;

    private static final float HFIELD_WIDTH = 4.0f;
    private static final float HFIELD_DEPTH = 8.0f;

    private static final float DENSITY = 5.0f;    // density of all objects
    private static final int MAX_CONTACTS = 64;        // maximum number of contact points per body

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;

    private boolean hasCollided = false;


    private DHeightfieldGetHeight heightfield_callback = this::heightfield_callback;

    private double heightfield_callback(Object pUserData, int x, int z) {
        double fx = (((double) x) - (HFIELD_WSTEP - 1) / 2) / (HFIELD_WSTEP - 1);
        double fz = (((double) z) - (HFIELD_DSTEP - 1) / 2) / (HFIELD_DSTEP - 1);

        // Create an interesting 'hump' shape
        double h = (1.0) + ((-16.0) * (fx * fx * fx + fz * fz * fz));

        return h;
    }


    private final DGeom.DNearCallback nearCallback = this::nearCallback;


    // this is called by dSpaceCollide when two objects in space are
    // potentially colliding.

    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        int i;
        // if (o1->body && o2->body) return;

        // exit without doing anything if the two bodies are connected by a joint
        DBody b1 = o1.getBody();
        DBody b2 = o2.getBody();
        if (b1 != null && b2 != null && areConnectedExcluding(b1, b2, DContactJoint.class)) {
            return;
        }

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
            hasCollided = true;
            DMatrix3 RI = new DMatrix3();
            RI.setIdentity();
            for (i = 0; i < numc; i++) {
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contacts.get(i));
                c.attach(b1, b2);
            }
        }
    }

    // called when a key pressed
    private DBody createConvex() {
        DMass m = OdeHelper.createMass();

        //
        // Geom Creation
        //

        DBody body = OdeHelper.createBody(world);

        DMatrix3 R = new DMatrix3();
        body.setPosition(
                (dRandReal() - 0.5) * HFIELD_WIDTH * 0.75,
                (dRandReal() - 0.5) * HFIELD_DEPTH * 0.75,
                dRandReal() + 2);
        dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
                dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);
        body.setRotation(R);

        // Create Convex
        m.setBox(DENSITY, 0.25, 0.25, 0.25);
        DConvex geom = OdeHelper.createConvex(space,
                IcosahedronGeom.planes,
                IcosahedronGeom.planecount,
                IcosahedronGeom.points,
                IcosahedronGeom.pointcount,
                IcosahedronGeom.polygons);

        geom.setBody(body);

        body.setMass(m);

        return body;
    }


    // simulation loop

    public void step() {

        space.collide(null, nearCallback);

        world.quickStep(0.05);

        // remove all contact joints
        contactgroup.empty();
    }


    @Test
    public void demo() {
        // create world
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createHashSpace(null);
        contactgroup = OdeHelper.createJointGroup();
        world.setGravity(0, 0, -0.05);
        world.setCFM(1e-5);
        world.setAutoDisableFlag(true);
        world.setContactMaxCorrectingVel(0.1);
        world.setContactSurfaceLayer(0.001);

        world.setAutoDisableAverageSamplesCount(1);

        // base plane to catch overspill
        OdeHelper.createPlane(space, 0, 0, 1, 0);


        // our heightfield floor

        DHeightfieldData height = OdeHelper.createHeightfieldData();

        // Create an finite heightfield.
        height.buildCallback(null, heightfield_callback,
                HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
                1.0, 0.0, 0.0, false);
        // alternative: create heightfield from array
//		double[] data = new double[HFIELD_WSTEP*HFIELD_DSTEP];
//		for (int x = 0; x < HFIELD_WSTEP; x++) {
//			for (int z = 0; z < HFIELD_DSTEP; z++) {
//				data[x+HFIELD_WSTEP*z] = heightfield_callback(null, x, z);
//			}
//		}
//		heightid.build(data, false, HFIELD_WIDTH, HFIELD_DEPTH, 
//				HFIELD_WSTEP, HFIELD_DSTEP, 1.0, 0.0, 0.0, false );

        // Give some very bounds which, while conservative,
        // makes AABB computation more accurate than +/-INF.
        height.setBounds((-4.0), (+6.0));

        // Our heightfield geom
        DGeom gheight = new DxTrimeshHeightfield(space, height, true);

        DVector3 pos = new DVector3();

        // Rotate so Z is up, not Y (which is the default orientation)
        DMatrix3 R = new DMatrix3();
        R.setIdentity();
        dRFromAxisAndAngle(R, 1, 0, 0, DEGTORAD * 90);

        // Place it.
        gheight.setRotation(R);
        gheight.setPosition(pos);

        // run simulation
        assertFalse(hasCollided);
        createConvex();
        for (int i = 0; i < 200; i++) {
            step();
            if (hasCollided) {
                break;
            }
        }
        assertTrue(hasCollided);

        contactgroup.destroy();
        space.destroy();
        world.destroy();

        // destroy heightfield data, because _we_ own it not ODE
        height.destroy();

        OdeHelper.closeODE();
    }
}