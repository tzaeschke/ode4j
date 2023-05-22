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
package org.ode4j.tests;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector4;
import org.ode4j.ode.*;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;
import static org.ode4j.ode.OdeConstants.*;

/**
 * Issue #79: Trimesh callbacks are not called.
 */
public class TestIssue0079_TriMeshCallback {

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;
    private final ArrayList<DJoint> contactJoints = new ArrayList<DJoint>();
    private int nContacts;

    private DTriMesh.DTriCallback defaultCallback = new DTriMesh.DTriCallback() {
        @Override
        public int call(DGeom TriMesh, DGeom RefObject, int TriangleIndex) {
            if (TriangleIndex != 1 && TriangleIndex != 2) {
                fail("Triangle index should be 1 or 2 but was: " + TriangleIndex);
            }
            // We ignore triangle ¨1"
            if (TriangleIndex == 1) {
                return 0;
            }
            // Triangle "2" should collide normally!
            return 1;
        }

        ;
    };

    @Before
    public void beforeTest() {
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createSapSpace(DSapSpace.AXES.XYZ);
        contactgroup = OdeHelper.createJointGroup();
    }

    @After
    public void afterTest() {
        OdeHelper.closeODE();

        contactJoints.clear();
        nContacts = 0;
    }

    /**
     * This test is designed such that every geom can collide with triangles "1" and "2" only.
     * Only triangle "2" will collide, "1" is ignored.
     */
    private void testTrimesh(DGeom geom, DTriMesh.DTriCallback callback) {
        float[] size = new float[]{5.0f, 5.0f, 2.5f};
        float[] vertices = new float[]{
                -size[0], -size[1], size[2],
                size[0], -size[1], size[2],
                size[0], size[1], size[2],
                -size[0], size[1], size[2],
                0f, 0f, 0f};
        int[] indices = new int[]{
                0, 1, 4,
                1, 2, 4,
                2, 3, 4,
                3, 0, 4
        };


        DTriMeshData Data = OdeHelper.createTriMeshData();
        Data.build(vertices, indices);
        OdeHelper.createTriMesh(space, Data, callback, null, null);

        if (geom instanceof DRay) {
            //space.add(geom);
            OdeHelper.spaceCollide2((DRay) geom, space, 0, rayCallback);
        } else {
            if (!(geom instanceof DTriMesh)) {
                DBody sBody = OdeHelper.createBody(world);
                geom.setBody(sBody);
                sBody.setPosition(1, 1, 1);
                space.add(geom);
            }

            OdeHelper.spaceCollide(space, 0, nearCallback);
        }
    }

    @Test
    public void testBoxNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createBox(1, 1, 1);
        testTrimesh(geom, null);
        assertEquals(2 * 4, contactJoints.size());
    }

    @Test
    public void testBox() {
        DGeom geom = OdeHelper.createBox(1, 1, 1);
        testTrimesh(geom, defaultCallback);
        assertEquals(1 * 4, contactJoints.size());
    }

    @Test
    public void testCapsuleNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createCapsule(1, 1);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testCapsule() {
        DGeom geom = OdeHelper.createCapsule(1, 1);
        testTrimesh(geom, defaultCallback);
        assertEquals(1, contactJoints.size());
    }

    @Test
    public void testRayNoOp() {
        // just verify that it does not fail!
        DRay geom = OdeHelper.createRay(10);
        geom.set(0f, 2.5f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, null);
        assertEquals(1, nContacts);
    }

    @Test
    public void testRay() {
        DRay geom = OdeHelper.createRay(10);
        geom.set(0f, 2.5f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, defaultCallback);
        assertEquals(1, nContacts);
    }

    @Test
    public void testRay2NoOp() {
        // just verify that it does not fail!
        DRay geom = OdeHelper.createRay(10);
        geom.set(2.5f, 0f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, null);
        assertEquals(1, nContacts);
    }

    @Test
    public void test2Ray() {
        DRay geom = OdeHelper.createRay(10);
        geom.set(2.5f, 0f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, defaultCallback);
        // Contact is ignored by our callback -> 0
        assertEquals(0, nContacts);
    }

    @Test
    public void testSphereNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createSphere(1);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testSphere() {
        DGeom geom = OdeHelper.createSphere(1);
        testTrimesh(geom, defaultCallback);
        assertEquals(1, contactJoints.size());
    }

    @Test
    public void testTrimesNoOp() {
        float[] size = new float[]{5.0f, 5.0f, 5f};
        float[] vertices = new float[]{
                size[0], 0, 0,
                0, size[1], 0,
                0f, 0f, size[2]};
        int[] indices = new int[]{
                0, 1, 2,
        };
        // just verify that it does not fail!
        DTriMeshData Data = OdeHelper.createTriMeshData();
        Data.build(vertices, indices);
        DGeom geom = OdeHelper.createTriMesh(space, Data);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testTrimesh() {
        float[] size = new float[]{5.0f, 5.0f, 5f};
        float[] vertices = new float[]{
                size[0], 0, 0,
                0, size[1], 0,
                0f, 0f, size[2]};
        int[] indices = new int[]{
                0, 1, 2,
        };
        DTriMeshData Data = OdeHelper.createTriMeshData();
        Data.build(vertices, indices);
        DGeom geom = OdeHelper.createTriMesh(space, Data);
        testTrimesh(geom, defaultCallback);
        assertEquals(1, contactJoints.size());
    }

    private final DGeom.DNearCallback rayCallback = new DGeom.DNearCallback() {
        @Override
        public void call(Object data, DGeom Geometry1, DGeom Geometry2) {
            // Check collisions
            DContactBuffer contacts = new DContactBuffer(32);
            int Count = OdeHelper.collide(Geometry1, Geometry2, 32, contacts.getGeomBuffer());//get(0).geom);
            nContacts += Count;
        }
    };

    private final DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };


    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        assert (o1 != null);
        assert (o2 != null);

        if (o1 instanceof DSpace || o2 instanceof DSpace) {
            OdeHelper.spaceCollide2(o1, o2, data, nearCallback);
            return;
        }

        final int N = 32;
        DContactBuffer contacts = new DContactBuffer(N);
        int n = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());
        nContacts = n;
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                DContact contact = contacts.get(i);
                contact.surface.slip1 = 0.1;
                contact.surface.slip2 = 0.1;
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactSlip1 | dContactSlip2;
                contact.surface.mu = OdeConstants.dInfinity;
                contact.surface.soft_erp = 0.99;
                contact.surface.soft_cfm = 0.10;
                contact.surface.bounce = 0;
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
                c.attach(contact.geom.g1.getBody(),
                        contact.geom.g2.getBody());
                contactJoints.add(c);
            }
        }
    }

}
