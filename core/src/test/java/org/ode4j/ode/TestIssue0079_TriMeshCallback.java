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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.ode.test.geoms.ConvexCubeGeom;

import java.util.ArrayList;

import static org.junit.Assert.*;

/**
 * Issue #79: Trimesh callbacks are not called.
 */
public class TestIssue0079_TriMeshCallback {

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;
    private final ArrayList<DContactJoint> contactJoints = new ArrayList<>();
    private int nContacts;

    // Second trimesh to collide with main trimesh
    private final float[] triangle2vertices = new float[]{
            5.0f, 0, 0,  // point 1 of colliding triangle
            0, 5.0f, 0,  // point 2 of colliding triangle
            0, 0, 5.0f,  // point 3 of colliding triangle
            6, 6, 6,  // point way outside
            6, 6, 7,  // point way outside
            6, 7, 6,  // point way outside
            7, 6, 6,  // point way outside
    };
    private final int[] triangle2indices = new int[]{
            3, 4, 5,
            4, 5, 6,
            3, 5, 6,
            0, 1, 2,  // overlapping triangle
            3, 4, 6,
    };


    private final DTriMesh.DTriCallback defaultCallback = (TriMesh, RefObject, TriangleIndex) -> {
        if (TriangleIndex != 1 && TriangleIndex != 2) {
            fail("Triangle index should be 1 or 2 but was: " + TriangleIndex);
        }
        // We ignore triangle ¨1"
        if (TriangleIndex == 1) {
            return 0;
        }
        // Triangle "2" should collide normally!
        assertEquals(2, TriangleIndex);
        return 1;
    };

    private final DTriMesh.DTriRayCallback defaultRayCallback = (TriMesh, RefObject, TriangleIndex, u, v) -> {
        if (TriangleIndex != 1 && TriangleIndex != 2) {
            fail("Triangle index should be 1 or 2 but was: " + TriangleIndex);
        }
        // We ignore triangle ¨1"
        if (TriangleIndex == 1) {
            return 0;
        }
        // Triangle "2" should collide normally!
        assertEquals(2, TriangleIndex);
        ++nContacts;
        return 1;
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
     * When using the callback, only triangle "2" will collide, "1" is ignored.
     */
    private void testTrimesh(DGeom geom, DTriMesh.DTriCallback callback) {
        testTrimesh(geom, callback, null);
    }

    private void testTrimesh(DGeom geom, DTriMesh.DTriCallback callback, DTriMesh.DTriRayCallback rayCallback) {
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
        if (geom instanceof DConvex) {
            Data.preprocess();
        }
        DTriMesh trimesh = OdeHelper.createTriMesh(space, Data, callback, null, rayCallback);
        DBody triBody = OdeHelper.createBody(world);
        trimesh.setBody(triBody);

        if (geom instanceof DRay) {
            //            OdeHelper.spaceCollide2(geom, space, 0, rayCallback);
        } else {
            DBody sBody = OdeHelper.createBody(world);
            geom.setBody(sBody);
            if (!(geom instanceof DTriMesh)) {
                sBody.setPosition(1, 1, 1);
            }
        }
        OdeHelper.spaceCollide(space, 0, this::nearCallback);
    }

    @Test
    public void testBoxNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createBox(space, 1, 1, 1);
        testTrimesh(geom, null);
        assertEquals(2 * 4, contactJoints.size());
    }

    @Test
    public void testBox() {
        DGeom geom = OdeHelper.createBox(space, 1, 1, 1);
        testTrimesh(geom, defaultCallback);
        verifyContactJoints();
        assertEquals(1 * 4, contactJoints.size());
    }

    @Test
    public void testCapsuleNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createCapsule(space, 1, 1);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testCapsule() {
        DGeom geom = OdeHelper.createCapsule(space, 1, 1);
        testTrimesh(geom, defaultCallback);
        verifyContactJoints();
        assertEquals(1, contactJoints.size());
    }

    @Test
    public void testConvexNoOp() {
        DGeom geom = OdeHelper.createConvex(space, ConvexCubeGeom.planes1,
                ConvexCubeGeom.points1,
                ConvexCubeGeom.polygons1);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testConvex() {
        DGeom geom = OdeHelper.createConvex(space, ConvexCubeGeom.planes1,
                ConvexCubeGeom.points1,
                ConvexCubeGeom.polygons1);
        testTrimesh(geom, defaultCallback);
        verifyContactJoints();
        assertEquals(1, contactJoints.size());
    }

    @Test
    public void testCylinderNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createCylinder(space, 1, 1);
        testTrimesh(geom, null);
        assertEquals(4, contactJoints.size());
    }

    @Test
    public void testCylinder() {
        DGeom geom = OdeHelper.createCylinder(space, 1, 1);
        testTrimesh(geom, defaultCallback);
        verifyContactJoints();
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testRayNoOp() {
        // Ray through triangle 2
        DRay geom = OdeHelper.createRay(space, 10);
        geom.set(0f, 2.5f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, null, null);
        assertEquals(1, nContacts);
    }

    @Test
    public void testRay() {
        // Ray through triangle 2
        DRay geom = OdeHelper.createRay(space, 10);
        //        geom.set(0f, 2.5f, 0f, 0f, 0f, 1f);
        geom.set(0f, 2.5f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, defaultCallback, null);
        verifyContactJoints();
        assertEquals(1, nContacts);
    }

    @Test
    public void testRayRCB() {
        // Ray through triangle 2
        DRay geom = OdeHelper.createRay(space, 10);
        //        geom.set(0f, 2.5f, 0f, 0f, 0f, 1f);
        geom.set(0f, 2.5f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, null, defaultRayCallback);
        verifyContactJoints();
        assertEquals(1, nContacts);
    }

    @Test
    public void testRay2NoOp() {
        // Ray through triangle 1
        DRay geom = OdeHelper.createRay(space, 10);
        geom.set(2.5f, 0f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, null, null);
        assertEquals(1, nContacts);
    }

    @Test
    public void test2Ray() {
        // Ray through triangle 1
        DRay geom = OdeHelper.createRay(space, 10);
        geom.set(2.5f, 0f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, defaultCallback, null);
        // Contact is ignored by our callback -> 0
        verifyContactJoints();
        assertEquals(0, nContacts);
    }

    @Test
    public void test2RayRCB() {
        // Ray through triangle 1
        DRay geom = OdeHelper.createRay(space, 10);
        geom.set(2.5f, 0f, 10f, 0f, 0f, -1f);
        testTrimesh(geom, null, defaultRayCallback);
        // Contact is ignored by our callback -> 0
        verifyContactJoints();
        assertEquals(0, nContacts);
    }

    @Test
    public void testSphereNoOp() {
        // just verify that it does not fail!
        DGeom geom = OdeHelper.createSphere(space, 1);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testSphere() {
        DGeom geom = OdeHelper.createSphere(space, 1);
        testTrimesh(geom, defaultCallback);
        verifyContactJoints();
        assertEquals(1, contactJoints.size());
    }

    @Test
    public void testTrimesNoOp() {
        // just verify that it does not fail!
        DTriMeshData Data = OdeHelper.createTriMeshData();
        Data.build(triangle2vertices, triangle2indices);
        DGeom geom = OdeHelper.createTriMesh(space, Data);
        testTrimesh(geom, null);
        assertEquals(2, contactJoints.size());
    }

    @Test
    public void testTrimesh() {
        DTriMeshData Data = OdeHelper.createTriMeshData();
        Data.build(triangle2vertices, triangle2indices);
        DGeom geom = OdeHelper.createTriMesh(space, Data);
        testTrimesh(geom, defaultCallback);
        verifyContactJoints();
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

    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        assertNotNull(o1);
        assertNotNull(o2);

        final int N = 32;
        DContactBuffer contacts = new DContactBuffer(N);
        int n = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());
        nContacts = n;
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                DContact contact = contacts.get(i);
                DContactJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
                c.attach(contact.geom.g1.getBody(),
                        contact.geom.g2.getBody());
                contactJoints.add(c);
            }
        }
    }

    private void verifyContactJoints() {
        for (DContactJoint j : contactJoints) {
            DContact contact = j.getContact();
            DGeom g1 = contact.getContactGeom().g1;
            if (g1 instanceof DTriMesh) {
                assertEquals(2, contact.getContactGeom().side1);
            }
            DGeom g2 = contact.getContactGeom().g2;
            if (g2 instanceof DTriMesh) {
                // Trimesh-trimesh collision, only triangl 3 collides
                assertEquals(3, contact.getContactGeom().side2);
            }
        }
    }
}
