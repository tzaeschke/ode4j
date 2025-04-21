/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.demo;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.DxMass;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;

// TriMesh collision demo.
// Serves as a test for the collision of trimesh geometries.
// By Davy (Dawei) Chen.
public class DemoTrimeshCollision extends dsFunctions {

    private static DWorld world;
    private static DHashSpace space;
    private static DJointGroup contactgroup;

    // 2D Convex hulls to create the TriMesh geometries
    private static final double[] HULL1 = {-1.000000, -35.000000, 1.000000, -34.000000, 5.000000, -31.000000, 16.000000, -20.000000, 16.000000, -11.000000, 5.000000, 0.000000, -1.000000, 3.000000, -7.000000, 4.000000, -13.000000, 3.000000, -16.000000, 2.000000, -20.000000, 0.000000, -24.000000, -4.000000, -26.000000, -8.000000, -27.000000, -12.000000, -27.000000, -15.000000, -27.000000, -19.000000, -26.000000, -23.000000, -24.000000, -27.000000, -20.000000, -31.000000, -16.000000, -34.000000};
    private static final double[] HULL2 = {23.000000, 28.000000, 28.000000, 29.000000, 30.000000, 30.000000, 34.000000, 33.000000, 35.000000, 34.000000, 38.000000, 39.000000, 39.000000, 42.000000, 40.000000, 48.000000, 39.000000, 53.000000, 38.000000, 56.000000, 35.000000, 61.000000, 34.000000, 62.000000, 30.000000, 65.000000, 28.000000, 66.000000, 25.000000, 67.000000, 20.000000, 68.000000, 19.000000, 68.000000, 14.000000, 67.000000, 11.000000, 66.000000, 9.000000, 65.000000, 5.000000, 62.000000, 4.000000, 61.000000, 1.000000, 56.000000, 0.000000, 53.000000, 0.000000, 48.000000, 0.000000, 42.000000, 1.000000, 39.000000, 4.000000, 34.000000, 5.000000, 33.000000, 9.000000, 30.000000, 11.000000, 29.000000};
    private static final double[] HULL3 = {-1.000000, -35.000000, 1.000000, -34.000000, 5.000000, -31.000000, 8.000000, -28.000000, 11.000000, -22.000000, 12.000000, -19.000000, 12.000000, -12.000000, 11.000000, -9.000000, 8.000000, -3.000000, 5.000000, 0.000000, -1.000000, 3.000000, -7.000000, 4.000000, -13.000000, 3.000000, -16.000000, 2.000000, -20.000000, 0.000000, -23.000000, -3.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -11.000000, -27.000000, -15.000000, -27.000000, -20.000000, -26.000000, -23.000000, -25.000000, -25.000000, -23.000000, -28.000000, -20.000000, -31.000000, -16.000000, -34.000000};
    private static final double[] HULL4 = {-2.000000, -35.000000, 6.000000, -31.000000, 9.000000, -27.000000, 10.000000, -25.000000, 11.000000, -22.000000, 11.000000, -9.000000, 10.000000, -6.000000, 9.000000, -4.000000, 6.000000, 0.000000, -2.000000, 3.000000, -7.000000, 4.000000, -19.000000, 3.000000, -20.000000, 2.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -11.000000, -27.000000, -15.000000, -27.000000, -20.000000, -26.000000, -23.000000, -25.000000, -25.000000, -20.000000, -34.000000};
    private static final double[] HULL5 = {-2.000000, -35.000000, 4.000000, -32.000000, 9.000000, -27.000000, 11.000000, -23.000000, 12.000000, -20.000000, 12.000000, -11.000000, 11.000000, -8.000000, 9.000000, -4.000000, 5.000000, 0.000000, -2.000000, 3.000000, -7.000000, 4.000000, -12.000000, 3.000000, -15.000000, 2.000000, -20.000000, 0.000000, -23.000000, -3.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -11.000000, -27.000000, -15.000000, -27.000000, -20.000000, -26.000000, -23.000000, -25.000000, -25.000000, -23.000000, -28.000000, -19.000000, -32.000000, -15.000000, -34.000000};
    private static final double[] HULL6 = {-1.000000, -35.000000, 25.000000, -29.000000, 26.000000, -28.000000, 30.000000, -18.000000, 30.000000, -14.000000, 26.000000, -6.000000, 25.000000, -5.000000, -1.000000, 3.000000, -7.000000, 4.000000, -13.000000, 3.000000, -20.000000, 0.000000, -23.000000, -3.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -12.000000, -27.000000, -15.000000, -27.000000, -19.000000, -26.000000, -23.000000, -25.000000, -25.000000, -23.000000, -28.000000, -20.000000, -31.000000, -17.000000, -33.000000};

    // Center points of the 2D Convex hulls
    private final double[] CENTER1 = {-5.500000, -15.500000};
    private final double[] CENTER2 = {20.000000, 48.000000};
    private final double[] CENTER3 = {-7.500000, -15.500000};
    private final double[] CENTER4 = {-8.000000, -15.500000};
    private final double[] CENTER5 = {-7.500000, -15.500000};
    private final double[] CENTER6 = {1.500000, -15.500000};

    // Where to position the TriMeshes on the Ground plane
    // In (X, Y) coordinates pairs
    private final float[][] BODY_POSITIONS = { //[][2] = {
            {-60.0f, -30.0f},
            {0.0f, -30.0f},
            {60.0f, -30.0f},
            {-60.0f, 30.0f},
            {0.0f, 30.0f},
            {60.0f, 30.0f}
    };

    private static final double[][] HULLS = {HULL1, HULL2, HULL3, HULL4, HULL5, HULL6};
    private final int[] HULL_SIZES = {
            HULL1.length / 2, HULL2.length / 2, HULL3.length / 2, HULL4.length / 2, HULL5.length / 2, HULL6.length / 2
    };
    private final double[][] CENTERS = {CENTER1, CENTER2, CENTER3, CENTER4, CENTER5, CENTER6};
    private static final int HULLS_COUNT = HULLS.length;

    private final float TRIMESH_HEIGHT = 2.0f;

    private static final float[][] odeVerts = new float[HULLS_COUNT][]; // *odeVerts[HULLS_COUNT];
    private static final int[][] odeInds = new int[HULLS_COUNT][];// *odeInds[HULLS_COUNT];
    private static final int[] odeIndsCount = new int[HULLS_COUNT];

    private static final DTriMeshData[] triMeshDataId = new DTriMeshData[HULLS_COUNT];
    private static final DGeom[] triMeshId = new DGeom[HULLS_COUNT];
    private static final DBody[] bodyId = new DBody[HULLS_COUNT];


    private final DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };

    // this is called by dSpaceCollide when two objects in space are
    // potentially colliding.

    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        if (o1.isSpace() || o2.isSpace()) {
            // colliding a space with something
            OdeHelper.spaceCollide2(o1, o2, data, nearCallback);
            // Note we do not want to test intersections within a space,
            // only between spaces.
            return;
        }

        final int N = 32;
        DContactBuffer contacts = new DContactBuffer(N);   // up to MAX_CONTACTS contacts per box-box
        int n = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                DContact contact = contacts.get(i);
                contact.surface.slip1 = 0.7;
                contact.surface.slip2 = 0.7;
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
                // Friction effect, if set to dInfinity, objects would be unmovable
                contact.surface.mu = 0.0f;
                contact.surface.soft_erp = 0.50;
                contact.surface.soft_cfm = 0.03;
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
                c.attach
                        (
                                contact.geom.g1.getBody(),
                                contact.geom.g2.getBody()
                        );
            }
        }
    }

    // start simulation - set viewpoint
    @Override
    public void start() {
        // dAllocateODEDataForThread(dAllocateMaskAll);
        float[] xyz = {-8, 0, 5};
        float[] hpr = {0.0f, -29.5000f, 0.0000f};
        dsSetViewpoint(xyz, hpr);
        System.err.println("Press SPACE to reset the simulation1.");
    }

    void reset() {
        for (int i = 0; i < HULLS_COUNT; i++) {
            bodyId[i].setPosition(
                    BODY_POSITIONS[i][0] * 0.05,
                    BODY_POSITIONS[i][1] * 0.05,
                    0.0f);

            DMatrix3 R = new DMatrix3();
            dRFromAxisAndAngle(R,
                    1.0f,
                    1.0f,
                    1.0f,
                    0.0f);
            bodyId[i].setRotation(R);

            // Enable the body as it might have been auto-disabled
            bodyId[i].enable();
        }
    }

    // called when a key pressed
    @Override
    public void command(char cmd) {
        switch (cmd) {
            case ' ':
                reset();
                break;
            default:
                break;
        }
    }

    @Override
    public void step(boolean pause) {
        double simstep = 1 / 240.0;
        double dt = dsElapsedTime();

        int nrofsteps = (int) Math.ceil(dt / simstep);
        nrofsteps = nrofsteps > 8 ? 8 : nrofsteps;

        for (int i = 0; i < nrofsteps && !pause; i++) {
            // Add force to TriMesh bodies, and make them gather towards world center
            for (int j = 0; j < HULLS_COUNT; j++) {
                DVector3C pos = bodyId[j].getPosition();
                // Calculate force tensity according to distance from world center
                double length = pos.length(); //dCalcVectorLengthSquare3(pos);
                DVector3 pos1 = new DVector3();
                dCopyVector3(pos1, pos);
                dNegateVector3(pos1);
                dNormalize3(pos1);
                pos1.scale(5.0f * length);

                bodyId[j].addForce(pos1);
            }

            space.collide(null, nearCallback);
            world.quickStep(simstep);
            contactgroup.empty();
        }

        dsSetColor(1, 1, 1);
        for (int i = 0; i < HULLS_COUNT; i++) {
            DVector3C Pos = bodyId[i].getPosition();
            DMatrix3C Rot = bodyId[i].getRotation();

            // Draw TriMeshes
            if (odeVerts != null) // TODO?
            {
                for (int j = 0; j < odeIndsCount[i] / 3; j++) {
                    DVector3C v0 = new DVector3(
                            odeVerts[i][odeInds[i][j * 3 + 0] * 3],
                            odeVerts[i][odeInds[i][j * 3 + 0] * 3 + 1],
                            odeVerts[i][odeInds[i][j * 3 + 0] * 3 + 2]
                    );
                    DVector3C v1 = new DVector3(
                            odeVerts[i][odeInds[i][j * 3 + 1] * 3],
                            odeVerts[i][odeInds[i][j * 3 + 1] * 3 + 1],
                            odeVerts[i][odeInds[i][j * 3 + 1] * 3 + 2]
                    );
                    DVector3C v2 = new DVector3(
                            odeVerts[i][odeInds[i][j * 3 + 2] * 3],
                            odeVerts[i][odeInds[i][j * 3 + 2] * 3 + 1],
                            odeVerts[i][odeInds[i][j * 3 + 2] * 3 + 2]
                    );
                    dsDrawTriangle(Pos, Rot, v0, v1, v2, true);
                }
            }
        }
    }

    public static void main(String[] args) {
        new DemoTrimeshCollision().demo(args);
    }

    private void demo(String[] args) {
        DMass m = new DxMass();

        // setup pointers to drawstuff callback functions
//    dsFunctions fn;
//    fn.version = DS_VERSION;
//    fn.start = &start;
//    fn.step = &simLoop;
//    fn.command = &command;
//    fn.stop = 0;
//    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

        // create world
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createHashSpace(null);
        space.setLevels(-3, 5);
        OdeHelper.createPlane(space, 0, 0, 1, 0);    // Add a ground plane.

        contactgroup = OdeHelper.createJointGroup();
        world.setGravity(0, 0, -1);
        world.setQuickStepNumIterations(32);
        world.setContactMaxCorrectingVel(40);
        world.setMaxAngularSpeed(62.8);
        world.setERP(0.7);
        world.setQuickStepW(0.75); // For increased stability.

        world.setAutoDisableFlag(true);
        world.setAutoDisableLinearThreshold(0.01);
        world.setAutoDisableAngularThreshold(0.03);
        world.setAutoDisableTime(0.15f);

        // Generate TriMesh geometries and bodies
        for (int i = 0; i < HULLS_COUNT; i++) {
            int hullSize = HULL_SIZES[i];
            // Vertices
            int odeVertsCount1 = hullSize * 3 * 2;
            int odeVertsCount = odeVertsCount1 + 2 * 3;
            odeVerts[i] = new float[odeVertsCount];
            for (int j = 0; j < hullSize; j++) {
                // Bottom layer
                odeVerts[i][j * 3] = (float) ((HULLS[i][j * 2] - CENTERS[i][0]) * 0.05f);
                odeVerts[i][j * 3 + 1] = (float) ((HULLS[i][j * 2 + 1] - CENTERS[i][1]) * 0.05f);
                odeVerts[i][j * 3 + 2] = 0.0f;
                // Top layer
                odeVerts[i][(hullSize + j) * 3] = (float) ((HULLS[i][j * 2] - CENTERS[i][0]) * 0.05f);
                odeVerts[i][(hullSize + j) * 3 + 1] = (float) ((HULLS[i][j * 2 + 1] - CENTERS[i][1]) * 0.05f);
                odeVerts[i][(hullSize + j) * 3 + 2] = TRIMESH_HEIGHT;
            }
            // Center vertex on bottom plane
            odeVerts[i][odeVertsCount1] = 0.0f;
            odeVerts[i][odeVertsCount1 + 1] = 0.0f;
            odeVerts[i][odeVertsCount1 + 2] = 0.0f;
            // Center vertex on top plane
            odeVerts[i][odeVertsCount1 + 3] = 0.0f;
            odeVerts[i][odeVertsCount1 + 3 + 1] = 0.0f;
            odeVerts[i][odeVertsCount1 + 3 + 2] = TRIMESH_HEIGHT;
            // Indices
            int odeIndsCount1 = hullSize * 6;
            odeIndsCount[i] = odeIndsCount1 * 2;
            odeInds[i] = new int[odeIndsCount[i]];
            for (int j = 0; j < hullSize; j++) {
                // Wall triangles
                // Wrap around index
                int n1 = j + 1 < hullSize ? j + 1 : 0;
                int n2 = hullSize + n1;
                odeInds[i][j * 6] = j;
                odeInds[i][j * 6 + 1] = n1;
                odeInds[i][j * 6 + 2] = hullSize + j;
                odeInds[i][j * 6 + 3] = hullSize + j;
                odeInds[i][j * 6 + 4] = n1;
                odeInds[i][j * 6 + 5] = n2;
                // Bottom and Top triangles
                odeInds[i][odeIndsCount1 + j * 6] = j;
                odeInds[i][odeIndsCount1 + j * 6 + 1] = n1;
                odeInds[i][odeIndsCount1 + j * 6 + 2] = hullSize * 2;
                odeInds[i][odeIndsCount1 + j * 6 + 3] = hullSize + j;
                odeInds[i][odeIndsCount1 + j * 6 + 4] = n2;
                odeInds[i][odeIndsCount1 + j * 6 + 5] = hullSize * 2 + 1;
            }

            bodyId[i] = OdeHelper.createBody(world);
            bodyId[i].setPosition(
                    BODY_POSITIONS[i][0] * 0.05,
                    BODY_POSITIONS[i][1] * 0.05,
                    0.0f);

            DMatrix3 R = new DMatrix3();
            dRFromAxisAndAngle(R,
                    1.0f,
                    1.0f,
                    1.0f,
                    0.0f);
            bodyId[i].setRotation(R);

            RefInt index = new RefInt(0);
            bodyId[i].setData(index);

            DMass m1 = OdeHelper.createMass();
            double[] sides = new double[3];
            sides[0] = 3.0f;
            sides[1] = 3.0f;
            sides[2] = 3.0f;
            final float DENSITY = 1.0f;
            m1.setBox(DENSITY, sides[0], sides[1], sides[2]);

            bodyId[i].setMass(m1);
            // Make bodies less bouncy
            bodyId[i].setLinearDamping(0.1);
            bodyId[i].setAngularDamping(0.1);

            triMeshDataId[i] = OdeHelper.createTriMeshData();
            triMeshDataId[i].build(odeVerts[i], odeInds[i]);
            triMeshDataId[i].preprocess2((1 << DTriMeshData.dTRIDATAPREPROCESS_BUILD.FACE_ANGLES), null);

            triMeshId[i] = OdeHelper.createTriMesh(space, triMeshDataId[i], null, null, null);
            triMeshId[i].setBody(bodyId[i]);
        }

        // run simulation
        dsSimulationLoop(args, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, this);

        for (int i = 0; i < HULLS_COUNT; i++) {
            triMeshDataId[i].destroy();
            triMeshId[i].destroy();
            bodyId[i].destroy();

            odeVerts[i] = null;
            odeInds[i] = null;
        }

        contactgroup.empty();
        contactgroup.destroy();
        space.destroy();
        world.destroy();
        OdeHelper.closeODE();
    }

    @Override
    public void stop() {
        // Nothing
    }
}
