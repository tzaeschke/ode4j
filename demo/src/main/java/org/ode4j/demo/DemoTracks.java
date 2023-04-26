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

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.Common;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;

/**
 *
 * @author Tilmann Zaeschke
 */
public class DemoTracks extends dsFunctions {

    private final double ball_radius = 0.4;
    private final double balls_sep = 2; // separation between the balls

    /* Choose one test case
     */
    private static int TEST_CASE = 0;
    private static double track_len = 10;
    private static double track_height = 1;
    private static double track_width = 0.1;
    private static double track_gauge = 1;
    private static double track_elevation = 2;
    private static double track_angle = 80 * Common.M_PI/180.;
    private static double track_incl = 10 * Common.M_PI/180.;

    {
        if (TEST_CASE == 0) {
            track_len = 10;
            track_height = 1;
            track_width = 0.1;
            track_gauge = 1;
            track_elevation = 2;
            track_angle = 80 * Common.M_PI/180.;
            track_incl = 10 * Common.M_PI/180.;
        } else if (TEST_CASE == 1) {
            track_len = 10;
            track_height = 1;
            track_width = 0.1;
            track_gauge = 1.9*ball_radius;
            track_elevation = 2;
            track_angle = 0 * Common.M_PI/180.;
            track_incl = 10 * Common.M_PI/180.;
        } else if (TEST_CASE == 2) {
            track_len = 10;
            track_height = 1;
            track_width = 0.1;
            track_gauge = 1.9*ball_radius;
            track_elevation = 2;
            track_angle = 15 * Common.M_PI/180.;
            track_incl = 10 * Common.M_PI/180.;
        } else if (TEST_CASE == 3) {
            track_len = 10;
            track_height = .7;
            track_width = 0.1;
            track_gauge = track_height*1.1;
            track_elevation = 2;
            track_angle = 90 * Common.M_PI/180.;
            track_incl = 10 * Common.M_PI/180.; 
        } else {
            throw new RuntimeException("TEST_CAST to a valid value!");
        }
    }



    private DWorld world;
    private DSpace space;
    private DJointGroup contact_group;
    private DGeom ground;
    private DGeom ball1_geom, ball2_geom;
    private DTriMeshData mesh_data;
    private DGeom mesh_geom;

    private DBody ball1_body, ball2_body;

    private final int n_box_verts = 8;
    private DVector3[] box_verts = new DVector3[] {
            new DVector3(-track_len/2, -track_width/2,  track_height/2), // 0
            new DVector3( track_len/2, -track_width/2,  track_height/2), // 1
            new DVector3( track_len/2,  track_width/2,  track_height/2), // 2
            new DVector3(-track_len/2,  track_width/2,  track_height/2), // 3
            new DVector3( track_len/2, -track_width/2, -track_height/2), // 4
            new DVector3(-track_len/2, -track_width/2, -track_height/2), // 5
            new DVector3(-track_len/2,  track_width/2, -track_height/2), // 6
            new DVector3( track_len/2,  track_width/2, -track_height/2)  // 7
    };

    private final int n_box_faces = 12;
    private int[] box_faces = new int[] {//[n_box_faces * 3] {
            0, 1, 2,
            0, 2, 3,
            1, 4, 7,
            1, 7, 2,
            4, 5, 6,
            4, 6, 7,
            5, 0, 3,
            5, 3, 6,
            3, 2, 7,
            3, 7, 6,
            0, 5, 4,
            0, 4, 1
    };


    private final int n_track_verts = n_box_verts * 2;
    private final int n_track_faces = n_box_faces * 2;

    float[] track_verts = new float[n_track_verts * 3];
    int[] track_faces = new int[n_track_faces * 3];



    void resetBall(DBody b, int idx)
    {
        b.setPosition(     0.5*track_len*Math.cos(track_incl) // Z
                - 0.5*track_height*Math.sin(track_incl)
                - ball_radius, // X
                balls_sep*idx, // Y
                track_elevation + ball_radius// Z
                + 0.5*track_len*Math.sin(track_incl)
                + 0.5*track_height*Math.cos(track_incl));
//        DMatrix3 r = new DMatrix3(1, 0, 0, 0,
//                0, 1, 0, 0,
//                0, 0, 1, 0);
        DMatrix3 r = new DMatrix3(1, 0, 0,
                0, 1, 0,
                0, 0, 1);
        b.setRotation(r);
        b.setLinearVel(0, 0, 0);
        b.setAngularVel(0, 0, 0);

    }


    void resetSim()
    {
        resetBall(ball1_body, 0);
        resetBall(ball2_body, 1);
    }


    @Override
    public void start() {
    	//dAllocateODEDataForThread(dAllocateMaskAll);

        world = OdeHelper.createWorld();
        world.setGravity (0,0,-9.8);

        contact_group = OdeHelper.createJointGroup();

        space = OdeHelper.createSimpleSpace();


        // first, the ground plane
        // it has to coincide with the plane we have in drawstuff
        ground = OdeHelper.createPlane(space, 0, 0, 1, 0);


        // now a ball
        DMass m = OdeHelper.createMass();
        m.setSphere(0.1, ball_radius);

        ball1_geom = OdeHelper.createSphere(space, ball_radius);
        ball1_body = OdeHelper.createBody(world);
        ball1_geom.setBody(ball1_body);
        ball1_body.setMass(m);

        ball2_geom = OdeHelper.createSphere(space, ball_radius);
        ball2_body = OdeHelper.createBody(world);
        ball2_geom.setBody(ball2_body);
        ball2_body.setMass(m);




        // tracks made out of boxes
        DGeom trk;
        DMatrix3 r1 = new DMatrix3(), r2 = new DMatrix3(), r3 = new DMatrix3();
        DVector3 ro = new DVector3(0, -(0.5*track_gauge + 0.5*track_width), track_elevation);
        DMatrix3 s1 = new DMatrix3(), s2 = new DMatrix3(), s3 = new DMatrix3();
        DVector3 so = new DVector3(0, 0.5*track_gauge + 0.5*track_width, track_elevation);

        dRFromAxisAndAngle(r1, 1, 0, 0,  track_angle);
        dRFromAxisAndAngle(r2, 0, 1, 0, -track_incl);
        dMultiply0_333(r3, r2, r1);

        dRFromAxisAndAngle(s1, 1, 0, 0, -track_angle);
        dRFromAxisAndAngle(s2, 0, 1, 0, -track_incl);
        dMultiply0_333(s3, s2, s1);

        trk = OdeHelper.createBox(space, track_len, track_width, track_height);
        trk.setPosition(ro.get0(), ro.get1() + balls_sep, ro.get2());
        trk.setRotation(r3);

        trk = OdeHelper.createBox(space, track_len, track_width, track_height);
        trk.setPosition(so.get0(), so.get1() + balls_sep, so.get2());
        trk.setRotation(s3);





        // tracks made out of trimesh
        for (int i=0; i<n_box_verts; ++i) {
            DVector3 p = new DVector3();
            dMultiply0_331(p, s3, box_verts[i]);
            p.add(so);//dAddVectors3(p, p, so);
            dCopyVector3(track_verts, i*3, p);
        }
        // trimesh tracks 2, transform all vertices by s3
        for (int i=0; i<n_box_verts; ++i) {
            DVector3 p = new DVector3();
            dMultiply0_331(p, r3, box_verts[i]);
            p.add(ro);//dAddVectors3(p, p, ro);
            dCopyVector3(track_verts, (n_box_verts + i)*3, p);
        }

        // copy face indices
        for (int i=0; i<n_box_faces; ++i)
            for (int j=0; j<3; ++j) // each face index
                track_faces[3*i+j] = box_faces[3*i+j];
        for (int i=0; i<n_box_faces; ++i)
            for (int j=0; j<3; ++j) // each face index
                track_faces[3*(i + n_box_faces)+j] = box_faces[3*i+j] + n_box_verts;

        mesh_data = OdeHelper.createTriMeshData();
        mesh_data.build(//hpr, box_faces)dGeomTriMeshDataBuildSimple(mesh_data,
                track_verts,//[0], n_track_verts,
                track_faces);//, 3*n_track_faces);
        mesh_geom = OdeHelper.createTriMesh(space, mesh_data, null, null, null);





        resetSim();


        // initial camera position
        //        static float xyz[3] = {-5.9414,-0.4804,2.9800};
        //        static float hpr[3] = {32.5000,-10.0000,0.0000};
        dsSetViewpoint (xyz,hpr);

        dsSetSphereQuality(3);
    }
    private static float[] xyz = {-5.9414f,-0.4804f,2.9800f};
    private static float[] hpr = {32.5000f,-10.0000f,0.0000f};


    private DNearCallback nearCallback = new DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };


    void nearCallback(Object data, DGeom a, DGeom b)
    {
        final int max_contacts = 8;
        DContactBuffer contacts = new DContactBuffer(max_contacts);

        //if (!dGeomGetBody(a) && !dGeomGetBody(b))
        if (a.getBody()==null && b.getBody()==null)
            return; // don't handle static geom collisions

        int n = OdeHelper.collide (a,b,max_contacts,contacts.getGeomBuffer());
        //clog << "got " << n << " contacts" << endl;

        /* Simple contact merging:
         * If we have contacts that are too close with the same normal, keep only
         * the one with maximum depth.
         * The epsilon that defines what "too close" means can be a heuristic.
         */
        int new_n = 0;
        double epsilon = 1e-1; // default
        /* If we know one of the geoms is a sphere, we can base the epsilon on the
         *  sphere's radius.
         */
        DGeom s = null;
        //        if ((dGeomGetClass(a) == dSphereClass && (s = a)) || 
        //                (dGeomGetClass(b) == dSphereClass && (s = b))) {
        //            epsilon = dGeomSphereGetRadius(s) * 0.3;
        //        }
        if (((a instanceof DSphere) && (s = a)!=null) || 
                ((b instanceof DSphere) && (s = b)!=null)) {
            epsilon = ((DSphere)s).getRadius() * 0.3;
        }


        for (int i=0; i<n; ++i) {
            DContact contact = contacts.get(i);

            // this block draws the contact points before merging, in red
            DMatrix3 r = new DMatrix3();
            r.setIdentity();
            dsSetColor(1, 0, 0);
            dsSetTexture(DS_TEXTURE_NUMBER.DS_NONE);
            dsDrawSphere(contact.geom.pos, r, 0.008);

            // let's offset the line a bit to avoid drawing overlap issues
            float[] xyzf = new float[3], hprf = new float[3];
            dsGetViewpoint(xyzf, hprf);
            DVector3 xyz = new DVector3(xyzf[0], xyzf[1], xyzf[2]);
            DVector3 v = new DVector3();
            v = contact.geom.pos.reSub(xyz);//dSubtractVectors3(v, contact.geom.pos, xyz);
            DVector3 c = new DVector3();
            dCalcVectorCross3(c, v, contact.geom.pos);
            dNormalize3(c);
            DVector3 pos1 = new DVector3();
            dAddScaledVectors3(pos1, contact.geom.pos, c, 1, 0.005);
            DVector3 pos2 = new DVector3();
            dAddScaledVectors3(pos2, pos1, contact.geom.normal, 1, 0.05);
            dsDrawLine(pos1, pos2);
            // end of contacts drawing code



            int closest_point = i;
            for (int j=0; j<new_n; ++j) {
                double alignment = dCalcVectorDot3(contact.geom.normal, contacts.get(j).geom.normal);
                if (alignment > 0.99 // about 8 degrees of difference
                        &&
                        contact.geom.pos.distance(contacts.get(j).geom.pos) < epsilon) {
                    // they are too close
                    closest_point = j;
                    //clog << "found close points: " << j << " and " << i << endl;
                    break;
                }
            }

            if (closest_point != i) {
                // we discard one of the points
                if (contact.geom.depth > contacts.get(closest_point).geom.depth)
                    // the new point is deeper, copy it over closest_point
                    contacts.set(closest_point, contacts.get(i));
                //contacts[closest_point] = contacts[i];
            } else {
                //contacts[new_n++] = contacts[i]; // the point is preserved
                contacts.set(new_n++, contacts.get(i)); // the point is preserved
            }
        }
        //clog << "reduced from " << n << " to " << new_n << endl;
        n = new_n;

        for (int i=0; i<n; ++i) {
            DContact contactI = contacts.get(i);
            contactI.surface.mode = dContactBounce | dContactApprox1 | dContactSoftERP;
            contactI.surface.mu = 10;
            contactI.surface.bounce = 0.2;
            contactI.surface.bounce_vel = 0;
            contactI.surface.soft_erp = 1e-3;
            //clog << "depth: " << contacts[i].geom.depth << endl;


            DJoint contact = OdeHelper.createContactJoint(world, contact_group, contactI);
            contact.attach(a.getBody(), b.getBody());

            DMatrix3 r = new DMatrix3();
            r.setIdentity();
            dsSetColor(0, 0, 1);
            dsSetTexture(DS_TEXTURE_NUMBER.DS_NONE);
            dsDrawSphere(contactI.geom.pos, r, 0.01);
            dsSetColor(0, 1, 0);
            DVector3 pos2 = new DVector3();
            dAddScaledVectors3(pos2, contactI.geom.pos, contactI.geom.normal, 1, 0.1);
            dsDrawLine(contactI.geom.pos, pos2);
        }
        //clog << "----" << endl;
    }



    @Override
    public void stop()
    {
        mesh_geom.destroy();
        mesh_data.destroy();

        ball1_body.destroy();
        ball2_body.destroy();

        ground.destroy();

        contact_group.destroy();

        space.destroy(); // will destroy all geoms

        world.destroy();
    }


    @Override
    public void command (char cmd)
    {
        switch (cmd) {
        case ' ':
            resetSim();
            break;
        }
    }


    private void drawGeom(DGeom g)
    {
        DVector3C pos = g.getPosition();
        DMatrix3C rot = g.getRotation();

        if (g instanceof DSphere) {
            dsSetColorAlpha(0f, 0.75f, 0.5f, 0.5f);
            dsSetTexture (DS_TEXTURE_NUMBER.DS_CHECKERED);
            dsDrawSphere(pos, rot, ((DSphere)g).getRadius());
        } else if (g instanceof DBox) {
            DVector3 lengths = new DVector3();
            dsSetColorAlpha(1f, 1f, 0f, 0.5f);
            dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
            ((DBox)g).getLengths(lengths);
            dsDrawBox(pos, rot, lengths);
        } else if (g instanceof DTriMesh) {
//            int numi = ((DTriMesh)g).dGeomTriMeshGetTriangleCount(g);
//
//            for (int i=0; i<numi; ++i) {
//                DVector3 v0 = new DVector3(), v1 = new DVector3(), v2 = new DVector3();
//                dGeomTriMeshGetTriangle(g, i, v0, v1, v2);
//
//                dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
//
//                dsSetDrawMode(DS_WIREFRAME);
//                dsSetColorAlpha(0f, 0f, 0f, 1.0f);
//                dsDrawTriangle(pos, rot, v0, v1, v2, true);
//
//                dsSetDrawMode(DS_POLYFILL);
//                dsSetColorAlpha(1f, 1f, 0f, 0.5f);
//                dsDrawTriangle(pos, rot, v0, v1, v2, true);
//            }
            int numi = track_faces.length/3;
            for (int ii=0; ii<numi; ++ii) {
                int v0 = track_faces[ii + 0] * 3;
                int v1 = track_faces[ii + 1] * 3;
                int v2 = track_faces[ii + 2] * 3;

                dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

                dsSetDrawMode(DS_WIREFRAME);
                dsSetColorAlpha(0f, 0f, 0f, 1.0f);
                dsDrawTriangle(pos, rot, track_verts, v0, v1, v2, true);

                dsSetDrawMode(DS_POLYFILL);
                dsSetColorAlpha(1f, 1f, 0f, 0.5f);
                dsDrawTriangle(pos, rot, track_verts, v0, v1, v2, true);
            }
        }
    }



    void simLoop (boolean pause)
    {
        if (!pause) {

            final double step = 0.02;
            final int nsteps = 1;

            for (int i=0; i<nsteps; ++i) {
                OdeHelper.spaceCollide(space, null, nearCallback);
                world.quickStep(step);
                contact_group.empty();
            }
        } else {
            OdeHelper.spaceCollide(space, null, nearCallback);
            contact_group.empty();
        }

        // now we draw everything
		for (DGeom g : space.getGeoms()) {

            if (g == ground)
                continue; // drawstuff is already drawing it for us

            drawGeom(g);
        }

        if (ball1_body.getPosition().get0() < -track_len)
            resetSim();
    }

    /**
     * @param args args
     */
    public static void main(final String[] args) {
        new DemoTracks().demo(args);
    }

    private void demo(String[] args) {
        // create world
        OdeHelper.initODE2(0);

        // run demo
        dsSimulationLoop (args, 800, 600, this);

        OdeHelper.closeODE();
    }


    @Override
    public void step(boolean pause) {
        simLoop(pause);
    }
}