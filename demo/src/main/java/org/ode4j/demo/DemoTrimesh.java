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
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DRay;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeHelper.*;
import static org.ode4j.ode.OdeMath.*;


// TriMesh test by Erwin de Vries
class DemoTrimesh extends dsFunctions {

	// some constants

	private static final int NUM = 200;			// max number of objects
	private static final double DENSITY = (5.0);		// density of all objects
	private static final int GPB = 3;			// maximum number of geometries per body
	private static final int MAX_CONTACTS = 40;		// maximum number of contact points per body


	// dynamics and collision objects

	private static class MyObject {
		DBody body;			// the body
		DGeom[] geom = new DGeom[GPB];		// geometries representing this body
	};

	private static int num=0;		// number of objects in simulation
	private static int nextobj=0;		// next object to recycle if num==NUM
	private static DWorld world;
	private static DSpace space;
	private static MyObject[] obj = new MyObject[NUM];
	private static DJointGroup contactgroup;
	private static int selected = -1;	// selected object
	private static boolean show_aabb = false;	// show geom AABBs?
	private static boolean show_contacts = false;	// show contact points?
	private static boolean random_pos = true;	// drop objects from random position?

	private static final int VertexCount = 5;
	private static final int IndexCount = 12;

	private static float[] Size;
	private static float[] Vertices = new float[VertexCount*3];
	private static int[] Indices = new int[IndexCount];

	private static DTriMesh TriMesh;
	private static DRay Ray;


	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback( data, o1, o2);
		}
	};

	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback (Object data, DGeom o1, DGeom o2)	{
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && areConnectedExcluding (b1,b2,DContactJoint.class)) return;

		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
		for (int i=0; i<MAX_CONTACTS; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer() );
		if (numc != 0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity ();
			DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (int i=0; i<numc; i++) {
				DContact contact = contacts.get(i);
				if (o1 instanceof DRay || o2 instanceof DRay){
					DMatrix3 Rotation = new DMatrix3();
					Rotation.setIdentity();
					dsDrawSphere(contact.geom.pos, Rotation, (0.01));

					DVector3 End = new DVector3();
					End.eqSum( contact.geom.pos, contact.geom.normal, contact.geom.depth );

					dsDrawLine(contact.geom.pos, End);
					continue;
				}

				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact );
				c.attach (b1,b2);
				if (show_contacts) dsDrawBox (contact.geom.pos,RI,ss);
			}
		}
	}


	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};

	// start simulation - set viewpoint

	@Override
	public void start()	{
		dsSetViewpoint (xyz,hpr);
		System.out.println ("To drop another object, press:");
		System.out.println ("   b for box.");
		System.out.println ("   s for sphere.");
		System.out.println ("   c for capsule.");
		System.out.println ("   y for cylinder.");
		System.out.println ("   v for a convex object.");
		System.out.println ("   x for a composite object.");
		System.out.println ("To select an object, press space.");
		System.out.println ("To disable the selected object, press d.");
		System.out.println ("To enable the selected object, press e.");
		System.out.println ("To toggle showing the geom AABBs, press a.");
		System.out.println ("To toggle showing the contact points, press t.");
		System.out.println ("To toggle dropping from random position/orientation, press r.");
	}


	// called when a key pressed

	@Override
	public void command (char cmd) {
		int i,j,k;
		double[] sides= new double[3];
		DMass m = OdeHelper.createMass();
		boolean setBody = false;

		cmd = Character.toLowerCase (cmd);
		if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y' || cmd == 'v') {
			if (num < NUM) {
				i = num;
				num++;
			}
			else {
				i = nextobj;
				nextobj++;
				if (nextobj >= num) nextobj = 0;

				// destroy the body and geoms for slot i
				obj[i].body.destroy ();
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) obj[i].geom[k].destroy ();
				}
				obj[i] = new MyObject();
			}

			obj[i].body = OdeHelper.createBody (world);
			for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if (random_pos) {
				double z = dRandReal()+1;
				double y = dRandReal()*2-1;
				double x = dRandReal()*2-1;
				obj[i].body.setPosition (x, y, z);
//				obj[i].body.setPosition (
//						dRandReal()*2-1,dRandReal()*2-1,dRandReal()+1);
				double d = dRandReal()*10.0-5.0;
				double c = dRandReal()*2.0-1.0;
				double b = dRandReal()*2.0-1.0;
				double a = dRandReal()*2.0-1.0;
				dRFromAxisAndAngle (R,a, b, c, d);
//				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
//						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			}
			else {
				double maxheight = 0;
				for (k=0; k<num; k++) {
					DVector3C pos = obj[k].body.getPosition ();
					if (pos.get2() > maxheight) maxheight = pos.get2();
				}
				obj[i].body.setPosition (0,0,maxheight+1);
				dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
			}
			obj[i].body.setRotation (R);
			obj[i].body.setData (i);

			if (cmd == 'b') {
				m.setBox (DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
			}
			else if (cmd == 'c') {
				sides[0] *= 0.5;
				m.setCapsule (DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCapsule (space,sides[0],sides[1]);
			} else if (cmd == 'v') {
				m.setBox (DENSITY,0.25,0.25,0.25);
				obj[i].geom[0] = OdeHelper.createConvex (space,
						ConvexCubeGeom.planes,
						ConvexCubeGeom.planecount,
						ConvexCubeGeom.points,
						ConvexCubeGeom.pointcount,
						ConvexCubeGeom.polygons);
			}
			else if (cmd == 'y') {
				sides[1] *= 0.5;
				m.setCylinder(DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCylinder (space,sides[0],sides[1]);
			}
			else if (cmd == 's') {
				sides[0] *= 0.5;
				m.setSphere (DENSITY,sides[0]);
				obj[i].geom[0] = OdeHelper.createSphere (space,sides[0]);
			}
			else if (cmd == 'x') {
				setBody = true;

				// start accumulating masses for the encapsulated geometries
				DMass m2 = OdeHelper.createMass();
				m.setZero ();

				DVector3[] dpos = DVector3.newArray(GPB);	// delta-positions for encapsulated geometries
				DMatrix3[] drot = DMatrix3.newArray(GPB); 

				// set random delta positions
				for (j=0; j<GPB; j++) {
					for (k=0; k<3; k++) {
						dpos[j].set(k, dRandReal()*0.3-0.15 );
					}
				}

				for (k=0; k<GPB; k++) {
					if (k==0) {
						double radius = dRandReal()*0.25+0.05;
						obj[i].geom[k] = OdeHelper.createSphere (space,radius);
						m2.setSphere (DENSITY,radius);
					}
					else if (k==1) {
						obj[i].geom[k] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
						m2.setBox (DENSITY,sides[0],sides[1],sides[2]);
					}
					else {
						double radius = dRandReal()*0.1+0.05;
						double length = dRandReal()*1.0+0.1;
						obj[i].geom[k] = OdeHelper.createCapsule (space,radius,length);
						m2.setCapsule (DENSITY,3,radius,length);
					}

					dRFromAxisAndAngle (drot[k],dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
							dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);

					m2.rotate (drot[k]);

					m2.translate (dpos[k]);

					// add to the total mass
					m.add (m2);
				}

				// move all encapsulated objects so that the center of mass is (0,0,0)
				DVector3 negC = new DVector3(m.getC()).scale(-1);
				for (k=0; k<GPB; k++) {
					obj[i].geom[k].setBody(obj[i].body);
	                obj[i].geom[k].setOffsetPosition(dpos[k].reAdd(negC));
	                obj[i].geom[k].setOffsetRotation(drot[k]);
				}
				m.translate(negC);
				obj[i].body.setMass(m);
			}

			if (!setBody) {
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) {
						obj[i].geom[k].setBody (obj[i].body);
					}
				}
	
				obj[i].body.setMass (m);
			}
		}

		if (cmd == ' ') {
			selected++;
			if (selected >= num) selected = 0;
			if (selected < 0) selected = 0;
		} else if (cmd == 'd' && selected >= 0 && selected < num) {
			obj[selected].body.disable ();
		} else if (cmd == 'e' && selected >= 0 && selected < num) {
			obj[selected].body.enable ();
		} else if (cmd == 'a') {
			show_aabb ^= true;
		} else if (cmd == 't') {
			show_contacts ^= true;
		} else if (cmd == 'r') {
			random_pos ^= true;
		}
	}


	// draw a geom

	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb)
	{
		if (g==null) return;
		if (pos==null) pos = g.getPosition ();
		if (R==null) R = g.getRotation ();

		if (g instanceof DBox) {
			DVector3C sides = ((DBox)g).getLengths();
			dsDrawBox (pos,R,sides);
		} else if (g instanceof DSphere) {
			dsDrawSphere( pos,R, ((DSphere)g).getRadius() );
		} else if (g instanceof DCapsule) {
			DCapsule c = (DCapsule) g;
			dsDrawCapsule( pos, R, c.getLength(), c.getRadius() );
		} else if (g instanceof DCylinder) {
			DCylinder c = (DCylinder) g;
			dsDrawCylinder (pos, R, c.getLength(), c.getRadius());
		} else if (g instanceof DConvex) {
			//dVector3 sides={0.50,0.50,0.50};
			dsDrawConvex(pos,R,ConvexCubeGeom.planes,
					ConvexCubeGeom.planecount,
					ConvexCubeGeom.points,
					ConvexCubeGeom.pointcount,
					ConvexCubeGeom.polygons);
		}

		if (show_aabb) {
			// draw the bounding box for this geom
			DAABBC aabb = g.getAABB();
			DVector3 bbpos = aabb.getCenter();
			DVector3 bbsides = aabb.getLengths();
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity ();
			dsSetColorAlpha (1f,0f,0f,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}


	// simulation loop

	@Override
	public void step (boolean pause)
	{
		dsSetColor (0,0,2);
		space.collide (0,nearCallback);
		if (!pause) world.step (0.05);
		//if (!pause) dWorldStepFast (world,0.05, 1);

		// remove all contact joints
		contactgroup.empty ();

		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++) {
			for (int j=0; j < GPB; j++) {
				if (i==selected) {
					dsSetColor (0f,0.7f,1f);
				} else if (! obj[i].body.isEnabled ()) {
					dsSetColor (1,0,0);
				} else {
					dsSetColor (1,1,0);
				}
				drawGeom (obj[i].geom[j], null, null, show_aabb);
			}
		}

		/*{
    for (int i = 1; i < IndexCount; i++) {
      dsDrawLine(Vertices[Indices[i - 1]], Vertices[Indices[i]]);
    }
  }*/

		if (TriMesh!=null) {
			DVector3C Pos = TriMesh.getPosition();
			DMatrix3C Rot = TriMesh.getRotation();
			for (int i = 0; i < IndexCount; i+=3){
				int p0 = Indices[i + 0] * 3;
				int p1 = Indices[i + 1] * 3;
				int p2 = Indices[i + 2] * 3;
				dsDrawTriangle(Pos, Rot, Vertices, p0, p1, p2, false);
			}
		}

		if (Ray!=null) {
			DVector3C Origin = Ray.getPosition();
			DVector3C Direction = Ray.getDirection();

			double Length = Ray.getLength();

			DVector3 End = new DVector3();
			End.eqSum( Origin, Direction, Length );

			dsDrawLine(Origin, End);
		}
	}

	public static void main(String[] args) {
		new DemoTrimesh().demo(args);
	}

	private void demo (String[] args) {
		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();

		space = OdeHelper.createSimpleSpace();
		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-0.5);
		world.setCFM (1e-5);
		//dCreatePlane (space,0,0,1,0);
		//memset (obj,0,sizeof(obj));
		for (int i = 0; i < obj.length; i++) obj[i] = new MyObject();

		Size = new float[]{ 5.0f, 5.0f, 2.5f };

		Vertices = new float[]{ 
				-Size[0], -Size[1], Size[2],
				 Size[0], -Size[1], Size[2],
				 Size[0],  Size[1], Size[2],
				-Size[0],  Size[1], Size[2],
				0f, 0f, 0f};

		Indices = new int[] {
				0, 1, 4,
				1, 2, 4,
				2, 3, 4,
				3, 0, 4
		};

		DTriMeshData Data = OdeHelper.createTriMeshData();

		//dGeomTriMeshDataBuildSimple(Data, (dReal*)Vertices, VertexCount, Indices, IndexCount);
		//dGeomTriMeshDataBuildSingle(Data, Vertices[0], 3 * sizeof(float), VertexCount, &Indices[0], IndexCount, 3 * sizeof(dTriIndex));
		//		Data.build(Data, Vertices[0], 3,// * sizeof(float), 
		//				VertexCount, Indices[0], IndexCount, 3);// * sizeof(dTriIndex));
		Data.build(Vertices, Indices);// * sizeof(dTriIndex));
		Data.preprocess();
		TriMesh = OdeHelper.createTriMesh(space, Data, null, null, null);

		//TriMesh.setPosition(0, 0, 1.0);

		Ray = OdeHelper.createRay(space, 0.9);
		DVector3 Origin, Direction;
		Origin = new DVector3( 0, 0, 0.5 );

		Direction = new DVector3( 0, 1.1, -1 );
		Direction.normalize();

		Ray.set(Origin, Direction);

		//TODO
//	    DThreadingImplementation threading = OdeHelper.allocateMultiThreaded();
//	    DThreadingThreadPool pool = OdeHelper.allocateThreadPool(4, 0, /*dAllocateFlagBasicData,*/ null);
//	    pool.serveMultiThreadedImplementation(threading);
//	    // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
//	    world.setStepThreadingImplementation(threading.dThreadingImplementationGetFunctions(), threading);

		// run simulation
		dsSimulationLoop (args,640,480,this);

//	    threading.shutdownProcessing();//dThreadingImplementationShutdownProcessing(threading);
//	    pool.freeThreadPool();
//	    world.setStepThreadingImplementation(null, null);
//	    threading.free();

		contactgroup.destroy ();
		space.destroy ();
		world.destroy ();
		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
		// Nothing
	}
}