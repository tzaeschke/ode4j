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

import static org.ode4j.demo.BunnyGeom.IndexCount;
import static org.ode4j.demo.BunnyGeom.Indices;
import static org.ode4j.demo.BunnyGeom.Vertices;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsDrawConvex;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsDrawTriangle;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactBounce;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
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
import org.ode4j.ode.DHeightfield;
import org.ode4j.ode.DHeightfield.DHeightfieldGetHeight;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.DxTrimeshHeightfield;


class DemoLayeredTrimeshHeightfield extends dsFunctions {

	private static final float DEGTORAD = 0.01745329251994329577f	; //!< PI / 180.0, convert degrees to radians

	// Our heightfield geom
	private DxTrimeshHeightfield gheight;
	// Our heightfield geom
	private DxTrimeshHeightfield gheight2;

	// Heightfield dimensions

	private static final int HFIELD_WSTEP =			15;			// Vertex count along edge >= 2
	private static final int HFIELD_DSTEP =			31;

	private static final float HFIELD_WIDTH =			4.0f;
	private static final float HFIELD_DEPTH =			8.0f;

	private static final float HFIELD_WSAMP =			( HFIELD_WIDTH / ( HFIELD_WSTEP-1 ) );
	private static final float HFIELD_DSAMP =			( HFIELD_DEPTH / ( HFIELD_DSTEP-1 ) );



	//<---- Convex Object
	private double[] planes= // planes for a cube
	{
			1.0f ,0.0f ,0.0f ,0.25f,
			0.0f ,1.0f ,0.0f ,0.25f,
			0.0f ,0.0f ,1.0f ,0.25f,
			0.0f ,0.0f ,-1.0f,0.25f,
			0.0f ,-1.0f,0.0f ,0.25f,
			-1.0f,0.0f ,0.0f ,0.25f
	};
	private final int planecount=6;

	private double points[]= // points for a cube
	{
			0.25f,0.25f,0.25f,  //  point 0
			-0.25f,0.25f,0.25f, //  point 1

			0.25f,-0.25f,0.25f, //  point 2
			-0.25f,-0.25f,0.25f,//  point 3

			0.25f,0.25f,-0.25f, //  point 4
			-0.25f,0.25f,-0.25f,//  point 5

			0.25f,-0.25f,-0.25f,//  point 6
			-0.25f,-0.25f,-0.25f,// point 7
	};
	private final int pointcount=8;
	private int polygons[] = //Polygons for a cube (6 squares)
	{
			4,0,2,6,4, // positive X
			4,1,0,4,5, // positive Y
			4,0,1,3,2, // positive Z
			4,3,1,5,7, // negative X
			4,2,3,7,6, // negative Y
			4,5,4,6,7, // negative Z
	};
	//----> Convex Object


	// some constants

	private static final int NUM = 100;			// max number of objects
	private static final float DENSITY = 5.0f	;	// density of all objects
	private static final int  GPB = 3;			// maximum number of geometries per body
	private static final int  MAX_CONTACTS = 64;		// maximum number of contact points per body


	// dynamics and collision objects

	private class MyObject {
		DBody body;			// the body
		DGeom[] geom = new DGeom[GPB];		// geometries representing this body

		// Trimesh only - double buffered matrices for 'last transform' setup
		//double[] matrix_dblbuff = new double[ 16 * 2 ];
		//int last_matrix_index;
	};

	private int num=0;		// number of objects in simulation
	private int nextobj=0;		// next object to recycle if num==NUM
	private DWorld world;
	private DSpace space;
	private MyObject[] obj = new MyObject[NUM];
	private DJointGroup contactgroup;
	private int selected = -1;	// selected object
	private boolean show_aabb = false;	// show geom AABBs?
	private boolean show_contacts = false;	// show contact points?
	private boolean random_pos = true;	// drop objects from random position?


	//============================

	//private DGeom TriMesh1;
	//private DGeom TriMesh2;
	//static dTriMeshDataID TriData1, TriData2;  // reusable static trimesh data

	//============================

	private DHeightfieldGetHeight heightfield_callback = new DHeightfieldGetHeight(){
		@Override
		public double call(Object pUserData, int x, int z) {
			return heightfield_callback(pUserData, x, z);
		}
	};

	private double heightfield_callback( Object pUserData, int x, int z ) {
		double fx = ( ((double)x) - ( HFIELD_WSTEP-1 )/2 ) / ( HFIELD_WSTEP-1 );
		double fz = ( ((double)z) - ( HFIELD_DSTEP-1 )/2 ) / ( HFIELD_DSTEP-1 );

		double h = ( 1.0 ) + ( ( -2 ) * ( fx + fz ) );
		if ((x *x+ z * z) % 30 == 1) {
			h = Double.NaN;
		}
		return h;
	}

	private DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};


	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback (Object data, DGeom o1, DGeom o2) {
		int i;
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && areConnectedExcluding (b1,b2,DContactJoint.class)) {
			return;
		}

		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
		for (i=0; i<MAX_CONTACTS; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());
		if (numc!=0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			final DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (i=0; i<numc; i++) {
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contacts.get(i));
				c.attach (b1,b2);
				if (show_contacts) {
					dsSetColor(0, 0, 1);
					dsDrawBox (contacts.get(i).geom.pos,RI,ss);
				}
			}
		}
	}

	private static float[] xyz = {1.1640f,-7.0079f,4.2600f};
	private static float[] hpr = {95.5000f,-12.0000f,0.0000f};

	// start simulation - set viewpoint
	@Override
	public void start() {
		dsSetViewpoint (xyz,hpr);
		System.out.println ("To drop another object, press:\n");
		System.out.println ("   b for box.");
		System.out.println ("   s for sphere.");
		System.out.println ("   c for capsule.");
		System.out.println ("   y for cylinder.");
		System.out.println ("   v for a convex object.");
		System.out.println ("   x for a composite object.");
		System.out.println ("   m for a trimesh.");
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
		int i;
		int j,k;
		double[] sides = new double[3];
		DMass m = OdeHelper.createMass();
	    boolean setBody = false;

		cmd = Character.toLowerCase(cmd);


		//
		// Geom Creation
		//

		if ( cmd == 'b' || cmd == 's' || cmd == 'c' || ( cmd == 'm' ) ||
				cmd == 'x' || cmd == 'y' || cmd == 'v' ) {
			if ( num < NUM ) {
				i = num;
				num++;
			} else {
				i = nextobj;
				nextobj++;
				nextobj %= num;

				// destroy the body and geoms for slot i
				obj[i].body.destroy();
				obj[i].body = null;
				
				for (k=0; k < GPB; k++)	{
					if (obj[i].geom[k]!=null) {
						obj[i].geom[k].destroy();
						obj[i].geom[k] = null;
					}
				}
				//memset (&obj[i],0,sizeof(obj[i]));
				obj[i] = new MyObject();
			}

			obj[i].body = OdeHelper.createBody (world);
			for (k=0; k<3; k++) {
				sides[k] = dRandReal()*0.2+0.2;
			}

			DMatrix3 R = new DMatrix3();
			if (random_pos) {
				obj[i].body.setPosition(
						(dRandReal()-0.5)*HFIELD_WIDTH*0.75,
						(dRandReal()-0.5)*HFIELD_DEPTH*0.75,
						dRandReal() + 5 );
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			} else {
				double maxheight = 0;
				for (k=0; k<num; k++) {
					final DVector3C pos = obj[k].body.getPosition();
					if (pos.get2() > maxheight) {
						maxheight = pos.get2();
					}
				}
				obj[i].body.setPosition(0,maxheight+1,0);
				dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
			}
			obj[i].body.setRotation (R);
			obj[i].body.setData (i);

			if (cmd == 'b') {
				m.setBox (DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
			} else if (cmd == 'c') {
				sides[0] *= 0.5;
				m.setCapsule (DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCapsule (space,sides[0],sides[1]);
			} else if (cmd == 'v') {
				m.setBox (DENSITY,0.25,0.25,0.25);
				obj[i].geom[0] = OdeHelper.createConvex (space,
						planes,
						planecount,
						points,
						pointcount,
						polygons);
			} else if (cmd == 'y') {
				m.setCylinder (DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCylinder (space,sides[0],sides[1]);
			} else if (cmd == 's') {
				sides[0] *= 0.5;
				m.setSphere (DENSITY,sides[0]);
				obj[i].geom[0] = OdeHelper.createSphere (space,sides[0]);
			} else if (cmd == 'm') {
				DTriMeshData new_tmdata = OdeHelper.createTriMeshData();
				new_tmdata.build(Vertices, Indices);

				obj[i].geom[0] = OdeHelper.createTriMesh(space, new_tmdata, null, null, null);

				m.setTrimesh( DENSITY, (DTriMesh)obj[i].geom[0] );
				DVector3 c = new DVector3(m.getC());
				c.scale(-1);
				obj[i].geom[0].setPosition(c);
				m.translate(c);
			} else if (cmd == 'x') {
				setBody = true;
				// start accumulating masses for the encapsulated geometries
				DMass m2 = OdeHelper.createMass();
				m.setZero ();

				DVector3[] dpos = DVector3.newArray(GPB);	// delta-positions for encapsulated geometries
				DMatrix3[] drot = DMatrix3.newArray(GPB);

				// set random delta positions
				for (j=0; j<GPB; j++) {
					for (k=0; k<3; k++) dpos[j].set(k, dRandReal()*0.3-0.15);
				}

				for (k=0; k<GPB; k++) {
					if (k==0) {
						double radius = dRandReal()*0.25+0.05;
						obj[i].geom[k] = OdeHelper.createSphere (space, radius);
						m2.setSphere (DENSITY,radius);
					} else if (k==1) {
						obj[i].geom[k] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
						m2.setBox (DENSITY,sides[0],sides[1],sides[2]);
					} else {
						double radius = dRandReal()*0.1+0.05;
						double length = dRandReal()*1.0+0.1;
						obj[i].geom[k] = OdeHelper.createCapsule (space,radius,length);
						m2.setCapsule (DENSITY,3,radius,length);
					}

					dRFromAxisAndAngle(drot[k], dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                                   dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
					m2.rotate (drot[k]);

	                m2.translate(dpos[k]);

					// add to the total mass
					m.add (m2);
				}

				// move all encapsulated objects so that the center of mass is (0,0,0)
				DVector3 c = new DVector3().set(m.getC());
				c.scale(-1);
				for (k=0; k<GPB; k++) {
					obj[i].geom[k].setBody(obj[i].body);
//					dGeomSetOffsetPosition (obj[i].geom[k],
//							dpos[k][0]-m.c[0],
//							dpos[k][1]-m.c[1],
//							dpos[k][2]-m.c[2]);
					obj[i].geom[k].setOffsetPosition(dpos[k].reAdd(c));
					obj[i].geom[k].setOffsetRotation(drot[k]);
				}
//				dMassTranslate (m,-m.c[0],-m.c[1],-m.c[2]);
				m.translate(c);
				obj[i].body.setMass(m);
			}

			if (!setBody) {
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) {
						obj[i].geom[k].setBody(obj[i].body);
					}
				}

				obj[i].body.setMass(m);
			}
		}


		//
		// Control Commands
		//

		if (cmd == ' ') {
			selected++;
			if (selected >= num) 
				selected = 0;
			if (selected < -1) 
				selected = 0;
		} else if (cmd == 'd' && selected >= 0 && selected < num) {
			obj[selected].body.disable();
		} else if (cmd == 'e' && selected >= 0 && selected < num) {
			obj[selected].body.enable();
		} else if (cmd == 'a') {
			show_aabb = !show_aabb;
		} else if (cmd == 't') {
			show_contacts = !show_contacts;
		} else if (cmd == 'r') {
			random_pos = !random_pos;
		}
	}


	// draw a geom

	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb) {
		if (g==null) 
			return;
		if (pos==null) 
			pos = g.getPosition ();
		if (R==null) 
			R = g.getRotation ();

		if (g instanceof DBox) {
			DVector3C sides = ((DBox)g).getLengths();
			dsDrawBox (pos,R,sides);
			
		} else if (g instanceof DSphere) {
			dsDrawSphere (pos, R, ((DSphere)g).getRadius());
			
		} else if (g instanceof DCapsule) {
			DCapsule cap = (DCapsule) g; 
			dsDrawCapsule (pos, R, cap.getLength(), cap.getRadius());
			
		} else if (g instanceof DConvex) {
			//dVector3 sides={0.50,0.50,0.50};
			dsDrawConvex(pos,R,planes,
					planecount,
					points,
					pointcount,
					polygons);
			
		} else if (g instanceof DCylinder) {
			DCylinder cyl = (DCylinder) g;
			dsDrawCylinder (pos, R, cyl.getLength(), cyl.getRadius());
			
		} else if (g instanceof DTriMesh) {
	        int[] Indices = BunnyGeom.Indices;

	        // assume all trimeshes are drawn as bunnies
	        for (int ii = 0; ii < IndexCount / 3; ii++) {
	            float[] v0 = { // explicit conversion from float to dReal
	                Vertices[Indices[ii * 3 + 0] * 3 + 0],
	                Vertices[Indices[ii * 3 + 0] * 3 + 1],
	                Vertices[Indices[ii * 3 + 0] * 3 + 2]};
	            float[] v3 = {
	                Vertices[Indices[ii * 3 + 1] * 3 + 0],
	                Vertices[Indices[ii * 3 + 1] * 3 + 1],
	                Vertices[Indices[ii * 3 + 1] * 3 + 2]};
	            float[] v6 = {
	                Vertices[Indices[ii * 3 + 2] * 3 + 0],
	                Vertices[Indices[ii * 3 + 2] * 3 + 1],
	                Vertices[Indices[ii * 3 + 2] * 3 + 2]
	            };
	            dsDrawTriangle(pos, R, v0, v3, v6, true);
	        }

	    } else if (g instanceof DHeightfield) {

	        // Set ox and oz to zero for DHEIGHTFIELD_CORNER_ORIGIN mode.
	        int ox = (int) ( -HFIELD_WIDTH/2 );
	        int oz = (int) ( -HFIELD_DEPTH/2 );

	        //	for ( int tx = -1; tx < 2; ++tx )
	        //	for ( int tz = -1; tz < 2; ++tz )
	        dsSetColorAlpha (0.5,1,0.5,0.5);
	        dsSetTexture( DS_TEXTURE_NUMBER.DS_WOOD );

	        for ( int i = 0; i < HFIELD_WSTEP - 1; ++i )
	            for ( int j = 0; j < HFIELD_DSTEP - 1; ++j ) {
	                float[] a = new float[3], b = new float[3];
	                float[] c = new float[3], d = new float[3];

	                a[ 0 ] = ox + ( i ) * HFIELD_WSAMP;
	                a[ 1 ] = (float) heightfield_callback( null, i, j );
	                a[ 2 ] = oz + ( j ) * HFIELD_DSAMP;

	                b[ 0 ] = ox + ( i + 1 ) * HFIELD_WSAMP;
	                b[ 1 ] = (float) heightfield_callback( null, i + 1, j );
	                b[ 2 ] = oz + ( j ) * HFIELD_DSAMP;

	                c[ 0 ] = ox + ( i ) * HFIELD_WSAMP;
	                c[ 1 ] = (float) heightfield_callback( null, i, j + 1 );
	                c[ 2 ] = oz + ( j + 1 ) * HFIELD_DSAMP;
	                
	                d[ 0 ] = ox + ( i + 1 ) * HFIELD_WSAMP;
	                d[ 1 ] = (float) heightfield_callback( null, i + 1, j + 1 );
	                d[ 2 ] = oz + ( j + 1 ) * HFIELD_DSAMP;
	                if (Float.isNaN(a[1]) || Float.isNaN(b[1]) || Float.isNaN(c[1]) || Float.isNaN(d[1]))
	                    continue;
	                dsDrawTriangle( pos, R, a, c, b, true );
	                dsDrawTriangle( pos, R, b, c, d, true );
	            }

		}

		drawAABB(g);
	}
	
	private void drawAABB(DGeom g) {
		if (show_aabb) {
			// draw the bounding box for this geom
			DAABBC aabb = g.getAABB();
			DVector3 bbpos = aabb.getCenter();
			DVector3 bbsides = aabb.getLengths();
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity ();
			dsSetColorAlpha (1,0,0,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}

	// simulation loop

	@Override
	public void step (boolean pause) {

		space.collide (null,nearCallback);

		if (!pause) {
			world.quickStep (0.05);
		}
		
		// remove all contact joints
		contactgroup.empty();


		//
		// Draw Heightfield
		//
		
		drawGeom(gheight, null, null, false);
		drawGeom(gheight2, null, null, false);

		dsSetColorAlpha (0.5f,1,0.5f,0.5f);
		dsSetTexture( DS_TEXTURE_NUMBER.DS_WOOD );
		for ( int i = 0; i < num; ++i ) {
			for ( int j = 0; j < GPB; ++j ) {
				if (i==selected) {
					dsSetColor (0,0.7,1);
				} else if (! obj[i].body.isEnabled ()) {
					dsSetColor (1,0.8,0);
				} else {
					dsSetColor (1,1,0);
				}

				drawGeom (obj[i].geom[j],null,null,show_aabb);
			}
		}
	}


	public static void main(String[] args) {
		new DemoLayeredTrimeshHeightfield().demo(args);
	}

	private void demo(String[] args) {
		System.out.println("ODE configuration: " + OdeHelper.getConfiguration());

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld ();
		space = OdeHelper.createHashSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0,0,-0.2);
		world.setCFM (1e-5);
		world.setAutoDisableFlag (true);
		world.setContactMaxCorrectingVel (0.1);
		world.setContactSurfaceLayer (0.001);
		for (int i = 0; i < obj.length; i++) {
			obj[i] = new MyObject();
		}

		world.setAutoDisableAverageSamplesCount( 1 );

		// base plane to catch overspill
		OdeHelper.createPlane( space, 0, 0, 1, 0 );


		// our heightfield floor

		DHeightfieldData height = OdeHelper.createHeightfieldData();

		// Create an finite heightfield.
		height.buildCallback( null, heightfield_callback,
				HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
				1.0, 0.0, 0.0, false );
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
		height.setBounds( ( -4.0 ), ( +6.0 ) );

		gheight = new DxTrimeshHeightfield(space, height, true, true);// OdeHelper.createHeightfield( space, height, true );
		gheight2 = new DxTrimeshHeightfield(space, height, true, true);// OdeHelper.createHeightfield( space, height, true );
		DVector3 pos = new DVector3(0, 0, 3.5);

		// Rotate so Z is up, not Y (which is the default orientation)
		DMatrix3 R = new DMatrix3();
		R.setIdentity();
		dRFromAxisAndAngle( R, 1, 0, 0, DEGTORAD * 90 );

		// Place it.
		gheight.setRotation( R );
		gheight.setPosition( pos );

		R = new DMatrix3();
		R.setIdentity();
		dRFromAxisAndAngle( R, 1, 0, 0, DEGTORAD * 90 );
		DMatrix3 R2 = new DMatrix3();
		R2.setIdentity();
		dRFromAxisAndAngle( R2, 0, 0, 1, DEGTORAD * 180 );
		DMatrix3 R3 = new DMatrix3(); 
		OdeMath.dMultiply0_333(R3, R2, R);
		gheight2.setRotation(R3);
		pos = new DVector3(2, 0, 0);
		gheight2.setPosition(pos);
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
 
		contactgroup.destroy();
		space.destroy();
		world.destroy();

		// destroy heightfield data, because _we_ own it not ODE
		height.destroy();

		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
		// Nothing
	}
}