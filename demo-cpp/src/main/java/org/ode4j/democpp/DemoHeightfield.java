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
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyDestroy;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyDisable;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyEnable;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyIsEnabled;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetData;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCapsule;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateConvex;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCylinder;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateHeightfield;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetAABB;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataBuildCallback;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataCreate;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataSetBounds;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSphereGetRadius;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dCreateTriMesh;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dGeomTriMeshDataBuildSingle;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dGeomTriMeshDataCreate;
import static org.ode4j.cpp.internal.ApiCppExportDIF.dWorldExportDIF;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdd;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassRotate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCapsule;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCylinder;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetTrimesh;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetZero;
import static org.ode4j.cpp.internal.ApiCppMass.dMassTranslate;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppOther.dAreConnectedExcluding;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetAutoDisableAverageSamplesCount;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetAutoDisableFlag;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetCFM;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetContactMaxCorrectingVel;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetContactSurfaceLayer;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.democpp.BunnyGeom.IndexCount;
import static org.ode4j.democpp.BunnyGeom.Indices;
import static org.ode4j.democpp.BunnyGeom.VertexCount;
import static org.ode4j.democpp.BunnyGeom.Vertices;
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
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fclose;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fopen;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
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
import org.ode4j.ode.internal.cpp4j.FILE;


class DemoHeightfield extends dsFunctions {

	private static final float DEGTORAD = 0.01745329251994329577f	; //!< PI / 180.0, convert degrees to radians

	// Our heightfield geom
	private DGeom gheight;

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
			/*
    1.0f ,0.0f ,0.0f ,2.0f,
    0.0f ,1.0f ,0.0f ,1.0f,
    0.0f ,0.0f ,1.0f ,1.0f,
    0.0f ,0.0f ,-1.0f,1.0f,
    0.0f ,-1.0f,0.0f ,1.0f,
    -1.0f,0.0f ,0.0f ,0.0f
			 */
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
		double[] matrix_dblbuff = new double[ 16 * 2 ];
		int last_matrix_index;
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
	private static boolean write_world = false;




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

	private double heightfield_callback( Object pUserData, int x, int z )
	{
		dIASSERT( x < HFIELD_WSTEP );
		dIASSERT( z < HFIELD_DSTEP );

		double fx = ( ((double)x) - ( HFIELD_WSTEP-1 )/2 ) / ( HFIELD_WSTEP-1 );
		double fz = ( ((double)z) - ( HFIELD_DSTEP-1 )/2 ) / ( HFIELD_DSTEP-1 );

		// Create an interesting 'hump' shape
		double h = ( 1.0 ) + ( ( -16.0 ) * ( fx*fx*fx + fz*fz*fz ) );

		return h;
	}


	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};


	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i;
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = dGeomGetBody(o1);
		DBody b2 = dGeomGetBody(o2);
		if (b1!=null && b2!=null && dAreConnectedExcluding (b1,b2,DContactJoint.class)) return;

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
		int numc = dCollide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());
		if (numc!=0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			final DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (i=0; i<numc; i++) {
				DJoint c = dJointCreateContact (world,contactgroup,contacts.get(i));
				dJointAttach (c,b1,b2);
				if (show_contacts) dsDrawBox (contacts.get(i).geom.pos,RI,ss);
			}
		}
	}

	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};

	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		printf ("To drop another object, press:\n");
		printf ("   b for box.\n");
		printf ("   s for sphere.\n");
		printf ("   c for capsule.\n");
		printf ("   y for cylinder.\n");
		printf ("   v for a convex object.\n");
		printf ("   x for a composite object.\n");
		printf ("   m for a trimesh.\n");
		printf ("To select an object, press space.\n");
		printf ("To disable the selected object, press d.\n");
		printf ("To enable the selected object, press e.\n");
		printf ("To toggle showing the geom AABBs, press a.\n");
		printf ("To toggle showing the contact points, press t.\n");
		printf ("To toggle dropping from random position/orientation, press r.\n");
		printf ("To save the current state to 'state.dif', press 1.\n");
	}


//	private char locase (char c)
//	{
//		if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
//		else return c;
//	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
		int i;
		int j,k;
		double[] sides = new double[3];
		DMass m = OdeHelper.createMass();
		boolean setBody;

		cmd = Character.toLowerCase(cmd);//locase (cmd);


		//
		// Geom Creation
		//

		if ( cmd == 'b' || cmd == 's' || cmd == 'c' || ( cmd == 'm' ) ||
				cmd == 'x' || cmd == 'y' || cmd == 'v' )
		{
			setBody = false;
			if ( num < NUM )
			{
				i = num;
				num++;
			}
			else
			{
				i = nextobj;
				nextobj++;
				if (nextobj >= num) nextobj = 0;

				// destroy the body and geoms for slot i
				dBodyDestroy (obj[i].body);
				for (k=0; k < GPB; k++)
				{
					if (obj[i].geom[k]!=null) dGeomDestroy (obj[i].geom[k]);
				}
				//memset (&obj[i],0,sizeof(obj[i]));
				obj[i] = new MyObject();
			}

			obj[i].body = dBodyCreate (world);
			for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if (random_pos) {
				dBodySetPosition (obj[i].body,
						(dRandReal()-0.5)*HFIELD_WIDTH*0.75,
						(dRandReal()-0.5)*HFIELD_DEPTH*0.75,
						dRandReal() + 2 );
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			}
			else {
				double maxheight = 0;
				for (k=0; k<num; k++) {
					final DVector3C pos = dBodyGetPosition (obj[k].body);
					if (pos.get2() > maxheight) maxheight = pos.get2();
				}
				dBodySetPosition (obj[i].body, 0,maxheight+1,0);
				dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
			}
			dBodySetRotation (obj[i].body,R);
			dBodySetData (obj[i].body,i);

			if (cmd == 'b')
			{
				dMassSetBox (m,DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
			}
			else if (cmd == 'c')
			{
				sides[0] *= 0.5;
				dMassSetCapsule (m,DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
			}
			//<---- Convex Object
			else if (cmd == 'v')
			{
				dMassSetBox (m,DENSITY,0.25,0.25,0.25);
				obj[i].geom[0] = dCreateConvex (space,
						planes,
						planecount,
						points,
						pointcount,
						polygons);
			}
			//----> Convex Object
			else if (cmd == 'y')
			{
				dMassSetCylinder (m,DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
			}
			else if (cmd == 's')
			{
				sides[0] *= 0.5;
				dMassSetSphere (m,DENSITY,sides[0]);
				obj[i].geom[0] = dCreateSphere (space,sides[0]);
			}
			else if (cmd == 'm')
			{
				DTriMeshData new_tmdata = dGeomTriMeshDataCreate();
//				dGeomTriMeshDataBuildSingle(new_tmdata, Vertices[0], 3,// * sizeof(float), 
//						VertexCount, 
//						Indices[0], IndexCount, 3);// * sizeof(dTriIndex));
				dGeomTriMeshDataBuildSingle(new_tmdata, Vertices, 3,// * sizeof(float), 
						VertexCount, 
						Indices, IndexCount, 3);// * sizeof(dTriIndex));

				obj[i].geom[0] = dCreateTriMesh(space, new_tmdata, null, null, null);

				dMassSetTrimesh( m, DENSITY, (DTriMesh)obj[i].geom[0] );
				DVector3 c = m.getC().copy();
				printf("mass at %f %f %f\n", c.get0(), c.get1(), c.get2());
				//dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
				//dMassTranslate(m, -m.c[0], -m.c[1], -m.c[2]);
				c.scale(-1);
				obj[i].geom[0].setPosition(c);
				m.translate(c);
			}
			else if (cmd == 'x') {
				setBody = true;
				// start accumulating masses for the encapsulated geometries
				DMass m2 = dMassCreate();
				dMassSetZero (m);

				double[][] dpos = new double[GPB][3];	// delta-positions for encapsulated geometries
				DMatrix3[] drot = new DMatrix3[GPB];

				// set random delta positions
				for (j=0; j<GPB; j++) {
					for (k=0; k<3; k++) dpos[j][k] = dRandReal()*0.3-0.15;
				}

				for (k=0; k<GPB; k++) {
					if (k==0) {
						double radius = dRandReal()*0.25+0.05;
						obj[i].geom[k] = dCreateSphere (space,radius);
						dMassSetSphere (m2,DENSITY,radius);
					}
					else if (k==1) {
						obj[i].geom[k] = dCreateBox (space,sides[0],sides[1],sides[2]);
						dMassSetBox (m2,DENSITY,sides[0],sides[1],sides[2]);
					}
					else {
						double radius = dRandReal()*0.1+0.05;
						double length = dRandReal()*1.0+0.1;
						obj[i].geom[k] = dCreateCapsule (space,radius,length);
						dMassSetCapsule (m2,DENSITY,3,radius,length);
					}

					drot[k] = new DMatrix3();
					dRFromAxisAndAngle (drot[k],dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
							dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
					dMassRotate (m2,drot[k]);

					dMassTranslate (m2,dpos[k][0],dpos[k][1],dpos[k][2]);

					// add to the total mass
					dMassAdd (m,m2);

				}
				DVector3C m_c = m.getC();
				for (k=0; k<GPB; k++) {
					dGeomSetBody (obj[i].geom[k],obj[i].body);
					dGeomSetOffsetPosition (obj[i].geom[k],
							dpos[k][0]-m_c.get(0),
							dpos[k][1]-m_c.get(1),
							dpos[k][2]-m_c.get(2));
					dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);
				}
				dMassTranslate (m,-m_c.get(0),-m_c.get(1),-m_c.get(2));
				dBodySetMass (obj[i].body,m);

			}
//			else if (cmd == 'x')
//			{
//				DGeom[] g2 = new DGeom[GPB];		// encapsulated geometries
//				DVector3[] dpos = new DVector3[GPB];	// delta-positions for encapsulated geometries
//
//				// start accumulating masses for the encapsulated geometries
//				DMass m2 = OdeHelper.createMass();
//				dMassSetZero (m);
//
//				// set random delta positions
//				for (j=0; j<GPB; j++) {
//					dpos[j] = new DVector3();
//					for (k=0; k<3; k++) dpos[j].set(k, dRandReal()*0.3-0.15);
//				}
//
//				for (k=0; k<GPB; k++) {
//					obj[i].geom[k] = dCreateGeomTransform (space);
//					dGeomTransformSetCleanup ((DGeomTransform)obj[i].geom[k],true);
//					if (k==0) {
//						double radius = dRandReal()*0.25+0.05;
//						g2[k] = dCreateSphere (null,radius);
//						dMassSetSphere (m2,DENSITY,radius);
//					}
//					else if (k==1) {
//						g2[k] = dCreateBox (null,sides[0],sides[1],sides[2]);
//						dMassSetBox (m2,DENSITY,sides[0],sides[1],sides[2]);
//					}
//					else {
//						double radius = dRandReal()*0.1+0.05;
//						double length = dRandReal()*1.0+0.1;
//						g2[k] = dCreateCapsule (null,radius,length);
//						dMassSetCapsule (m2,DENSITY,3,radius,length);
//					}
//					dGeomTransformSetGeom ((DGeomTransform)obj[i].geom[k],g2[k]);
//
//					// set the transformation (adjust the mass too)
////					dGeomSetPosition (g2[k],dpos[k][0],dpos[k][1],dpos[k][2]);
////					dMassTranslate (m2,dpos[k][0],dpos[k][1],dpos[k][2]);
//					g2[k].setPosition (dpos[k]);
//					m2.translate(dpos[k]);
//					DMatrix3 Rtx = new DMatrix3();
//					dRFromAxisAndAngle (Rtx,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
//							dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
//					dGeomSetRotation (g2[k],Rtx);
//					dMassRotate (m2,Rtx);
//
//					// add to the total mass
//					dMassAdd (m,m2);
//				}
//
//				// move all encapsulated objects so that the center of mass is (0,0,0)
//				DVector3 c = m.getC().copy();
//				c.scale(-1);
//				for (k=0; k<2; k++) {
////					dGeomSetPosition (g2[k],
////							dpos[k][0]-m.c[0],
////							dpos[k][1]-m.c[1],
////							dpos[k][2]-m.c[2]);
//				g2[k].setPosition(dpos[k].reAdd(c));
//				}
////				dMassTranslate (m,-m.c[0],-m.c[1],-m.c[2]);
//				m.translate(c);
//			}

			if (!setBody)
				for (k=0; k < GPB; k++)
				{
					if (obj[i].geom[k]!=null) dGeomSetBody (obj[i].geom[k],obj[i].body);
				}

			dBodySetMass (obj[i].body,m);
		}


		//
		// Control Commands
		//

		if (cmd == ' ') {
			selected++;
			if (selected >= num) selected = 0;
			if (selected < 0) selected = 0;
		}
		else if (cmd == 'd' && selected >= 0 && selected < num) {
			dBodyDisable (obj[selected].body);
		}
		else if (cmd == 'e' && selected >= 0 && selected < num) {
			dBodyEnable (obj[selected].body);
		}
		else if (cmd == 'a') {
			show_aabb ^= true;
		}
		else if (cmd == 't') {
			show_contacts ^= true;
		}
		else if (cmd == 'r') {
			random_pos ^= true;
		}
		else if (cmd == '1') {
			write_world = true;
		}
	}


	// draw a geom

	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb)
	{
		int i;

		if (g==null) return;
		if (pos==null) pos = dGeomGetPosition (g);
		if (R==null) R = dGeomGetRotation (g);

		if (g instanceof DBox) {
			DVector3 sides = new DVector3();
			dGeomBoxGetLengths ((DBox)g,sides);
			dsDrawBox (pos,R,sides);
		}
		else if (g instanceof DSphere) {
			dsDrawSphere (pos,R,dGeomSphereGetRadius ((DSphere)g));
		}
		else if (g instanceof DCapsule) {
			//double radius,length;
			//dGeomCapsuleGetParams (g,&radius,&length);
			DCapsule cap = (DCapsule) g; 
			dsDrawCapsule (pos,R,cap.getLength(),cap.getRadius());
		}
		//<---- Convex Object
		else if (g instanceof DConvex)
		{
			//dVector3 sides={0.50,0.50,0.50};
			dsDrawConvex(pos,R,planes,
					planecount,
					points,
					pointcount,
					polygons);
		}
		//----> Convex Object
		else if (g instanceof DCylinder) {
			//double radius,length;
			//dGeomCylinderGetParams (g,&radius,&length);
			DCylinder cyl = (DCylinder) g;
			dsDrawCylinder (pos,R,cyl.getLength(),cyl.getRadius());
		}

		if (show_aabb) {
			// draw the bounding box for this geom
			DAABB aabb = new DAABB();
			dGeomGetAABB (g,aabb);
			DVector3 bbpos = new DVector3();
			for (i=0; i<3; i++) bbpos.set(i, 0.5*(aabb.getMin(i) + aabb.getMax(i)) );
			DVector3 bbsides = new DVector3();
			for (i=0; i<3; i++) bbsides.set(i, aabb.getMax(i) - aabb.getMin(i) );
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha (1,0,0,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}

	}

	// simulation loop

	private void simLoop (boolean pause)
	{
		dsSetColor (0,0,2);

		dSpaceCollide (space,0,nearCallback);

		//if (!pause) dWorldStep (world,0.05);
		if (!pause) dWorldQuickStep (world,0.05);


		if (write_world) {
			FILE f = fopen ("state.dif","wt");
			if (f!=null) {
				dWorldExportDIF (world,f,"X");
				fclose (f);
			}
			write_world = false;
		}

		// remove all contact joints
		dJointGroupEmpty (contactgroup);



		final DVector3C pReal = dGeomGetPosition( gheight );

		final DMatrix3C RReal = dGeomGetRotation( gheight );

		//
		// Draw Heightfield
		//

		// Set ox and oz to zero for DHEIGHTFIELD_CORNER_ORIGIN mode.
		int ox = (int) ( -HFIELD_WIDTH/2 );
		int oz = (int) ( -HFIELD_DEPTH/2 );

		//	for ( int tx = -1; tx < 2; ++tx )
		//	for ( int tz = -1; tz < 2; ++tz )
		{
			dsSetColorAlpha (0.5f,1,0.5f,0.5f);
			dsSetTexture( DS_TEXTURE_NUMBER.DS_WOOD );

			DVector3 a = new DVector3(), b = new DVector3(), c = new DVector3(), d = new DVector3();
			for ( int i = 0; i < HFIELD_WSTEP - 1; ++i )
				for ( int j = 0; j < HFIELD_DSTEP - 1; ++j )
				{

					a.set( ox + ( i ) * HFIELD_WSAMP,
							heightfield_callback( null, i, j ),
							oz + ( j ) * HFIELD_DSAMP);

					b.set( ox + ( i + 1 ) * HFIELD_WSAMP,
							heightfield_callback( null, i + 1, j ),
							oz + ( j ) * HFIELD_DSAMP);

					c.set( ox + ( i ) * HFIELD_WSAMP,
							heightfield_callback( null, i, j + 1 ),
							oz + ( j + 1 ) * HFIELD_DSAMP);

					d.set( ox + ( i + 1 ) * HFIELD_WSAMP,
							heightfield_callback( null, i + 1, j + 1 ),
							oz + ( j + 1 ) * HFIELD_DSAMP);

					dsDrawTriangle( pReal, RReal, a, c, b, true );
					dsDrawTriangle( pReal, RReal, b, c, d, true );
				}
		}





		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++)
		{
			for (int j=0; j < GPB; j++)
			{
				if (i==selected)
				{
					dsSetColor (0,0.7f,1);
				}
				else if (! dBodyIsEnabled (obj[i].body))
				{
					dsSetColor (1,0.8f,0);
				}
				else 
				{
					dsSetColor (1,1,0);
				}

				if ( obj[i].geom[j]!=null && obj[i].geom[j] instanceof DTriMesh )
				{
					//dTriIndex* Indices = (dTriIndex*)::Indices;
					int[] Indices = BunnyGeom.Indices;

					// assume all trimeshes are drawn as bunnies
					final DVector3C Pos = dGeomGetPosition(obj[i].geom[j]);
					final DMatrix3C Rot = dGeomGetRotation(obj[i].geom[j]);

					for (int ii = 0; ii < IndexCount / 3; ii++)
					{
						final float[] v = { // explicit conversion from float to dReal
								Vertices[Indices[ii * 3+0] * 3 + 0],
								Vertices[Indices[ii * 3+0] * 3 + 1],
								Vertices[Indices[ii * 3+0] * 3 + 2],
								Vertices[Indices[ii * 3+1] * 3 + 0],
								Vertices[Indices[ii * 3+1] * 3 + 1],
								Vertices[Indices[ii * 3+1] * 3 + 2],
								Vertices[Indices[ii * 3+2] * 3 + 0],
								Vertices[Indices[ii * 3+2] * 3 + 1],
								Vertices[Indices[ii * 3+2] * 3 + 2]
						};
						//dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
						dsDrawTriangle(Pos, Rot, v, 0, 3, 6, true);
					}

					// tell the tri-tri collider the current transform of the trimesh --
					// this is fairly important for good results.

					// Fill in the (4x4) matrix.
//					double* p_matrix = obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 );
//
//					p_matrix[ 0 ] = Rot[ 0 ];	p_matrix[ 1 ] = Rot[ 1 ];	p_matrix[ 2 ] = Rot[ 2 ];	p_matrix[ 3 ] = 0;
//					p_matrix[ 4 ] = Rot[ 4 ];	p_matrix[ 5 ] = Rot[ 5 ];	p_matrix[ 6 ] = Rot[ 6 ];	p_matrix[ 7 ] = 0;
//					p_matrix[ 8 ] = Rot[ 8 ];	p_matrix[ 9 ] = Rot[ 9 ];	p_matrix[10 ] = Rot[10 ];	p_matrix[11 ] = 0;
//					p_matrix[12 ] = Pos[ 0 ];	p_matrix[13 ] = Pos[ 1 ];	p_matrix[14 ] = Pos[ 2 ];	p_matrix[15 ] = 1;
//
//					// Flip to other matrix.
//					obj[i].last_matrix_index = !obj[i].last_matrix_index;
//
//					// Apply the 'other' matrix which is the oldest.
//					dGeomTriMeshSetLastTransform( obj[i].geom[j], 
//							*(dMatrix4*)( obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 ) ) );
					double[] p_matrixA = obj[i].matrix_dblbuff;
					int of = ( obj[i].last_matrix_index * 16 );

					p_matrixA[ of+0 ] = Rot.get00();	p_matrixA[ of+1 ] = Rot.get01();	p_matrixA[ of+2 ] = Rot.get02();	p_matrixA[ of+3 ] = 0;
					p_matrixA[ of+4 ] = Rot.get10();	p_matrixA[ of+5 ] = Rot.get11();	p_matrixA[ of+6 ] = Rot.get12();	p_matrixA[ of+7 ] = 0;
					p_matrixA[ of+8 ] = Rot.get20();	p_matrixA[ of+9 ] = Rot.get21();	p_matrixA[of+10 ] = Rot.get22();	p_matrixA[of+11 ] = 0;
					p_matrixA[of+12 ] = Pos.get0();	p_matrixA[of+13 ] = Pos.get1();	p_matrixA[of+14 ] = Pos.get2();	p_matrixA[of+15 ] = 1;
					
										// Flip to other matrix.
					//obj[i].last_matrix_index = !obj[i].last_matrix_index;
					if (obj[i].last_matrix_index != 0)
						obj[i].last_matrix_index = 0;
					else
						obj[i].last_matrix_index = 1;
					
					// Apply the 'other' matrix which is the oldest.
					// -> not implemented for GIMPACT
//					dGeomTriMeshSetLastTransform( obj[i].geom[j],
//							new DoubleArray( obj[i].matrix_dblbuff, ( obj[i].last_matrix_index * 16 ) ) );
				}
				else
				{
					drawGeom (obj[i].geom[j],null,null,show_aabb);
				}
			}
		}

		if ( show_aabb )
		{
			// draw the bounding box for this geom
			DAABB aabb = new DAABB();
			dGeomGetAABB (gheight,aabb);
			DVector3 bbpos = new DVector3();
			for (int i=0; i<3; i++) bbpos.set(i, 0.5*(aabb.getMin(i) + aabb.getMax(i)) );
			DVector3 bbsides = new DVector3();
			for (int i=0; i<3; i++) bbsides.set(i, aabb.getMax(i) - aabb.getMin(i) );
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha (1,0,0,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}


	public static void main(String[] args) {
		new DemoHeightfield().demo(args);
	}

	private void demo(String[] args) {
		printf("ODE configuration: %s\n", OdeHelper.getConfiguration());

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = this;
		//fn.version = DS_VERSION;
		//	fn.start = &start;
		//	fn.step = &simLoop;
		//	fn.command = &command;
		//	fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		world = dWorldCreate();
		space = dHashSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world,0,0,-0.05);
		dWorldSetCFM (world,1e-5);
		dWorldSetAutoDisableFlag (world,true);
		dWorldSetContactMaxCorrectingVel (world,0.1);
		dWorldSetContactSurfaceLayer (world,0.001);
		//memset (obj,0,sizeof(obj));
		for (int i = 0; i < obj.length; i++) {
			obj[i] = new MyObject();
		}

		if (true) {//#if 1

			dWorldSetAutoDisableAverageSamplesCount( world, 1 );

		} //#endif

		// base plane to catch overspill
		dCreatePlane( space, 0, 0, 1, 0 );


		// our heightfield floor

		DHeightfieldData heightid = dGeomHeightfieldDataCreate();

		// Create an finite heightfield.
		dGeomHeightfieldDataBuildCallback( heightid, null, heightfield_callback,
				HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
				( 1.0 ), ( 0.0 ), ( 0.0 ), false );

		// Give some very bounds which, while conservative,
		// makes AABB computation more accurate than +/-INF.
		dGeomHeightfieldDataSetBounds( heightid, ( -4.0 ), ( +6.0 ) );

		gheight = dCreateHeightfield( space, heightid, true );

		DVector3 pos = new DVector3();

		// Rotate so Z is up, not Y (which is the default orientation)
		DMatrix3 R = new DMatrix3();
		R.setIdentity();
		dRFromAxisAndAngle( R, 1, 0, 0, DEGTORAD * 90 );

		// Place it.
		dGeomSetRotation( gheight, R );
		dGeomSetPosition( gheight, pos.get0(), pos.get1(), pos.get2() );

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);

		// destroy heightfield data, because _we_ own it not ODE
		dGeomHeightfieldDataDestroy( heightid );

		dCloseODE();
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
		// Nothing
	}
}