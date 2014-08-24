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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyIsEnabled;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCapsule;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateConvex;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCylinder;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCapsuleGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCylinderGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetAABB;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSphereGetRadius;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppExportDIF.dWorldExportDIF;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetFeedback;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdd;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassRotate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCapsule;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCylinder;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetZero;
import static org.ode4j.cpp.internal.ApiCppMass.dMassTranslate;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppOther.dAreConnectedExcluding;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetAngularDamping;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetAutoDisableAverageSamplesCount;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetAutoDisableFlag;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetCFM;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetContactMaxCorrectingVel;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetContactSurfaceLayer;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetLinearDamping;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetMaxAngularSpeed;
import static org.ode4j.democpp.IcosahedronGeom.Sphere_planecount;
import static org.ode4j.democpp.IcosahedronGeom.Sphere_planes;
import static org.ode4j.democpp.IcosahedronGeom.Sphere_pointcount;
import static org.ode4j.democpp.IcosahedronGeom.Sphere_points;
import static org.ode4j.democpp.IcosahedronGeom.Sphere_polygons;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsDrawConvex;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
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
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.cpp4j.FILE;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

class DemoBoxstack extends dsFunctions {

	//<---- Convex Object
	private double planes[]= // planes for a cube, these should coincide with the face array
	{
			1.0f ,0.0f ,0.0f ,0.25f,
			0.0f ,1.0f ,0.0f ,0.25f,
			0.0f ,0.0f ,1.0f ,0.25f,
			-1.0f,0.0f ,0.0f ,0.25f,
			0.0f ,-1.0f,0.0f ,0.25f,
			0.0f ,0.0f ,-1.0f,0.25f
			/*
    1.0f ,0.0f ,0.0f ,2.0f,
    0.0f ,1.0f ,0.0f ,1.0f,
    0.0f ,0.0f ,1.0f ,1.0f,
    0.0f ,0.0f ,-1.0f,1.0f,
    0.0f ,-1.0f,0.0f ,1.0f,
    -1.0f,0.0f ,0.0f ,0.0f
			 */
	};
	//final unsigned 
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
	//final unsigned 
	private final int pointcount=8;
	//unsigned 
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
	private static final double DENSITY =(5.0);		// density of all objects
	private static final int GPB = 3;			// maximum number of geometries per body
	private static final int MAX_CONTACTS = 8;          // maximum number of contact points per body
	private static final int MAX_FEEDBACKNUM = 20;
	private static final double GRAVITY = 0.5f;

	// dynamics and collision objects

	private static class MyObject {
		DBody body;			// the body
		DGeom[] geom = new DGeom[GPB];		// geometries representing this body
	}

	private static int num=0;		// number of objects in simulation
	private static int nextobj=0;		// next object to recycle if num==NUM
	private static DWorld world;
	private static DSpace space;
	private static MyObject[] obj=new MyObject[NUM];
	private static DJointGroup contactgroup;
	private static int selected = -1;	// selected object
	private static boolean show_aabb = false;	// show geom AABBs?
	private static boolean show_contacts = false;	// show contact points?
	private static boolean random_pos = true;	// drop objects from random position?
	private static boolean write_world = false;
	private static boolean show_body = false;

	private class MyFeedback {
		DJoint.DJointFeedback fb;
		boolean first;
	}
	private static boolean doFeedback=false;
	private static MyFeedback[] feedbacks=new MyFeedback[MAX_FEEDBACKNUM];
	private static int fbnum=0;

	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	//static void nearCallback (void *data, dGeom o1, dGeom o2)
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i;
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = dGeomGetBody(o1);
		DBody b2 = dGeomGetBody(o2);
		if (b1!=null && b2!=null && dAreConnectedExcluding (b1,b2,DContactJoint.class)) 
			return;

		//dContact[] contact=new dContact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
		for (i=0; i<MAX_CONTACTS; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		//	if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
		//			sizeof(dContact))) {
		int numc = dCollide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());//, sizeof(dContact));
		if (numc!=0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			final DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (i=0; i<numc; i++) {
				DJoint c = dJointCreateContact (world,contactgroup,contacts.get(i));
				dJointAttach (c,b1,b2);
				if (show_contacts) {
	                dsSetColor(0,0,1);
					dsDrawBox (contacts.get(i).geom.pos,RI,ss);
				}

				if (doFeedback && (b1==obj[selected].body || b2==obj[selected].body))
				{
					if (fbnum<MAX_FEEDBACKNUM)
					{
						feedbacks[fbnum].first = b1==obj[selected].body;
						dJointSetFeedback (c,feedbacks[fbnum++].fb);
					}
					else fbnum++;
				}
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

		//  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
		//  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
		printf ("To drop another object, press:\n");
		printf ("   b for box.\n");
		printf ("   s for sphere.\n");
		printf ("   c for capsule.\n");
		printf ("   y for cylinder.\n");
		printf ("   v for a convex object.\n");
		printf ("   x for a composite object.\n");
		printf ("To select an object, press space.\n");
		printf ("To disable the selected object, press d.\n");
		printf ("To enable the selected object, press e.\n");
		printf ("To dump transformation data for the selected object, press p.\n");
		printf ("To toggle showing the geom AABBs, press a.\n");
		printf ("To toggle showing the contact points, press t.\n");
		printf ("To toggle dropping from random position/orientation, press r.\n");
		printf ("To save the current state to 'state.dif', press 1.\n");
		printf ("To show joint feedbacks of selected object, press f.\n");
	}


	private char locase(char c)
	{
		//  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
		//  else return c;
		return Character.toLowerCase(c);
	}


	// called when a key pressed
	@Override
	@SuppressWarnings("unused")
	public void command (char cmd)
	{
		int i;//size_t i;
		int j,k;
		double[] sides= new double[3];
		DMass m = dMassCreate();
		boolean setBody = false;

		cmd = locase (cmd);
		if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y' || cmd == 'v') {
			if (num < NUM) {
				i = num;
				num++;
			} else {
	            // recycle existing object
	            i = nextobj++;
	            nextobj %= num; // wrap-around if needed

				// destroy the body and geoms for slot i
				dBodyDestroy (obj[i].body);
	            obj[i].body = null;

	            for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) {
						dGeomDestroy (obj[i].geom[k]);
	                    obj[i].geom[k] = null;
					}
				}
				//memset (obj[i],0);//,sizeof(obj[i]));
				obj[i] = new MyObject();
			}

			obj[i].body = dBodyCreate (world);
			for (k=0; k<3; k++) 
				sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if (random_pos)	{
				dBodySetPosition (obj[i].body,
						dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2);
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			} else {
	            // higher than highest body position
				double maxheight = 0;
				for (k=0; k<num; k++) {
					final DVector3C pos = dBodyGetPosition (obj[k].body);
					if (pos.get(2) > maxheight) 
						maxheight = pos.get(2);
				}
				dBodySetPosition (obj[i].body, 0,0,maxheight+1);
				R.setIdentity();
				//dRFromAxisAndAngle (R,0,0,1,/*dRandReal()*10.0-5.0*/0);
			}
			dBodySetRotation (obj[i].body,R);

			if (cmd == 'b') {
				dMassSetBox (m,DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
				
			} else if (cmd == 'c') {
				sides[0] *= 0.5;
				dMassSetCapsule (m,DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
				
			} else if (cmd == 'v') {
				dMassSetBox (m,DENSITY,0.25,0.25,0.25);
				if (false) { //#if 0
					obj[i].geom[0] = dCreateConvex (space,
							planes,
							planecount,
							points,
							pointcount,
							polygons);
				} else { //#else
					obj[i].geom[0] = dCreateConvex (space,
									Sphere_planes,
									Sphere_planecount,
									Sphere_points,
									Sphere_pointcount,
									Sphere_polygons);
				} //#endif
				
			} else if (cmd == 'y') {
				dMassSetCylinder (m,DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
				
			} else if (cmd == 's') {
				sides[0] *= 0.5;
				dMassSetSphere (m,DENSITY,sides[0]);
				obj[i].geom[0] = dCreateSphere (space,sides[0]);
				
			} else if (cmd == 'x') {
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
					} else if (k==1) {
						obj[i].geom[k] = dCreateBox (space,sides[0],sides[1],sides[2]);
						dMassSetBox (m2,DENSITY,sides[0],sides[1],sides[2]);
					} else {
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
	
			if (!setBody) {// avoid calling for composite geometries
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) 
						dGeomSetBody (obj[i].geom[k],obj[i].body);
				}

				dBodySetMass (obj[i].body,m);
			}
		}

		if (cmd == ' ') {
			selected++;
			if (selected >= num) 
				selected = 0;
			if (selected < 0) 
				selected = 0;
			
		} else if (cmd == 'd' && selected >= 0 && selected < num) {
			dBodyDisable (obj[selected].body);
			
		} else if (cmd == 'e' && selected >= 0 && selected < num) {
			dBodyEnable (obj[selected].body);
			
		} else if (cmd == 'a') {
			show_aabb = !show_aabb;
			
		} else if (cmd == 't') {
			show_contacts = !show_contacts;
			
		} else if (cmd == 'r') {
			random_pos =!random_pos;
			
		} else if (cmd == '1') {
			write_world = true;
			
		} else if (cmd == 'p'&& selected >= 0) {
			final DVector3C pos = dGeomGetPosition(obj[selected].geom[0]);
			final DMatrix3C rot = dGeomGetRotation(obj[selected].geom[0]);
			printf("POSITION:\n\t[%f,%f,%f]\n\n",pos.get(0),pos.get(1),pos.get(2));
			printf("ROTATION:\n\t[%f,%f,%f,%f]\n\t[%f,%f,%f,%f]\n\t[%f,%f,%f,%f]\n\n",
					rot.get00(),rot.get01(),rot.get02(),//rot.get(3),
					rot.get10(),rot.get11(),rot.get12(),//rot.get(7),
					rot.get20(),rot.get21(),rot.get22());//,rot.get(11));
			
		} else if (cmd == 'f' && selected >= 0 && selected < num) {
			if (dBodyIsEnabled(obj[selected].body))
				doFeedback = true;
		}
	}


	// draw a geom

	//void drawGeom (dGeom g, final double *pos, final double *R, boolean show_aabb)
	@SuppressWarnings("unused")
	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb)
	{
		int i;

		if (g==null) 
			return;
		if (pos==null) 
			pos = dGeomGetPosition (g);
		if (R==null) 
			R = dGeomGetRotation (g);

		if (g instanceof DBox) {
			DVector3 sides = new DVector3();
			dGeomBoxGetLengths ((DBox)g,sides);
			dsDrawBox (pos,R,sides);
			
		} else if (g instanceof DSphere) {
			dsDrawSphere (pos,R,dGeomSphereGetRadius ((DSphere)g));
			
		} else if (g instanceof DCapsule) {
			RefDouble radius = new RefDouble(0),length=new RefDouble(0);
			dGeomCapsuleGetParams ((DCapsule)g,radius,length);
			dsDrawCapsule (pos,R,length.getF(),radius.getF());
			
		} else if (g instanceof DConvex) {
			//dVector3 sides={0.50,0.50,0.50};
			if (false) {//if#
				dsDrawConvex(pos,R,planes,
						planecount,
						points,
						pointcount,
						polygons);
			} else { //#else
				dsDrawConvex(pos,R,
						Sphere_planes,
						Sphere_planecount,
						Sphere_points,
						Sphere_pointcount,
						Sphere_polygons);
			} //#endif
		}
		//----> Convex Object
		else if (g instanceof DCylinder) {
			RefDouble radius = new RefDouble(0),length=new RefDouble(0);
			dGeomCylinderGetParams ((DCylinder)g,radius,length);
			dsDrawCylinder (pos,R,length.getF(),radius.getF());
		}

		if (show_body) {
			DBody body = dGeomGetBody(g);
			if (body!=null) {
				final DVector3C bodypos = dBodyGetPosition (body); 
				final DMatrix3C bodyr = dBodyGetRotation (body); 
				DVector3 bodySides = new DVector3( 0.1, 0.1, 0.1 );
				dsSetColorAlpha(0,1,0,1);
				dsDrawBox(bodypos,bodyr,bodySides); 
			}
		}
		
		if (show_aabb) {
			// draw the bounding box for this geom
			DAABB aabb=new DAABB();
			dGeomGetAABB (g,aabb);
			DVector3 bbpos = new DVector3();
			for (i=0; i<3; i++) 
				bbpos.set(i, 0.5*(aabb.getMin(i) + aabb.getMax(i)));
			DVector3 bbsides = new DVector3();
			for (i=0; i<3; i++) 
				bbsides.set(i, aabb.getMax(i) - aabb.getMin(i));
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha (1,0,0,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}


	// simulation loop

	private void simLoop (boolean pause)
	{
		//  dSpaceCollide (space,null,nearCallback);
		dSpaceCollide (space,null,new DNearCallback(){
			@Override
			public void call(Object data, DGeom o1, DGeom o2) {
				nearCallback(data, o1, o2);
			}});
		if (!pause) 
			dWorldQuickStep (world,0.02);

		if (write_world) {
			FILE f = fopen ("state.dif","wt");
			if (f!=null) {
				dWorldExportDIF (world,f,"X");
				fclose (f);
			}
			write_world = false;
		}


		if (doFeedback) {
			if (fbnum>MAX_FEEDBACKNUM) {
				printf("joint feedback buffer overflow!\n");
			} else {
				DVector3 sum = new DVector3(0, 0, 0);
				printf("\n");
				for (int i=0; i<fbnum; i++) {
					DVector3 f = feedbacks[i].first?feedbacks[i].fb.f1:feedbacks[i].fb.f2;
					printf("%f %f %f\n", f.get(0), f.get(0), f.get(0));
					//        sum[0] += f[0];
					//        sum[1] += f[1];
					//        sum[2] += f[2];
					sum.add(f);
				}
				printf("Sum: %f %f %f\n", sum.get(0), sum.get(1), sum.get(2));
				DMass m = dMassCreate();
				dBodyGetMass(obj[selected].body, m);
				printf("Object G=%f\n", GRAVITY*m.getMass());
			}
			doFeedback = false;
			fbnum = 0;
		}

		// remove all contact joints
		dJointGroupEmpty (contactgroup);

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++) {
			for (int j=0; j < GPB; j++) {
				if (i==selected) {
					dsSetColor (0,0.7f,1);
				} else if (! dBodyIsEnabled (obj[i].body)) {
					dsSetColor (1,0.8f,0);
				} else {
					dsSetColor (1,1,0);
				}
				drawGeom (obj[i].geom[j],null,null,show_aabb);
			}
		}
	}


	public static void main (String[] args)
	{
		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoBoxstack();
		//fn.version = DrawStuff.DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = &command;
		//  fn.stop = 0;
		//fn.path_to_textures = DrawStuff.DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		world = dWorldCreate();
		space = dHashSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world,0,0,-GRAVITY);
		dWorldSetCFM (world,1e-5);
		dWorldSetAutoDisableFlag (world,true);

		if (true) {//#if 1
			dWorldSetAutoDisableAverageSamplesCount( world, 10 );
		}//#endif

		dWorldSetLinearDamping(world, 0.00001);
		dWorldSetAngularDamping(world, 0.005);
		dWorldSetMaxAngularSpeed(world, 200);

		dWorldSetContactMaxCorrectingVel (world,0.1);
		dWorldSetContactSurfaceLayer (world,0.001);
		dCreatePlane (space,0,0,1,0);
		//memset (obj,0);//,sizeof(obj));
		for (int i = 0; i < obj.length; i++) obj[i] = new MyObject();

		// run simulation
		dsSimulationLoop (args,640,480,new DemoBoxstack());

		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
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