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

import static org.ode4j.demo.IcosahedronGeom.Sphere_planecount;
import static org.ode4j.demo.IcosahedronGeom.Sphere_planes;
import static org.ode4j.demo.IcosahedronGeom.Sphere_pointcount;
import static org.ode4j.demo.IcosahedronGeom.Sphere_points;
import static org.ode4j.demo.IcosahedronGeom.Sphere_polygons;
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
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DMassC;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.OdeHelper;

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
	private static final int NUM = 10;			// max number of objects
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
	private static boolean show_body = false;

	private class MyFeedback {
		DJoint.DJointFeedback fb;
		boolean first;
	}
	private static boolean doFeedback=false;
	private static MyFeedback[] feedbacks=new MyFeedback[MAX_FEEDBACKNUM];
	private static int fbnum=0;

	
	private DNearCallback nearCallback = new DNearCallback(){
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}};
	
	
	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	//static void nearCallback (void *data, dGeom o1, dGeom o2)
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i;
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && OdeHelper.areConnectedExcluding (b1,b2,DContactJoint.class)) {
			return;
		}

		//dContact[] contact=new dContact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
		for (DContact contact: contacts) {
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		//	if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
		//			sizeof(dContact))) {
		int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());//, sizeof(dContact));
		if (numc!=0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			final DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (i=0; i<numc; i++) {
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contacts.get(i));
				c.attach (b1,b2);
				if (show_contacts) {
	                dsSetColor(0,0,1);
	                dsDrawBox (contacts.get(i).geom.pos,RI,ss);
				}

				if (doFeedback && (b1==obj[selected].body || b2==obj[selected].body))
				{
					if (fbnum<MAX_FEEDBACKNUM)
					{
						feedbacks[fbnum].first = b1==obj[selected].body;
						c.setFeedback (feedbacks[fbnum++].fb);
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
		dsSetViewpoint (xyz,hpr);
		System.out.println("To drop another object, press:");
		System.out.println("   b for box.");
		System.out.println("   s for sphere.");
		System.out.println("   c for capsule.");
		System.out.println("   y for cylinder.");
		System.out.println("   v for a convex object.");
		System.out.println("   x for a composite object.");
		System.out.println("To select an object, press space.");
		System.out.println("To disable the selected object, press d.");
		System.out.println("To enable the selected object, press e.");
		System.out.println("To dump transformation data for the selected object, press p.");
		System.out.println("To toggle showing the geom AABBs, press a.");
		System.out.println("To toggle showing the contact points, press t.");
		System.out.println("To toggle dropping from random position/orientation, press r.");
		System.out.println("To save the current state to 'state.dif', press 1.");
		System.out.println("To show joint feedbacks of selected object, press f.");
	}


	// called when a key pressed
	@SuppressWarnings("unused")
	@Override
	public void command (char cmd)
	{
		int i;//size_t i;
		int j,k;
		double[] sides= new double[3];
		DMass m = OdeHelper.createMass();
		boolean setBody = false;

		cmd = Character.toLowerCase (cmd);
		if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y' || cmd == 'v') {
			if (num < NUM) {
				i = num;
				num++;
			} else {
	            // recycle existing object
				i = nextobj++;
	            nextobj %= num; // wrap-around if needed

				// destroy the body and geoms for slot i
				obj[i].body.destroy ();
				obj[i].body = null;
				
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) obj[i].geom[k].destroy();
                    obj[i].geom[k] = null;
				}
				//memset (obj[i],0);//,sizeof(obj[i]));
				obj[i] = new MyObject();
			}

			obj[i].body = OdeHelper.createBody(world);
			for (k=0; k<3; k++) {
				sides[k] = dRandReal()*0.5+0.1;
			}

			DMatrix3 R = new DMatrix3();
			if (random_pos) {
				obj[i].body.setPosition(
						dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2);
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			} else {
	            // higher than highest body position
				double maxheight = 0;
				for (k=0; k<num; k++) {
					final DVector3C pos = obj[k].body.getPosition();
					if (pos.get2() > maxheight) 
						maxheight = pos.get2();
				}
				obj[i].body.setPosition( 0,0,maxheight+1);
				R.setIdentity();
				//dRFromAxisAndAngle (R,0,0,1,/*dRandReal()*10.0-5.0*/0);
			}
			obj[i].body.setRotation(R);

			if (cmd == 'b') {
				m.setBox(DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = OdeHelper.createBox(space,sides[0],sides[1],sides[2]);
			} else if (cmd == 'c') {
				sides[0] *= 0.5;
				m.setCapsule(DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCapsule (space,sides[0],sides[1]);
			} else if (cmd == 'v') {
				m.setBox(DENSITY,0.25,0.25,0.25);
				if (false) {//
					obj[i].geom[0] = OdeHelper.createConvex (space,
							planes,
							planecount,
							points,
							pointcount,
							polygons);
				} else { //#else
					obj[i].geom[0] = OdeHelper.createConvex (space,
							Sphere_planes,
							Sphere_planecount,
							Sphere_points,
							Sphere_pointcount,
							Sphere_polygons);
				} //#endif
			} else if (cmd == 'y') {
				m.setCylinder(DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCylinder (space,sides[0],sides[1]);
			} else if (cmd == 's') {
				sides[0] *= 0.5;
				m.setSphere(DENSITY,sides[0]);
				obj[i].geom[0] = OdeHelper.createSphere (space,sides[0]);
			} else if (cmd == 'x') {
				setBody = true;
				// start accumulating masses for the encapsulated geometries
				DMass m2 = OdeHelper.createMass();
				m.setZero();

				DVector3[] dpos = DVector3.newArray(GPB);	// delta-positions for encapsulated geometries
				DMatrix3[] drot = DMatrix3.newArray(GPB);

				// set random delta positions
				for (j=0; j<GPB; j++) {
					for (k=0; k<3; k++) {
						dpos[j].set(k, dRandReal()*0.3-0.15);
					}
				}

				for (k=0; k<GPB; k++) {
					if (k==0) {
						double radius = dRandReal()*0.25+0.05;
						obj[i].geom[k] = OdeHelper.createSphere (space,radius);
						m2.setSphere(DENSITY,radius);
					} else if (k==1) {
						obj[i].geom[k] = OdeHelper.createBox(space,sides[0],sides[1],sides[2]);
						m2.setBox(DENSITY,sides[0],sides[1],sides[2]);
					} else {
						double radius = dRandReal()*0.1+0.05;
						double length = dRandReal()*1.0+0.1;
						obj[i].geom[k] = OdeHelper.createCapsule (space,radius,length);
						m2.setCapsule(DENSITY,3,radius,length);
					}

					dRFromAxisAndAngle (drot[k],dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
							dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
					m2.rotate(drot[k]);

					m2.translate(dpos[k]);

					// add to the total mass
					m.add(m2);

				}
				DVector3C negC = m.getC().copy().scale(-1);
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
						obj[i].geom[k].setBody(obj[i].body);
					}
				}

				obj[i].body.setMass(m);
			}
		}

		if (cmd == ' ') {
			selected++;
			if (selected >= num) 
				selected = 0;
			if (selected == -1) 
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
		} else if (cmd == 'p'&& selected >= 0) {
			final DVector3C pos = obj[selected].geom[0].getPosition();
			final DMatrix3C rot = obj[selected].geom[0].getRotation();
			System.out.println("POSITION:\n\t" + pos);
			System.out.println("ROTATION:\n\t" + rot);
		} else if (cmd == 'f' && selected >= 0 && selected < num) {
			if (obj[selected].body.isEnabled())
				doFeedback = true;
		}
	}


	// draw a geom

	//void drawGeom (dGeom g, final double *pos, final double *R, boolean show_aabb)
	@SuppressWarnings("unused")
	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb)
	{
		if (g==null) 
			return;
		if (pos==null)
			pos = g.getPosition();
		if (R==null) 
			R = g.getRotation();

		if (g instanceof DBox) {
			DVector3C sides = ((DBox)g).getLengths();
			dsDrawBox (pos,R,sides);
		} else if (g instanceof DSphere) {
			dsDrawSphere (pos,R,((DSphere)g).getRadius());
		} else if (g instanceof DCapsule) {
			DCapsule cap = (DCapsule) g;
			dsDrawCapsule (pos,R,cap.getLength(),cap.getRadius());
		} else if (g instanceof DConvex) {
			//DVector3 sides={0.50,0.50,0.50};
			if (false) {
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
		} else if (g instanceof DCylinder) {
			DCylinder cyl = (DCylinder) g;
			dsDrawCylinder (pos,R,cyl.getLength(),cyl.getRadius());
		} 
		
		if (show_body) {
			DBody body = g.getBody();
			if (body != null) {
				final DVector3C bodypos = body.getPosition(); 
				final DMatrix3C bodyr = body.getRotation(); 
				DVector3 bodySides = new DVector3( 0.1, 0.1, 0.1 );
				dsSetColorAlpha(0,1,0,1);
				dsDrawBox(bodypos,bodyr,bodySides); 
			}
		}
		if (show_aabb) {
			// draw the bounding box for this geom
			DAABBC aabb = g.getAABB();
			DVector3 bbpos = new DVector3();
			//for (i=0; i<3; i++) bbpos.set(i, 0.5*(aabb.get(i*2) + aabb.get(i*2+1)));
			bbpos.set( 0.5* (aabb.getMin0()+aabb.getMax0()),
					0.5* (aabb.getMin1()+aabb.getMax1()),
					0.5* (aabb.getMin2()+aabb.getMax2()));
			DVector3 bbsides = aabb.getLengths();
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha (1,0,0,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}


	// simulation loop

	@Override
	public void step (boolean pause)
	{
		//  dSpaceCollide (space,null,nearCallback);
		space.collide(null,nearCallback);
		if (!pause) 
			world.quickStep (0.02);

//		if (write_world) {
//			File f = new File("state.dif");
//			OdeHelper.worldExportDIF(world, f, "X");
//			FILE f = fopen ("state.dif","wt");
//			if (f!=null) {
//				OdeHelper.dWorldExportDIF (world,f,"X");
//				fclose (f);
//			}
//			write_world = false;
//		}


		if (doFeedback)	{
			if (fbnum>MAX_FEEDBACKNUM) {
				System.out.println("joint feedback buffer overflow!");
			} else {
				DVector3 sum = new DVector3();
				System.out.println();
				for (int i=0; i<fbnum; i++) {
					DVector3 f = feedbacks[i].first?feedbacks[i].fb.f1:feedbacks[i].fb.f2;
					System.out.println(f);
					sum.add(f);
				}
				System.out.println("Sum: " + sum);
				DMassC m = obj[selected].body.getMass();
				System.out.println("Object G=" + GRAVITY*m.getMass());
			}
			doFeedback = false;
			fbnum = 0;
		}

		// remove all contact joints
		contactgroup.empty();

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++) {
			for (int j=0; j < GPB; j++) {
				if (i==selected) {
					dsSetColor (0,0.7f,1);
				} else if (! obj[i].body.isEnabled()) {
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
		new DemoBoxstack().demo(args);
	}
	
	private void demo(String[] args) {
		OdeConfig.setLibCCDEndabled(true);
		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace(null);
		contactgroup = OdeHelper.createJointGroup();
		world.setGravity(0,0,-GRAVITY);
		world.setCFM (1e-5);
		world.setAutoDisableFlag (true);

		if (true) {//#if 1
			world.setAutoDisableAverageSamplesCount( 10 );
		}//#endif

		world.setLinearDamping(0.00001);
		world.setAngularDamping(0.005);
		world.setMaxAngularSpeed(200);

		world.setContactMaxCorrectingVel (0.1);
		world.setContactSurfaceLayer (0.001);
		OdeHelper.createPlane (space,0,0,1,0);
		//memset (obj,0);//,sizeof(obj));
		for (int i = 0; i < obj.length; i++) obj[i] = new MyObject();

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