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
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCylinderGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomIsSpace;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide2;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceAdd;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateFeedback;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateHinge;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateSlider;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetBody;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetFeedback;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHingeAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHingeAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHingeParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetSliderAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetSliderParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetQuickStepNumIterations;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsElapsedTime;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.internal.cpp4j.Cmath.ceilf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stderr;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


/**
 *  Test for breaking joints, by Bram Stolk
 */
class DemoFeedback extends dsFunctions {

	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;

	private static final int   STACKCNT=10;	// nr of weights on bridge
	private static final int   SEGMCNT=16;	// nr of segments in bridge
	private static final float[] SEGMDIM = { 0.9f, 4, 0.1f };

	//private static DGeom  groundgeom;
	private static DBody[]  segbodies = new DBody[SEGMCNT];
	private static DGeom[]  seggeoms = new DGeom[SEGMCNT];
	private static DBody[]  stackbodies = new DBody[STACKCNT];
	private static DGeom[]  stackgeoms = new DGeom[STACKCNT];
	private static DHingeJoint[] hinges = new DHingeJoint[SEGMCNT-1];
	private static DSliderJoint[] sliders = new DSliderJoint[2];
	private static DJoint.DJointFeedback[] jfeedbacks = new DJoint.DJointFeedback[SEGMCNT-1];
	private static double[] colours = new double[SEGMCNT];
	private static int[] stress = new int[SEGMCNT-1];

	private static DJointGroup contactgroup;

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}};
	
	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		assert(o1!=null);
		assert(o2!=null);

		if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
		{
			fprintf(stderr,"testing space %s %s\n", o1,o2);
			// colliding a space with something
			dSpaceCollide2(o1,o2,data,nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		final int N = 32;
		//dContact contact[N];
		DContactBuffer contacts = new DContactBuffer(N);
		int n = dCollide (o1,o2,N,contacts.getGeomBuffer());//,sizeof(dContact));
		if (n > 0) 
		{
			for (int i=0; i<n; i++) 
			{
				DContact contact = contacts.get(i);
				contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact.surface.mu = 100.0;
				contact.surface.soft_erp = 0.96;
				contact.surface.soft_cfm = 0.02;
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c,
						dGeomGetBody(contact.geom.g1),
						dGeomGetBody(contact.geom.g2));
			}
		}
	}


	private static float[] xyz = { -6, 8, 6};
	private static float[] hpr = { -65.0f, -27.0f, 0.0f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = { -6, 8, 6};
		//  static float hpr[3] = { -65.0f, -27.0f, 0.0f};
		dsSetViewpoint (xyz,hpr);
	}



	// called when a key pressed

	@Override
	public void command (char cmd)
	{
	}



	private void drawGeom (DGeom g)
	{
		final DVector3C pos = dGeomGetPosition(g);
		final DMatrix3C R   = dGeomGetRotation(g);

		//int type = dGeomGetClass (g);
		if (g instanceof DBox)
		{
			DVector3 sides = new DVector3();
			dGeomBoxGetLengths ((DBox)g, sides);
			dsDrawBox (pos,R,sides);
		}
		if (g instanceof DCylinder)
		{
			RefDouble r = new RefDouble(0), l = new RefDouble(0);
			dGeomCylinderGetParams((DCylinder)g, r, l);
			dsDrawCylinder (pos, R, l.getF(), r.getF());
		}
	}


	private static void inspectJoints()
	{
		final double forcelimit = 2000.0;
		int i;
		for (i=0; i<SEGMCNT-1; i++)
		{
			if (dJointGetBody(hinges[i], 0)!=null)
			{
				// This joint has not snapped already... inspect it.
				double l0 = jfeedbacks[i].f1.length();// dLENGTH(jfeedbacks[i].f1);
				double l1 = jfeedbacks[i].f2.length();// dLENGTH(jfeedbacks[i].f2);
				colours[i+0] = 0.95*colours[i+0] + 0.05 * l0/forcelimit;
				colours[i+1] = 0.95*colours[i+1] + 0.05 * l1/forcelimit;
				if (l0 > forcelimit || l1 > forcelimit)
					stress[i]++;
				else
					stress[i]=0;
				if (stress[i]>4)
				{
					// Low-pass filter the noisy feedback data.
					// Only after 4 consecutive timesteps with excessive load, snap.
					fprintf(stderr,"SNAP! (that was the sound of joint %d breaking)\n", i);
					dJointAttach (hinges[i], null, null);
				}
			}
		}
	}


	// simulation loop

	public void simLoop (boolean pause)
	{
		int i;

		float simstep = 0.002f; // 2ms simulation steps
		float dt = (float) dsElapsedTime();
		int nrofsteps = (int) ceilf(dt/simstep);
		for (i=0; i<nrofsteps && !pause; i++)
		{
			dSpaceCollide (space,null,nearCallback);
			dWorldQuickStep (world, simstep);
			dJointGroupEmpty (contactgroup);
			inspectJoints();
		}

		for (i=0; i<SEGMCNT; i++)
		{
			float r=0,g=0,b=0.2f;
			float v = (float) colours[i];
			if (v>1.0) v=1.0f;
			if (v<0.5) 
			{
				r=2*v;
				g=1.0f;
			}
			else
			{
				r=1.0f;
				g=2*(1.0f-v);
			}
			dsSetColor (r,g,b);
			drawGeom(seggeoms[i]);
		}
		dsSetColor (1,1,1);
		for (i=0; i<STACKCNT; i++)
			drawGeom(stackgeoms[i]);
	}



	public static void main(String[] args) {
		new DemoFeedback().demo(args);
	}
	
	private void demo(String[] args) {
		DMass m = dMassCreate();

		// setup pointers to drawstuff callback functions
//		dsFunctions fn = new DemoFeedback();
//		fn.version = DS_VERSION;
//		fn.start = &start;
//		fn.step = &simLoop;
//		fn.command = &command;
//		fn.stop = 0;
//		fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		world = dWorldCreate();
		space = dHashSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world,0,0,-9.8);
		dWorldSetQuickStepNumIterations (world, 20);

		int i;
		for (i=0; i<SEGMCNT; i++)
		{
			segbodies[i] = dBodyCreate (world);
			dBodySetPosition(segbodies[i], i - SEGMCNT/2.0, 0, 5);
			dMassSetBox (m, 1, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
			dBodySetMass (segbodies[i], m);
			seggeoms[i] = dCreateBox (null, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
			dGeomSetBody (seggeoms[i], segbodies[i]);
			dSpaceAdd (space, seggeoms[i]);
		}

		for (i=0; i<SEGMCNT-1; i++)
		{
			//TZ
			jfeedbacks[i] = dJointCreateFeedback();
			hinges[i] = dJointCreateHinge (world,null);
			dJointAttach (hinges[i], segbodies[i],segbodies[i+1]);
			dJointSetHingeAnchor (hinges[i], i + 0.5 - SEGMCNT/2.0, 0, 5);
			dJointSetHingeAxis (hinges[i], 0,1,0);
			dJointSetHingeParam (hinges[i],dParamFMax,  8000.0);
			// NOTE:
			// Here we tell ODE where to put the feedback on the forces for this hinge
			dJointSetFeedback (hinges[i], jfeedbacks[i]);
			stress[i]=0;
		}

		for (i=0; i<STACKCNT; i++)
		{
			stackbodies[i] = dBodyCreate(world);
			dMassSetBox (m, 2.0, 2, 2, 0.6);
			dBodySetMass(stackbodies[i],m);

			stackgeoms[i] = dCreateBox(null, 2, 2, 0.6);
			dGeomSetBody(stackgeoms[i], stackbodies[i]);
			dBodySetPosition(stackbodies[i], 0,0,8+2*i);
			dSpaceAdd(space, stackgeoms[i]);
		}

		sliders[0] = dJointCreateSlider (world,null);
		dJointAttach(sliders[0], segbodies[0], null);
		dJointSetSliderAxis (sliders[0], 1,0,0);
		dJointSetSliderParam (sliders[0],dParamFMax,  4000.0);
		dJointSetSliderParam (sliders[0],dParamLoStop,   0.0);
		dJointSetSliderParam (sliders[0],dParamHiStop,   0.2);

		sliders[1] = dJointCreateSlider (world,null);
		dJointAttach(sliders[1], segbodies[SEGMCNT-1], null);
		dJointSetSliderAxis (sliders[1], 1,0,0);
		dJointSetSliderParam (sliders[1],dParamFMax,  4000.0);
		dJointSetSliderParam (sliders[1],dParamLoStop,   0.0);
		dJointSetSliderParam (sliders[1],dParamHiStop,  -0.2);

		dCreatePlane(space, 0,0,1,0);

		for (i=0; i<SEGMCNT; i++)
			colours[i]=0.0;

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dJointGroupEmpty (contactgroup);
		dJointGroupDestroy (contactgroup);

		// First destroy seggeoms, then space, then the world.
		for (i=0; i<SEGMCNT; i++)
			dGeomDestroy (seggeoms[i]);
		for (i=0; i<STACKCNT; i++)
			dGeomDestroy (stackgeoms[i]);

		dSpaceDestroy(space);
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


