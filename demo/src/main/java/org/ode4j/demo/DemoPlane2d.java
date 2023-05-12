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

import static org.ode4j.drawstuff.DrawStuff.*;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DPlane2DJoint;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;


/**
 * Test my Plane2D constraint.
 * Uses ode-0.35 collision API.
 * 
 */
class DemoPlane2d extends dsFunctions {

	public static final int RAND_MAX = 2147483647;
	private static final double drand48() {
		//return ((double) rand()) / ((double) RAND_MAX);
		int i1 = OdeMath.dRandInt(RAND_MAX);
		return ((double) i1) / ((double) RAND_MAX);
	}
	//TZ: below is the new version from 0.11.1, but it yields different results
	//in Java and C++
	//#   define drand48()  ((double) (((double) rand()) / ((double) RAND_MAX)))
//	private static final double drand48() {
//		return ((double)rand()) / ((double)RAND_MAX);
//	}

	private static final int N_BODIES = 40;
	private static final float STAGE_SIZE = 8.0f; // in m

	private static final double TIME_STEP = 0.01;
	//private static final double K_SPRING = 10.0;
	//private static final double K_DAMP = 10.0; 

	private static class GlobalVars {
		DWorld dyn_world;
		DBody[] dyn_bodies = new DBody[N_BODIES];
		DVector3[] bodies_sides = new DVector3[N_BODIES];

		DSpace coll_space_id;
		DPlane2DJoint[] plane2d_joint_ids = new DPlane2DJoint[N_BODIES];
		DJointGroup coll_contacts;
	}

	private static GlobalVars g_globals_ptr = null;

//	private static DWorld   dyn_world;
//	private static DBody[]    dyn_bodies = new DBody[N_BODIES];
//	private static DVector3[]    bodies_sides = new DVector3[N_BODIES];
//
//	private static DSpace coll_space_id;
//	private static DPlane2DJoint[] plane2d_joint_ids=new DPlane2DJoint[N_BODIES];
//	private static DJointGroup coll_contacts;


	private static float[]    xyz = { 0.5f*STAGE_SIZE, 0.5f*STAGE_SIZE, 0.65f*STAGE_SIZE};
	private static float[]    hpr = { 90.0f, -90.0f, 0 };

	private static void cb_start ()	{
		dsSetViewpoint (xyz, hpr);
	}


	private static void     cb_near_collision (Object data, DGeom o1, DGeom o2) {
		DBody     b1 = o1.getBody();
		DBody     b2 = o2.getBody();
		//dContact    contact = new dContact();
		DContactBuffer contacts = new DContactBuffer(1);

		// exit without doing anything if the two bodies are static
		if (b1 == null && b2 == null) {
			return;
		}

		// exit without doing anything if the two bodies are connected by a joint
		if (b1!=null && b2!=null && OdeHelper.areConnected(b1, b2))
		{
			/* MTRAP; */
			return;
		}

		contacts.get(0).surface.mode = 0;
		contacts.get(0).surface.mu = 0; // frictionless

		if (OdeHelper.collide (o1, o2, 1, contacts.getGeomBuffer())!=0)
		{
			DJoint c = OdeHelper.createContactJoint (g_globals_ptr.dyn_world,
					g_globals_ptr.coll_contacts, contacts.get(0));
			c.attach (b1, b2);
		}
	}


	private static void     track_to_pos (DBody body, DPlane2DJoint joint_id,
			double target_x, double target_y) {
		double  curr_x = body.getPosition().get0();
		double  curr_y = body.getPosition().get1();

		joint_id.setXParamVel (1 * (target_x - curr_x));
		joint_id.setYParamVel (1 * (target_y - curr_y));
	}


	private DNearCallback myNearCallBack = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			cb_near_collision(data, o1, o2);
		}
	};

	private static double angle = 0;
	
	@Override
	@SuppressWarnings("unused")
	public void step (boolean pause) {
		if (! pause) {
			//        static double angle = 0;

			angle +=  0.01 ;

			track_to_pos (g_globals_ptr.dyn_bodies[0], g_globals_ptr.plane2d_joint_ids[0],
					( STAGE_SIZE/2 + STAGE_SIZE/2.0 * Math.cos (angle) ),
					( STAGE_SIZE/2 + STAGE_SIZE/2.0 * Math.sin (angle) ));

			/* double   f0 = 0.001; */
			/* for (int b = 0; b < N_BODIES; b ++) */
			/* { */
			/* double   p = 1 + b / (double) N_BODIES; */
			/* double   q = 2 - b / (double) N_BODIES; */
			/* dyn_bodies[b].addForce (f0 * cos (p*angle), f0 * sin (q*angle), 0); */
			/* } */
			/* dyn_bodies[0].addTorque (0, 0, 0.1); */

			final int n = 10;
			for (int i = 0; i < n; i ++)
			{
				//dSpaceCollide (coll_space_id, 0, cb_near_collision);
				OdeHelper.spaceCollide (g_globals_ptr.coll_space_id, null, myNearCallBack );
				// TODO See issue #15, step() causes weird spinning and console errors
				// g_globals_ptr.dyn_world.step (TIME_STEP/n);
				g_globals_ptr.dyn_world.quickStep (TIME_STEP/n);
				g_globals_ptr.coll_contacts.empty ();
			}
		}

		if (true) //# if 1  /* [ */
		{
			// @@@ hack Plane2D constraint error reduction here:
			for (int b = 0; b < N_BODIES; b ++)
			{
				DVector3C rot = g_globals_ptr.dyn_bodies[b].getAngularVel();
				//final double     []quat_ptr;
				DQuaternionC     quat_ptr;
				DQuaternion           quat= new DQuaternion();
				double                quat_len;

				double q0, q3;
				quat_ptr = g_globals_ptr.dyn_bodies[b].getQuaternion();
				//            quat[0] = quat_ptr[0];
				//            quat[1] = 0;
				//            quat[2] = 0;
				//            quat[3] = quat_ptr[3];
				q0 = quat_ptr.get(0);
				q3 = quat_ptr.get(3);
				quat_len = Math.sqrt (q0 * q0 + q3 * q3);
				q0 /= quat_len;
				q3 /= quat_len;
				quat.set(q0, 0, 0, q3);
				g_globals_ptr.dyn_bodies[b].setQuaternion (quat);
				g_globals_ptr.dyn_bodies[b].setAngularVel (0, 0, rot.get2());
			}
		}//# endif  /* ] */


		if (false)//# if 0  /* [ */
		{
			// @@@ friction
			for (int b = 0; b < N_BODIES; b ++)
			{
				DVector3C vel = g_globals_ptr.dyn_bodies[b].getLinearVel(),
				rot = g_globals_ptr.dyn_bodies[b].getAngularVel();
				double       s = 1.00;
				double       t = 0.99;

				g_globals_ptr.dyn_bodies[b].setLinearVel ( vel.reScale(s) );
				g_globals_ptr.dyn_bodies[b].setAngularVel( rot.reScale(t));
			}
		}//# endif  /* ] */


		{
			// ode  drawstuff

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
			for (int b = 0; b < N_BODIES; b ++)
			{
				if (b == 0)
					dsSetColor(1.0f, 0.5f, 1.0f);
				else
					dsSetColor(0f, 0.5f, 1.0f);
				dsDrawBox(g_globals_ptr.dyn_bodies[b].getPosition(),
						g_globals_ptr.dyn_bodies[b].getRotation(),
						g_globals_ptr.bodies_sides[b]);
			}
		}
	}



	/**
	 * @param args args
	 */
	public static void main(String[] args) {
		int         b;
		dsFunctions drawstuff_functions = new DemoPlane2d();


		OdeHelper.initODE2(0);

		g_globals_ptr = new GlobalVars();

		// dynamic world

		double  cf_mixing;// = 1 / TIME_STEP * K_SPRING + K_DAMP;
		double  err_reduct;// = TIME_STEP * K_SPRING * cf_mixing;
		err_reduct =  0.5 ;
		cf_mixing =  0.001 ;
		//TZ
		g_globals_ptr.dyn_world = OdeHelper.createWorld();
		g_globals_ptr.dyn_world.setERP (err_reduct);
		g_globals_ptr.dyn_world.setCFM (cf_mixing);
		g_globals_ptr.dyn_world.setGravity (0, 0.0, -1.0);

		g_globals_ptr.coll_space_id = OdeHelper.createSimpleSpace (null);
		//coll_space_id = dSweepAndPruneSpaceCreate(null, dSAP_AXES_XYZ);
		//coll_space_id = dHashSpaceCreate(null);

		// dynamic bodies
		for (b = 0; b < N_BODIES; b ++)
		{
			int     l = (int) (1 + Math.sqrt (N_BODIES));
			double  x = (0.5 + (b / l)) / l * STAGE_SIZE;
			double  y = (0.5 + (b % l)) / l * STAGE_SIZE;
			//double  z = REAL( 1.0 ) + REAL( 0.1 ) * (double)drand48();
			double  z = 1.0 + 0.1  * drand48();

			//TZ
//			bodies_sides[b] = new dVector3(
//					(double)( 5. * (0.2 + 0.7*drand48()) / sqrt((double)N_BODIES) ),
//					(double)( 5. * (0.2 + 0.7*drand48()) / sqrt((double)N_BODIES) ),
//					z);
			double r2 = drand48();
			double r1 = drand48();
			g_globals_ptr.bodies_sides[b] = new DVector3(
					5. * (0.2 + 0.7*r2) / Math.sqrt(N_BODIES),
					5. * (0.2 + 0.7*r1) / Math.sqrt(N_BODIES),
					z);

			g_globals_ptr.dyn_bodies[b] = OdeHelper.createBody(g_globals_ptr.dyn_world);
			g_globals_ptr.dyn_bodies[b].setPosition (x, y, z/2);
			g_globals_ptr.dyn_bodies[b].setData (b);//(void*) (size_t)b);
//			dBodySetLinearVel (dyn_bodies[b],
//			(double)( 3. * (drand48 () - 0.5) ), 
//			(double)( 3. * (drand48 () - 0.5) ), 0);
			r2 = drand48();
			r1 = drand48();
			g_globals_ptr.dyn_bodies[b].setLinearVel (
			3. * (r1 - 0.5), 
			3. * (r2 - 0.5), 0);

			DMass m = OdeHelper.createMass();
			m.setBox(1, g_globals_ptr.bodies_sides[b]);
			m.adjust(0.1 * g_globals_ptr.bodies_sides[b].get0() * g_globals_ptr.bodies_sides[b].get1());
			g_globals_ptr.dyn_bodies[b].setMass(m);

			g_globals_ptr.plane2d_joint_ids[b] = OdeHelper.createPlane2DJoint(g_globals_ptr.dyn_world, null);
			g_globals_ptr.plane2d_joint_ids[b].attach(g_globals_ptr.dyn_bodies[b], null);
		}
		g_globals_ptr.plane2d_joint_ids[0].setXParamFMax(10);
		g_globals_ptr.plane2d_joint_ids[0].setYParamFMax(10);


		// collision geoms and joints
		OdeHelper.createPlane (g_globals_ptr.coll_space_id,  1, 0, 0, 0);
		OdeHelper.createPlane (g_globals_ptr.coll_space_id, -1, 0, 0, -STAGE_SIZE);
		OdeHelper.createPlane (g_globals_ptr.coll_space_id,  0,  1, 0, 0);
		OdeHelper.createPlane (g_globals_ptr.coll_space_id,  0, -1, 0, -STAGE_SIZE);

		for (b = 0; b < N_BODIES; b ++)
		{
			DGeom coll_box_id;
			coll_box_id = OdeHelper.createBox (g_globals_ptr.coll_space_id, g_globals_ptr.bodies_sides[b]);
			coll_box_id.setBody (g_globals_ptr.dyn_bodies[b]);
		}

		g_globals_ptr.coll_contacts = OdeHelper.createJointGroup();

		dsSimulationLoop (args, 352,288,drawstuff_functions);

		OdeHelper.closeODE();
	}


	@Override
	public void command(char cmd) {
		// Nothing
	}


	@Override
	public void start() {
		cb_start();	
	}


	@Override
	public void stop() {
		// Nothing
	}
}
