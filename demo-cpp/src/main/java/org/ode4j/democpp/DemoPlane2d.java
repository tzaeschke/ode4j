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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSimpleSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePlane2D;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPlane2DXParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPlane2DYParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamVel;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppOther.dAreConnected;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetCFM;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetERP;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.internal.cpp4j.Cmath.cos;
import static org.ode4j.ode.internal.cpp4j.Cmath.sin;
import static org.ode4j.ode.internal.cpp4j.Cmath.sqrt;
import static org.ode4j.ode.internal.cpp4j.Cstdlib.RAND_MAX;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPlane2DJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;


/**
 * Test my Plane2D constraint.
 * Uses ode-0.35 collision API.
 * 
 */
class DemoPlane2d extends dsFunctions {

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

	//
	private static final int N_BODIES = 40;
	private static final float STAGE_SIZE = 8.0f; // in m

	private static final double TIME_STEP = 0.01;
	//private static final double K_SPRING = 10.0;
	//private static final double K_DAMP = 10.0; 

	private static DWorld   dyn_world;
	private static DBody[]    dyn_bodies = new DBody[N_BODIES];
	private static DVector3[]    bodies_sides = new DVector3[N_BODIES];

	private static DSpace coll_space_id;
	private static DPlane2DJoint[] plane2d_joint_ids=new DPlane2DJoint[N_BODIES];
	private static DJointGroup coll_contacts;


	private static float[]    xyz = { 0.5f*STAGE_SIZE, 0.5f*STAGE_SIZE, 0.65f*STAGE_SIZE};
	private static float[]    hpr = { 90.0f, -90.0f, 0 };

	private static void     cb_start ()
	/** ********************** */
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);
		dsSetViewpoint (xyz, hpr);
	}



	private static void     cb_near_collision (Object data, DGeom o1, DGeom o2)
	/********************************************************************/
	{
		DBody     b1 = dGeomGetBody (o1);
		DBody     b2 = dGeomGetBody (o2);
		//dContact    contact = new dContact();
		DContactBuffer contacts = new DContactBuffer(1);

		// exit without doing anything if the two bodies are static
		if (b1 == null && b2 == null) {
			return;
		}

		// exit without doing anything if the two bodies are connected by a joint
		if (b1!=null && b2!=null && dAreConnected(b1, b2))
		{
			/* MTRAP; */
			return;
		}

		contacts.get(0).surface.mode = 0;
		contacts.get(0).surface.mu = 0; // frictionless

		if (dCollide (o1, o2, 1, contacts.getGeomBuffer())!=0)//, sizeof (dContactGeom)))
		{
			DJoint c = dJointCreateContact (dyn_world,
					coll_contacts, contacts.get(0));
			dJointAttach (c, b1, b2);
		}
	}


	private static void     track_to_pos (DBody body, DPlane2DJoint joint_id,
			double target_x, double target_y)
	/************************************************************************/
	{
		double  curr_x = body.getPosition().get0();
		double  curr_y = body.getPosition().get1();

		dJointSetPlane2DXParam (joint_id, dParamVel, 1 * (target_x - curr_x));
		dJointSetPlane2DYParam (joint_id, dParamVel, 1 * (target_y - curr_y));
	}


	private DNearCallback myNearCallBack = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			cb_near_collision(data, o1, o2);
		}
	};

	private static double angle = 0;
	@SuppressWarnings("unused")
	private void cb_sim_step (boolean pause)
	{
		if (! pause)
		{
			//        static double angle = 0;

			angle +=  0.01 ;

			track_to_pos (dyn_bodies[0], plane2d_joint_ids[0],
					( STAGE_SIZE/2 + STAGE_SIZE/2.0 * cos (angle) ),
					( STAGE_SIZE/2 + STAGE_SIZE/2.0 * sin (angle) ));

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
				dSpaceCollide (coll_space_id, null, myNearCallBack );
				dyn_world.step (TIME_STEP/n);
				coll_contacts.empty ();
			}
		}

		if (true) //# if 1  /* [ */
		{
			// @@@ hack Plane2D constraint error reduction here:
			for (int b = 0; b < N_BODIES; b ++)
			{
				final DVector3C rot = dBodyGetAngularVel (dyn_bodies[b]);
				//final double     []quat_ptr;
				DQuaternionC     quat_ptr;
				DQuaternion           quat= new DQuaternion();
				double                quat_len;

				double q0, q3;
				quat_ptr = dBodyGetQuaternion (dyn_bodies[b]);
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
				dBodySetQuaternion (dyn_bodies[b], quat);
				dBodySetAngularVel (dyn_bodies[b], 0, 0, rot.get2());
			}
		}//# endif  /* ] */


		if (false)//# if 0  /* [ */
		{
			// @@@ friction
			for (int b = 0; b < N_BODIES; b ++)
			{
				DVector3C vel = dBodyGetLinearVel (dyn_bodies[b]),
				rot = dBodyGetAngularVel (dyn_bodies[b]);
				double       s = 1.00;
				double       t = 0.99;

				dBodySetLinearVel (dyn_bodies[b], s*vel.get0(),s*vel.get1(),s*vel.get2());
				dBodySetAngularVel (dyn_bodies[b],t*rot.get0(),t*rot.get1(),t*rot.get2());
			}
		}//# endif  /* ] */


		{
			// ode  drawstuff

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
			for (int b = 0; b < N_BODIES; b ++)
			{
				if (b == 0)
					dsSetColor (1.0f, 0.5f, 1.0f);
				else
					dsSetColor (0f, 0.5f, 1.0f);
				dsDrawBox (dyn_bodies[b].getPosition(), dyn_bodies[b].getRotation(), bodies_sides[b]);
			}
		}
	}



	/**
	 * @param args arguments 
	**/
	public static void main(String[] args)
	{
		new DemoPlane2d().demo(args);
	}
	
	private void demo(String[] args) {
		int         b;
		
		dInitODE2(0);

		// dynamic world

		double  cf_mixing;// = 1 / TIME_STEP * K_SPRING + K_DAMP;
		double  err_reduct;// = TIME_STEP * K_SPRING * cf_mixing;
		err_reduct =  0.5 ;
		cf_mixing =  0.001 ;
		//TZ
		dyn_world = dWorldCreate();
		dWorldSetERP (dyn_world, err_reduct);
		dWorldSetCFM (dyn_world, cf_mixing);
		dyn_world.setGravity (0, 0.0, -1.0);

		coll_space_id = dSimpleSpaceCreate (null);
		//coll_space_id = dSweepAndPruneSpaceCreate(null, dSAP_AXES_XYZ);
		//coll_space_id = dHashSpaceCreate(null);

		// dynamic bodies
		for (b = 0; b < N_BODIES; b ++)
		{
			int     l = (int) (1 + sqrt ((double) N_BODIES));
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
			bodies_sides[b] = new DVector3(
					5. * (0.2 + 0.7*r2) / sqrt((double)N_BODIES),
					5. * (0.2 + 0.7*r1) / sqrt((double)N_BODIES),
					z);

			dyn_bodies[b] = dBodyCreate(dyn_world);
			dyn_bodies[b].setPosition (x, y, z/2);
			dyn_bodies[b].setData (b);//(void*) (size_t)b);
//			dBodySetLinearVel (dyn_bodies[b],
//			(double)( 3. * (drand48 () - 0.5) ), 
//			(double)( 3. * (drand48 () - 0.5) ), 0);
			r2 = drand48();
			r1 = drand48();
			dBodySetLinearVel (dyn_bodies[b],
			3. * (r1 - 0.5), 
			3. * (r2 - 0.5), 0);

			DMass m = OdeHelper.createMass();
			m.setBox (1, bodies_sides[b].get0(),bodies_sides[b].get1(),bodies_sides[b].get2());
			m.adjust (0.1 * bodies_sides[b].get0() * bodies_sides[b].get1());
			dyn_bodies[b].setMass (m);

			plane2d_joint_ids[b] = dJointCreatePlane2D (dyn_world, null);
			dJointAttach (plane2d_joint_ids[b], dyn_bodies[b], null);
		}
		dJointSetPlane2DXParam (plane2d_joint_ids[0], dParamFMax, 10);
		dJointSetPlane2DYParam (plane2d_joint_ids[0], dParamFMax, 10);


		// collision geoms and joints
		dCreatePlane (coll_space_id,  1, 0, 0, 0);
		dCreatePlane (coll_space_id, -1, 0, 0, -STAGE_SIZE);
		dCreatePlane (coll_space_id,  0,  1, 0, 0);
		dCreatePlane (coll_space_id,  0, -1, 0, -STAGE_SIZE);

		for (b = 0; b < N_BODIES; b ++)
		{
			DGeom coll_box_id;
			coll_box_id = dCreateBox (coll_space_id,
					bodies_sides[b].get0(), bodies_sides[b].get1(), bodies_sides[b].get2());
			dGeomSetBody (coll_box_id, dyn_bodies[b]);
		}

		coll_contacts = OdeHelper.createJointGroup();

		{
			// simulation loop (by drawstuff lib)
			//drawstuff_functions.setVersion(DS_VERSION);
			//		drawstuff_functions.start = &cb_start;
			//		drawstuff_functions.step = &cb_sim_step;
			//		drawstuff_functions.command = 0;
			//		drawstuff_functions.stop = 0;
			//drawstuff_functions.setPathToTextures(DRAWSTUFF_TEXTURE_PATH);

			dsSimulationLoop (args, 352,288,this);
		}

		dCloseODE();
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
	public void step(boolean pause) {
		cb_sim_step(pause);
	}


	@Override
	public void stop() {
		// Nothing
	}
}
