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
package org.ode4j.ode.internal;

import org.cpp4j.java.Ref;

import static org.ode4j.ode.OdeMath.*;

import org.ode4j.ode.internal.Objects_H.dxAutoDisable;
import org.ode4j.ode.internal.Objects_H.dxContactParameters;
import org.ode4j.ode.internal.Objects_H.dxDampingParameters;
import org.ode4j.ode.internal.Objects_H.dxQuickStepParameters;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DWorld;

public class DxWorld extends DBase implements DWorld {

	//TODO
	public final Ref<DxBody> firstbody = new Ref<DxBody>();
	public final Ref<DxJoint> firstjoint = new Ref<DxJoint>();
	//	List<dxBody> bodies = new LinkedList<dxBody>();
	//	List<dxBody> joints = new LinkedList<dxBody>();
	//	 public dxBody firstbody;		// body linked list
	//	  dxJoint firstjoint;		// joint linked list
	public int nb;			// number of bodies and joints in lists
	public int nj;
	DVector3 gravity;		// gravity vector (m/s/s)
	public double global_erp;		// global error reduction parameter
	public double global_cfm;		// global costraint force mixing parameter
	public dxAutoDisable adis;		// auto-disable parameters
	public int body_flags;               // flags for new bodies
	dxQuickStepParameters qs;
	public dxContactParameters contactp;
	public dxDampingParameters dampingp; // damping parameters
	double max_angular_speed;      // limit the angular velocity to this magnitude

	//****************************************************************************
	// world
	private DxWorld() {
		//private
		super();
	}


	public static DxWorld dWorldCreate()
	{
		DxWorld w = new DxWorld();
		w.firstbody.set(null);
		w.firstjoint.set(null);
		w.nb = 0;
		w.nj = 0;
		//dSetZero (w.gravity,4);
		w.gravity = new DVector3();
		w.global_erp = 0.2;
		//	#if defined(dSINGLE)
		//	  w.global_cfm = 1e-5f;
		//	#elif defined(dDOUBLE)
		if (dDOUBLE) {
			w.global_cfm = 1e-10;
		} else {
			w.global_cfm = 1e-5;
		}
		//	#else
		//	  #error dSINGLE or dDOUBLE must be defined
		//	#endif

		w.body_flags = 0; // everything disabled

		w.adis = new dxAutoDisable();
		w.adis.idle_steps = 10;
		w.adis.idle_time = 0;
		w.adis.average_samples = 1;		// Default is 1 sample => Instantaneous velocity
		w.adis.angular_average_threshold = (0.01)*(0.01);	// (magnitude squared)
		w.adis.linear_average_threshold = (0.01)*(0.01);		// (magnitude squared)

		w.qs = new dxQuickStepParameters();
		w.qs.num_iterations = 20;
		w.qs.w = (1.3);

		w.contactp = new dxContactParameters();
		w.contactp.max_vel = dInfinity;
		w.contactp.min_depth = 0;

		w.dampingp = new dxDampingParameters();
		w.dampingp.linear_scale = 0;
		w.dampingp.angular_scale = 0;
		w.dampingp.linear_threshold = (0.01) * (0.01);
		w.dampingp.angular_threshold = (0.01) * (0.01);  
		w.max_angular_speed = dInfinity;

		return w;
	}


	public void dWorldDestroy ()
	{
		// delete all bodies and joints
//		dAASSERT (w);
		DxBody nextb, b = firstbody.get();
		while (b != null) {
			nextb = (DxBody) b.getNext();
			// TODO: remove those 2 ifs
			if(b.average_lvel_buffer != null)
			{
//TZ				delete[] (b.average_lvel_buffer);
				b.average_lvel_buffer = null;
			}
			if(b.average_avel_buffer != null)
			{
//TZ				delete[] (b.average_avel_buffer);
				b.average_avel_buffer = null;
			}
			b.dBodyDestroy(); // calling here dBodyDestroy for correct destroying! (i.e. the average buffers)
			b = nextb;
		}
		DxJoint nextj, j = firstjoint.get();
		while (j != null) {
			nextj = (DxJoint)j.getNext();
			if ((j.flags & DxJoint.dJOINT_INGROUP) != 0) {
				// the joint is part of a group, so "deactivate" it instead
				j.world = null;
				j.node[0].body = null;
				j.node[0].next = null;
				j.node[1].body = null;
				j.node[1].next = null;
				dMessage (0,"warning: destroying world containing grouped joints");
			}
			else {
				//TZ size_t sz = j.size();
				j.DESTRUCTOR();
				//TZ dFree (j,sz);
			}
			j = nextj;
		}
//		delete w;
		DESTRUCTOR();
	}


//	public void dWorldSetGravity (dxWorld w, double x, double y, double z)
	public void dWorldSetGravity (double x, double y, double z)
	{
		//dAASSERT (w);
		gravity.set(x, y, z);
//		gravity.v[0] = x;
//		gravity.v[1] = y;
//		gravity.v[2] = z;
	}


	void dWorldGetGravity (DxWorld w, DVector3 g)
	{
		dAASSERT (w);
		g.set(gravity);
		//	  g[0] = w.gravity[0];
		//	  g[1] = w.gravity[1];
		//	  g[2] = w.gravity[2];
	}


//	public void dWorldSetERP (dxWorld w, double erp)
	public void dWorldSetERP (double erp)
	{
		//dAASSERT (w);
		global_erp = erp;
	}


	double dWorldGetERP (DxWorld w)
	{
		dAASSERT (w);
		return w.global_erp;
	}


//	public void dWorldSetCFM (dxWorld w, double cfm)
	public void dWorldSetCFM (double cfm)
	{
		//dAASSERT (w);
		global_cfm = cfm;
	}


	double dWorldGetCFM (DxWorld w)
	{
		dAASSERT (w);
		return w.global_cfm;
	}


//	void dWorldStep (dxWorld w, double stepsize)
	public void dWorldStep (double stepsize)
	{
		//dUASSERT (w,"bad world argument");
		dUASSERT (stepsize > 0,"stepsize must be > 0");
		dxProcessIslands (this,stepsize,Step.INSTANCE);//&dInternalStepIsland);
	}


	//	public void dWorldQuickStep (dxWorld w, double stepsize)
	public void dWorldQuickStep (double stepsize)
	{
		//	  dUASSERT (w,"bad world argument");
		dUASSERT (stepsize > 0,"stepsize must be > 0");
		dxProcessIslands (this,stepsize,DxQuickStep.INSTANCE);//dxQuickStepper);
	}


	void dWorldImpulseToForce (DxWorld w, double stepsize,
			double ix, double iy, double iz,
			DVector3 force)
	{
		dAASSERT (w);
		stepsize = dRecip(stepsize);
//		force.v[0] = stepsize * ix;
//		force.v[1] = stepsize * iy;
//		force.v[2] = stepsize * iz;
		force.set(stepsize * ix, stepsize * iy, stepsize * iz);
		// @@@ force[3] = 0;
	}


	// world auto-disable functions

	double dWorldGetAutoDisableLinearThreshold (DxWorld w)
	{
		dAASSERT(w);
		return dSqrt (w.adis.linear_average_threshold);
	}


	void dWorldSetAutoDisableLinearThreshold (DxWorld w, double linear_average_threshold)
	{
		dAASSERT(w);
		w.adis.linear_average_threshold = linear_average_threshold * linear_average_threshold;
	}


	double dWorldGetAutoDisableAngularThreshold (DxWorld w)
	{
		dAASSERT(w);
		return dSqrt (w.adis.angular_average_threshold);
	}


	void dWorldSetAutoDisableAngularThreshold (DxWorld w, double angular_average_threshold)
	{
		dAASSERT(w);
		w.adis.angular_average_threshold = angular_average_threshold * angular_average_threshold;
	}


	int dWorldGetAutoDisableAverageSamplesCount (DxWorld w)
	{
		dAASSERT(w);
		return w.adis.average_samples;
	}


	//	void dWorldSetAutoDisableAverageSamplesCount (dxWorld w, 
	//			unsigned int average_samples_count)
	public void dWorldSetAutoDisableAverageSamplesCount ( 
			int average_samples_count)
	{
		//dAASSERT(w);
		adis.average_samples = average_samples_count;
	}


	int dWorldGetAutoDisableSteps (DxWorld w)
	{
		dAASSERT(w);
		return w.adis.idle_steps;
	}


	void dWorldSetAutoDisableSteps (DxWorld w, int steps)
	{
		dAASSERT(w);
		w.adis.idle_steps = steps;
	}


	double dWorldGetAutoDisableTime (DxWorld w)
	{
		dAASSERT(w);
		return w.adis.idle_time;
	}


	void dWorldSetAutoDisableTime (DxWorld w, double time)
	{
		dAASSERT(w);
		w.adis.idle_time = time;
	}


	int dWorldGetAutoDisableFlag (DxWorld w)
	{
		dAASSERT(w);
		return w.body_flags & DxBody.dxBodyAutoDisable;
	}


	//	void dWorldSetAutoDisableFlag (dxWorld w, int do_auto_disable)
	public void dWorldSetAutoDisableFlag (boolean do_auto_disable)
	{
		//dAASSERT(w);
		if (do_auto_disable)
			body_flags |= DxBody.dxBodyAutoDisable;
		else
			body_flags &= ~DxBody.dxBodyAutoDisable;
	}


	// world damping functions

	double dWorldGetLinearDampingThreshold(DxWorld w)
	{
		dAASSERT(w);
		return dSqrt(w.dampingp.linear_threshold);
	}

	void dWorldSetLinearDampingThreshold(DxWorld w, double threshold)
	{
		dAASSERT(w);
		w.dampingp.linear_threshold = threshold*threshold;
	}

	double dWorldGetAngularDampingThreshold(DxWorld w)
	{
		dAASSERT(w);
		return dSqrt(w.dampingp.angular_threshold);
	}

	void dWorldSetAngularDampingThreshold(DxWorld w, double threshold)
	{
		dAASSERT(w);
		w.dampingp.angular_threshold = threshold*threshold;
	}

	double dWorldGetLinearDamping(DxWorld w)
	{
		dAASSERT(w);
		return w.dampingp.linear_scale;
	}

	public void dWorldSetLinearDamping(double scale)
	{
		//dAASSERT(w);
		if (scale != 0)
			body_flags |= DxBody.dxBodyLinearDamping;
		else
			body_flags &= ~DxBody.dxBodyLinearDamping;
		dampingp.linear_scale = scale;
	}

//	double dWorldGetAngularDamping(dxWorld w)
	public double dWorldGetAngularDamping()
	{
		//dAASSERT(w);
		return dampingp.angular_scale;
	}

	//public void dWorldSetAngularDamping(dxWorld w, double scale)
	public void dWorldSetAngularDamping(double scale)
	{
		//dAASSERT(w);
		if (scale != 0)
			body_flags |= DxBody.dxBodyAngularDamping;
		else
			body_flags &= ~DxBody.dxBodyAngularDamping;
		dampingp.angular_scale = scale;
	}

	void dWorldSetDamping(DxWorld w, double linear_scale, double angular_scale)
	{
		dAASSERT(w);
		w.dWorldSetLinearDamping(linear_scale);
		w.dWorldSetAngularDamping(angular_scale);
	}

	double dWorldGetMaxAngularSpeed(DxWorld w)
	{
		dAASSERT(w);
		return w.max_angular_speed;
	}

//	public void dWorldSetMaxAngularSpeed(dxWorld w, double max_speed)
	public void dWorldSetMaxAngularSpeed(double max_speed)
	{
		//dAASSERT(w);
		if (max_speed < dInfinity)
			body_flags |= DxBody.dxBodyMaxAngularSpeed;
		else
			body_flags &= ~DxBody.dxBodyMaxAngularSpeed;
		max_angular_speed = max_speed;
	}


//	void dWorldSetQuickStepNumIterations (dxWorld w, int num)
	public void dWorldSetQuickStepNumIterations (int num)
	{
		//dAASSERT(w);
		qs.num_iterations = num;
	}


	int dWorldGetQuickStepNumIterations (DxWorld w)
	{
		dAASSERT(w);
		return w.qs.num_iterations;
	}


	void dWorldSetQuickStepW (DxWorld w, double param)
	{
		dAASSERT(w);
		w.qs.w = param;
	}


	double dWorldGetQuickStepW (DxWorld w)
	{
		dAASSERT(w);
		return w.qs.w;
	}


//	void dWorldSetContactMaxCorrectingVel (dxWorld w, double vel)
	public void dWorldSetContactMaxCorrectingVel (double vel)
	{
		//dAASSERT(w);
		contactp.max_vel = vel;
	}


	double dWorldGetContactMaxCorrectingVel (DxWorld w)
	{
		dAASSERT(w);
		return w.contactp.max_vel;
	}


	//public void dWorldSetContactSurfaceLayer (dxWorld w, double depth)
	public void dWorldSetContactSurfaceLayer (double depth)
	{
		//dAASSERT(w);
		contactp.min_depth = depth;
	}


	double dWorldGetContactSurfaceLayer (DxWorld w)
	{
		dAASSERT(w);
		return w.contactp.min_depth;
	}

	//**** FROM util.cpp TZ

	//	typedef void (*dstepper_fn_t) (dxWorld *world, dxBody * const *body, int nb,
			//	        dxJoint * const *_joint, int nj, dReal stepsize);
	public interface dstepper_fn_t {
		public void run(DxWorld world, DxBody[]body, int nb,
				DxJoint []_joint, int nj, double stepsize);
	}

	//****************************************************************************
	// island processing

	// this groups all joints and bodies in a world into islands. all objects
	// in an island are reachable by going through connected bodies and joints.
	// each island can be simulated separately.
	// note that joints that are not attached to anything will not be included
	// in any island, an so they do not affect the simulation.
	//
	// this function starts new island from unvisited bodies. however, it will
	// never start a new islands from a disabled body. thus islands of disabled
	// bodies will not be included in the simulation. disabled bodies are
	// re-enabled if they are found to be part of an active island.

	//	void dxProcessIslands (dxWorld *world, dReal stepsize, dstepper_fn_t stepper)
	void dxProcessIslands (DxWorld world, double stepsize, dstepper_fn_t stepper)
	{
		//		  dxBody *b,*bb,**body;
		//		  dxJoint *j,**joint;
		DxBody b,bb,body[];
		DxJoint j,joint[];

		// nothing to do if no bodies
		if (world.nb <= 0) return;

		// handle auto-disabling of bodies
		dInternalHandleAutoDisabling (world,stepsize);

		// make arrays for body and joint lists (for a single island) to go into
		body = new DxBody[world.nb];//(dxBody**) ALLOCA (world.nb * sizeof(dxBody*));
		joint = new DxJoint[world.nj];//(dxJoint**) ALLOCA (world.nj * sizeof(dxJoint*));
		int bcount = 0;	// number of bodies in `body'
		int jcount = 0;	// number of joints in `joint'

		// set all body/joint tags to 0
		for (b=world.firstbody.get(); b!=null; b=(DxBody)b.getNext()) b.tag = 0;
		for (j=world.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) j.tag = 0;

		// allocate a stack of unvisited bodies in the island. the maximum size of
		// the stack can be the lesser of the number of bodies or joints, because
		// new bodies are only ever added to the stack by going through untagged
		// joints. all the bodies in the stack must be tagged!
		int stackalloc = (world.nj < world.nb) ? world.nj : world.nb;
		//	  dxBody **stack = (dxBody**) ALLOCA (stackalloc * sizeof(dxBody*));
		DxBody[] stack = new DxBody[stackalloc];//**) ALLOCA (stackalloc * sizeof(dxBody*));

		for (bb=world.firstbody.get(); bb!=null; bb=(DxBody)bb.getNext()) {
			// get bb = the next enabled, untagged body, and tag it
			if (bb.tag!=0 || ((bb.flags & DxBody.dxBodyDisabled)!=0)) continue;
			bb.tag = 1;

			// tag all bodies and joints starting from bb.
			int stacksize = 0;
			b = bb;
			body[0] = bb;
			bcount = 1;
			jcount = 0;
			//TZ goto quickstart;
			boolean firstRound = true;
			while (stacksize > 0 || firstRound) {
				if (!firstRound) {
					b = stack[--stacksize];	// pop body off stack
					body[bcount++] = b;	// put body on body list
				}
				firstRound = false;//quickstart:

					// traverse and tag all body's joints, add untagged connected bodies
					// to stack
					for (DxJointNode n=b.firstjoint.get(); n!=null; n=n.next) {
						if (n.joint.tag==0) {
							n.joint.tag = 1;
							joint[jcount++] = n.joint;
							if (n.body!=null && (n.body.tag==0)) {
								n.body.tag = 1;
								stack[stacksize++] = n.body;
							}
						}
					}
				dIASSERT(stacksize <= world.nb);
				dIASSERT(stacksize <= world.nj);
			}

			// now do something with body and joint lists
			stepper.run (world,body,bcount,joint,jcount,stepsize);

			// what we've just done may have altered the body/joint tag values.
			// we must make sure that these tags are nonzero.
			// also make sure all bodies are in the enabled state.
			int i;
			for (i=0; i<bcount; i++) {
				body[i].tag = 1;
				body[i].flags &= ~DxBody.dxBodyDisabled;
			}
			for (i=0; i<jcount; i++) joint[i].tag = 1;
		}

		// if debugging, check that all objects (except for disabled bodies,
		// unconnected joints, and joints that are connected to disabled bodies)
		// were tagged.
		if (!dNODEBUG) {//# ifndef dNODEBUG
			for (b=world.firstbody.get(); b != null; b=(DxBody)b.getNext()) {
				if ((b.flags & DxBody.dxBodyDisabled) != 0) {
					if (b.tag!=0) dDebug (0,"disabled body tagged");
				}
				else {
					if (b.tag==0) dDebug (0,"enabled body not tagged");
				}
			}
			for (j=world.firstjoint.get(); j != null; j=(DxJoint)j.getNext()) {
				if ((j.node[0].body!=null && (j.node[0].body.flags & DxBody.dxBodyDisabled)==0) ||
						(j.node[1].body!=null && (j.node[1].body.flags & DxBody.dxBodyDisabled)==0)) {
					if (j.tag==0) dDebug (0,"attached enabled joint not tagged");
				}
				else {
					if (j.tag!=0) dDebug (0,"unattached or disabled joint tagged");
				}
			}
		}//dNoDEBUG # endif
	}


	//****************************************************************************
	// Auto disabling

	void dInternalHandleAutoDisabling (DxWorld world, double stepsize)
	{
		DxBody bb;
		for ( bb=world.firstbody.get(); bb!=null; bb=(DxBody)bb.getNext() )
		{
			// don't freeze objects mid-air (patch 1586738)
			if ( bb.firstjoint.get() == null ) continue;

			// nothing to do unless this body is currently enabled and has
			// the auto-disable flag set
			if ( (bb.flags & (DxBody.dxBodyAutoDisable|DxBody.dxBodyDisabled)) != 
				DxBody.dxBodyAutoDisable ) continue;

			// if sampling / threshold testing is disabled, we can never sleep.
			if ( bb.adis.average_samples == 0 ) continue;

			//
			// see if the body is idle
			//

			if (!dNODEBUG) {//#ifndef dNODEBUG
				// sanity check
				if ( bb.average_counter >= bb.adis.average_samples )
				{
					dUASSERT( bb.average_counter < bb.adis.average_samples, "buffer overflow" );

					// something is going wrong, reset the average-calculations
					bb.average_ready = 0; // not ready for average calculation
					bb.average_counter = 0; // reset the buffer index
				}
			}//#endif // dNODEBUG

			// sample the linear and angular velocity
//			bb.average_lvel_buffer[bb.average_counter].v[0] = bb.lvel.v[0];
//			bb.average_lvel_buffer[bb.average_counter].v[1] = bb.lvel.v[1];
//			bb.average_lvel_buffer[bb.average_counter].v[2] = bb.lvel.v[2];
			bb.average_lvel_buffer[bb.average_counter].set(bb.lvel);
//			bb.average_avel_buffer[bb.average_counter].v[0] = bb.avel.v[0];
//			bb.average_avel_buffer[bb.average_counter].v[1] = bb.avel.v[1];
//			bb.average_avel_buffer[bb.average_counter].v[2] = bb.avel.v[2];
			bb.average_avel_buffer[bb.average_counter].set(bb.avel);
			bb.average_counter++;

			// buffer ready test
			if ( bb.average_counter >= bb.adis.average_samples )
			{
				bb.average_counter = 0; // fill the buffer from the beginning
				bb.average_ready = 1; // this body is ready now for average calculation
			}

			boolean idle = false; // Assume it's in motion unless we have samples to disprove it.

			// enough samples?
			if ( bb.average_ready != 0)
			{
				idle = true; // Initial assumption: IDLE

				// the sample buffers are filled and ready for calculation
				DVector3 average_lvel = new DVector3(), average_avel = new DVector3();

				// Store first velocity samples
//				average_lvel.v[0] = bb.average_lvel_buffer[0].v[0];
//				average_avel.v[0] = bb.average_avel_buffer[0].v[0];
//				average_lvel.v[1] = bb.average_lvel_buffer[0].v[1];
//				average_avel.v[1] = bb.average_avel_buffer[0].v[1];
//				average_lvel.v[2] = bb.average_lvel_buffer[0].v[2];
//				average_avel.v[2] = bb.average_avel_buffer[0].v[2];
				average_lvel.set( bb.average_lvel_buffer[0] );
				average_avel.set( bb.average_avel_buffer[0] );

				// If we're not in "instantaneous mode"
				if ( bb.adis.average_samples > 1 )
				{
					// add remaining velocities together
					for ( int i = 1; i < bb.adis.average_samples; ++i )
					{
//						average_lvel.v[0] += bb.average_lvel_buffer[i].v[0];
//						average_avel.v[0] += bb.average_avel_buffer[i].v[0];
//						average_lvel.v[1] += bb.average_lvel_buffer[i].v[1];
//						average_avel.v[1] += bb.average_avel_buffer[i].v[1];
//						average_lvel.v[2] += bb.average_lvel_buffer[i].v[2];
//						average_avel.v[2] += bb.average_avel_buffer[i].v[2];
						average_lvel.add( bb.average_lvel_buffer[i] );
						average_avel.add( bb.average_avel_buffer[i] );
					}

					// make average
					double r1 = (double) 1.0  / (double) bb.adis.average_samples ;

//					average_lvel.v[0] *= r1;
//					average_avel.v[0] *= r1;
//					average_lvel.v[1] *= r1;
//					average_avel.v[1] *= r1;
//					average_lvel.v[2] *= r1;
//					average_avel.v[2] *= r1;
					average_lvel.scale( r1 );
					average_avel.scale( r1 );
				}

				// threshold test
				double av_lspeed, av_aspeed;
				av_lspeed = dDOT( average_lvel, average_lvel );
				if ( av_lspeed > bb.adis.linear_average_threshold )
				{
					idle = false; // average linear velocity is too high for idle
				}
				else
				{
					av_aspeed = dDOT( average_avel, average_avel );
					if ( av_aspeed > bb.adis.angular_average_threshold )
					{
						idle = false; // average angular velocity is too high for idle
					}
				}
			}

			// if it's idle, accumulate steps and time.
			// these counters won't overflow because this code doesn't run for disabled bodies.
			if (idle) {
				bb.adis_stepsleft--;
				bb.adis_timeleft -= stepsize;
			}
			else {
				// Reset countdowns
				bb.adis_stepsleft = bb.adis.idle_steps;
				bb.adis_timeleft = bb.adis.idle_time;
			}

			// disable the body if it's idle for a long enough time
			if ( bb.adis_stepsleft <= 0 && bb.adis_timeleft <= 0 )
			{
				bb.flags |= DxBody.dxBodyDisabled; // set the disable flag

				// disabling bodies should also include resetting the velocity
				// should prevent jittering in big "islands"
//				bb.lvel.v[0] = 0;
//				bb.lvel.v[1] = 0;
//				bb.lvel.v[2] = 0;
				bb.lvel.setValues(0);
//				bb.avel.v[0] = 0;
//				bb.avel.v[1] = 0;
//				bb.avel.v[2] = 0;
				bb.avel.setValues(0);
			}
		}
	}

	// ************************************************************
	// API dWorld
	// ************************************************************
	
	// intentionally undefined, don't use these
	//		  dWorld (const dWorld &);
	//		  void operator= (const dWorld &);
//
//	public dWorld()
//	{ _id = dxWorld.dWorldCreate(); }
	//~dWorld()
	@Override
	public void DESTRUCTOR()
//	{ dWorldDestroy (); super.DESTRUCTOR(); }
	{ super.DESTRUCTOR(); }

//	public dWorld id() 
//	{ return _id; }
	//TZ 		  operator dWorld() const
	//		    { return _id; }

	public void setGravity (double x, double y, double z)
	{ dWorldSetGravity (x,y,z); }
	public void setGravity (final DVector3 g)
	{ setGravity (g.v[0], g.v[1], g.v[2]); }
	public void getGravity (DVector3 g) 
	{ dWorldGetGravity (this,g); }

	public void setERP (double erp)
	{ dWorldSetERP(erp); }
	public double getERP() 
	{ return dWorldGetERP(this); }

	public void setCFM (double cfm)
	{ dWorldSetCFM(cfm); }
	public double getCFM() 
	{ return dWorldGetCFM(this); }

	public void step (double stepsize)
	{ dWorldStep (stepsize); }

	public void stepFast1 (double stepsize, int maxiterations)
	{ //TODO dWorldStepFast1 (this,stepsize,maxiterations); 
		throw new UnsupportedOperationException();
	}
	//TZ TODO report:		  void setAutoEnableDepthSF1(dWorld, int depth)
	public void setAutoEnableDepthSF1(int depth)
	{ //TODO dWorldSetAutoEnableDepthSF1 (this, depth); }
		throw new UnsupportedOperationException();
	}
	//TZ TODO report:		  int getAutoEnableDepthSF1(dWorld) 
	public int getAutoEnableDepthSF1() 
	{ //TODO return dWorldGetAutoEnableDepthSF1 (this); }
		throw new UnsupportedOperationException();
	}

	public void quickStep(double stepsize)
	{ dWorldQuickStep (stepsize); }
	public void setQuickStepNumIterations(int num)
	{ dWorldSetQuickStepNumIterations (num); }
	public int getQuickStepNumIterations() 
	{ return dWorldGetQuickStepNumIterations (this); }
	public void setQuickStepW(double over_relaxation)
	{ dWorldSetQuickStepW (this, over_relaxation); }
	public double getQuickStepW() 
	{ return dWorldGetQuickStepW (this); }

	public void  setAutoDisableLinearThreshold (double threshold) 
	{ dWorldSetAutoDisableLinearThreshold (this,threshold); }
	public double getAutoDisableLinearThreshold()
	{ return dWorldGetAutoDisableLinearThreshold (this); }
	public void setAutoDisableAngularThreshold (double threshold)
	{ dWorldSetAutoDisableAngularThreshold (this,threshold); }
	public double getAutoDisableAngularThreshold()
	{ return dWorldGetAutoDisableAngularThreshold (this); }
	public void setAutoDisableSteps (int steps)
	{ dWorldSetAutoDisableSteps (this,steps); }
	public int getAutoDisableSteps()
	{ return dWorldGetAutoDisableSteps (this); }
	public void setAutoDisableTime (double time)
	{ dWorldSetAutoDisableTime (this,time); }
	public double getAutoDisableTime() 
	{ return dWorldGetAutoDisableTime (this); }
	public void setAutoDisableFlag (boolean do_auto_disable)
	{ dWorldSetAutoDisableFlag (do_auto_disable); }
	public int getAutoDisableFlag() 
	{ return dWorldGetAutoDisableFlag (this); }

	public double getLinearDampingThreshold() 
	{ return dWorldGetLinearDampingThreshold(this); }
	public void setLinearDampingThreshold(double threshold)
	{ dWorldSetLinearDampingThreshold(this, threshold); }
	public double getAngularDampingThreshold() 
	{ return dWorldGetAngularDampingThreshold(this); }
	public void setAngularDampingThreshold(double threshold)
	{ dWorldSetAngularDampingThreshold(this, threshold); }
	public double getLinearDamping() 
	{ return dWorldGetLinearDamping(this); }
	public void setLinearDamping(double scale)
	{ dWorldSetLinearDamping(scale); }
	public double getAngularDamping() 
	{ return dWorldGetAngularDamping(); }
	public void setAngularDamping(double scale)
	{ dWorldSetAngularDamping(scale); }
	public void setDamping(double linear_scale, double angular_scale)
	{ dWorldSetDamping(this, linear_scale, angular_scale); }

	public 	double getMaxAngularSpeed() 
	{ return dWorldGetMaxAngularSpeed(this); }
	public void setMaxAngularSpeed(double max_speed)
	{ dWorldSetMaxAngularSpeed(max_speed); }

	public void setContactSurfaceLayer(double depth)
	{ dWorldSetContactSurfaceLayer (depth); }
	public double getContactSurfaceLayer() 
	{ return dWorldGetContactSurfaceLayer (this); }

	public void impulseToForce (double stepsize, double ix, double iy, double iz,
			DVector3 force)
	{ dWorldImpulseToForce (this,stepsize,ix,iy,iz,force); }


	@Override
	public void setAutoDisableAverageSamplesCount(int average_samples_count) {
		dWorldSetAutoDisableAverageSamplesCount(average_samples_count);
	}


	@Override
	public void setContactMaxCorrectingVel(double vel) {
		dWorldSetContactMaxCorrectingVel(vel);
	}


	@Override
	public void destroy() {
		dWorldDestroy();
	}
}
