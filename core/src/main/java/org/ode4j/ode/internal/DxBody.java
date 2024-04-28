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
package org.ode4j.ode.internal;

import static org.ode4j.ode.OdeMath.dAddVectorCross3;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply1_331;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.ode.OdeMath.dNormalize4;
import static org.ode4j.ode.OdeMath.dOrthogonalizeR;
import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.Matrix.dInvertPDMatrix;
import static org.ode4j.ode.internal.Rotation.dDQfromW;
import static org.ode4j.ode.internal.Rotation.dQMultiply0;
import static org.ode4j.ode.internal.Rotation.dQfromR;
import static org.ode4j.ode.internal.Rotation.dRfromQ;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.Objects_H.DxPosR;
import org.ode4j.ode.internal.Objects_H.DxPosRC;
import org.ode4j.ode.internal.Objects_H.dxAutoDisable;
import org.ode4j.ode.internal.Objects_H.dxDampingParameters;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;
import org.ode4j.ode.internal.processmem.DxWorldProcessContext;

import java.util.Iterator;

/**
 * rigid body (dynamics object).
 */
public class DxBody extends DObject implements DBody {

	// some body flags

	//	enum Body {
	//	  dxBodyFlagFiniteRotation     	(1),	// use finite rotations
	//	  dxBodyFlagFiniteRotationAxis 	(2),	// use finite rotations only along axis
	//	  dxBodyDisabled				(4),	// body is disabled
	//	  dxBodyNoGravity				(8),	// body is not influenced by gravity
	//	  dxBodyAutoDisable 			(16),	// enable auto-disable on body
	//	  dxBodyLinearDamping 			(32),   // using linear damping
	//	  dxBodyAngularDamping 			(64),   // using angular damping
	//	  dxBodyMaxAngularSpeed			(128);  // using maximum angular speed
	//	  private final int _i;
	//	  Body(int i) {
	//		  _i = i;
	//	  }
	//	};
	private static final int dxBodyFlagFiniteRotation = 1;	// use finite rotations
	private static final int dxBodyFlagFiniteRotationAxis= 2;	// use finite rotations only along axis
	static final int dxBodyDisabled				=  4;		// body is disabled
	static final int dxBodyNoGravity				=8;		// body is not influenced by gravity
	static final int dxBodyAutoDisable 			=16;	// enable auto-disable on body
	static final int dxBodyLinearDamping 			=32;   	// using linear damping
	static final int dxBodyAngularDamping 			=64;  	// use angular damping
	static final int dxBodyMaxAngularSpeed			=128;  	// use maximum angular speed
	private static final int dxBodyGyroscopic 				=256;	// use gyroscopic term


	//	  public dxJointNode firstjoint;	// list of attached joints
	//TODO
	public final Ref<DxJointNode> firstjoint = new Ref<>();	// list of attached joints
	//unsigned
	int flags;			// some dxBodyFlagXXX flags
	//  public dGeom geom;			// first collision geom associated with body
	public DxGeom geom;			// first collision geom associated with body
	DxMass mass;			// mass parameters about POR
	DMatrix3 invI;		// inverse of mass.I
	public double invMass;		// 1 / mass.mass
	public DxPosR _posr;			// position and orientation of point of reference
	public DQuaternion _q;		// orientation quaternion
	public DVector3 lvel;		// linear and angular velocity of POR
	public DVector3 avel;
	DVector3 facc,tacc;		// force and torque accumulators
	DVector3 finite_rot_axis;	// finite rotation axis, unit length or 0=none

	// auto-disable information
	final dxAutoDisable adis = new dxAutoDisable();		// auto-disable parameters
	double adis_timeleft;		// time left to be idle
	int adis_stepsleft;		// steps left to be idle
	//  dVector3* average_lvel_buffer;      // buffer for the linear average velocity calculation
	//  dVector3* average_avel_buffer;      // buffer for the angular average velocity calculation
	//  unsigned int average_counter;      // counter/index to fill the average-buffers
	DVector3[] average_lvel_buffer;      // buffer for the linear average velocity calculation
	DVector3[] average_avel_buffer;      // buffer for the angular average velocity calculation
	//unsigned
	int average_counter;      // counter/index to fill the average-buffers
	int average_ready;            // indicates ( with = 1 ), if the Body's buffers are ready for average-calculations

	BodyMoveCallBack moved_callback; // let the user know the body moved
	private final dxDampingParameters dampingp = new dxDampingParameters(); // damping parameters, depends on flags
	double max_angular_speed;      // limit the angular velocity to this magnitude

	protected DxBody(DxWorld w)
	{
		super(w);
	}


	DxWorld dBodyGetWorld ()
	{
		return world;
	}

	public static DxBody dBodyCreate (DxWorld w)
	{
		dAASSERT (w);
		DxBody b = new DxBody(w);
		b.firstjoint.set(null);
		b.flags = 0;
		b.geom = null;
		b.average_lvel_buffer = null;
		b.average_avel_buffer = null;
		//TZ
		b.mass = new DxMass();
		b.mass.dMassSetParameters (1, 0,0,0,1,1,1,0,0,0);
		b.invI = new DMatrix3();
		//MAT.dSetZero (b.invI.v,4*3);
		b.invI.set00( 1 );
		b.invI.set11( 1 );
		b.invI.set22( 1 );
		b.invMass = 1;
		b._posr = new DxPosR();
		//MAT.dSetZero (b.posr.pos.v,4);
		b._q = new DQuaternion();
		//MAT.dSetZero (b._q.v,4);
		b._q.set( 0, 1 );
		b._posr.Rw().setIdentity();
		b.lvel = new DVector3();
		//MAT.dSetZero (b.lvel.v,4);
		b.avel = new DVector3();
		//MAT.dSetZero (b.avel.v,4);
		b.facc = new DVector3();
		//MAT.dSetZero (b.facc.v,4);
		b.tacc = new DVector3();
		//MAT.dSetZero (b.tacc.v,4);
		b.finite_rot_axis = new DVector3();
		//MAT.dSetZero (b.finite_rot_axis.v,4);
		//addObjectToList (b,(dObject **) &w.firstbody);
		addObjectToList(b, w.firstbody);
		w.nb++;

		// set auto-disable parameters
		b.average_avel_buffer = b.average_lvel_buffer = null; // no buffer at beginning
		b.dBodySetAutoDisableDefaults ();	// must do this after adding to world
		b.adis_stepsleft = b.adis.idle_steps;
		b.adis_timeleft = b.adis.idle_time;
		b.average_counter = 0;
		b.average_ready = 0; // average buffer not filled on the beginning
		b.dBodySetAutoDisableAverageSamplesCount(b.adis.average_samples);

		b.moved_callback = null;

		b.dBodySetDampingDefaults();	// must do this after adding to world

		b.flags |= w.body_flags & dxBodyMaxAngularSpeed;
		b.max_angular_speed = w.max_angular_speed;

		b.flags |= dxBodyGyroscopic;

		return b;
	}


	//		public void dBodyDestroy (dxBody b)
	public void dBodyDestroy ()
	{
		//dAASSERT (b);
		// ode4j special: We set world=null, so we can check here
		if (world == null) {
			return; // already destroyed
		}

		// all geoms that link to this body must be notified that the body is about
		// to disappear. note that the call to dGeomSetBody(geom,0) will result in
		// dGeomGetBodyNext() returning 0 for the body, so we must get the next body
		// before setting the body to 0.
		DxGeom next_geom = null;
		for (DxGeom geom2 = geom; geom2 != null; geom2 = next_geom) {
			next_geom = geom2.dGeomGetBodyNext ();
			geom2.dGeomSetBody (null);
		}

		// detach all neighbouring joints, then delete this body.
		DxJointNode n = firstjoint.get();
		while (n != null) {
			// sneaky trick to speed up removal of joint references (black magic)
			//TODO use equals?
			n.joint.node[(n == n.joint.node[0])?1:0].body = null;

			DxJointNode next = n.next;
			n.next = null;
			n.joint.removeJointReferencesFromAttachedBodies ();
			n = next;
		}
		removeObjectFromList ();
		world.nb--;

		// ode4j special: We set world=null, so we can check precondition in destroy()
		world = null;

		// delete the average buffers
		//TZ nothing to do
		//			if(average_lvel_buffer!= null)
		//			{
		//				delete[] (average_lvel_buffer);
		//				average_lvel_buffer = null;
		//			}
		//			if(average_avel_buffer!= null)
		//			{
		//				delete[] (average_avel_buffer);
		//				average_avel_buffer = null;
		//			}
		//
		//			delete b;
		DESTRUCTOR();
	}


	//public void dBodySetData (dxBody b, Object data)
	public void dBodySetData (Object data)
	{
		userdata = data;
	}


	public Object dBodyGetData ()
	{
		return userdata;
	}


	public void dBodySetPosition (double x, double y, double z)
	{
		_posr.pos.set(x, y, z);

		// notify all attached geoms that this body has moved
		for (DxGeom geom2 = geom; geom2 != null; geom2 = geom2.dGeomGetBodyNext ())
			geom2.dGeomMoved ();
	}

	//TZ
	public void dBodySetPosition (DVector3C xyz)
	{
		_posr.pos.set(xyz);

		// notify all attached geoms that this body has moved
		for (DxGeom geom2 = geom; geom2 != null; geom2 = geom2.dGeomGetBodyNext ())
			geom2.dGeomMoved ();
	}


	//		public void dBodySetRotation (dxBody b, final dMatrix3 R)
	public void dBodySetRotation (DMatrix3C R)
	{
		//memcpy(b->posr.R, R, sizeof(dMatrix3));
		_posr.Rw().set(R);

		boolean bOrthogonalizeResult = dOrthogonalizeR(_posr.Rw());
		dAVERIFY(bOrthogonalizeResult);

		dQfromR(_q, R);
		dNormalize4(_q);
		  
		// notify all attached geoms that this body has moved
		for (DxGeom geom2 = geom; geom2 != null; geom2 = geom2.dGeomGetBodyNext ()) {
			geom2.dGeomMoved ();
		}
	}


	//		void dBodySetQuaternion (dxBody b, final dQuaternion q)
	public void dBodySetQuaternion (DQuaternionC q)
	{
		//dAASSERT (q);
		//			_q.v[0] = q.v[0];
		//			_q.v[1] = q.v[1];
		//			_q.v[2] = q.v[2];
		//			_q.v[3] = q.v[3];
		_q.set(q);
		dNormalize4 (_q);
		//dQtoR (b.q, b.posr.R);
		dRfromQ(_posr.Rw(), _q);

		// notify all attached geoms that this body has moved
		for (DxGeom geom2 = geom; geom2 != null; geom2 = geom2.dGeomGetBodyNext ())
			geom2.dGeomMoved ();
	}


	public void dBodySetLinearVel  (double x, double y, double z)
	{
		lvel.set(x, y, z);
	}
	public void dBodySetLinearVel  (DVector3C xyz)
	{
		lvel.set(xyz);
	}


	public void dBodySetAngularVel (double x, double y, double z)
	{
		avel.set(x, y, z);
	}
	public void dBodySetAngularVel (DVector3C xyz)
	{
		avel.set(xyz);
	}


	//		public final double[] dBodyGetPosition (dxBody b)
	public DVector3C dBodyGetPosition ()
	{
		return _posr.pos();
	}


	void dBodyCopyPosition (DxBody b, DVector3 pos)
	{
		//			dAASSERT (b);
		//double[] src = b.posr.pos.v;
		//			pos.v[0] = src[0];
		//			pos.v[1] = src[1];
		//			pos.v[2] = src[2];
		pos.set(b._posr.pos());
	}


	//		public final double[] dBodyGetRotation (dxBody b)
	public DMatrix3C dBodyGetRotation ()
	{
		return _posr.R();
	}


	void dBodyCopyRotation (DxBody b, DMatrix3 R)
	{
		//			dAASSERT (b);
		//			final double[] src = b.posr.R.v;
		//			R.v[0] = src[0];
		//			R.v[1] = src[1];
		//			R.v[2] = src[2];
		//			R.v[3] = src[3];
		//			R.v[4] = src[4];
		//			R.v[5] = src[5];
		//			R.v[6] = src[6];
		//			R.v[7] = src[7];
		//			R.v[8] = src[8];
		//			R.v[9] = src[9];
		//			R.v[10] = src[10];
		//			R.v[11] = src[11];
		//TODO clean up commented code and other TODO below (add(f), e.t.c) 
		R.set(b._posr.R());
	}


	//		final double[] dBodyGetQuaternion (dxBody b)
	public DQuaternionC dBodyGetQuaternion ()
	{
		return _q;
	}


	void dBodyCopyQuaternion (DxBody b, DQuaternion quat)
	{
		quat.set(b._q);
	}


	//		final double[] dBodyGetLinearVel (dxBody b)
	public DVector3C dBodyGetLinearVel ()
	{
		return lvel;
	}


	//		final double[] dBodyGetAngularVel (dxBody b)
	public DVector3C dBodyGetAngularVel ()
	{
		return avel;
	}


	//void dBodySetMass (dxBody b, final dMass mass)
	public void dBodySetMass (DMassC mass2)
	{
		//dAASSERT (mass2 );
		dIASSERT(mass2.check());

		// The centre of mass must be at the origin.
		// Use dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] ) 
		// to correct it.
		DVector3C mass2c = mass2.getC();
		dUASSERT( Math.abs( mass2c.get0() ) <= DBL_EPSILON &&
				Math.abs( mass2c.get1() ) <= DBL_EPSILON &&
				Math.abs( mass2c.get2() ) <= DBL_EPSILON, 
		"The centre of mass must be at the origin." );

		//memcpy (b.mass,mass,sizeof(dMass));
		mass.set(mass2);
		if (!dInvertPDMatrix (mass._I, invI)) {
			dDEBUGMSG ("inertia must be positive definite!");
			invI.setIdentity();
		}
		invMass = dRecip(mass._mass);
	}


	public void dBodyGetMass (DxMass mass2)
	//void dBodyGetMass (dxBody b, dMass *mass)
	{
		mass2.set(mass);
		//memcpy (mass,b.mass,sizeof(dMass));
	}


	//		public void dBodyAddForce (dxBody b, double fx, double fy, double fz)
	public void dBodyAddForce (double fx, double fy, double fz)
	{
		facc.add(fx, fy, fz);
	}
	public void dBodyAddForce (DVector3C f)
	{
		facc.add(f);
	}


	//		public void dBodyAddTorque (dxBody b, double fx, double fy, double fz)
	public void dBodyAddTorque (double fx, double fy, double fz)
	{
		tacc.add(fx, fy, fz);
	}
	public void dBodyAddTorque (DVector3C f)
	{
		tacc.add(f);
	}


	void dBodyAddRelForce (DVector3C f)
	{
		DVector3 t2 = new DVector3();
		dMultiply0_331 (t2,_posr.R(),f);
		facc.add(t2);
	}


	//	void dBodyAddRelTorque (dxBody b, double fx, double fy, double fz)
	public void dBodyAddRelTorque (DVector3C f)
	{
		DVector3 t2 = new DVector3();
		dMultiply0_331 (t2,_posr.R(),f);
		tacc.add(t2);
	}


	void dBodyAddForceAtPos (DVector3C f, DVector3C p)
	{
		facc.add(f);
		DVector3 q = p.reSub(_posr.pos());
		dAddVectorCross3 (tacc,q,f);
	}


	void dBodyAddForceAtRelPos (DVector3C f, DVector3C prel)
	{
		DVector3 p = new DVector3();
		dMultiply0_331 (p,_posr.R(),prel);
		facc.add(f);
		dAddVectorCross3 (tacc,p,f);
	}


	void dBodyAddRelForceAtPos (DVector3C frel, DVector3C p)
	{
		DVector3 f = new DVector3();
		dMultiply0_331 (f,_posr.R(),frel);
		facc.add(f);
		DVector3 q = p.reSub(_posr.pos());
		dAddVectorCross3 (tacc,q,f);
	}


	//	void dBodyAddRelForceAtRelPos (dxBody b, double fx, double fy, double fz,
	//	double px, double py, double pz)
	void dBodyAddRelForceAtRelPos (DVector3C fRel, DVector3C pRel)
	{
		//		dVector3 frel = new dVector3(fx, fy, fz);
		//		dVector3 prel = new dVector3(px, py, pz);
		DVector3 f = new DVector3();
		DVector3 p = new DVector3();
		//		frel.v[0] = fx;
		//		frel.v[1] = fy;
		//		frel.v[2] = fz;
		//		frel.v[3] = 0;
		//		prel.v[0] = px;
		//		prel.v[1] = py;
		//		prel.v[2] = pz;
		//		prel.v[3] = 0;
		dMultiply0_331 (f,_posr.R(),fRel);
		dMultiply0_331 (p,_posr.R(),pRel);
		//		b.facc.v[0] += f.v[0];
		//		b.facc.v[1] += f.v[1];
		//		b.facc.v[2] += f.v[2];
		facc.add(f);
		dAddVectorCross3 (tacc,p,f);
	}


	DVector3C dBodyGetForce ()
	{
		//dAASSERT (b);
		return facc;
	}


	DVector3C dBodyGetTorque ()
	{
		return tacc;
	}


	void dBodySetForce (double x, double y, double z)
	{
		facc.set(x, y, z);
	}
	void dBodySetForce (DVector3C xyz)
	{
		facc.set(xyz);
	}


	void dBodySetTorque (double x, double y, double z)
	{
		tacc.set(x, y, z);
	}
	void dBodySetTorque (DVector3C t)
	{
		tacc.set(t);
	}


	//	void dBodyGetRelPointPos (dxBody b, double px, double py, double pz,
	//			dVector3 result)
	void dBodyGetRelPointPos (DVector3C prel, DVector3 result)
	{
		//dVector3 prel = new dVector3(px, py, pz);
		//		dVector3 p = new dVector3();
		//	  prel.v[0] = px;
		//	  prel.v[1] = py;
		//	  prel.v[2] = pz;
		//	  prel.v[3] = 0;
		//		dMULTIPLY0_331 (p,b._posr.R,prel);
		//		result.v[0] = p.v[0] + b._posr.pos.v[0];
		//		result.v[1] = p.v[1] + b._posr.pos.v[1];
		//		result.v[2] = p.v[2] + b._posr.pos.v[2];
		//		result.sum(p, b._posr.pos);
		dMultiply0_331 (result,_posr.R(),prel);
		result.add( _posr.pos() );
	}


	//	public void dBodyGetRelPointVel (double px, double py, double pz,
	//			dVector3 result)
	public void dBodyGetRelPointVel (DVector3C prel, DVector3 result)
	{
		//dVector3 prel = new dVector3(px, py, pz);
		DVector3 p = new DVector3();
		dMultiply0_331 (p,_posr.R(),prel);
		result.set(lvel);
		dAddVectorCross3 (result,avel,p);
	}


	//	void dBodyGetPointVel (dxBody b, double px, double py, double pz,
	//			dVector3 result)
	void dBodyGetPointVel (DVector3C prel, DVector3 result)
	{
		DVector3 p = new DVector3(prel).sub(_posr.pos());
		result.set(lvel);
		dAddVectorCross3 (result,avel,p);
	}


	//	void dBodyGetPosRelPoint (dxBody b, double px, double py, double pz,
	//			dVector3 result)
	void dBodyGetPosRelPoint (DVector3C p, DVector3 result)
	{
		DVector3 prel = p.reSub(_posr.pos());
		dMultiply1_331 (result,_posr.R(),prel);
	}


	//	void dBodyVectorToWorld (dxBody b, double px, double py, double pz,
	//			dVector3 result)
	void dBodyVectorToWorld (DVector3C p, DVector3 result)
	{
		dMultiply0_331 (result,_posr.R(),p);
	}


	//	void dBodyVectorFromWorld (dxBody b, double px, double py, double pz,
	//			dVector3 result)
	void dBodyVectorFromWorld (DVector3C p, DVector3 result)
	{
		dMultiply1_331 (result,_posr.R(),p);
	}


	//	void dBodySetFiniteRotationMode (dxBody b, int mode)
	void dBodySetFiniteRotationMode (boolean mode)
	{
		flags &= ~(dxBodyFlagFiniteRotation | dxBodyFlagFiniteRotationAxis);
		if (mode) {
			flags |= dxBodyFlagFiniteRotation;
			if (finite_rot_axis.get0() != 0 || finite_rot_axis.get1() != 0 ||
					finite_rot_axis.get2() != 0) {
				flags |= dxBodyFlagFiniteRotationAxis;
			}
		}
	}


	//	void dBodySetFiniteRotationAxis (dxBody b, double x, double y, double z)
	void dBodySetFiniteRotationAxis (DVector3C xyz)
	{
		finite_rot_axis.set(xyz);
		if (xyz.get0() != 0 || xyz.get1() != 0 || xyz.get2() != 0) {
			dNormalize3 (finite_rot_axis);
			flags |= dxBodyFlagFiniteRotationAxis;
		}
		else {
			flags &= ~dxBodyFlagFiniteRotationAxis;
		}
	}


	boolean dBodyGetFiniteRotationMode ()
	{
		return (flags & dxBodyFlagFiniteRotation) != 0;
	}


	void dBodyGetFiniteRotationAxis (DVector3 result)
	{
		result.set(finite_rot_axis);
	}


	int dBodyGetNumJoints ()
	{
		int count=0;
		//TODO return array size
		for (DxJointNode n=firstjoint.get(); n != null; n=n.next, count++);
		return count;
	}


	DxJoint dBodyGetJoint (int index)
	{
		int i=0;
		//TODO return array size
		for (DxJointNode n=firstjoint.get(); n != null; n=n.next, i++) {
			if (i == index) return n.joint;
		}
		return null;
	}


	void dBodySetDynamic ()
	{
	  dBodySetMass(mass);
	}

	void dBodySetKinematic ()
	{
		//dSetZero (b->invI,4*3);
		invI.setZero();
		invMass = 0; 
	}

	boolean dBodyIsKinematic ()
	{
	  return invMass == 0;
	}

	//	void dBodyEnable (dxBody b)
    public void dBodyEnable ()
    {
        flags &= ~dxBodyDisabled;
        adis_stepsleft = adis.idle_steps;
        adis_timeleft = adis.idle_time;
        // no code for average-processing needed here
    }

    /**
     * flags &amp;= ~dxBodyDisabled.
     */
    //(TZ)
    public void dBodyEnable_noAdis ()
    {
        flags &= ~dxBodyDisabled;
    }


	//	void dBodyDisable (dxBody b)
	public void dBodyDisable ()
	{
		flags |= dxBodyDisabled;
	}


	/**
	 * @return (flags &amp; dxBodyDisabled) == 0
	 */
	//	public boolean dBodyIsEnabled (dxBody b)
	public boolean dBodyIsEnabled ()
	{
		return ((flags & dxBodyDisabled) == 0);
	}


	//	void dBodySetGravityMode (dxBody b, int mode)
	void dBodySetGravityMode (boolean mode)
	{
		if (mode) flags &= ~dxBodyNoGravity;
		else flags |= dxBodyNoGravity;
	}


	/**
	 * @return (flags & dxBodyNoGravity) == 0
	 */
	boolean dBodyGetGravityMode ()
	{
		return (flags & dxBodyNoGravity) == 0;
	}


	// body auto-disable functions

	double dBodyGetAutoDisableLinearThreshold ()
	{
		return dSqrt (adis.linear_average_threshold);
	}


	void dBodySetAutoDisableLinearThreshold (double linear_average_threshold)
	{
		adis.linear_average_threshold = linear_average_threshold * linear_average_threshold;
	}


	double dBodyGetAutoDisableAngularThreshold ()
	{
		return dSqrt (adis.angular_average_threshold);
	}


	void dBodySetAutoDisableAngularThreshold (double angular_average_threshold)
	{
		adis.angular_average_threshold = angular_average_threshold * angular_average_threshold;
	}


	int dBodyGetAutoDisableAverageSamplesCount ()
	{
		return adis.average_samples;
	}


	//	void dBodySetAutoDisableAverageSamplesCount (dxBody b, unsigned int average_samples_count)
	void dBodySetAutoDisableAverageSamplesCount (int average_samples_count)
	{
		adis.average_samples = average_samples_count;
		// update the average buffers
		if(average_lvel_buffer != null)
		{
			//TZ			delete[] b.average_lvel_buffer;
			average_lvel_buffer = null;
		}
		if(average_avel_buffer != null)
		{
			//TZ			delete[] b.average_avel_buffer;
			average_avel_buffer = null;
		}
		if(adis.average_samples > 0)
		{
			average_lvel_buffer = DVector3.newArray(adis.average_samples);
			average_avel_buffer = DVector3.newArray(adis.average_samples);
		}
		else
		{
			average_lvel_buffer = null;
			average_avel_buffer = null;
		}
		// new buffer is empty
		average_counter = 0;
		average_ready = 0;
	}


	int dBodyGetAutoDisableSteps ()
	{
		return adis.idle_steps;
	}


	void dBodySetAutoDisableSteps (int steps)
	{
		adis.idle_steps = steps;
	}


	double dBodyGetAutoDisableTime ()
	{
		return adis.idle_time;
	}


	void dBodySetAutoDisableTime (double time)
	{
		adis.idle_time = time;
	}


	boolean dBodyGetAutoDisableFlag ()
	{
		return ((flags & dxBodyAutoDisable) != 0);
	}


	//	void dBodySetAutoDisableFlag (dxBody b, int do_auto_disable)
	void dBodySetAutoDisableFlag (boolean do_auto_disable)
	{
		if (!do_auto_disable)
		{
			flags &= ~dxBodyAutoDisable;
			// (mg) we should also reset the IsDisabled state to correspond to the DoDisabling flag
			flags &= ~dxBodyDisabled;
			adis.idle_steps = world.getAutoDisableSteps();
			adis.idle_time = world.getAutoDisableTime();
			// resetting the average calculations too
			dBodySetAutoDisableAverageSamplesCount( 
					world.getAutoDisableAverageSamplesCount() );
		}
		else
		{
			flags |= dxBodyAutoDisable;
		}
	}


	void dBodySetAutoDisableDefaults ()
	{
		DxWorld w = world;
		adis.set(w.adis);
		dBodySetAutoDisableFlag ( (w.body_flags & dxBodyAutoDisable)!=0);
	}


	// body damping functions

	double dBodyGetLinearDamping()
	{
		return dampingp.linear_scale;
	}

	void dBodySetLinearDamping(double scale)
	{
		if (scale != 0)
			flags |= dxBodyLinearDamping;
		else
			flags &= ~dxBodyLinearDamping;
		dampingp.linear_scale = scale;
	}

	double dBodyGetAngularDamping()
	{
		return dampingp.angular_scale;
	}

	void dBodySetAngularDamping(double scale)
	{
		if (scale != 0)
			flags |= dxBodyAngularDamping;
		else
			flags &= ~dxBodyAngularDamping;
		dampingp.angular_scale = scale;
	}

	void dBodySetDamping(double linear_scale, double angular_scale)
	{
		dBodySetLinearDamping(linear_scale);
		dBodySetAngularDamping(angular_scale);
	}

	double dBodyGetLinearDampingThreshold()
	{
		return dSqrt(dampingp.linear_threshold);
	}

	void dBodySetLinearDampingThreshold(double threshold)
	{
		dampingp.linear_threshold = threshold*threshold;
	}


	double dBodyGetAngularDampingThreshold()
	{
		return dSqrt(dampingp.angular_threshold);
	}

	void dBodySetAngularDampingThreshold(double threshold)
	{
		dampingp.angular_threshold = threshold*threshold;
	}

	void dBodySetDampingDefaults()
	{
		DxWorld w = world;
		dampingp.set(w.dampingp);
		//unsigned
		final int mask = dxBodyLinearDamping | dxBodyAngularDamping;
		flags &= ~mask; // zero them
		flags |= w.body_flags & mask;
	}

	double dBodyGetMaxAngularSpeed()
	{
		return max_angular_speed;
	}

	void dBodySetMaxAngularSpeed(double max_speed)
	{
		if (max_speed < Double.MAX_VALUE)
			flags |= dxBodyMaxAngularSpeed;
		else
			flags &= ~dxBodyMaxAngularSpeed;
		max_angular_speed = max_speed;
	}

	//	void dBodySetMovedCallback(dxBody b, void (*callback)(dxBody))
	public void dBodySetMovedCallback(BodyMoveCallBack callback)
	{
		moved_callback = callback;
	}


	//		dxGeom dBodyGetFirstGeom(dxBody b)
	DxGeom dBodyGetFirstGeom()
	{
		//			dAASSERT(b);
		return geom;
	}

	//TODO check calls for invalid geom (not of this body)
	DxGeom dBodyGetNextGeom(DxGeom geom)
	{
		//dAASSERT(geom);
		return geom.dGeomGetBodyNext();
	}

	boolean dBodyGetGyroscopicMode()
	{
	        return (flags & dxBodyGyroscopic) != 0;
	}

	void dBodySetGyroscopicMode(boolean enabled)
	{
	        if (enabled)
	                flags |= dxBodyGyroscopic;
	        else
	                flags &= ~dxBodyGyroscopic;
	}


	//****************************************************************************
	// body rotation

	// return sin(x)/x. this has a singularity at 0 so special handling is needed
	// for small arguments.

	private static double sinc (double x)
	{
		// if |x| < 1e-4 then use a taylor series expansion. this two term expansion
		// is actually accurate to one LS bit within this range if double precision
		// is being used - so don't worry!
		if (dFabs(x) < 1.0e-4) return 1.0 - x*x*0.166666666666666666667;
		else return dSin(x)/x;
	}


	// given a body b, apply its linear and angular rotation over the time
	// interval h, thereby adjusting its position and orientation.

	void dxStepBody (double h)
	{
		// cap the angular velocity
		if ((flags & dxBodyMaxAngularSpeed) != 0) {
			final double max_ang_speed = max_angular_speed;
			final double aspeed = dCalcVectorDot3( avel, avel );
			if (aspeed > max_ang_speed*max_ang_speed) {
				final double coef = max_ang_speed/dSqrt(aspeed);
				avel.scale(coef);//dOPEC(avel.v, OP.MUL_EQ /* *= */, coef);
			}
		}
		// end of angular velocity cap

		// handle linear velocity
		//for (j=0; j<3; j++) _posr.pos.v[j] += h * lvel.v[j];
		_posr.pos.eqSum(_posr.pos(), lvel, h);

		if ((flags & dxBodyFlagFiniteRotation) != 0) {
			DVector3 irv = new DVector3();	// infitesimal rotation vector
			DQuaternion q = new DQuaternion();	// quaternion for finite rotation

			if ((flags & dxBodyFlagFiniteRotationAxis) != 0) {
				// split the angular velocity vector into a component along the finite
				// rotation axis, and a component orthogonal to it.
				DVector3 frv = new DVector3();		// finite rotation vector
				double k = dCalcVectorDot3 (finite_rot_axis,avel);
				//				frv.v[0] = finite_rot_axis.v[0] * k;
				//				frv.v[1] = finite_rot_axis.v[1] * k;
				//				frv.v[2] = finite_rot_axis.v[2] * k;
				frv.set( finite_rot_axis ).scale( k );
				//				irv.v[0] = avel.v[0] - frv.v[0];
				//				irv.v[1] = avel.v[1] - frv.v[1];
				//				irv.v[2] = avel.v[2] - frv.v[2];
				irv.eqDiff(avel, frv);

				// make a rotation quaternion q that corresponds to frv * h.
				// compare this with the full-finite-rotation case below.
				h *= 0.5;
				double theta = k * h;
				double s = sinc(theta) * h;
				//				q.v[0] = dCos(theta);
				//				q.v[1] = frv.v[0] * s;
				//				q.v[2] = frv.v[1] * s;
				//				q.v[3] = frv.v[2] * s;
				q.set( dCos(theta), frv.get0()*s, frv.get1()*s, frv.get2()*s);
			}
			else {
				// make a rotation quaternion q that corresponds to w * h
				double wlen = avel.length();//dSqrt (avel.v[0]*avel.v[0] + avel.v[1]*avel.v[1] +
				//avel.v[2]*avel.v[2]);
				h *= 0.5;
				double theta = wlen * h;
				double s = sinc(theta) * h;
				//				q.v[0] = dCos(theta);
				//				q.v[1] = avel.v[0] * s;
				//				q.v[2] = avel.v[1] * s;
				//				q.v[3] = avel.v[2] * s;
				q.set( dCos(theta), avel.get0()*s, avel.get1()*s, avel.get2()*s);
			}

			// do the finite rotation
			DQuaternion q2 = new DQuaternion();
			dQMultiply0 (q2,q,_q);
			//for (j=0; j<4; j++) _q.v[j] = q2.v[j];
			_q.set(q2);

			// do the infitesimal rotation if required
			if ((flags & dxBodyFlagFiniteRotationAxis) != 0) {
				DQuaternion dq = new DQuaternion();
				dDQfromW (dq,irv,_q);
				//for (j=0; j<4; j++) _q.v[j] += h * dq[j];
				_q.sum( _q, dq, h);
			}
		}
		else {
			// the normal way - do an infitesimal rotation
			DQuaternion dq = new DQuaternion();
			dDQfromW (dq,avel,_q);
			//for (j=0; j<4; j++) _q.v[j] += h * dq[j];
			_q.sum( _q, dq, h);
		}

		// normalize the quaternion and convert it to a rotation matrix
		dNormalize4 (_q);
		dRfromQ (_posr.Rw(),_q);


		// notify all attached geoms that this body has moved
	    DxWorldProcessContext world_process_context = world.UnsafeGetWorldProcessingContext(); 
	    for (DxGeom geom2 = geom; geom2 != null; geom2 = geom2.dGeomGetBodyNext ()) {
	        world_process_context.LockForStepbodySerialization();
	        geom2.dGeomMoved ();
	        world_process_context.UnlockForStepbodySerialization();
	    }

		// notify the user
		if (moved_callback != null)
			moved_callback.run(this);


		// damping
		if ((flags & dxBodyLinearDamping)!=0) {
			final double lin_threshold = dampingp.linear_threshold;
			final double lin_speed = dCalcVectorDot3( lvel, lvel );
			if ( lin_speed > lin_threshold) {
				final double k = 1 - dampingp.linear_scale;
				lvel.scale(k);//dOPEC(lvel.v, OP.MUL_EQ, k);
			}
		}
		if ((flags & dxBodyAngularDamping)!=0) {
			final double ang_threshold = dampingp.angular_threshold;
			final double ang_speed = dCalcVectorDot3( avel, avel );
			if ( ang_speed > ang_threshold) {
				final double k = 1 - dampingp.angular_scale;
				avel.scale(k);//dOPEC(avel.v, OP.MUL_EQ, k);
			}
		}
	}

	@Override
	public String toString() {
		return super.toString();  
	}

	public DxPosRC posr() {
		return _posr;
	}
	
	
	// ******************************************************
	// dBody API
	// ******************************************************

	//~dBody()
	@Override
	//	  public void DESTRUCTOR()
	//	    { dBodyDestroy (); super.DESTRUCTOR(); }
	public void DESTRUCTOR() { super.DESTRUCTOR(); }

	//void setData (void *data)
	@Override
	public void setData (Object data)
	{ dBodySetData (data); }
	//void *getData() 
	@Override
	public Object getData() 
	{ return dBodyGetData (); }

	@Override
	public void setPosition (double x, double y, double z)
	{ dBodySetPosition (x,y,z); }
	@Override
	public void setPosition (DVector3C p)
	{ dBodySetPosition(p); }

	@Override
	public void setRotation (DMatrix3C R)
	{ dBodySetRotation (R); }
	@Override
	public void setQuaternion (DQuaternionC q)
	{ dBodySetQuaternion (q); }
	@Override
	public void setLinearVel (double x, double y, double z)
	{ dBodySetLinearVel (x,y,z); }
	@Override
	public void setLinearVel (DVector3C v)
	{ dBodySetLinearVel(v); }
	@Override
	public void addLinearVel (double x, double y, double z)	{
		lvel.add(x, y, z);
	}
	@Override
	public void addLinearVel (DVector3C v)
	{ lvel.add(v); }
	@Override
	public void setAngularVel (double x, double y, double z)
	{ dBodySetAngularVel (x,y,z); }
	@Override
	public void setAngularVel (DVector3C v)
	{ dBodySetAngularVel (v); }

	@Override
	public DVector3C getPosition() 
	{ return dBodyGetPosition (); }
	@Override
	public DMatrix3C getRotation() //const
	{ return dBodyGetRotation (); }
	@Override
	public DQuaternionC getQuaternion() //const
	{ return dBodyGetQuaternion (); }
	@Override
	public DVector3C getLinearVel() //const
	{ return dBodyGetLinearVel (); }
	@Override
	public DVector3C getAngularVel() //const
	{ return dBodyGetAngularVel (); }

	//  void setMass (final dMass *mass)
	//  { dBodySetMass (_id,mass); }
	//void setMass (final dMass &mass)
	//  { setMass (&mass); }
	@Override
	public void setMass (DMassC mass)
	{ dBodySetMass (mass); }
	@Override
	public DMass getMass () //const
	{ DMass mass = new DxMass(); dBodyGetMass ((DxMass) mass); return mass; }

	@Override
	public void addForce (double fx, double fy, double fz)
	{ dBodyAddForce (fx, fy, fz); }
	@Override
	public void addForce (DVector3C f)
	{ dBodyAddForce (f); }
	@Override
	public void addTorque (double fx, double fy, double fz)
	{ dBodyAddTorque (fx, fy, fz); }
	@Override
	public void addTorque (DVector3C t)
	{ dBodyAddTorque(t); }

	@Override
	public void addRelForce (double fx, double fy, double fz)
	{ dBodyAddRelForce ( new DVector3(fx, fy, fz)); }
	@Override
	public void addRelForce (DVector3C f)
	{ dBodyAddRelForce (f); }
	@Override
	public void addRelTorque (double fx, double fy, double fz)
	{ dBodyAddRelTorque (new DVector3(fx, fy, fz)); }
	@Override
	public void addRelTorque (DVector3C t)
	{ dBodyAddRelTorque (t); }

	@Override
	public void addForceAtPos (double fx, double fy, double fz,
			double px, double py, double pz)
	{ dBodyAddForceAtPos (new DVector3(fx, fy, fz), new DVector3(px, py, pz)); }
	@Override
	public void addForceAtPos (DVector3C f, DVector3C p)
	{ dBodyAddForceAtPos (f,p); }

	@Override
	public void addForceAtRelPos (double fx, double fy, double fz,
			double px, double py, double pz)
	{ dBodyAddForceAtRelPos (new DVector3(fx, fy, fz), new DVector3(px, py, pz)); }
	@Override
	public void addForceAtRelPos (DVector3C f, DVector3C p)
	{ dBodyAddForceAtRelPos (f, p); }

	@Override
	public void addRelForceAtPos (double fx, double fy, double fz,
			double px, double py, double pz)
	{ dBodyAddRelForceAtPos (new DVector3(fx, fy, fz), new DVector3(px, py, pz)); }
	@Override
	public void addRelForceAtPos (DVector3C f, DVector3C p)
	{ dBodyAddRelForceAtPos (f, p); }

	@Override
	public void addRelForceAtRelPos (double fx, double fy, double fz,
			double px, double py, double pz)
	{ dBodyAddRelForceAtRelPos (new DVector3(fx, fy, fz), new DVector3(px, py, pz)); }
	@Override
	public void addRelForceAtRelPos (DVector3C f, DVector3C p)
	{ dBodyAddRelForceAtRelPos (f, p); }

	@Override
	public DVector3C getForce() //const
	{ return dBodyGetForce(); }
	@Override
	public DVector3C getTorque() //const
	{ return dBodyGetTorque(); }
	@Override
	public void setForce (double x, double y, double z)
	{ dBodySetForce (x,y,z); }
	@Override
	public void setForce (DVector3C f)
	{ dBodySetForce (f); }
	@Override
	public void setTorque (double x, double y, double z)
	{ dBodySetTorque (x,y,z); }
	@Override
	public void setTorque (DVector3C t)
	{ dBodySetTorque (t); }

	@Override
	public void setDynamic()
	{ dBodySetDynamic (); }
	@Override
	public void setKinematic()
	{ dBodySetKinematic (); }
	@Override
	public boolean isKinematic()
	{ return dBodyIsKinematic (); }

	@Override
	public void enable()
	{ dBodyEnable (); }
	@Override
	public void disable()
	{ dBodyDisable (); }
	@Override
	public boolean isEnabled() //const
	{ return dBodyIsEnabled (); }

	//TZ
	public boolean isFlagsGyroscopic() {
		return (flags & dxBodyGyroscopic) != 0;
	}

	@Override
	public void getRelPointPos (double px, double py, double pz, DVector3 result) //const
	{ dBodyGetRelPointPos (new DVector3(px, py, pz), result); }
	@Override
	public void getRelPointPos (DVector3C p, DVector3 result) //const
	{ dBodyGetRelPointPos (p, result); }

	@Override
	public void getRelPointVel (double px, double py, double pz, DVector3 result) //const
	{ dBodyGetRelPointVel (new DVector3(px, py, pz), result); }
	@Override
	public void getRelPointVel (DVector3C p, DVector3 result) //const
	{ dBodyGetRelPointVel (p, result); }

	@Override
	public void getPointVel (double px, double py, double pz, DVector3 result) //const
	{ dBodyGetPointVel ( new DVector3(px, py, pz), result); }
	@Override
	public void getPointVel (DVector3C p, DVector3 result) //const
	{ dBodyGetPointVel (p, result); }

	@Override
	public void getPosRelPoint (double px, double py, double pz, DVector3 result) //const
	{ dBodyGetPosRelPoint ( new DVector3(px, py, pz), result); }
	@Override
	public void getPosRelPoint (DVector3C p, DVector3 result) //const
	{ dBodyGetPosRelPoint (p, result); }

	@Override
	public void vectorToWorld (double px, double py, double pz, DVector3 result) //const
	{ dBodyVectorToWorld ( new DVector3(px, py, pz), result); }
	@Override
	public void vectorToWorld (DVector3C p, DVector3 result) //const
	{ dBodyVectorToWorld (p, result); }

	@Override
	public void vectorFromWorld (double px, double py, double pz, DVector3 result) //const
	{ dBodyVectorFromWorld (new DVector3(px,py,pz),result); }
	@Override
	public void vectorFromWorld (DVector3C p, DVector3 result) //const
	{ dBodyVectorFromWorld (p, result); }

	@Override
	public void setFiniteRotationMode (boolean mode)
	{ dBodySetFiniteRotationMode (mode); }

	@Override
	public void setFiniteRotationAxis (double x, double y, double z)
	{ dBodySetFiniteRotationAxis( new DVector3(x, y, z)); }
	@Override
	public void setFiniteRotationAxis (DVector3C a)
	{ dBodySetFiniteRotationAxis (a); }

	@Override
	public boolean getFiniteRotationMode() //const
	{ return dBodyGetFiniteRotationMode (); }
	@Override
	public void getFiniteRotationAxis (DVector3 result) //const
	{ dBodyGetFiniteRotationAxis ( result); }

	@Override
	public int getNumJoints() //const
	{ return dBodyGetNumJoints (); }
	@Override
	public DJoint getJoint (int index) //const
	{ return dBodyGetJoint ( index); }

	@Override
	public void setGravityMode (boolean mode)
	{ dBodySetGravityMode (mode); }
	/** @see DxBody#dBodyGetGravityMode() */
	@Override
	public boolean getGravityMode() 
	{ return dBodyGetGravityMode (); }

	@Override
	public void setGyroscopicMode (boolean mode)
	{ dBodySetGyroscopicMode(mode); }
	@Override
	public boolean getGyroscopicMode() 
	{ return dBodyGetGyroscopicMode (); }

	@Override
	public boolean isConnectedTo (DBody body) //const
	{ return OdeHelper.areConnected (this, body); }

	@Override
	public void  setAutoDisableLinearThreshold (double threshold)
	{ dBodySetAutoDisableLinearThreshold (threshold); }
	@Override
	public double getAutoDisableLinearThreshold() //const
	{ return dBodyGetAutoDisableLinearThreshold (); }
	@Override
	public void setAutoDisableAngularThreshold (double threshold)
	{ dBodySetAutoDisableAngularThreshold (threshold); }
	@Override
	public double getAutoDisableAngularThreshold() //const
	{ return dBodyGetAutoDisableAngularThreshold (); }
	@Override
	public void setAutoDisableSteps (int steps)
	{ dBodySetAutoDisableSteps (steps); }
	@Override
	public int getAutoDisableSteps() 
	{ return dBodyGetAutoDisableSteps (); }
	@Override
	public void setAutoDisableTime (double time)
	{ dBodySetAutoDisableTime (time); }
	@Override
	public double getAutoDisableTime() 
	{ return dBodyGetAutoDisableTime (); }
	@Override
	public void setAutoDisableFlag (boolean do_auto_disable)
	{ dBodySetAutoDisableFlag ( do_auto_disable); }
	@Override
	public boolean getAutoDisableFlag() 
	{ return dBodyGetAutoDisableFlag (); }

	@Override
	public double getLinearDamping()
	{ return dBodyGetLinearDamping(); }
	@Override
	public void setLinearDamping(double scale)
	{ dBodySetLinearDamping(scale); }
	@Override
	public double getAngularDamping()
	{ return dBodyGetAngularDamping(); }
	@Override
	public void setAngularDamping(double scale)
	{ dBodySetAngularDamping( scale); }
	@Override
	public void setDamping(double linear_scale, double angular_scale)
	{ dBodySetDamping(linear_scale, angular_scale); }
	@Override
	public double getLinearDampingThreshold()
	{ return dBodyGetLinearDampingThreshold(); }
	@Override
	public void setLinearDampingThreshold(double threshold)
	{ dBodySetLinearDampingThreshold(threshold); }
	@Override
	public double getAngularDampingThreshold()
	{ return dBodyGetAngularDampingThreshold(); }
	@Override
	public void setAngularDampingThreshold(double threshold)
	{ dBodySetAngularDampingThreshold(threshold); }
	@Override
	public void setDampingDefaults()
	{ dBodySetDampingDefaults(); }

	@Override
	public double getMaxAngularSpeed()
	{ return dBodyGetMaxAngularSpeed(); }
	@Override
	public void setMaxAngularSpeed(double max_speed)
	{ dBodySetMaxAngularSpeed(max_speed); }

	@Override
	public void destroy() {
		dBodyDestroy();
	}


	@Override
	public int getAutoDisableAverageSamplesCount() {
		return dBodyGetAutoDisableAverageSamplesCount();
	}


	@Override
	public void setAutoDisableAverageSamplesCount(int average_samples_count) {
		dBodySetAutoDisableAverageSamplesCount(average_samples_count);
	}


	@Override
	public void setAutoDisableDefaults() {
		dBodySetAutoDisableDefaults();
	}

    @Override
	public DGeom getFirstGeom() {
		return dBodyGetFirstGeom();
	}


	/** @deprecated */
	@Deprecated
    @Override
	public DGeom getNextGeom(DGeom geom) {
		return dBodyGetNextGeom((DxGeom) geom);
	}

	@Override
	public Iterator<DGeom> getGeomIterator() {
		return new GeomIterator(geom);
	}

	@Override
	public void setMovedCallback(BodyMoveCallBack callback) {
		dBodySetMovedCallback(callback);
	}

	private static class GeomIterator implements Iterator<DGeom> {
		DxGeom current;

		GeomIterator(DxGeom start) {
			current = start;
		}

		@Override
		public boolean hasNext() {
			return current != null;
		}

		@Override
		public DGeom next() {
			DGeom ret = current;
			current = current.body_next;
			return ret;
		}
	}
}
