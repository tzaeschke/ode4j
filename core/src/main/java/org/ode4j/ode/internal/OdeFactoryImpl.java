/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2013 Tilmann Zaeschke     *
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

import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.joints.OdeJointsFactoryImpl;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.cpp4j.Cstdio.*;


/**
 * this source file is mostly concerned with the data structures, not the
 * numerics.
 */
public class OdeFactoryImpl extends OdeJointsFactoryImpl {

	// misc defines
	//#define ALLOCA dALLOCA16
	//private static final String ALLOCA = "ALLOCA16";

	//****************************************************************************
	// utility

	//Moved to Objects_H.java
	//dObject::dObject(dxWorld *w)
	//{
	//    world = w;
	//    next = 0;
	//    tome = 0;
	//    userdata = 0;
	//    tag = 0;
	//}


//	// add an object `obj' to the list who's head pointer is pointed 
//	//to by `first'.
//
//	//static void addObjectToList (dObject *obj, dObject **first)
//	public static <T extends dObject>void addObjectToList (T obj, 
//			MutableReference<T> first)
//	{
//		//  obj.next = *first;
//		//  obj.tome = first;
//		//  if (first != null) first.tome = &obj.next;
//		//  first = obj;
//		obj.next = first.get();
//		obj.tome = null;//first;
//		if (first.get() != null) first.get().tome = obj;
//		first.set(obj);
//	}
//
//
//	// remove the object from the linked list
//
//	static <T extends dObject>void removeObjectFromList (T obj)
//	{
//		//	  if (obj.next) obj.next.tome = obj.tome;
//		//	  *(obj.tome) = obj.next;
//		//	  // safeguard
//		//	  obj.next = 0;
//		//	  obj.tome = 0;
//		if (obj.next != null) obj.next.tome = obj.tome;
//		(obj.tome) = obj.next;
//		// safeguard
//		obj.next = null;
//		obj.tome = null;
//	}


	//// remove the joint from neighbour lists of all connected bodies
	//
	//static void removeJointReferencesFromAttachedBodies (dxJoint j)
	//{
	//	for (int i=0; i<2; i++) {
	//		dxBody body = j.node[i].body;
	//		if (body) {
	//			dxJointNode n = body.firstjoint;
	//			dxJointNode last = null;
	//			while (n != null) {
	//				if (n.joint == j) {
	//					if (last) last->next = n.next;
	//					else body.firstjoint = n.next;
	//					break;
	//				}
	//				last = n;
	//				n = n.next;
	//			}
	//		}
	//	}
	//	j.node[0].body = 0;
	//	j.node[0].next = 0;
	//	j.node[1].body = 0;
	//	j.node[1].next = 0;
	//}
	//
	//****************************************************************************
	// debugging

	// see if an object list loops on itself (if so, it's bad).

	static <T extends DObject> boolean listHasLoops (Ref<T>  first)
	{
		//  if (first==null || first.next==null) return 0;
		//  dObject *a=first,*b=first->next;
		//  int skip=0;
		//  while (b) {
		//    if (a==b) return 1;
		//    b = b.next;
		//    if (skip != 0) a = a.next;
		//    skip ^= 1;
		//  }
		//  return 0;
		if (first.get()==null || first.get().getNext()==null) return false;
		DObject a=first.get(),b=first.get().getNext();
		int skip=0;
		while (b != null) {
			if (a==b) return true;
			b = b.getNext();
			if (skip != 0) a = a.getNext();
			skip ^= 1;
		}
		return false;
	}


	// check the validity of the world data structures

	private static int g_world_check_tag_generator = 0;

	static int generateWorldCheckTag()
	{
		// Atomicity is not necessary here
		return ++g_world_check_tag_generator;
	}

	static void checkWorld (DxWorld w)
	{
		DxBody b;
		DxJoint j;

		// check there are no loops
		if (listHasLoops (w.firstbody)) dDebug (0,"body list has loops");
		if (listHasLoops (w.firstjoint)) dDebug (0,"joint list has loops");

		//TODO
		if (true) throw new UnsupportedOperationException();
//		// check lists are well formed (check `tome' pointers)
//		for (b=w.firstbody.get(); b!=null; b=(dxBody)b.getNext()) {
//			if (b.getNext()!=null && b.getNext().tome != b.getNext())
//				dDebug (0,"bad tome pointer in body list");
//		}
//		for (j=w.firstjoint.get(); j!=null; j=(dxJoint)j.getNext()) {
//			if (j.getNext()!=null && j.getNext().tome != j.getNext())
//				dDebug (0,"bad tome pointer in joint list");
//		}

		// check counts
		int nn = 0;
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) nn++;
		if (w.nb != nn) dDebug (0,"body count incorrect");
		nn = 0;
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) nn++;
		if (w.nj != nn) dDebug (0,"joint count incorrect");

		// set all tag values to a known value
		int count = generateWorldCheckTag();
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) b.tag = count;
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) j.tag = count;

		// check all body/joint world pointers are ok
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) if (b.world != w)
			dDebug (0,"bad world pointer in body list");
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) if (j.world != w)
			dDebug (0,"bad world pointer in joint list");

		/*
  // check for half-connected joints - actually now these are valid
  for (j=w.firstjoint; j; j=(dxJoint*)j.next) {
    if (j.node[0].body || j.node[1].body) {
      if (!(j.node[0].body && j.node[1].body))
	dDebug (0,"half connected joint found");
    }
  }
		 */

		// check that every joint node appears in the joint lists of both bodies it
		// attaches
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			for (int i=0; i<2; i++) {
				if (j.node[i].body!=null) {
					int ok = 0;
					for (DxJointNode n=j.node[i].body.firstjoint.get(); n!=null; n=n.next) {
						if (n.joint == j) ok = 1;
					}
					if (ok==0) dDebug (0,"joint not in joint list of attached body");
				}
			}
		}

		// check all body joint lists (correct body ptrs)
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) {
			for (DxJointNode n=b.firstjoint.get(); n!=null; n=n.next) {
				if (n.joint.node[0] == n) {
					if (n.joint.node[1].body != b)
						dDebug (0,"bad body pointer in joint node of body list (1)");
				}
				else {
					if (n.joint.node[0].body != b)
						dDebug (0,"bad body pointer in joint node of body list (2)");
				}
				if (n.joint.tag != count) dDebug (0,"bad joint node pointer in body");
			}
		}

		// check all body pointers in joints, check they are distinct
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			if (j.node[0].body!=null && (j.node[0].body == j.node[1].body))
				dDebug (0,"non-distinct body pointers in joint");
			if ((j.node[0].body!=null && j.node[0].body.tag != count) ||
					(j.node[1].body!=null && j.node[1].body.tag != count))
				dDebug (0,"bad body pointer in joint");
		}
	}


	void dWorldCheck (DxWorld w)
	{
		checkWorld (w);
	}

	//****************************************************************************
	// body

	//TZ moved to dxBody

	//dxBody::dxBody(dxWorld w) :
	//    dObject(w)
	//{
	//    
	//}
	//
	//
	//dxWorld dBodyGetWorld (dxBody b)
	//{
	//  dAASSERT (b);
	//  return b.world;
	//}
	//
	//dxBody *dBodyCreate (dxWorld *w)
	//{
	//  dAASSERT (w);
	//  dxBody *b = new dxBody(w);
	//  b->firstjoint = 0;
	//  b->flags = 0;
	//  b->geom = 0;
	//  b->average_lvel_buffer = 0;
	//  b->average_avel_buffer = 0;
	//  dMassSetParameters (&b->mass,1,0,0,0,1,1,1,0,0,0);
	//  dSetZero (b->invI,4*3);
	//  b->invI[0] = 1;
	//  b->invI[5] = 1;
	//  b->invI[10] = 1;
	//  b->invMass = 1;
	//  dSetZero (b->posr.pos,4);
	//  dSetZero (b->q,4);
	//  b->q[0] = 1;
	//  dRSetIdentity (b->posr.R);
	//  dSetZero (b->lvel,4);
	//  dSetZero (b->avel,4);
	//  dSetZero (b->facc,4);
	//  dSetZero (b->tacc,4);
	//  dSetZero (b->finite_rot_axis,4);
	//  addObjectToList (b,(dObject **) &w->firstbody);
	//  w->nb++;
	//
	//  // set auto-disable parameters
	//  b->average_avel_buffer = b->average_lvel_buffer = 0; // no buffer at beginning
	//  dBodySetAutoDisableDefaults (b);	// must do this after adding to world
	//  b->adis_stepsleft = b->adis.idle_steps;
	//  b->adis_timeleft = b->adis.idle_time;
	//  b->average_counter = 0;
	//  b->average_ready = 0; // average buffer not filled on the beginning
	//  dBodySetAutoDisableAverageSamplesCount(b, b->adis.average_samples);
	//
	//  b->moved_callback = 0;
	//
	//  dBodySetDampingDefaults(b);	// must do this after adding to world
	//
	//  b->flags |= w->body_flags & dxBodyMaxAngularSpeed;
	//  b->max_angular_speed = w->max_angular_speed;
	//
	//  return b;
	//}
	//
	//
	//void dBodyDestroy (dxBody b)
	//{
	//  dAASSERT (b);
	//
	//  // all geoms that link to this body must be notified that the body is about
	//  // to disappear. note that the call to dGeomSetBody(geom,0) will result in
	//  // dGeomGetBodyNext() returning 0 for the body, so we must get the next body
	//  // before setting the body to 0.
	//  dxGeom next_geom = null;
	//  for (dxGeom geom = b.geom; geom != null; geom = next_geom) {
	//    next_geom = dGeomGetBodyNext (geom);
	//    dGeomSetBody (geom,0);
	//  }
	//
	//  // detach all neighbouring joints, then delete this body.
	//  dxJointNode n = b.firstjoint;
	//  while (n != null) {
	//    // sneaky trick to speed up removal of joint references (black magic)
	//	  //TODO use equals?
	//    n.joint.node[(n == n.joint.node)].body = 0;
	//
	//    dxJointNode next = n.next;
	//    n.next = null;
	//    removeJointReferencesFromAttachedBodies (n.joint);
	//    n = next;
	//  }
	//  removeObjectFromList (b);
	//  b.world.nb--;
	//
	//  // delete the average buffers
	//  if(b.average_lvel_buffer)
	//  {
	//	  delete[] (b.average_lvel_buffer);
	//	  b.average_lvel_buffer = 0;
	//  }
	//  if(b.average_avel_buffer)
	//  {
	//	  delete[] (b.average_avel_buffer);
	//	  b.average_avel_buffer = 0;
	//  }
	//
	//  delete b;
	//}
	//
	//
	//void dBodySetData (dxBody b, void data)
	//{
	//  dAASSERT (b);
	//  b.userdata = data;
	//}
	//
	//
	//Object dBodyGetData (dBody b)
	//{
	//  dAASSERT (b);
	//  return b.userdata;
	//}
	//
	//
	//void dBodySetPosition (dBody b, dReal x, dReal y, dReal z)
	//{
	//  dAASSERT (b);
	//  b.posr.pos[0] = x;
	//  b.posr.pos[1] = y;
	//  b.posr.pos[2] = z;
	//
	//  // notify all attached geoms that this body has moved
	//  for (dxGeom geom = b.geom; geom != null; geom = dGeomGetBodyNext (geom))
	//    dGeomMoved (geom);
	//}
	//
	//
	//void dBodySetRotation (dBody b, final dMatrix3 R)
	//{
	//  dAASSERT (b, R);
	//  dQuaternion q;
	//  dRtoQ (R,q);
	//  dNormalize4 (q);
	//  b.q[0] = q[0];
	//  b.q[1] = q[1];
	//  b.q[2] = q[2];
	//  b.q[3] = q[3];
	//  dQtoR (b.q,b.posr.R);
	//
	//  // notify all attached geoms that this body has moved
	//  for (dxGeom geom = b.geom; geom != null; geom = dGeomGetBodyNext (geom))
	//    dGeomMoved (geom);
	//}
	//
	//
	//void dBodySetQuaternion (dBody b, final dQuaternion q)
	//{
	//  dAASSERT (b, q);
	//  b.q[0] = q[0];
	//  b.q[1] = q[1];
	//  b.q[2] = q[2];
	//  b.q[3] = q[3];
	//  dNormalize4 (b.q);
	//  dQtoR (b.q,b.posr.R);
	//
	//  // notify all attached geoms that this body has moved
	//  for (dxGeom geom = b.geom; geom != null; geom = dGeomGetBodyNext (geom))
	//    dGeomMoved (geom);
	//}
	//
	//
	//void dBodySetLinearVel  (dBody b, dReal x, dReal y, dReal z)
	//{
	//  dAASSERT (b);
	//  b.lvel[0] = x;
	//  b.lvel[1] = y;
	//  b.lvel[2] = z;
	//}
	//
	//
	//void dBodySetAngularVel (dBody b, dReal x, dReal y, dReal z)
	//{
	//  dAASSERT (b);
	//  b.avel[0] = x;
	//  b.avel[1] = y;
	//  b.avel[2] = z;
	//}
	//
	//
	//final double[] dBodyGetPosition (dBody b)
	//{
	//  dAASSERT (b);
	//  return b.posr.pos;
	//}
	//
	//
	//void dBodyCopyPosition (dBody b, dVector3 pos)
	//{
	//	dAASSERT (b);
	//	double[] src = b.posr.pos;
	//	pos[0] = src[0];
	//	pos[1] = src[1];
	//	pos[2] = src[2];
	//}
	//
	//
	//final double[] dBodyGetRotation (dBody b)
	//{
	//  dAASSERT (b);
	//  return b.posr.R;
	//}
	//
	//
	//void dBodyCopyRotation (dBody b, dMatrix3 R)
	//{
	//	dAASSERT (b);
	//	final double[] src = b.posr.R;
	//	R[0] = src[0];
	//	R[1] = src[1];
	//	R[2] = src[2];
	//	R[3] = src[3];
	//	R[4] = src[4];
	//	R[5] = src[5];
	//	R[6] = src[6];
	//	R[7] = src[7];
	//	R[8] = src[8];
	//	R[9] = src[9];
	//	R[10] = src[10];
	//	R[11] = src[11];
	//}
	//
	//
	//final double[] dBodyGetQuaternion (dBody b)
	//{
	//  dAASSERT (b);
	//  return b.q;
	//}
	//
	//
	//void dBodyCopyQuaternion (dBody b, dQuaternion quat)
	//{
	//	dAASSERT (b);
	//	double[] src = b.q;
	//	quat[0] = src[0];
	//	quat[1] = src[1];
	//	quat[2] = src[2];
	//	quat[3] = src[3];
	//}
	//
	//
	//final double[] dBodyGetLinearVel (dBody b)
	//{
	//  dAASSERT (b);
	//  return b.lvel;
	//}
	//
	//
	//final double[] dBodyGetAngularVel (dBody b)
	//{
	//  dAASSERT (b);
	//  return b.avel;
	//}
	//
	//
	////void dBodySetMass (dBody b, final dMass mass)
	//void dBodySetMass (dxBody b, final dMass mass)
	//{
	//  dAASSERT (b, mass );
	//  dIASSERT(dMassCheck(mass));
	//
	//  // The centre of mass must be at the origin.
	//  // Use dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] ) to correct it.
	//  dUASSERT( fabs( mass.c[0] ) <= dEpsilon &&
	//			fabs( mass.c[1] ) <= dEpsilon &&
	//			fabs( mass.c[2] ) <= dEpsilon, "The centre of mass must be at the origin." )
	//
	//  memcpy (b.mass,mass,sizeof(dMass));
	//  if (dInvertPDMatrix (b.mass.I,b.invI,3)==0) {
	//    dDEBUGMSG ("inertia must be positive definite!");
	//    dRSetIdentity (b.invI);
	//  }
	//  b.invMass = dRecip(b.mass.mass);
	//}
	//
	//
	//void dBodyGetMass (dxBody b, dMass mass)
	////void dBodyGetMass (dBody b, dMass *mass)
	//{
	//  dAASSERT (b && mass);
	//  memcpy (mass,&b->mass,sizeof(dMass));
	//}
	//
	//
	//void dBodyAddForce (dBody b, dReal fx, dReal fy, dReal fz)
	//{
	//  dAASSERT (b);
	//  b->facc[0] += fx;
	//  b->facc[1] += fy;
	//  b->facc[2] += fz;
	//}
	//
	//
	//void dBodyAddTorque (dBody b, dReal fx, dReal fy, dReal fz)
	//{
	//  dAASSERT (b);
	//  b->tacc[0] += fx;
	//  b->tacc[1] += fy;
	//  b->tacc[2] += fz;
	//}
	//
	//
	//void dBodyAddRelForce (dBody b, dReal fx, dReal fy, dReal fz)
	//{
	//  dAASSERT (b);
	//  dVector3 t1,t2;
	//  t1[0] = fx;
	//  t1[1] = fy;
	//  t1[2] = fz;
	//  t1[3] = 0;
	//  dMULTIPLY0_331 (t2,b->posr.R,t1);
	//  b->facc[0] += t2[0];
	//  b->facc[1] += t2[1];
	//  b->facc[2] += t2[2];
	//}
	//
	//
	//void dBodyAddRelTorque (dBody b, dReal fx, dReal fy, dReal fz)
	//{
	//  dAASSERT (b);
	//  dVector3 t1,t2;
	//  t1[0] = fx;
	//  t1[1] = fy;
	//  t1[2] = fz;
	//  t1[3] = 0;
	//  dMULTIPLY0_331 (t2,b->posr.R,t1);
	//  b->tacc[0] += t2[0];
	//  b->tacc[1] += t2[1];
	//  b->tacc[2] += t2[2];
	//}
	//
	//
	//void dBodyAddForceAtPos (dBody b, dReal fx, dReal fy, dReal fz,
	//			 dReal px, dReal py, dReal pz)
	//{
	//  dAASSERT (b);
	//  b->facc[0] += fx;
	//  b->facc[1] += fy;
	//  b->facc[2] += fz;
	//  dVector3 f,q;
	//  f[0] = fx;
	//  f[1] = fy;
	//  f[2] = fz;
	//  q[0] = px - b->posr.pos[0];
	//  q[1] = py - b->posr.pos[1];
	//  q[2] = pz - b->posr.pos[2];
	//  dCROSS (b->tacc,+=,q,f);
	//}
	//
	//
	//void dBodyAddForceAtRelPos (dBody b, dReal fx, dReal fy, dReal fz,
	//			    dReal px, dReal py, dReal pz)
	//{
	//  dAASSERT (b);
	//  dVector3 prel,f,p;
	//  f[0] = fx;
	//  f[1] = fy;
	//  f[2] = fz;
	//  f[3] = 0;
	//  prel[0] = px;
	//  prel[1] = py;
	//  prel[2] = pz;
	//  prel[3] = 0;
	//  dMULTIPLY0_331 (p,b->posr.R,prel);
	//  b->facc[0] += f[0];
	//  b->facc[1] += f[1];
	//  b->facc[2] += f[2];
	//  dCROSS (b->tacc,+=,p,f);
	//}
	//
	//
	//void dBodyAddRelForceAtPos (dBody b, dReal fx, dReal fy, dReal fz,
	//			    dReal px, dReal py, dReal pz)
	//{
	//  dAASSERT (b);
	//  dVector3 frel,f;
	//  frel[0] = fx;
	//  frel[1] = fy;
	//  frel[2] = fz;
	//  frel[3] = 0;
	//  dMULTIPLY0_331 (f,b->posr.R,frel);
	//  b->facc[0] += f[0];
	//  b->facc[1] += f[1];
	//  b->facc[2] += f[2];
	//  dVector3 q;
	//  q[0] = px - b->posr.pos[0];
	//  q[1] = py - b->posr.pos[1];
	//  q[2] = pz - b->posr.pos[2];
	//  dCROSS (b->tacc,+=,q,f);
	//}
	//
	//
	//void dBodyAddRelForceAtRelPos (dBody b, dReal fx, dReal fy, dReal fz,
	//			       dReal px, dReal py, dReal pz)
	//{
	//  dAASSERT (b);
	//  dVector3 frel,prel,f,p;
	//  frel[0] = fx;
	//  frel[1] = fy;
	//  frel[2] = fz;
	//  frel[3] = 0;
	//  prel[0] = px;
	//  prel[1] = py;
	//  prel[2] = pz;
	//  prel[3] = 0;
	//  dMULTIPLY0_331 (f,b->posr.R,frel);
	//  dMULTIPLY0_331 (p,b->posr.R,prel);
	//  b->facc[0] += f[0];
	//  b->facc[1] += f[1];
	//  b->facc[2] += f[2];
	//  dCROSS (b->tacc,+=,p,f);
	//}
	//
	//
	//const dReal * dBodyGetForce (dBody b)
	//{
	//  dAASSERT (b);
	//  return b->facc;
	//}
	//
	//
	//const dReal * dBodyGetTorque (dBody b)
	//{
	//  dAASSERT (b);
	//  return b->tacc;
	//}
	//
	//
	//void dBodySetForce (dBody b, dReal x, dReal y, dReal z)
	//{
	//  dAASSERT (b);
	//  b->facc[0] = x;
	//  b->facc[1] = y;
	//  b->facc[2] = z;
	//}
	//
	//
	//void dBodySetTorque (dBody b, dReal x, dReal y, dReal z)
	//{
	//  dAASSERT (b);
	//  b->tacc[0] = x;
	//  b->tacc[1] = y;
	//  b->tacc[2] = z;
	//}
	//
	//
	//void dBodyGetRelPointPos (dBody b, dReal px, dReal py, dReal pz,
	//			  dVector3 result)
	//{
	//  dAASSERT (b);
	//  dVector3 prel,p;
	//  prel[0] = px;
	//  prel[1] = py;
	//  prel[2] = pz;
	//  prel[3] = 0;
	//  dMULTIPLY0_331 (p,b->posr.R,prel);
	//  result[0] = p[0] + b->posr.pos[0];
	//  result[1] = p[1] + b->posr.pos[1];
	//  result[2] = p[2] + b->posr.pos[2];
	//}
	//
	//
	//void dBodyGetRelPointVel (dBody b, dReal px, dReal py, dReal pz,
	//			  dVector3 result)
	//{
	//  dAASSERT (b);
	//  dVector3 prel,p;
	//  prel[0] = px;
	//  prel[1] = py;
	//  prel[2] = pz;
	//  prel[3] = 0;
	//  dMULTIPLY0_331 (p,b->posr.R,prel);
	//  result[0] = b->lvel[0];
	//  result[1] = b->lvel[1];
	//  result[2] = b->lvel[2];
	//  dCROSS (result,+=,b->avel,p);
	//}
	//
	//
	//void dBodyGetPointVel (dBody b, dReal px, dReal py, dReal pz,
	//		       dVector3 result)
	//{
	//  dAASSERT (b);
	//  dVector3 p;
	//  p[0] = px - b->posr.pos[0];
	//  p[1] = py - b->posr.pos[1];
	//  p[2] = pz - b->posr.pos[2];
	//  p[3] = 0;
	//  result[0] = b->lvel[0];
	//  result[1] = b->lvel[1];
	//  result[2] = b->lvel[2];
	//  dCROSS (result,+=,b->avel,p);
	//}
	//
	//
	//void dBodyGetPosRelPoint (dBody b, dReal px, dReal py, dReal pz,
	//			  dVector3 result)
	//{
	//  dAASSERT (b);
	//  dVector3 prel;
	//  prel[0] = px - b->posr.pos[0];
	//  prel[1] = py - b->posr.pos[1];
	//  prel[2] = pz - b->posr.pos[2];
	//  prel[3] = 0;
	//  dMULTIPLY1_331 (result,b->posr.R,prel);
	//}
	//
	//
	//void dBodyVectorToWorld (dBody b, dReal px, dReal py, dReal pz,
	//			 dVector3 result)
	//{
	//  dAASSERT (b);
	//  dVector3 p;
	//  p[0] = px;
	//  p[1] = py;
	//  p[2] = pz;
	//  p[3] = 0;
	//  dMULTIPLY0_331 (result,b->posr.R,p);
	//}
	//
	//
	//void dBodyVectorFromWorld (dBody b, dReal px, dReal py, dReal pz,
	//			   dVector3 result)
	//{
	//  dAASSERT (b);
	//  dVector3 p;
	//  p[0] = px;
	//  p[1] = py;
	//  p[2] = pz;
	//  p[3] = 0;
	//  dMULTIPLY1_331 (result,b->posr.R,p);
	//}
	//
	//
	//void dBodySetFiniteRotationMode (dBody b, int mode)
	//{
	//  dAASSERT (b);
	//  b->flags &= ~(dxBodyFlagFiniteRotation | dxBodyFlagFiniteRotationAxis);
	//  if (mode) {
	//    b->flags |= dxBodyFlagFiniteRotation;
	//    if (b->finite_rot_axis[0] != 0 || b->finite_rot_axis[1] != 0 ||
	//	b->finite_rot_axis[2] != 0) {
	//      b->flags |= dxBodyFlagFiniteRotationAxis;
	//    }
	//  }
	//}
	//
	//
	//void dBodySetFiniteRotationAxis (dBody b, dReal x, dReal y, dReal z)
	//{
	//  dAASSERT (b);
	//  b->finite_rot_axis[0] = x;
	//  b->finite_rot_axis[1] = y;
	//  b->finite_rot_axis[2] = z;
	//  if (x != 0 || y != 0 || z != 0) {
	//    dNormalize3 (b->finite_rot_axis);
	//    b->flags |= dxBodyFlagFiniteRotationAxis;
	//  }
	//  else {
	//    b->flags &= ~dxBodyFlagFiniteRotationAxis;
	//  }
	//}
	//
	//
	//int dBodyGetFiniteRotationMode (dBody b)
	//{
	//  dAASSERT (b);
	//  return ((b->flags & dxBodyFlagFiniteRotation) != 0);
	//}
	//
	//
	//void dBodyGetFiniteRotationAxis (dBody b, dVector3 result)
	//{
	//  dAASSERT (b);
	//  result[0] = b->finite_rot_axis[0];
	//  result[1] = b->finite_rot_axis[1];
	//  result[2] = b->finite_rot_axis[2];
	//}
	//
	//
	//int dBodyGetNumJoints (dBody b)
	//{
	//  dAASSERT (b);
	//  int count=0;
	//  for (dxJointNode *n=b->firstjoint; n; n=n->next, count++);
	//  return count;
	//}
	//
	//
	//dJoint dBodyGetJoint (dBody b, int index)
	//{
	//  dAASSERT (b);
	//  int i=0;
	//  for (dxJointNode *n=b->firstjoint; n; n=n->next, i++) {
	//    if (i == index) return n->joint;
	//  }
	//  return 0;
	//}
	//
	//
	//void dBodyEnable (dBody b)
	//{
	//  dAASSERT (b);
	//  b->flags &= ~dxBodyDisabled;
	//  b->adis_stepsleft = b->adis.idle_steps;
	//  b->adis_timeleft = b->adis.idle_time;
	//  // no code for average-processing needed here
	//}
	//
	//
	//void dBodyDisable (dBody b)
	//{
	//  dAASSERT (b);
	//  b->flags |= dxBodyDisabled;
	//}
	//
	//
	//int dBodyIsEnabled (dBody b)
	//{
	//  dAASSERT (b);
	//  return ((b->flags & dxBodyDisabled) == 0);
	//}
	//
	//
	//void dBodySetGravityMode (dBody b, int mode)
	//{
	//  dAASSERT (b);
	//  if (mode) b->flags &= ~dxBodyNoGravity;
	//  else b->flags |= dxBodyNoGravity;
	//}
	//
	//
	//int dBodyGetGravityMode (dBody b)
	//{
	//  dAASSERT (b);
	//  return ((b->flags & dxBodyNoGravity) == 0);
	//}
	//
	//
	//// body auto-disable functions
	//
	//dReal dBodyGetAutoDisableLinearThreshold (dBody b)
	//{
	//	dAASSERT(b);
	//	return dSqrt (b->adis.linear_average_threshold);
	//}
	//
	//
	//void dBodySetAutoDisableLinearThreshold (dBody b, dReal linear_average_threshold)
	//{
	//	dAASSERT(b);
	//	b->adis.linear_average_threshold = linear_average_threshold * linear_average_threshold;
	//}
	//
	//
	//dReal dBodyGetAutoDisableAngularThreshold (dBody b)
	//{
	//	dAASSERT(b);
	//	return dSqrt (b->adis.angular_average_threshold);
	//}
	//
	//
	//void dBodySetAutoDisableAngularThreshold (dBody b, dReal angular_average_threshold)
	//{
	//	dAASSERT(b);
	//	b->adis.angular_average_threshold = angular_average_threshold * angular_average_threshold;
	//}
	//
	//
	//int dBodyGetAutoDisableAverageSamplesCount (dBody b)
	//{
	//	dAASSERT(b);
	//	return b->adis.average_samples;
	//}
	//
	//
	//void dBodySetAutoDisableAverageSamplesCount (dBody b, unsigned int average_samples_count)
	//{
	//	dAASSERT(b);
	//	b->adis.average_samples = average_samples_count;
	//	// update the average buffers
	//	if(b->average_lvel_buffer)
	//	{
	//		delete[] b->average_lvel_buffer;
	//		b->average_lvel_buffer = 0;
	//	}
	//	if(b->average_avel_buffer)
	//	{
	//		delete[] b->average_avel_buffer;
	//		b->average_avel_buffer = 0;
	//	}
	//	if(b->adis.average_samples > 0)
	//	{
	//		b->average_lvel_buffer = new dVector3[b->adis.average_samples];
	//		b->average_avel_buffer = new dVector3[b->adis.average_samples];
	//	}
	//	else
	//	{
	//		b->average_lvel_buffer = 0;
	//		b->average_avel_buffer = 0;
	//	}
	//	// new buffer is empty
	//	b->average_counter = 0;
	//	b->average_ready = 0;
	//}
	//
	//
	//int dBodyGetAutoDisableSteps (dBody b)
	//{
	//	dAASSERT(b);
	//	return b->adis.idle_steps;
	//}
	//
	//
	//void dBodySetAutoDisableSteps (dBody b, int steps)
	//{
	//	dAASSERT(b);
	//	b->adis.idle_steps = steps;
	//}
	//
	//
	//dReal dBodyGetAutoDisableTime (dBody b)
	//{
	//	dAASSERT(b);
	//	return b->adis.idle_time;
	//}
	//
	//
	//void dBodySetAutoDisableTime (dBody b, dReal time)
	//{
	//	dAASSERT(b);
	//	b->adis.idle_time = time;
	//}
	//
	//
	//int dBodyGetAutoDisableFlag (dBody b)
	//{
	//	dAASSERT(b);
	//	return ((b->flags & dxBodyAutoDisable) != 0);
	//}
	//
	//
	//void dBodySetAutoDisableFlag (dBody b, int do_auto_disable)
	//{
	//	dAASSERT(b);
	//	if (!do_auto_disable)
	//	{
	//		b->flags &= ~dxBodyAutoDisable;
	//		// (mg) we should also reset the IsDisabled state to correspond to the DoDisabling flag
	//		b->flags &= ~dxBodyDisabled;
	//		b->adis.idle_steps = dWorldGetAutoDisableSteps(b->world);
	//		b->adis.idle_time = dWorldGetAutoDisableTime(b->world);
	//		// resetting the average calculations too
	//		dBodySetAutoDisableAverageSamplesCount(b, dWorldGetAutoDisableAverageSamplesCount(b->world) );
	//	}
	//	else
	//	{
	//		b->flags |= dxBodyAutoDisable;
	//	}
	//}
	//
	//
	//void dBodySetAutoDisableDefaults (dBody b)
	//{
	//	dAASSERT(b);
	//	dWorld w = b->world;
	//	dAASSERT(w);
	//	b->adis = w->adis;
	//	dBodySetAutoDisableFlag (b, w->body_flags & dxBodyAutoDisable);
	//}
	//
	//
	//// body damping functions
	//
	//dReal dBodyGetLinearDamping(dBody b)
	//{
	//        dAASSERT(b);
	//        return b->dampingp.linear_scale;
	//}
	//
	//void dBodySetLinearDamping(dBody b, dReal scale)
	//{
	//        dAASSERT(b);
	//        if (scale)
	//                b->flags |= dxBodyLinearDamping;
	//        else
	//                b->flags &= ~dxBodyLinearDamping;
	//        b->dampingp.linear_scale = scale;
	//}
	//
	//dReal dBodyGetAngularDamping(dBody b)
	//{
	//        dAASSERT(b);
	//        return b->dampingp.angular_scale;
	//}
	//
	//void dBodySetAngularDamping(dBody b, dReal scale)
	//{
	//        dAASSERT(b);
	//        if (scale)
	//                b->flags |= dxBodyAngularDamping;
	//        else
	//                b->flags &= ~dxBodyAngularDamping;
	//        b->dampingp.angular_scale = scale;
	//}
	//
	//void dBodySetDamping(dBody b, dReal linear_scale, dReal angular_scale)
	//{
	//        dAASSERT(b);
	//        dBodySetLinearDamping(b, linear_scale);
	//        dBodySetAngularDamping(b, angular_scale);
	//}
	//
	//dReal dBodyGetLinearDampingThreshold(dBody b)
	//{
	//        dAASSERT(b);
	//        return dSqrt(b->dampingp.linear_threshold);
	//}
	//
	//void dBodySetLinearDampingThreshold(dBody b, dReal threshold)
	//{
	//        dAASSERT(b);
	//        b->dampingp.linear_threshold = threshold*threshold;
	//}
	//
	//
	//dReal dBodyGetAngularDampingThreshold(dBody b)
	//{
	//        dAASSERT(b);
	//        return dSqrt(b->dampingp.angular_threshold);
	//}
	//
	//void dBodySetAngularDampingThreshold(dBody b, dReal threshold)
	//{
	//        dAASSERT(b);
	//        b->dampingp.angular_threshold = threshold*threshold;
	//}
	//
	//void dBodySetDampingDefaults(dBody b)
	//{
	//        dAASSERT(b);
	//        dWorld w = b->world;
	//        dAASSERT(w);
	//        b->dampingp = w->dampingp;
	//        int mask = dxBodyLinearDamping | dxBodyAngularDamping;
	//        b->flags &= ~mask; // zero them
	//        b->flags |= w->body_flags & mask;
	//}
	//
	//dReal dBodyGetMaxAngularSpeed(dBody b)
	//{
	//        dAASSERT(b);
	//        return b->max_angular_speed;
	//}
	//
	//void dBodySetMaxAngularSpeed(dBody b, dReal max_speed)
	//{
	//        dAASSERT(b);
	//        if (max_speed < dInfinity)
	//                b->flags |= dxBodyMaxAngularSpeed;
	//        else
	//                b->flags &= ~dxBodyMaxAngularSpeed;
	//        b->max_angular_speed = max_speed;
	//}
	//
	//void dBodySetMovedCallback(dBody b, void (*callback)(dBody))
	//{
	//        dAASSERT(b);
	//        b->moved_callback = callback;
	//}
	//
	//
	//dGeom dBodyGetFirstGeom(dBody b)
	//{
	//        dAASSERT(b);
	//        return b->geom;
	//}
	//
	//
	//dGeom dBodyGetNextGeom(dGeom geom)
	//{
	//        dAASSERT(geom);
	//        return dGeomGetBodyNext(geom);
	//}


//	//****************************************************************************
//	// joints
//
//
//
//	//template<class T>
//	//dxJoint XXcreateJoint(dWorld w, dxJointGroup group)
//	//{
//	//    dxJoint j;
//	////    if (group != null) {
//	////        j = (dxJoint) group.stack.alloc(sizeof(T));
//	////        group.num++;
//	////    } else
//	////        j = (dxJoint) dAlloc(sizeof(T));
//	////    
//	////    new(j) T(w);
//	//    if (group != null)
//	//        j.flags |= dJOINT_INGROUP;
//	//    
//	//    return j;
//	//}
//	static <T extends dxJoint> T createJoint(T j, dxJointGroup group)
//	{
//		if (group != null)
//			j.flags |= dxJoint.dJOINT_INGROUP;
//
//		return j;
//	}
//
//
//	dxJoint dJointCreateBall (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointBall>(w,group);
//	}
//
//
//	dxJoint dJointCreateHinge (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointHinge>(w,group);
//	}
//
//
//	dxJoint dJointCreateSlider (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointSlider>(w,group);
//	}
//
//
////	dxJoint dJointCreateContact (dWorld w, dJointGroup group,
////			final dContact[] c)
//	public dxJointContact dJointCreateContact (dxWorld w, dxJointGroup group,
//			final dContact c)
//	{
//		dAASSERT (w, c);
//		dxJointContact j = createJoint(new dxJointContact(w), group);
//		j.contact = c;
//		return j;
//	}
//
//
//	public static dxJointHinge2 dJointCreateHinge2 (dxWorld w, dxJointGroup group)
//	{
//		dAASSERT (w);
//		//    return createJoint<dxJointHinge2> (w,group);
//		return createJoint(new dxJointHinge2(w),group);
//	}
//
//
//	public static dxJointUniversal dJointCreateUniversal (dxWorld w, dxJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint(new dxJointUniversal(w), group);
//	}
//
//	dxJoint  dJointCreatePR (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointPR> (w,group);
//	}
//
//	dxJoint  dJointCreatePU (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointPU> (w,group);
//	}
//
//	dxJoint  dJointCreatePiston (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointPiston> (w,group);
//	}
//
//	dxJoint  dJointCreateFixed (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointFixed> (w,group);
//	}
//
//
//	dxJoint  dJointCreateNull (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointNull> (w,group);
//	}
//
//
//	dxJoint  dJointCreateAMotor (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointAMotor> (w,group);
//	}
//
//	dxJoint  dJointCreateLMotor (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointLMotor> (w,group);
//	}
//
//	dxJoint  dJointCreatePlane2D (dWorld w, dJointGroup group)
//	{
//		dAASSERT (w);
//		return createJoint<dxJointPlane2D> (w,group);
//	}
//
//	void dJointDestroy (dxJoint j)
//	{
//		dAASSERT (j);
//		size_t sz = j.size();
//		if (j.flags & dJOINT_INGROUP) return;
//		removeJointReferencesFromAttachedBodies (j);
//		removeObjectFromList (j);
//		j.world.nj--;
//		j.~dxJoint();
//		dFree (j, sz);
//	}
//
//
//	//dJointGroup dJointGroupCreate (int max_size)
//	//{
//	//    // not any more ... dUASSERT (max_size > 0,"max size must be > 0");
//	//    dxJointGroup *group = new dxJointGroup;
//	//    group->num = 0;
//	//    return group;
//	//}
//	//
//	//
//	//void dJointGroupDestroy (dJointGroup group)
//	//{
//	//    dAASSERT (group);
//	//    dJointGroupEmpty (group);
//	//    delete group;
//	//}
//	//
//	//
//	//void dJointGroupEmpty (dJointGroup group)
//	//{
//	//    // the joints in this group are detached starting from the most recently
//	//    // added (at the top of the stack). this helps ensure that the various
//	//    // linked lists are not traversed too much, as the joints will hopefully
//	//    // be at the start of those lists.
//	//    // if any group joints have their world pointer set to 0, their world was
//	//    // previously destroyed. no special handling is required for these joints.
//	//    
//	//    dAASSERT (group);
//	//    int i;
//	//    dxJoint **jlist = (dxJoint**) ALLOCA (group->num * sizeof(dxJoint*));
//	//    dxJoint *j = (dxJoint*) group->stack.rewind();
//	//    for (i=0; i < group->num; i++) {
//	//        jlist[i] = j;
//	//        j = (dxJoint*) (group->stack.next (j->size()));
//	//    }
//	//    for (i=group->num-1; i >= 0; i--) {
//	//        if (jlist[i]->world) {
//	//            removeJointReferencesFromAttachedBodies (jlist[i]);
//	//            removeObjectFromList (jlist[i]);
//	//            jlist[i]->world->nj--;
//	//            jlist[i]->~dxJoint();
//	//        }
//	//    }
//	//    group->num = 0;
//	//    group->stack.freeAll();
//	//}
//	//
//	//int dJointGetNumBodies(dxJoint joint)
//	//{
//	//    // check arguments
//	//    dUASSERT (joint,"bad joint argument");
//	//
//	//    if ( !joint.node[0].body )
//	//        return 0;
//	//    else if ( !joint.node[1].body )
//	//        return 1;
//	//    else
//	//        return 2;
//	//}
//	//
//	//
//
//	public static void dJointAttach (dJoint joint, dBody body1, dBody body2)
//	{	
//		((ODE) joint).dJointAttach(joint, body1, body2);
//	}
//
//	//void dJointAttach (dxJoint joint, dxBody body1, dxBody body2)
//	//{
//	//  // check arguments
//	//  dUASSERT (joint,"bad joint argument");
//	//  dUASSERT (body1 == 0 || body1 != body2,"can't have body1==body2");
//	//  dxWorld world = joint.world;
//	//  dUASSERT ( (!body1 || body1.world == world) &&
//	//	     (!body2 || body2.world == world),
//	//	     "joint and bodies must be in same world");
//	//
//	//  // check if the joint can not be attached to just one body
//	//  dUASSERT (!((jointflags & dJOINT_TWOBODIES) &&
//	//	      ((body1 != 0) ^ (body2 != 0))),
//	//	    "joint can not be attached to just one body");
//	//
//	//  // remove any existing body attachments
//	//  if (joint.node[0].body || joint.node[1].body) {
//	//    removeJointReferencesFromAttachedBodies (joint);
//	//  }
//	//
//	//  // if a body is zero, make sure that it is body2, so 0 --> node[1].body
//	//  if (body1==null) {
//	//    body1 = body2;
//	//    body2 = null;
//	//    joint.flags |= dJOINT_REVERSE;
//	//  }
//	//  else {
//	//    joint.flags &= (~dJOINT_REVERSE);
//	//  }
//	//
//	//  // attach to new bodies
//	//  joint.node[0].body = body1;
//	//  joint.node[1].body = body2;
//	//  if (body1!=null) {
//	//    joint.node[1].next = body1.firstjoint;
//	//    body1.firstjoint = joint.node[1];
//	//  }
//	//  else joint.node[1].next = 0;
//	//  if (body2!=null) {
//	//    joint.node[0].next = body2.firstjoint;
//	//    body2.firstjoint = joint.node[0];
//	//  }
//	//  else {
//	//    joint.node[0].next = 0;
//	//  }
//	//}
//	//
//	//
//	//void dJointSetData (dxJoint joint, void data)
//	//{
//	//  dAASSERT (joint);
//	//  joint.userdata = data;
//	//}
//	//
//	//
//	//void dJointGetData (dxJoint joint)
//	//{
//	//  dAASSERT (joint);
//	//  return joint.userdata;
//	//}
//	//
//	//
//	//dJointType dJointGetType (dxJoint joint)
//	//{
//	//  dAASSERT (joint);
//	//  return joint.type();
//	//}
//	//
//	//
//	//dBody dJointGetBody (dxJoint joint, int index)
//	//{
//	//  dAASSERT (joint);
//	//  if (index == 0 || index == 1) {
//	//    if (joint.flags & dJOINT_REVERSE) return joint.node[1-index].body;
//	//    else return joint.node[index].body;
//	//  }
//	//  else return 0;
//	//}
//	//
//	//
//	//void dJointSetFeedback (dxJoint joint, dJointFeedback f)
//	//{
//	//  dAASSERT (joint);
//	//  joint.feedback = f;
//	//}
//	//
//	//
//	//dJointFeedback dJointGetFeedback (dxJoint joint)
//	//{
//	//  dAASSERT (joint);
//	//  return joint.feedback;
//	//}
//	//
//	//
//	//
//	dJoint dConnectingJoint (dBody in_b1, dBody in_b2)
//	{
//		dAASSERT (in_b1!=null || in_b2!=null);
//
//		dxBody b1, b2;
//
//		if (in_b1 == null) {
//			b1 = (dxBody) in_b2;
//			b2 = (dxBody) in_b1;
//		}
//		else {
//			b1 = (dxBody) in_b1;
//			b2 = (dxBody) in_b2;
//		}
//
//		// look through b1's neighbour list for b2
//		for (dxJointNode n=b1.firstjoint.get(); n!=null; n=n.next) {
//			if (n.body == b2) return n.joint;
//		}
//
//		return null;
//	}
//
//
//
//	int dConnectingJointList (dxBody in_b1, dxBody in_b2, dxJoint[] out_list)
//	{
//		dAASSERT (in_b1!=null || in_b2!=null);
//
//
//		dxBody b1, b2;
//
//		if (in_b1 == null) {
//			b1 = in_b2;
//			b2 = in_b1;
//		}
//		else {
//			b1 = in_b1;
//			b2 = in_b2;
//		}
//
//		// look through b1's neighbour list for b2
//		int numConnectingJoints = 0;
//		for (dxJointNode n=b1.firstjoint.get(); n!=null; n=n.next) {
//			if (n.body == b2)
//				out_list[numConnectingJoints++] = n.joint;
//		}
//
//		return numConnectingJoints;
//	}
//
//
//	int dAreConnected (dBody b1, dBody b2)
//	{
//		dAASSERT (b1!=null && b2!=null);
//		// look through b1's neighbour list for b2
//		for (dxJointNode n=((dxBody)b1).firstjoint.get(); n!=null; n=n.next) {
//			if (n.body == b2) return 1;
//		}
//		return 0;
//	}
//
//
//	int dAreConnectedExcluding (dBody b1, dBody b2, dJointType joint_type)
//	{
//		dAASSERT (b1!=null && b2!=null);
//		// look through b1's neighbour list for b2
//		for (dxJointNode n=((dxBody)b1).firstjoint.get(); n!=null; n=n.next) {
//			if (n.joint.dJointGetType (n.joint) != joint_type && n.body == b2) return 1;
//		}
//		return 0;
//	}
//

	//****************************************************************************
	// testing

	//#define NUM 100
	private static final int NUM = 100; 

	//#define DO(x)


	//extern "C" 
	public void dTestDataStructures()
	{
		int i;
		//DO(
		printf ("testDynamicsStuff()\n");

		DxBody[] body = new DxBody[NUM];
		int nb = 0;
		DxJoint[] joint = new DxJoint[NUM];
		int nj = 0;

		for (i=0; i<NUM; i++) body[i] = null;
		for (i=0; i<NUM; i++) joint[i] = null;

		//DO(
		printf ("creating world\n");
		DxWorld w = DxWorld.dWorldCreate();
		checkWorld (w);

		for (;;) {
			if (nb < NUM && dRandReal() > 0.5) {
				//DO(
				printf ("creating body\n");
				body[nb] = DxBody.dBodyCreate (w);
				//DO(
				printf ("\t--> %p\n",body[nb].toString());
				nb++;
				checkWorld (w);
				//DO(
				printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
			if (nj < NUM && nb > 2 && dRandReal() > 0.5) {
				DxBody b1 = body [(int) (dRand() % nb)];
				DxBody b2 = body [(int) (dRand() % nb)];
				if (b1 != b2) {
					//DO(
					printf ("creating joint, attaching to %p,%p\n",b1,b2);
					joint[nj] = dJointCreateBall (w,null);
					//DO(
					printf ("\t-->%p\n",joint[nj]);
					checkWorld (w);
					joint[nj].dJointAttach (b1,b2);
					nj++;
					checkWorld (w);
					//DO(
					printf ("%d BODIES, %d JOINTS\n",nb,nj);
				}
			}
			if (nj > 0 && nb > 2 && dRandReal() > 0.5) {
				DxBody b1 = body [(int) (dRand() % nb)];
				DxBody b2 = body [(int) (dRand() % nb)];
				if (b1 != b2) {
					int k = (int) (dRand() % nj);
					//DO(
					printf ("reattaching joint %p\n",joint[k]);
					joint[k].dJointAttach (b1,b2);
					checkWorld (w);
					//DO(
					printf ("%d BODIES, %d JOINTS\n",nb,nj);
				}
			}
			if (nb > 0 && dRandReal() > 0.5) {
				int k = (int) (dRand() % nb);
				//DO(
				printf ("destroying body %p\n",body[k]);
				body[k].dBodyDestroy ();
				checkWorld (w);
				for (; k < (NUM-1); k++) body[k] = body[k+1];
				nb--;
				printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
			if (nj > 0 && dRandReal() > 0.5) {
				int k = (int) (dRand() % nj);
				printf ("destroying joint %p\n",joint[k]);
				dJointDestroy (joint[k]);
				checkWorld (w);
				for (; k < (NUM-1); k++) joint[k] = joint[k+1];
				nj--;
				printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
		}

		/*
  printf ("creating world\n");
  dWorld w = dWorldCreate();
  checkWorld (w);
  printf ("creating body\n");
  dBody b1 = dBodyCreate (w);
  checkWorld (w);
  printf ("creating body\n");
  dBody b2 = dBodyCreate (w);
  checkWorld (w);
  printf ("creating joint\n");
  dJoint j = dJointCreateBall (w);
  checkWorld (w);
  printf ("attaching joint\n");
  dJointAttach (j,b1,b2);
  checkWorld (w);
  printf ("destroying joint\n");
  dJointDestroy (j);
  checkWorld (w);
  printf ("destroying body\n");
  dBodyDestroy (b1);
  checkWorld (w);
  printf ("destroying body\n");
  dBodyDestroy (b2);
  checkWorld (w);
  printf ("destroying world\n");
  dWorldDestroy (w);
		 */
	}

	//****************************************************************************
	// configuration
//	#if 1
//	#define REGISTER_EXTENSION( __a )  #__a " "
//	#else
//		#define REGISTER_EXTENSION( __a )  "__a "
//	#endif
	//private Object REGISTER_EXTENSION;
	//TODO ?
//	if (true) {
//		REGISTER_EXTENSION( __a )  #__a " "
//	#else
//		#define REGISTER_EXTENSION( __a )  "__a "
//	#endif
	private static void REGISTER_EXTENSION(String s) {
		ode_configuration += s + " ";
	}

	//  String[] ?!?!?
	private static String ode_configuration = "ODE ";

		// EXTENSION LIST BEGIN
		//**********************************

//		#ifdef dNODEBUG
//		REGISTER_EXTENSION( ODE_EXT_no_debug )
//	#endif // dNODEBUG
//
//	#if dTRIMESH_ENABLED
//	REGISTER_EXTENSION( ODE_EXT_trimesh )
//
//	// tri-mesh extensions
//	#if dTRIMESH_OPCODE
//	REGISTER_EXTENSION( ODE_EXT_opcode )
//
//	// opcode extensions
//	#if dTRIMESH_16BIT_INDICES
//	REGISTER_EXTENSION( ODE_OPC_16bit_indices )
//	#endif
//
//	#if !dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER
//	REGISTER_EXTENSION( ODE_OPC_new_collider )
//	#endif
//
//	#endif // dTRIMESH_OPCODE
//
//	#if dTRIMESH_GIMPACT
//	REGISTER_EXTENSION( ODE_EXT_gimpact )
//
//	// gimpact extensions
//	#endif
//
//	#endif // dTRIMESH_ENABLED
//
//	#if dTLS_ENABLED
//	REGISTER_EXTENSION( ODE_EXT_mt_collisions )
//	#endif // dTLS_ENABLED
	static {
	if (dNODEBUG)
		REGISTER_EXTENSION( "ODE_EXT_no_debug" );

//	if (dUSE_MALLOC_FOR_ALLOCA)
//		REGISTER_EXTENSION( "ODE_EXT_malloc_not_alloca" );

	if (dTRIMESH_ENABLED) {
		REGISTER_EXTENSION( "ODE_EXT_trimesh" );

		// tri-mesh extensions
		if (dTRIMESH_OPCODE) {
			REGISTER_EXTENSION( "ODE_EXT_opcode" );

			// opcode extensions
			if (dTRIMESH_16BIT_INDICES)
				REGISTER_EXTENSION( "ODE_OPC_16bit_indices" );

			if (!dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER)
				REGISTER_EXTENSION( "ODE_OPC_new_collider" );
		} // dTRIMESH_OPCODE

		if (dTRIMESH_GIMPACT) {
			REGISTER_EXTENSION( "ODE_EXT_gimpact" );
		} // gimpact extensions

	} // dTRIMESH_ENABLED

//	if (dTLS_ENABLED) {
//		REGISTER_EXTENSION( "ODE_EXT_mt_collisions" );
//	} // dTLS_ENABLED

	//**********************************
	// EXTENSION LIST END

	// These tokens are mutually exclusive, and always present
	//#ifdef dSINGLE
	//"ODE_single_precision"
	//#else
	//TZ TODO correct?
	ode_configuration +=
		"ODE_double_precision";
	//#endif // dDOUBLE

		//; // END
	}
	
	//const char* 
	public String _dGetConfiguration ()
	{
		return ode_configuration;
	}


	// Helper to check for a feature of ODE
//	int dCheckConfiguration( const char* extension )
	public boolean _dCheckConfiguration( final String extension )
	{
		//final String start;
		int start;
		//char * where, terminator;
		int where;
		int terminator;

		/* Feature names should not have spaces. */
//		where = (char*)strchr(extension, ' ');
//		if ( where || *extension == '\0')
//			return 1;
		if (extension.indexOf(' ') >= 0 || extension.length() == 0)
			return true;  //TODO TZ report. should this not return 'false' instead?

		final String config = getConfiguration();

		final int ext_length = extension.length();//strlen(extension);

		/* It takes a bit of care to be fool-proof. Don't be fooled by sub-strings, etc. */
		start = 0;//config;
		for (  ; ;  )
		{
			//where = (char*)strstr((const char *) start, extension);
			where = config.indexOf(extension, start);
			if (where == -1)//if (!where)
				break;

			terminator = where + ext_length;

//			if ( (where == start || *(where - 1) == ' ') && 
//					(*terminator == ' ' || *terminator == '\0') )
			if ( (where == start || extension.charAt(where - 1) == ' ') && 
					(extension.charAt(terminator) == ' ' || 
							terminator == extension.length()) )
			{
				return true;
			}

			start = terminator;
		}

		return false;
	}
}
