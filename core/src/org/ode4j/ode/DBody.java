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
package org.ode4j.ode;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

/**
 * @defgroup bodies Rigid Bodies
 *
 * A rigid body has various properties from the point of view of the
 * simulation. Some properties change over time:
 * <p>
 *  <li> Position vector (x,y,z) of the body's point of reference.
 *      Currently the point of reference must correspond to the body's center of mass.
 *  <li> Linear velocity of the point of reference, a vector (vx,vy,vz).
 *  <li> Orientation of a body, represented by a quaternion (qs,qx,qy,qz) or
 *      a 3x3 rotation matrix.
 *  <li> Angular velocity vector (wx,wy,wz) which describes how the orientation
 *      changes over time.
 * <p>
 * Other body properties are usually constant over time:
 * <p>
 *  <li> Mass of the body.
 *  <li> Position of the center of mass with respect to the point of reference.
 *      In the current implementation the center of mass and the point of
 *      reference must coincide.
 *  <li> Inertia matrix. This is a 3x3 matrix that describes how the body's mass
 *      is distributed around the center of mass. Conceptually each body has an
 *      x-y-z coordinate frame embedded in it that moves and rotates with the body.
 * <p>
 * The origin of this coordinate frame is the body's point of reference. Some values
 * in ODE (vectors, matrices etc) are relative to the body coordinate frame, and others
 * are relative to the global coordinate frame.
 * <p>
 * Note that the shape of a rigid body is not a dynamical property (except insofar as
 * it influences the various mass properties). It is only collision detection that cares
 * about the detailed shape of the body.
 * <p>
 * From odecpp.h.
 */
public interface DBody {

	//~dBody()
	//void DESTRUCTOR();
	void destroy();

	//	  public void create (dWorld world) {
	//		  if (_id!=null) dBodyDestroy (_id);
	//		  _id = dBodyCreate (world);
	//	  }

	//void setData (void *data)
	void setData (Object data);
	//void *getData() 
	Object getData();

	void setPosition (double x, double y, double z);
	void setPosition (DVector3C p);

	void setRotation (DMatrix3C R);
	void setQuaternion (DQuaternionC q);
	void setLinearVel (double x, double y, double z);
	void setLinearVel (DVector3C v);
	void setAngularVel (double x, double y, double z);
	void setAngularVel (DVector3C v);

	DVector3C getPosition();
	DMatrix3C getRotation();
	DQuaternionC getQuaternion();
	DVector3C getLinearVel();
	DVector3C getAngularVel();

	//  void setMass (final dMass *mass)
	//  { dBodySetMass (_id,mass); }
	//void setMass (final dMass &mass)
	//  { setMass (&mass); }
	void setMass (DMassC mass);
	DMassC getMass ();

	void addForce (double fx, double fy, double fz);
	void addForce (DVector3C f);
	void addTorque (double fx, double fy, double fz);
	void addTorque (DVector3C t);

	void addRelForce (double fx, double fy, double fz);
	void addRelForce (DVector3C f);
	void addRelTorque (double fx, double fy, double fz);
	void addRelTorque (DVector3C t);

	void addForceAtPos (double fx, double fy, double fz,
			double px, double py, double pz);
	void addForceAtPos (DVector3C f, DVector3C p);

	void addForceAtRelPos (double fx, double fy, double fz,
			double px, double py, double pz);
	void addForceAtRelPos (DVector3C f, DVector3C p);

	void addRelForceAtPos (double fx, double fy, double fz,
			double px, double py, double pz);
	void addRelForceAtPos (DVector3C f, DVector3C p);

	void addRelForceAtRelPos (double fx, double fy, double fz,
			double px, double py, double pz);
	void addRelForceAtRelPos (DVector3C f, DVector3C p);

	DVector3C getForce();
	DVector3C getTorque();
	void setForce (double x, double y, double z);
	void setForce (DVector3C f);
	void setTorque (double x, double y, double z);
	void setTorque (DVector3C t);

	void enable();
	void disable();
	boolean isEnabled();

	void getRelPointPos (double px, double py, double pz, DVector3 result);
	void getRelPointPos (DVector3C p, DVector3 result);

	void getRelPointVel (double px, double py, double pz, DVector3 result);
	void getRelPointVel (DVector3C p, DVector3 result);

	void getPointVel (double px, double py, double pz, DVector3 result);
	void getPointVel (DVector3C p, DVector3 result);

	void getPosRelPoint (double px, double py, double pz, DVector3 result);
	void getPosRelPoint (DVector3C p, DVector3 result);

	void vectorToWorld (double px, double py, double pz, DVector3 result);
	void vectorToWorld (DVector3C p, DVector3 result);

	void vectorFromWorld (double px, double py, double pz, DVector3 result);
	void vectorFromWorld (DVector3C p, DVector3 result);

	void setFiniteRotationMode (boolean mode);

	void setFiniteRotationAxis (double x, double y, double z);
	void setFiniteRotationAxis (DVector3C a);

	boolean getFiniteRotationMode();
	void getFiniteRotationAxis (DVector3 result);

	int getNumJoints();
	DJoint getJoint (int index);

	void setGravityMode (boolean mode);
	boolean getGravityMode();

	boolean isConnectedTo (DBody body);

	void  setAutoDisableLinearThreshold (double threshold);
	double getAutoDisableLinearThreshold();
	void setAutoDisableAngularThreshold (double threshold);
	double getAutoDisableAngularThreshold();
	void setAutoDisableSteps (int steps);
	int getAutoDisableSteps();
	void setAutoDisableTime (double time);
	double getAutoDisableTime();
	void setAutoDisableFlag (boolean do_auto_disable);
	boolean getAutoDisableFlag();

	double getLinearDamping();
	void setLinearDamping(double scale);
	 double getAngularDamping();
	void setAngularDamping(double scale);
	void setDamping(double linear_scale, double angular_scale);
	double getLinearDampingThreshold();
	void setLinearDampingThreshold(double threshold);
	double getAngularDampingThreshold();
	void setAngularDampingThreshold(double threshold);
	void setDampingDefaults();

	double getMaxAngularSpeed();
	void setMaxAngularSpeed(double max_speed);

	//	private dxBody _id;
	//  // intentionally undefined, don't use these
	//  //dBody (const dBody &);
	//  //void operator= (const dBody &);
	//
	////public:
		//  public dBody()
	//    { _id = null; }
	//  dBody (dWorld world)
	//    { _id = dBodyCreate (world); }
	//  dBody (dWorld world)
	//    { _id = dBodyCreate (world); }
	//  //~dBody()
	//  @Override
	//  protected void DESTRUCTOR()
	//    { if (_id!=null) dBodyDestroy (_id); super.DESTRUCTOR(); }
	//
	//  public void create (dWorld world) {
	//	  if (_id!=null) dBodyDestroy (_id);
	//	  _id = dBodyCreate (world);
	//  }
	//
	//  public dBody id() 
	//    { return _id; }
	//  //TODO
	////  operator dBody() const
	////    { return _id; }
	//
	//  //void setData (void *data)
	//  public void setData (Object data)
	//     { dBodySetData (_id,data); }
	//  //void *getData() 
	//  public Object getData() 
	//    { return dBodyGetData (_id); }
	//
	//  public void setPosition (double x, double y, double z)
	//    { dBodySetPosition (_id,x,y,z); }
	//  void setPosition (final dVector3 p)
	//    { setPosition(p.v[0], p.v[1], p.v[2]); }
	//
	//  void setRotation (final dMatrix3 R)
	//    { dBodySetRotation (_id,R); }
	//  void setQuaternion (final dQuaternion q)
	//    { dBodySetQuaternion (_id,q); }
	//  void setLinearVel (double x, double y, double z)
	//    { dBodySetLinearVel (_id,x,y,z); }
	//  void setLinearVel (final dVector3 v)
	//    { setLinearVel(v.v[0], v.v[1], v.v[2]); }
	//  void setAngularVel (double x, double y, double z)
	//    { dBodySetAngularVel (_id,x,y,z); }
	//  void setAngularVel (final dVector3 v)
	//    { setAngularVel (v.v[0], v.v[1], v.v[2]); }
	//
	//  public final dVector3 getPosition() 
	//    { return dBodyGetPosition (_id); }
	//  public final dMatrix3 getRotation() //const
	//    { return dBodyGetRotation (_id); }
	//  final dQuaternion getQuaternion() //const
	//    { return dBodyGetQuaternion (_id); }
	//  final dVector3 getLinearVel() //const
	//    { return dBodyGetLinearVel (_id); }
	//  final dVector3 getAngularVel() //const
	//    { return dBodyGetAngularVel (_id); }
	//
	////  void setMass (final dMass *mass)
	////  { dBodySetMass (_id,mass); }
	////void setMass (final dMass &mass)
	////  { setMass (&mass); }
	//  public void setMass (final dMass mass)
	//  { _id.dBodySetMass ((dxMass) mass); }
	//  dMass getMass () //const
	//    { dMass mass = new dxMass(); _id.dBodyGetMass ((dxMass) mass); return mass; }
	//
	//  void addForce (double fx, double fy, double fz)
	//    { dBodyAddForce (_id, fx, fy, fz); }
	//  void addForce (final dVector3 f)
	//    { addForce (f.v[0], f.v[1], f.v[2]); }
	//  void addTorque (double fx, double fy, double fz)
	//    { dBodyAddTorque (_id, fx, fy, fz); }
	//  void addTorque (final dVector3 t)
	//    { addTorque(t.v[0], t.v[1], t.v[2]); }
	//
	//  void addRelForce (double fx, double fy, double fz)
	//    { dBodyAddRelForce (_id, fx, fy, fz); }
	//  void addRelForce (final dVector3 f)
	//    { addRelForce (f.v[0], f.v[1], f.v[2]); }
	//  void addRelTorque (double fx, double fy, double fz)
	//    { dBodyAddRelTorque (_id, fx, fy, fz); }
	//  void addRelTorque (final dVector3 t)
	//    { addRelTorque (t.v[0], t.v[1], t.v[2]); }
	//
	//  void addForceAtPos (double fx, double fy, double fz,
	//		      double px, double py, double pz)
	//    { dBodyAddForceAtPos (_id, fx, fy, fz, px, py, pz); }
	//  void addForceAtPos (final dVector3 f, final dVector3 p)
	//    { addForceAtPos (f.v[0], f.v[1], f.v[2], p.v[0], p.v[1], p.v[2]); }
	//
	//  void addForceAtRelPos (double fx, double fy, double fz,
	//                         double px, double py, double pz)
	//    { dBodyAddForceAtRelPos (_id, fx, fy, fz, px, py, pz); }
	//  void addForceAtRelPos (final dVector3 f, final dVector3 p)
	//    { addForceAtRelPos (f.v[0], f.v[1], f.v[2], p.v[0], p.v[1], p.v[2]); }
	//
	//  void addRelForceAtPos (double fx, double fy, double fz,
	//			 double px, double py, double pz)
	//    { dBodyAddRelForceAtPos (_id, fx, fy, fz, px, py, pz); }
	//  void addRelForceAtPos (final dVector3 f, final dVector3 p)
	//    { addRelForceAtPos (f.v[0], f.v[1], f.v[2], p.v[0], p.v[1], p.v[2]); }
	//
	//  void addRelForceAtRelPos (double fx, double fy, double fz,
	//			    double px, double py, double pz)
	//    { dBodyAddRelForceAtRelPos (_id, fx, fy, fz, px, py, pz); }
	//  void addRelForceAtRelPos (final dVector3 f, final dVector3 p)
	//    { addRelForceAtRelPos (f.v[0], f.v[1], f.v[2], p.v[0], p.v[1], p.v[2]); }
	//
	//  final double [] getForce() //const
	//    { return dBodyGetForce(_id); }
	//  final double [] getTorque() //const
	//    { return dBodyGetTorque(_id); }
	//  void setForce (double x, double y, double z)
	//    { dBodySetForce (_id,x,y,z); }
	//  void setForce (final dVector3 f)
	//    { setForce (f.v[0], f.v[1], f.v[2]); }
	//  void setTorque (double x, double y, double z)
	//    { dBodySetTorque (_id,x,y,z); }
	//  void setTorque (final dVector3 t)
	//  { setTorque (t.v[0], t.v[1], t.v[2]); }
	//
	//  void enable()
	//    { dBodyEnable (_id); }
	//  void disable()
	//    { dBodyDisable (_id); }
	//  boolean isEnabled() //const
	//    { return dBodyIsEnabled (_id) != 0; }
	//
	//  void getRelPointPos (double px, double py, double pz, dVector3 result) //const
	//    { dBodyGetRelPointPos (_id, px, py, pz, result); }
	//  void getRelPointPos (final dVector3 p, dVector3 result) //const
	//    { getRelPointPos (p.v[0], p.v[1], p.v[2], result); }
	//
	//  void getRelPointVel (double px, double py, double pz, dVector3 result) //const
	//    { dBodyGetRelPointVel (_id, px, py, pz, result); }
	//  void getRelPointVel (final dVector3 p, dVector3 result) //const
	//    { getRelPointVel (p.v[0], p.v[1], p.v[2], result); }
	//
	//  void getPointVel (double px, double py, double pz, dVector3 result) //const
	//    { dBodyGetPointVel (_id, px, py, pz, result); }
	//  void getPointVel (final dVector3 p, dVector3 result) //const
	//    { getPointVel (p.v[0], p.v[1], p.v[2], result); }
	//
	//  void getPosRelPoint (double px, double py, double pz, dVector3 result) //const
	//    { dBodyGetPosRelPoint (_id, px, py, pz, result); }
	//  void getPosRelPoint (final dVector3 p, dVector3 result) //const
	//    { getPosRelPoint (p.v[0], p.v[1], p.v[2], result); }
	//
	//  void vectorToWorld (double px, double py, double pz, dVector3 result) //const
	//    { dBodyVectorToWorld (_id, px, py, pz, result); }
	//  void vectorToWorld (final dVector3 p, dVector3 result) //const
	//    { vectorToWorld (p.v[0], p.v[1], p.v[2], result); }
	//
	//  void vectorFromWorld (double px, double py, double pz, dVector3 result) //const
	//    { dBodyVectorFromWorld (_id,px,py,pz,result); }
	//  void vectorFromWorld (final dVector3 p, dVector3 result) //const
	//    { vectorFromWorld (p.v[0], p.v[1], p.v[2], result); }
	//
	//  void setFiniteRotationMode (bool mode)
	//    { dBodySetFiniteRotationMode (_id, mode); }
	//
	//  void setFiniteRotationAxis (double x, double y, double z)
	//    { dBodySetFiniteRotationAxis (_id, x, y, z); }
	//  void setFiniteRotationAxis (final dVector3 a)
	//    { setFiniteRotationAxis (a.v[0], a.v[1], a.v[2]); }
	//
	//  boolean getFiniteRotationMode() //const
	//    { return dBodyGetFiniteRotationMode (_id) != 0; }
	//  void getFiniteRotationAxis (dVector3 result) //const
	//    { dBodyGetFiniteRotationAxis (_id, result); }
	//
	//  int getNumJoints() //const
	//    { return dBodyGetNumJoints (_id); }
	//  dJoint getJoint (int index) //const
	//    { return dBodyGetJoint (_id, index); }
	//
	//  void setGravityMode (boolean mode)
	//    { dBodySetGravityMode (_id,mode); }
	//  boolean getGravityMode() //const
	//    { return dBodyGetGravityMode (_id) != 0; }
	//
	//  boolean isConnectedTo (dBody body) //const
	//    { return dAreConnected (_id, body) != 0; }
	//
	//  void  setAutoDisableLinearThreshold (double threshold)
	//    { dBodySetAutoDisableLinearThreshold (_id,threshold); }
	//  double getAutoDisableLinearThreshold() //const
	//    { return dBodyGetAutoDisableLinearThreshold (_id); }
	//  void setAutoDisableAngularThreshold (double threshold)
	//    { dBodySetAutoDisableAngularThreshold (_id,threshold); }
	//  double getAutoDisableAngularThreshold() //const
	//    { return dBodyGetAutoDisableAngularThreshold (_id); }
	//  void setAutoDisableSteps (int steps)
	//    { dBodySetAutoDisableSteps (_id,steps); }
	//  int getAutoDisableSteps() //const
	//    { return dBodyGetAutoDisableSteps (_id); }
	//  void setAutoDisableTime (double time)
	//    { dBodySetAutoDisableTime (_id,time); }
	//  double getAutoDisableTime() //const
	//    { return dBodyGetAutoDisableTime (_id); }
	//  void setAutoDisableFlag (boolean do_auto_disable)
	//    { dBodySetAutoDisableFlag (_id,do_auto_disable); }
	//  boolean getAutoDisableFlag() //const
	//    { return dBodyGetAutoDisableFlag (_id) != 0; }
	//
	//  double getLinearDamping() //const
	//    { return dBodyGetLinearDamping(_id); }
	//  void setLinearDamping(double scale)
	//    { dBodySetLinearDamping(_id, scale); }
	//  double getAngularDamping() //const
	//    { return dBodyGetAngularDamping(_id); }
	//  void setAngularDamping(double scale)
	//    { dBodySetAngularDamping(_id, scale); }
	//  void setDamping(double linear_scale, double angular_scale)
	//    { dBodySetDamping(_id, linear_scale, angular_scale); }
	//   double getLinearDampingThreshold() //const
	//    { return dBodyGetLinearDampingThreshold(_id); }
	//   void setLinearDampingThreshold(double threshold) //const
	//    { dBodySetLinearDampingThreshold(_id, threshold); }
	//   double getAngularDampingThreshold() //const
	//    { return dBodyGetAngularDampingThreshold(_id); }
	//   void setAngularDampingThreshold(double threshold)
	//    { dBodySetAngularDampingThreshold(_id, threshold); }
	//   void setDampingDefaults()
	//    { dBodySetDampingDefaults(_id); }
	//
	//   double getMaxAngularSpeed() //const
	//    { return dBodyGetMaxAngularSpeed(_id); }
	//   void setMaxAngularSpeed(double max_speed)
	//    { dBodySetMaxAngularSpeed(_id, max_speed); }

}
