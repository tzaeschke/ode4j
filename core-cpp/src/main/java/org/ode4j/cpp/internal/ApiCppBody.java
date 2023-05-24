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
package org.ode4j.cpp.internal;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DMassC;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBody.BodyMoveCallBack;

@SuppressWarnings("deprecation")
public abstract class ApiCppBody extends ApiCppJoint {

	/**
	 * Get auto disable linear average threshold.
	 * 
	 * @param b body
	 * @return the threshold
	 */
	//ODE_API 
	public static double dBodyGetAutoDisableLinearThreshold (DBody b){
		return b.getAutoDisableLinearThreshold();
	}

	/**
	 * Set auto disable linear average threshold.
	 * 
	 * @param b body
	 * @param linear_average_threshold the threshold
	 */
	//ODE_API 
	public static void dBodySetAutoDisableLinearThreshold (
			DBody b, double linear_average_threshold){
		b.setAutoDisableLinearThreshold(linear_average_threshold);
	}

	/**
	 * Get auto disable angular average threshold.
	 * 
	 * @param b body
	 * @return the threshold
	 */
	//ODE_API 
	public static double dBodyGetAutoDisableAngularThreshold (DBody b){
		return b.getAutoDisableAngularThreshold();
	}

	/**
	 * Set auto disable angular average threshold.
	 * 
	 * @param b body
	 * @param angular_average_threshold the threshold
	 */
	//ODE_API 
	public static void dBodySetAutoDisableAngularThreshold (
			DBody b, double angular_average_threshold){
		b.setAutoDisableAngularThreshold(angular_average_threshold);
	}

	/**
	 * Get auto disable average size (samples count).
	 * 
	 * @param b body
	 * @return the nr of steps/size.
	 */
	//ODE_API 
	public static int dBodyGetAutoDisableAverageSamplesCount (DBody b){
		return b.getAutoDisableAverageSamplesCount();
	}

	/**
	 * Set auto disable average buffer size (average steps).
	 * 
	 * @param b body
	 * @param average_samples_count the nr of samples to review.
	 */
	//ODE_API 
	//	 void dBodySetAutoDisableAverageSamplesCount (dBody b, 
	//			 unsigned int average_samples_count){
	public static void dBodySetAutoDisableAverageSamplesCount (DBody b, 
			int average_samples_count){
		b.setAutoDisableAverageSamplesCount(average_samples_count);
	}


	/**
	 * Get auto steps a body must be thought of as idle to disable
	 * 
	 * @param b body
	 * @return the nr of steps
	 */
	//ODE_API 
	public static int dBodyGetAutoDisableSteps (DBody b){
		return b.getAutoDisableSteps();
	}

	/**
	 * Set auto disable steps.
	 * 
	 * @param b body
	 * @param steps the nr of steps.
	 */
	//ODE_API 
	public static void dBodySetAutoDisableSteps (DBody b, int steps){
		b.setAutoDisableSteps(steps);
	}

	/**
	 * Get auto disable time.
	 * 
	 * @param b body
	 * @return nr of seconds
	 */
	//ODE_API 
	public static double dBodyGetAutoDisableTime (DBody b){
		return b.getAutoDisableTime();
	}

	/**
	 * Set auto disable time.
	 * 
	 * @param b body
	 * @param time nr of seconds.
	 */
	//ODE_API 
	public static void dBodySetAutoDisableTime (DBody b, double time){
		b.setAutoDisableTime(time);
	}

	/**
	 * Get auto disable flag.
	 * 
	 * @param b body
	 * @return 0 or 1
	 */
	//ODE_API 
	public static boolean dBodyGetAutoDisableFlag (DBody b){
		return b.getAutoDisableFlag();
	}

	/**
	 * Set auto disable flag.
	 * 
	 * @param b body
	 * @param do_auto_disable 0 or 1
	 */
	//ODE_API 
	public static void dBodySetAutoDisableFlag (DBody b, boolean do_auto_disable){
		b.setAutoDisableFlag(do_auto_disable);
	}

	/**
	 * Set auto disable defaults.
	 * <p>REMARKS:
	 * Set the values for the body to those set as default for the world.
	 * 
	 * @param b body
	 */
	//ODE_API 
	public static void dBodySetAutoDisableDefaults (DBody b){
		b.setAutoDisableDefaults();
	}


	/**
	 * Retrieves the world attached to te given body.
	 * @param b body
	 * @return World
	 */
	//ODE_API 
	public static DWorld dBodyGetWorld (DBody b){
		return b.getWorld();
	}

	/**
	 * Create a body in given world.
	 * 
	 * Default mass parameters are at position (0,0,0).
	 * @param w w
	 * @return ret
	 */
	//ODE_API 
	public static DBody dBodyCreate (DWorld w){
		return OdeHelper.createBody(w);
	}

	/**
	 * Destroy a body.
	 * 
	 * All joints that are attached to this body will be put into limbo:
	 * i.e. unattached and not affecting the simulation, but they will NOT be
	 * deleted.
	 * @param b b
	 * 
	 */
	//ODE_API 
	public static void dBodyDestroy (DBody b){
		b.destroy();
	}

	/**
	 * Set the body's user-data pointer.
	 * @param b b
	 * 
	 * @param data arbitraty pointer
	 */
	//ODE_API 
	public static void  dBodySetData (DBody b, Object data){
		b.setData(data);
	}

	/**
	 * Get the body's user-data pointer.
	 * @param b b
	 * 
	 * @return a pointer to the user's data.
	 */
	//ODE_API 
	public static Object dBodyGetData (DBody b){
		return b.getData();
	}

	/**
	 * Set position of a body.
	 * 
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param b b
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dBodySetPosition   (DBody b, double x, double y, double z){
		b.setPosition(x, y, z);
	}

	/**
	 * Set the orientation of a body.
	 * 
	 * <p>REMARKS:
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param b b
	 * @param R R
	 */
	//ODE_API 
	public static void dBodySetRotation   (DBody b, final DMatrix3 R){
		b.setRotation(R);
	}

	/**
	 * Set the orientation of a body.
	 * 
	 * <p>REMARKS:
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param b b
	 * @param q q
	 */
	//ODE_API 
	public static void dBodySetQuaternion (DBody b, final DQuaternion q){
		b.setQuaternion(q);
	}

	/**
	 * Set the linear velocity of a body.
	 * @param b b
	 * @param x x
	 * @param y y
	 * @param z z
	 * 
	 */
	//ODE_API 
	public static void dBodySetLinearVel  (DBody b, double x, double y, double z){
		b.setLinearVel(x, y, z);
	}

	/**
	 * Set the angular velocity of a body.
	 * @param b b
	 * @param x x
	 * @param y y
	 * @param z z
	 * 
	 */
	//ODE_API 
	public static void dBodySetAngularVel (DBody b, double x, double y, double z){
		b.setAngularVel(x, y, z);
	}

	/**
	 * Get the position of a body.
	 * 
	 * <p>REMARKS:
	 * When getting, the returned values are pointers to internal data structures,
	 * so the vectors are valid until any changes are made to the rigid body
	 * system structure.
	 * @param b b
	 * @return ret
	 * @see #dBodyCopyPosition(DBody, DVector3)
	 */
	//ODE_API 
	//const 
	public static DVector3C dBodyGetPosition (DBody b){
		return b.getPosition();
	}


	/**
	 * Copy the position of a body into a vector.
	 * 
	 * @param body  the body to query
	 * @param pos   a copy of the body position
	 * @see #dBodyGetPosition(DBody)
	 */
	//ODE_API 
	public static void dBodyCopyPosition (DBody body, DVector3 pos){
		pos.set(body.getPosition());
	}


	/**
	 * Get the rotation of a body.
	 * @param b b
	 * 
	 * @return pointer to a 4x3 rotation matrix.
	 */
	//ODE_API 
	//const 
	public static DMatrix3C dBodyGetRotation (DBody b){
		return b.getRotation();
	}


	/**
	 * Copy the rotation of a body.
	 * 
	 * @param b		the body to query
	 * @param R    	a copy of the rotation matrix
	 * @see #dBodyGetRotation(DBody)
	 */
	//ODE_API 
	public static void dBodyCopyRotation (DBody b, DMatrix3 R){
		R.set(b.getRotation());
	}


	/**
	 * Get the rotation of a body.
	 * @param b b
	 * 
	 * @return pointer to 4 scalars that represent the quaternion.
	 */
	//ODE_API 
	//const double * 
	public static DQuaternionC dBodyGetQuaternion (DBody b){
		return b.getQuaternion();
	}


	/**
	 * Copy the orientation of a body into a quaternion.
	 * 
	 * @param body  the body to query
	 * @param quat  a copy of the orientation quaternion
	 * @see #dBodyGetQuaternion(DBody)
	 */
	//ODE_API 
	public static void dBodyCopyQuaternion(DBody body, DQuaternion quat){
		quat.set(body.getQuaternion());
	}


	/**
	 * Get the linear velocity of a body.
	 * @param b b
	 * @return ret
	 * 
	 */
	//ODE_API 
	//const double * 
	public static DVector3C dBodyGetLinearVel (DBody b){
		return b.getLinearVel();
	}

	/**
	 * Get the angular velocity of a body.
	 * @param b b
	 * @return ret
	 * 
	 */
	//ODE_API 
	//const double * 
	public static DVector3C dBodyGetAngularVel (DBody b){
		return b.getAngularVel();
	}

	/**
	 * Set the mass of a body.
	 * @param b b
	 * @param mass mass 
	 * 
	 */
	//ODE_API 
	public static void dBodySetMass (DBody b, DMassC mass){
		b.setMass(mass);
	}

	/**
	 * Get the mass of a body.
	 * @param b b
	 * @param mass mass 
	 * 
	 */
	//ODE_API 
	//	 void dBodyGetMass (dBody b, dMass *mass){
	public static void dBodyGetMass (DBody b, DMass mass){
		mass.setI(b.getMass().getI());
		mass.setC(b.getMass().getC());
		mass.setMass(b.getMass().getMass());
	}

	/**
	 * Add force at centre of mass of body in absolute coordinates.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddForce (DBody b, double fx, double fy, double fz){
		b.addForce(fx, fy, fz);
	}

	/**
	 * Add torque at centre of mass of body in absolute coordinates.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddTorque (DBody b, double fx, double fy, double fz){
		b.addTorque(fx, fy, fz);
	}

	/**
	 * Add force at centre of mass of body in coordinates relative to body.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddRelForce (DBody b, double fx, double fy, double fz){
		b.addRelForce(fx, fy, fz);
	}

	/**
	 * Add torque at centre of mass of body in coordinates relative to body.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddRelTorque (DBody b, double fx, double fy, double fz){
		b.addRelTorque(new DVector3(fx, fy, fz));
	}

	/**
	 * Add force at specified point in body in global coordinates.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddForceAtPos (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		b.addForceAtPos(fx, fy, fz, px, py, pz);
	}
	/**
	 * Add force at specified point in body in local coordinates.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddForceAtRelPos (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		b.addForceAtRelPos(fx, fy, fz, px, py, pz);
	}
	/**
	 * Add force at specified point in body in global coordinates.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddRelForceAtPos (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		b.addRelForceAtPos(fx, fy, fz, px, py, pz);
	}
	/**
	 * Add force at specified point in body in local coordinates.
	 * @param b b
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 */
	//ODE_API 
	public static void dBodyAddRelForceAtRelPos (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		b.addRelForceAtRelPos(fx, fy, fz, px, py, pz);
	}

	/**
	 * Return the current accumulated force vector.
	 * <p>REMARKS:
	 * The returned values are pointers to internal data structures, so
	 * the vectors are only valid until any changes are made to the rigid
	 * body system.
	 * @param b b
	 * 
	 * @return points to an array of 3 reals.
	 */
	//ODE_API 
	//const double * 
	public static DVector3C dBodyGetForce (DBody b){
		return b.getForce();
	}

	/**
	 * Return the current accumulated torque vector.
	 * <p>REMARKS:
	 * The returned values are pointers to internal data structures, so
	 * the vectors are only valid until any changes are made to the rigid
	 * body system.
	 * @param b b
	 * 
	 * @return points to an array of 3 reals.
	 */
	//ODE_API 
	//const double * 
	public static DVector3C dBodyGetTorque (DBody b){
		return b.getTorque();
	}

	/**
	 * Set the body force accumulation vector.
	 * <p>REMARKS:
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @param b b
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dBodySetForce  (DBody b, double x, double y, double z){
		b.setForce(x, y, z);
	}

	/**
	 * Set the body torque accumulation vector.
	 * <p>REMARKS:
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @param b b
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dBodySetTorque (DBody b, double x, double y, double z){
		b.setTorque(x, y, z);
	}

	/**
	 * Get world position of a relative point on body.
	 * @param b b
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 * @param result will contain the result.
	 */
	//ODE_API 
	public static void dBodyGetRelPointPos (
			DBody b, double px, double py, double pz,
			DVector3 result){
		b.getRelPointPos(px, py, pz, result);
	}

	/**
	 * Get velocity vector in global coords of a relative point on body.
	 * @param b b
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 * @param result will contain the result.
	 */
	//ODE_API 
	public static void dBodyGetRelPointVel(
			DBody b, double px, double py, double pz,
			DVector3 result){
		b.getRelPointVel(px, py, pz, result);
	}

	/**
	 * Get velocity vector in global coords of a globally
	 * specified point on a body.
	 * @param b b
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 * @param result will contain the result.
	 */
	//ODE_API 
	public static void dBodyGetPointVel
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		b.getPointVel(px, py, pz, result);
	}

	/**
	 * Takes a point in global coordinates and returns
	 * the point's position in body-relative coordinates.
	 * <p>REMARKS:
	 * This is the inverse of dBodyGetRelPointPos()
	 * @param b b
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 * @param result will contain the result.
	 */
	//ODE_API 
	public static void dBodyGetPosRelPoint
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		b.getPosRelPoint(px, py, pz, result);
	}

	/**
	 * Convert from local to world coordinates.
	 * @param b b
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 * @param result will contain the result.
	 */
	//ODE_API 
	public static void dBodyVectorToWorld
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		b.vectorToWorld(px, py, pz, result);
	}

	/**
	 * Convert from world to local coordinates.
	 * @param b b
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * 
	 * @param result will contain the result.
	 */
	//ODE_API 
	public static void dBodyVectorFromWorld
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		b.vectorFromWorld(px, py, pz, result);
	}

	/**
	 * Controls the way a body's orientation is updated at each timestep.
	 * @param b b
	 * 
	 * @param mode can be 0 or 1:
	 * <ul>
	 * <li> 0: An ``infinitesimal'' orientation update is used.
	 * This is fast to compute, but it can occasionally cause inaccuracies
	 * for bodies that are rotating at high speed, especially when those
	 * bodies are joined to other bodies.
	 * This is the default for every new body that is created.</li>
	 * <li> 1: A ``finite'' orientation update is used.
	 * This is more costly to compute, but will be more accurate for high
	 * speed rotations. </li>
	 * </ul>
	 * <p>REMARKS:
	 * Note however that high speed rotations can result in many types of
	 * error in a simulation, and the finite mode will only fix one of those
	 * sources of error.
	 */
	//ODE_API 
	public static void dBodySetFiniteRotationMode (DBody b, boolean mode){
		b.setFiniteRotationMode(mode);
	}

	/**
	 * Sets the finite rotation axis for a body.
	 * 
	 * <p>REMARKS:
	 * This is axis only has meaning when the finite rotation mode is set
	 * If this axis is zero (0,0,0), full finite rotations are performed on
	 * the body.
	 * If this axis is nonzero, the body is rotated by performing a partial finite
	 * rotation along the axis direction followed by an infinitesimal rotation
	 * along an orthogonal direction.
	 * <p>REMARKS:
	 * This can be useful to alleviate certain sources of error caused by quickly
	 * spinning bodies. For example, if a car wheel is rotating at high speed
	 * you can call this function with the wheel's hinge axis as the argument to
	 * try and improve its behavior.
	 * @param body body
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dBodySetFiniteRotationAxis (
			DBody body, double x, double y, double z){
		body.setFiniteRotationAxis(x, y, z);
	}

	/**
	 * Get the way a body's orientation is updated each timestep.
	 * @param body body
	 * 
	 * @return the mode 0 (infitesimal) or 1 (finite).
	 */
	//ODE_API 
	public static boolean dBodyGetFiniteRotationMode (DBody body){
		return body.getFiniteRotationMode();
	}

	/**
	 * Get the finite rotation axis.
	 * @param body body
	 * @param result will contain the axis.
	 * 
	 */
	//ODE_API 
	public void dBodyGetFiniteRotationAxis (DBody body, DVector3 result){
		body.getFiniteRotationAxis(result);
	}

	/**
	 * Get the number of joints that are attached to this body.
	 * @param b b
	 * 
	 * @return nr of joints
	 */
	//ODE_API 
	public static int dBodyGetNumJoints (DBody b){
		return b.getNumJoints();
	}

	/**
	 * Return a joint attached to this body, given by index.
	 * @param body body
	 * 
	 * @param index valid range is  0 to n-1 where n is the value returned by
	 * dBodyGetNumJoints().
	 * @return ret
	 */
	//ODE_API 
	public DJoint dBodyGetJoint (DBody body, int index){
		return body.getJoint(index);
	}

	/**
	 * Set rigid body to dynamic state (default).
	 * @param body identification of body.
	 * 
	 */
	//ODE_API
	public static void dBodySetDynamic (DBody body) {
		body.setDynamic();
	}

	/**
	 * Set rigid body to kinematic state.
	 * When in kinematic state the body isn't simulated as a dynamic
	 * body (it's "unstoppable", doesn't respond to forces),
	 * but can still affect dynamic bodies (e.g. in joints).
	 * Kinematic bodies can be controlled by position and velocity.
	 * <p>NOTE: A kinematic body has infinite mass. If you set its mass
	 * to something else, it loses the kinematic state and behaves
	 * as a normal dynamic body.
	 * @param body identification of body.
	 * 
	 */
	//ODE_API
	public static void dBodySetKinematic (DBody body) {
		body.setKinematic();
	}

	/**
	 * Check wether a body is in kinematic state.
	 * @param body body
	 * 
	 * @return 1 if a body is kinematic or 0 if it is dynamic.
	 */
	//ODE_API
	public static boolean dBodyIsKinematic (DBody body ) {
		return body.isKinematic();
	}

	/**
	 * Manually enable a body.
	 * @param body identification of body.
	 * 
	 */
	//ODE_API 
	public static void dBodyEnable (DBody body){
		body.enable();
	}

	/**
	 * Manually disable a body.
	 * 
	 * <p>REMARKS:
	 * A disabled body that is connected through a joint to an enabled body will
	 * be automatically re-enabled at the next simulation step.
	 * @param body body
	 */
	//ODE_API 
	public static void dBodyDisable (DBody body){
		body.disable();
	}

	/**
	 * Check wether a body is enabled.
	 * @param body body
	 * 
	 * @return 1 if a body is currently enabled or 0 if it is disabled.
	 */
	//ODE_API 
	public static boolean dBodyIsEnabled (DBody body){
		return body.isEnabled();
	}

	/**
	 * Set whether the body is influenced by the world's gravity or not.
	 * <p>REMARKS:
	 * Newly created bodies are always influenced by the world's gravity.
	 * @param b b
	 * 
	 * @param mode when nonzero gravity affects this body.
	 */
	//ODE_API 
	public static void dBodySetGravityMode (DBody b, boolean mode){
		b.setGravityMode(mode);
	}

	/**
	 * Get whether the body is influenced by the world's gravity or not.
	 * @param b b
	 * 
	 * @return nonzero means gravity affects this body.
	 */
	//ODE_API 
	public static boolean dBodyGetGravityMode (DBody b){
		return b.getGravityMode();
	}


	/**
	 * Set the 'moved' callback of a body.
	 *
	 * Whenever a body has its position or rotation changed during the
	 * timestep, the callback will be called (with body as the argument).
	 * Use it to know which body may need an update in an external
	 * structure (like a 3D engine).
	 *
	 * @param b the body that needs to be watched.
	 * @param callback the callback to be invoked when the body moves. Set to zero
	 * to disable.
	 * 
	 */
	//ODE_API 
	void dBodySetMovedCallback(DBody b, BodyMoveCallBack callback){
		b.setMovedCallback(callback);
	}


	/**
	 * Return the first geom associated with the body.
	 *
	 * You can traverse through the geoms by repeatedly calling
	 * dBodyGetNextGeom().
	 *
	 * @return the first geom attached to this body, or 0.
	 * 
	 */
	//ODE_API 
	DGeom dBodyGetFirstGeom (DBody b){
		return b.getFirstGeom();
	}


	/**
	 * returns the next geom associated with the same body.
	 * @param g a geom attached to some body.
	 * @return the next geom attached to the same body, or 0.
	 * @see #dBodyGetFirstGeom
	 * 
	 */
	//ODE_API 
	DGeom dBodyGetNextGeom (DGeom g){
		return g.getBody().getNextGeom(g);
	}


	/**
	 * Resets the damping settings to the current world's settings.
	 * @param b b
	 * 
	 */
	//ODE_API 
	public static void dBodySetDampingDefaults(DBody b){
		b.setDampingDefaults();
	}

	/**
	 * Get the body's linear damping scale.
	 * @param b b
	 * @return ret
	 * 
	 */
	//ODE_API 
	public static double dBodyGetLinearDamping (DBody b){
		return b.getLinearDamping();
	}

	/**
	 * Set the body's linear damping scale.
	 * 
	 * <p>REMARKS:
	 * From now on the body will not use the world's linear damping
	 * scale until dBodySetDampingDefaults() is called.
	 * @param b b
	 * 
	 * @param scale The linear damping scale. Should be in the interval [0, 1].
	 * @see #dBodySetDampingDefaults(DBody)
	 */
	//ODE_API 
	public static void dBodySetLinearDamping(DBody b, double scale){
		b.setLinearDamping(scale);
	}

	/**
	 * Get the body's angular damping scale.
	 * 
	 * <p>REMARKS:
	 * If the body's angular damping scale was not set, this function
	 * returns the world's angular damping scale.
	 * @param b b
	 * @return ret
	 */
	//ODE_API 
	public static double dBodyGetAngularDamping (DBody b){
		return b.getAngularDamping();
	}

	/**
	 * Set the body's angular damping scale.
	 * 
	 * <p>REMARKS:
	 * From now on the body will not use the world's angular damping
	 * scale until dBodyResetAngularDamping() is called.
	 * @param b b
	 * 
	 * @param scale The angular damping scale. Should be in the interval [0, 1].
	 */
	//ODE_API 
	public static void dBodySetAngularDamping(DBody b, double scale){
		b.setAngularDamping(scale);
	}

	/**
	 * Convenience function to set linear and angular scales at once.
	 * @param b b
	 * @param linear_scale The linear damping scale. Should be in the interval [0, 1].
	 * @param angular_scale The angular damping scale. Should be in the interval [0, 1].
	 * 
	 * @see #dBodySetLinearDamping(DBody, double) 
	 * @see #dBodySetAngularDamping(DBody, double)
	 */
	//ODE_API 
	public static void dBodySetDamping(DBody b, double linear_scale, double angular_scale){
		b.setDamping(linear_scale, angular_scale);
	}

	/**
	 * Get the body's linear damping threshold.
	 * @param b b
	 * @return ret
	 * 
	 */
	//ODE_API 
	public static double dBodyGetLinearDampingThreshold (DBody b){
		return b.getLinearDampingThreshold();
	}

	/**
	 * Set the body's linear damping threshold.
	 * @param b b
	 * @param threshold The linear threshold to be used. Damping
	 *      is only applied if the linear speed is above this limit.
	 * 
	 */
	//ODE_API 
	public static void dBodySetLinearDampingThreshold(DBody b, double threshold){
		b.setLinearDampingThreshold(threshold);
	}

	/**
	 * Get the body's angular damping threshold.
	 * @param b b
	 * @return ret
	 * 
	 */
	//ODE_API 
	public static double dBodyGetAngularDampingThreshold (DBody b){
		return b.getAngularDampingThreshold();
	}

	/**
	 * Set the body's angular damping threshold.
	 * @param b b
	 * @param threshold The angular threshold to be used. Damping is
	 *      only used if the angular speed is above this limit.
	 * 
	 */
	//ODE_API 
	public static void dBodySetAngularDampingThreshold(DBody b, double threshold){
		b.setAngularDampingThreshold(threshold);
	}

	/**
	 * Get the body's maximum angular speed.
	 * @param b b
	 * @return ret
	 * @see ApiCppWorld#dWorldGetMaxAngularSpeed(DWorld)
	 */
	//ODE_API 
	public static double dBodyGetMaxAngularSpeed (DBody b){
		return b.getMaxAngularSpeed();
	}

	/**
	 * Set the body's maximum angular speed.
	 * 
	 * The default value is dInfinity, but it's a good idea to limit
	 * it at less than 500 if the body has the gyroscopic term
	 * enabled.
	 * @param b b
	 * @param max_speed max speed 
	 * 
	 * @see ApiCppWorld#dWorldSetMaxAngularSpeed(DWorld, double) 
	 */
	//ODE_API 
	public static void dBodySetMaxAngularSpeed(DBody b, double max_speed) {
		b.setMaxAngularSpeed(max_speed);
	}

	/**
	 * Get the body's gyroscopic state.
	 * @param b b
	 *
	 * @return nonzero if gyroscopic term computation is enabled (default),
	 * zero otherwise.
	 * 
	 */
	//ODE_API 
	public static boolean dBodyGetGyroscopicMode(DBody b) {
		return b.getGyroscopicMode();
	}


	/**
	 * Enable/disable the body's gyroscopic term.
	 *
	 * Disabling the gyroscopic term of a body usually improves
	 * stability. It also helps turning spining objects, like cars'
	 * wheels.
	 * @param b b
	 *
	 * @param enabled   nonzero (default) to enable gyroscopic term, 0
	 * to disable.
	 * 
	 */
	//ODE_API 
	public static void dBodySetGyroscopicMode(DBody b, boolean enabled) {
		b.setGyroscopicMode(enabled);
	}

	protected ApiCppBody() {}
}
