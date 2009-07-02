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
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxMass;
import org.ode4j.ode.internal.DxWorld;

public abstract class ApiCppBody extends ApiCppJoint {

	/**
	 * @brief Get auto disable linear average threshold.
	 * @ingroup bodies disable
	 * @return the threshold
	 */
	//ODE_API 
	double dBodyGetAutoDisableLinearThreshold (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable linear average threshold.
	 * @ingroup bodies disable
	 * @return the threshold
	 */
	//ODE_API 
	void  dBodySetAutoDisableLinearThreshold (DBody b, double linear_average_threshold){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get auto disable angular average threshold.
	 * @ingroup bodies disable
	 * @return the threshold
	 */
	//ODE_API 
	double dBodyGetAutoDisableAngularThreshold (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable angular average threshold.
	 * @ingroup bodies disable
	 * @return the threshold
	 */
	//ODE_API 
	void  dBodySetAutoDisableAngularThreshold (DBody b, double angular_average_threshold){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get auto disable average size (samples count).
	 * @ingroup bodies disable
	 * @return the nr of steps/size.
	 */
	//ODE_API 
	int dBodyGetAutoDisableAverageSamplesCount (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable average buffer size (average steps).
	 * @ingroup bodies disable
	 * @param average_samples_count the nr of samples to review.
	 */
	//ODE_API 
	//	 void dBodySetAutoDisableAverageSamplesCount (dBody b, 
	//			 unsigned int average_samples_count){
	void dBodySetAutoDisableAverageSamplesCount (DBody b, 
			int average_samples_count){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto steps a body must be thought of as idle to disable
	 * @ingroup bodies disable
	 * @return the nr of steps
	 */
	//ODE_API 
	int dBodyGetAutoDisableSteps (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable steps.
	 * @ingroup bodies disable
	 * @param steps the nr of steps.
	 */
	//ODE_API 
	void dBodySetAutoDisableSteps (DBody b, int steps){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get auto disable time.
	 * @ingroup bodies disable
	 * @return nr of seconds
	 */
	//ODE_API 
	double dBodyGetAutoDisableTime (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable time.
	 * @ingroup bodies disable
	 * @param time nr of seconds.
	 */
	//ODE_API 
	void  dBodySetAutoDisableTime (DBody b, double time){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get auto disable flag.
	 * @ingroup bodies disable
	 * @return 0 or 1
	 */
	//ODE_API 
	int dBodyGetAutoDisableFlag (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable flag.
	 * @ingroup bodies disable
	 * @param do_auto_disable 0 or 1
	 */
	//ODE_API 
	void dBodySetAutoDisableFlag (DBody b, int do_auto_disable){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set auto disable defaults.
	 * @remarks
	 * Set the values for the body to those set as default for the world.
	 * @ingroup bodies disable
	 */
	//ODE_API 
	void  dBodySetAutoDisableDefaults (DBody b){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Retrieves the world attached to te given body.
	 * @remarks
	 *
	 * @ingroup bodies
	 */
	//ODE_API 
	DWorld dBodyGetWorld (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Create a body in given world.
	 * @remarks
	 * Default mass parameters are at position (0,0,0).
	 * @ingroup bodies
	 */
	//ODE_API 
	public static DxBody dBodyCreate (DWorld w){
		return DxBody.dBodyCreate((DxWorld) w);
	}

	/**
	 * @brief Destroy a body.
	 * @remarks
	 * All joints that are attached to this body will be put into limbo:
	 * i.e. unattached and not affecting the simulation, but they will NOT be
	 * deleted.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodyDestroy (DBody b){
		((DxBody)b).dBodyDestroy();
	}

	/**
	 * @brief Set the body's user-data pointer.
	 * @ingroup bodies
	 * @param data arbitraty pointer
	 */
	//ODE_API 
	//	 void  dBodySetData (dBody b, void *data){
	public static void  dBodySetData (DBody b, Object data){
		((DxBody)b).dBodySetData(data);
	}

	/**
	 * @brief Get the body's user-data pointer.
	 * @ingroup bodies
	 * @return a pointer to the user's data.
	 */
	//ODE_API 
	//void *dBodyGetData (dBody b){
	public static Object dBodyGetData (DBody b){
		return ((DxBody)b).dBodyGetData();
	}

	/**
	 * @brief Set position of a body.
	 * @remarks
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodySetPosition   (DBody b, double x, double y, double z){
		((DxBody)b).dBodySetPosition(x, y, z);
	}

	/**
	 * @brief Set the orientation of a body.
	 * @ingroup bodies
	 * @remarks
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 */
	//ODE_API 
	public static void dBodySetRotation   (DBody b, final DMatrix3 R){
		((DxBody)b).dBodySetRotation(R);
	}

	/**
	 * @brief Set the orientation of a body.
	 * @ingroup bodies
	 * @remarks
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 */
	//ODE_API 
	public static void dBodySetQuaternion (DBody b, final DQuaternion q){
		((DxBody)b).dBodySetQuaternion(q);
	}

	/**
	 * @brief Set the linear velocity of a body.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodySetLinearVel  (DBody b, double x, double y, double z){
		((DxBody)b).dBodySetLinearVel(x, y, z);
	}

	/**
	 * @brief Set the angular velocity of a body.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodySetAngularVel (DBody b, double x, double y, double z){
		((DxBody)b).dBodySetAngularVel(x, y, z);
	}

	/**
	 * @brief Get the position of a body.
	 * @ingroup bodies
	 * @remarks
	 * When getting, the returned values are pointers to internal data structures,
	 * so the vectors are valid until any changes are made to the rigid body
	 * system structure.
	 * @see dBodyCopyPosition
	 */
	//ODE_API 
	//const 
	public static DVector3C dBodyGetPosition (DBody b){
		return ((DxBody)b).dBodyGetPosition();
	}


	/**
	 * @brief Copy the position of a body into a vector.
	 * @ingroup bodies
	 * @param body  the body to query
	 * @param pos   a copy of the body position
	 * @see dBodyGetPosition
	 */
	//ODE_API 
	void dBodyCopyPosition (DBody body, DVector3 pos){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the rotation of a body.
	 * @ingroup bodies
	 * @return pointer to a 4x3 rotation matrix.
	 */
	//ODE_API 
	//const 
	public static DMatrix3C dBodyGetRotation (DBody b){
		return ((DxBody)b).dBodyGetRotation();
	}


	/**
	 * @brief Copy the rotation of a body.
	 * @ingroup bodies
	 * @param body   the body to query
	 * @param R      a copy of the rotation matrix
	 * @see dBodyGetRotation
	 */
	//ODE_API 
	void dBodyCopyRotation (DBody b, DMatrix3 R){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the rotation of a body.
	 * @ingroup bodies
	 * @return pointer to 4 scalars that represent the quaternion.
	 */
	//ODE_API 
	//const double * 
	public static DQuaternionC dBodyGetQuaternion (DBody b){
		return ((DxBody)b).dBodyGetQuaternion();
	}


	/**
	 * @brief Copy the orientation of a body into a quaternion.
	 * @ingroup bodies
	 * @param body  the body to query
	 * @param quat  a copy of the orientation quaternion
	 * @see dBodyGetQuaternion
	 */
	//ODE_API 
	void dBodyCopyQuaternion(DBody body, DQuaternion quat){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the linear velocity of a body.
	 * @ingroup bodies
	 */
	//ODE_API 
	//const double * 
	public static DVector3C dBodyGetLinearVel (DBody b){
		return ((DxBody)b).dBodyGetLinearVel();
	}

	/**
	 * @brief Get the angular velocity of a body.
	 * @ingroup bodies
	 */
	//ODE_API 
	//const double * 
	public static DVector3C dBodyGetAngularVel (DBody b){
		return ((DxBody)b).dBodyGetAngularVel();
	}

	/**
	 * @brief Set the mass of a body.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodySetMass (DBody b, final DMass mass){
		((DxBody)b).dBodySetMass((DxMass) mass);
	}

	/**
	 * @brief Get the mass of a body.
	 * @ingroup bodies
	 */
	//ODE_API 
	//	 void dBodyGetMass (dBody b, dMass *mass){
	public static void dBodyGetMass (DBody b, DMass mass){
		((DxBody)b).dBodyGetMass((DxMass) mass);
	}

	/**
	 * @brief Add force at centre of mass of body in absolute coordinates.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodyAddForce            (DBody b, double fx, double fy, double fz){
		((DxBody)b).dBodyAddForce(fx, fy, fz);
	}

	/**
	 * @brief Add torque at centre of mass of body in absolute coordinates.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodyAddTorque           (DBody b, double fx, double fy, double fz){
		((DxBody)b).dBodyAddTorque(fx, fy, fz);
	}

	/**
	 * @brief Add force at centre of mass of body in coordinates relative to body.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodyAddRelForce         (DBody b, double fx, double fy, double fz){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Add torque at centre of mass of body in coordinates relative to body.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodyAddRelTorque (DBody b, double fx, double fy, double fz){
		((DxBody)b).dBodyAddRelTorque(new DVector3(fx, fy, fz));
	}

	/**
	 * @brief Add force at specified point in body in global coordinates.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodyAddForceAtPos       (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		throw new UnsupportedOperationException();
	}
	/**
	 * @brief Add force at specified point in body in local coordinates.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodyAddForceAtRelPos    (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		throw new UnsupportedOperationException();
	}
	/**
	 * @brief Add force at specified point in body in global coordinates.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodyAddRelForceAtPos    (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		throw new UnsupportedOperationException();
	}
	/**
	 * @brief Add force at specified point in body in local coordinates.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodyAddRelForceAtRelPos (DBody b, double fx, double fy, double fz,
			double px, double py, double pz){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Return the current accumulated force vector.
	 * @return points to an array of 3 reals.
	 * @remarks
	 * The returned values are pointers to internal data structures, so
	 * the vectors are only valid until any changes are made to the rigid
	 * body system.
	 * @ingroup bodies
	 */
	//ODE_API 
	//const double * 
	double[] dBodyGetForce (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Return the current accumulated torque vector.
	 * @return points to an array of 3 reals.
	 * @remarks
	 * The returned values are pointers to internal data structures, so
	 * the vectors are only valid until any changes are made to the rigid
	 * body system.
	 * @ingroup bodies
	 */
	//ODE_API 
	//const double * 
	double[] dBodyGetTorque (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body force accumulation vector.
	 * @remarks
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodySetForce  (DBody b, double x, double y, double z){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body torque accumulation vector.
	 * @remarks
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodySetTorque (DBody b, double x, double y, double z){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get world position of a relative point on body.
	 * @ingroup bodies
	 * @param result will contain the result.
	 */
	//ODE_API 
	void dBodyGetRelPointPos
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get velocity vector in global coords of a relative point on body.
	 * @ingroup bodies
	 * @param result will contain the result.
	 */
	//ODE_API 
	void dBodyGetRelPointVel
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get velocity vector in global coords of a globally
	 * specified point on a body.
	 * @ingroup bodies
	 * @param result will contain the result.
	 */
	//ODE_API 
	void dBodyGetPointVel
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief takes a point in global coordinates and returns
	 * the point's position in body-relative coordinates.
	 * @remarks
	 * This is the inverse of dBodyGetRelPointPos()
	 * @ingroup bodies
	 * @param result will contain the result.
	 */
	//ODE_API 
	void dBodyGetPosRelPoint
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Convert from local to world coordinates.
	 * @ingroup bodies
	 * @param result will contain the result.
	 */
	//ODE_API 
	void dBodyVectorToWorld
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Convert from world to local coordinates.
	 * @ingroup bodies
	 * @param result will contain the result.
	 */
	//ODE_API 
	void dBodyVectorFromWorld
	(
			DBody b, double px, double py, double pz,
			DVector3 result
	){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief controls the way a body's orientation is updated at each timestep.
	 * @ingroup bodies
	 * @param mode can be 0 or 1:
	 * \li 0: An ``infinitesimal'' orientation update is used.
	 * This is fast to compute, but it can occasionally cause inaccuracies
	 * for bodies that are rotating at high speed, especially when those
	 * bodies are joined to other bodies.
	 * This is the default for every new body that is created.
	 * \li 1: A ``finite'' orientation update is used.
	 * This is more costly to compute, but will be more accurate for high
	 * speed rotations.
	 * @remarks
	 * Note however that high speed rotations can result in many types of
	 * error in a simulation, and the finite mode will only fix one of those
	 * sources of error.
	 */
	//ODE_API 
	void dBodySetFiniteRotationMode (DBody b, int mode){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief sets the finite rotation axis for a body.
	 * @ingroup bodies
	 * @remarks
	 * This is axis only has meaning when the finite rotation mode is set
	 * If this axis is zero (0,0,0), full finite rotations are performed on
	 * the body.
	 * If this axis is nonzero, the body is rotated by performing a partial finite
	 * rotation along the axis direction followed by an infinitesimal rotation
	 * along an orthogonal direction.
	 * @remarks
	 * This can be useful to alleviate certain sources of error caused by quickly
	 * spinning bodies. For example, if a car wheel is rotating at high speed
	 * you can call this function with the wheel's hinge axis as the argument to
	 * try and improve its behavior.
	 */
	//ODE_API 
	void dBodySetFiniteRotationAxis (DBody body, double x, double y, double z){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the way a body's orientation is updated each timestep.
	 * @ingroup bodies
	 * @return the mode 0 (infitesimal) or 1 (finite).
	 */
	//ODE_API 
	int dBodyGetFiniteRotationMode (DBody body){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the finite rotation axis.
	 * @param result will contain the axis.
	 * @ingroup bodies
	 */
	//ODE_API 
	void dBodyGetFiniteRotationAxis (DBody body, DVector3 result){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the number of joints that are attached to this body.
	 * @ingroup bodies
	 * @return nr of joints
	 */
	//ODE_API 
	int dBodyGetNumJoints (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Return a joint attached to this body, given by index.
	 * @ingroup bodies
	 * @param index valid range is  0 to n-1 where n is the value returned by
	 * dBodyGetNumJoints().
	 */
	//ODE_API 
	DJoint dBodyGetJoint (DBody body, int index){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set rigid body to dynamic state (default).
	 * @param dBodyID identification of body.
	 * @ingroup bodies
	 */
	//ODE_API
	void dBodySetDynamic (DBody body) {
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set rigid body to kinematic state.
	 * When in kinematic state the body isn't simulated as a dynamic
	 * body (it's "unstoppable", doesn't respond to forces),
	 * but can still affect dynamic bodies (e.g. in joints).
	 * Kinematic bodies can be controlled by position and velocity.
	 * @note A kinematic body has infinite mass. If you set its mass
	 * to something else, it loses the kinematic state and behaves
	 * as a normal dynamic body.
	 * @param dBodyID identification of body.
	 * @ingroup bodies
	 */
	//ODE_API
	void dBodySetKinematic (DBody body) {
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Check wether a body is in kinematic state.
	 * @ingroup bodies
	 * @return 1 if a body is kinematic or 0 if it is dynamic.
	 */
	//ODE_API
	int dBodyIsKinematic (DBody body ) {
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Manually enable a body.
	 * @param DBody identification of body.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodyEnable (DBody body){
		((DxBody)body).dBodyEnable();
	}

	/**
	 * @brief Manually disable a body.
	 * @ingroup bodies
	 * @remarks
	 * A disabled body that is connected through a joint to an enabled body will
	 * be automatically re-enabled at the next simulation step.
	 */
	//ODE_API 
	public static void dBodyDisable (DBody body){
		((DxBody)body).dBodyDisable();
	}

	/**
	 * @brief Check wether a body is enabled.
	 * @ingroup bodies
	 * @return 1 if a body is currently enabled or 0 if it is disabled.
	 */
	//ODE_API 
	public static boolean dBodyIsEnabled (DBody body){
		return ((DxBody)body).dBodyIsEnabled();
	}

	/**
	 * @brief Set whether the body is influenced by the world's gravity or not.
	 * @ingroup bodies
	 * @param mode when nonzero gravity affects this body.
	 * @remarks
	 * Newly created bodies are always influenced by the world's gravity.
	 */
	//ODE_API 
	void dBodySetGravityMode (DBody b, int mode){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get whether the body is influenced by the world's gravity or not.
	 * @ingroup bodies
	 * @return nonzero means gravity affects this body.
	 */
	//ODE_API 
	int dBodyGetGravityMode (DBody b){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the 'moved' callback of a body.
	 *
	 * Whenever a body has its position or rotation changed during the
	 * timestep, the callback will be called (with body as the argument).
	 * Use it to know which body may need an update in an external
	 * structure (like a 3D engine).
	 *
	 * @param b the body that needs to be watched.
	 * @param callback the callback to be invoked when the body moves. Set to zero
	 * to disable.
	 * @ingroup bodies
	 */
	//ODE_API 
	//	 void dBodySetMovedCallback(dBody b, void (*callback)(dBody body)){
	void dBodySetMovedCallback(DBody b, BodyMoveCallBack callback){
		throw new UnsupportedOperationException();
		//((dxBody)b).dBodySetMovedCallback(callback);
	}


	/**
	 * @brief Return the first geom associated with the body.
	 *
	 * You can traverse through the geoms by repeatedly calling
	 * dBodyGetNextGeom().
	 *
	 * @return the first geom attached to this body, or 0.
	 * @ingroup bodies
	 */
	//ODE_API 
	DGeom dBodyGetFirstGeom (DBody b){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief returns the next geom associated with the same body.
	 * @param g a geom attached to some body.
	 * @return the next geom attached to the same body, or 0.
	 * @see dBodyGetFirstGeom
	 * @ingroup bodies
	 */
	//ODE_API 
	DGeom dBodyGetNextGeom (DGeom g){
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Resets the damping settings to the current world's settings.
	 * @ingroup bodies damping
	 */
	//ODE_API 
	void dBodySetDampingDefaults(DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the body's linear damping scale.
	 * @ingroup bodies damping
	 */
	//ODE_API 
	double dBodyGetLinearDamping (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body's linear damping scale.
	 * @param scale The linear damping scale. Should be in the interval [0, 1].
	 * @ingroup bodies damping
	 * @remarks From now on the body will not use the world's linear damping
	 * scale until dBodySetDampingDefaults() is called.
	 * @see dBodySetDampingDefaults()
	 */
	//ODE_API 
	void dBodySetLinearDamping(DBody b, double scale){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the body's angular damping scale.
	 * @ingroup bodies damping
	 * @remarks If the body's angular damping scale was not set, this function
	 * returns the world's angular damping scale.
	 */
	//ODE_API 
	double dBodyGetAngularDamping (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body's angular damping scale.
	 * @param scale The angular damping scale. Should be in the interval [0, 1].
	 * @ingroup bodies damping
	 * @remarks From now on the body will not use the world's angular damping
	 * scale until dBodyResetAngularDamping() is called.
	 * @see dBodyResetAngularDamping()
	 */
	//ODE_API 
	void dBodySetAngularDamping(DBody b, double scale){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Convenience function to set linear and angular scales at once.
	 * @param linear_scale The linear damping scale. Should be in the interval [0, 1].
	 * @param angular_scale The angular damping scale. Should be in the interval [0, 1].
	 * @ingroup bodies damping
	 * @see dBodySetLinearDamping() dBodySetAngularDamping()
	 */
	//ODE_API 
	void dBodySetDamping(DBody b, double linear_scale, double angular_scale){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the body's linear damping threshold.
	 * @ingroup bodies damping
	 */
	//ODE_API 
	double dBodyGetLinearDampingThreshold (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body's linear damping threshold.
	 * @param threshold The linear threshold to be used. Damping
	 *      is only applied if the linear speed is above this limit.
	 * @ingroup bodies damping
	 */
	//ODE_API 
	void dBodySetLinearDampingThreshold(DBody b, double threshold){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the body's angular damping threshold.
	 * @ingroup bodies damping
	 */
	//ODE_API 
	double dBodyGetAngularDampingThreshold (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body's angular damping threshold.
	 * @param threshold The angular threshold to be used. Damping is
	 *      only used if the angular speed is above this limit.
	 * @ingroup bodies damping
	 */
	//ODE_API 
	void dBodySetAngularDampingThreshold(DBody b, double threshold){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the body's maximum angular speed.
	 * @ingroup damping bodies
	 * @see dWorldGetMaxAngularSpeed()
	 */
	//ODE_API 
	double dBodyGetMaxAngularSpeed (DBody b){
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Set the body's maximum angular speed.
	 * @ingroup damping bodies
	 * @see dWorldSetMaxAngularSpeed() dBodyResetMaxAngularSpeed()
	 * The default value is dInfinity, but it's a good idea to limit
	 * it at less than 500 if the body has the gyroscopic term
	 * enabled.
	 */
	//ODE_API 
	void dBodySetMaxAngularSpeed(DBody b, double max_speed) {
		throw new UnsupportedOperationException();
	}

	/**
	 * @brief Get the body's gyroscopic state.
	 *
	 * @return nonzero if gyroscopic term computation is enabled (default),
	 * zero otherwise.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static boolean dBodyGetGyroscopicMode(DBody b) {
		return b.getGyroscopicMode();
	}


	/**
	 * @brief Enable/disable the body's gyroscopic term.
	 *
	 * Disabling the gyroscopic term of a body usually improves
	 * stability. It also helps turning spining objects, like cars'
	 * wheels.
	 *
	 * @param enabled   nonzero (default) to enable gyroscopic term, 0
	 * to disable.
	 * @ingroup bodies
	 */
	//ODE_API 
	public static void dBodySetGyroscopicMode(DBody b, boolean enabled) {
		b.setGyroscopicMode(enabled);
	}
}
