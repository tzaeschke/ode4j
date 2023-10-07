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
package org.ode4j.ode;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

import java.util.Iterator;

/**
 * A rigid body has various properties from the point of view of the
 * simulation. Some properties change over time:
 * <ul>
 *  <li> Position vector (x,y,z) of the body's point of reference.
 *      Currently the point of reference must correspond to the body's center of mass.
 *  <li> Linear velocity of the point of reference, a vector (vx,vy,vz).
 *  <li> Orientation of a body, represented by a quaternion (qs,qx,qy,qz) or
 *      a 3x3 rotation matrix.
 *  <li> Angular velocity vector (wx,wy,wz) which describes how the orientation
 *      changes over time.
 * </ul>
 * <p>
 * Other body properties are usually constant over time:
 * <ul>
 *  <li> Mass of the body.
 *  <li> Position of the center of mass with respect to the point of reference.
 *      In the current implementation the center of mass and the point of
 *      reference must coincide.
 *  <li> Inertia matrix. This is a 3x3 matrix that describes how the body's mass
 *      is distributed around the center of mass. Conceptually each body has an
 *      x-y-z coordinate frame embedded in it that moves and rotates with the body.
 * </ul>
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

	 /**
	  * Whenever a body has its position or rotation changed during the
	  * timestep, the callback will be called (with body as the argument).
	  * Use it to know which body may need an update in an external
	  * structure (like a 3D engine).
	  */
	 interface BodyMoveCallBack {
		 void run(DBody b);
	 }

	
	//~dBody()
	void DESTRUCTOR();
	/**
	 * Destroy a body.
	 * 
	 * <p>REMARK:
	 * All joints that are attached to this body will be put into limbo:
	 * i.e. unattached and not affecting the simulation, but they will NOT be
	 * deleted.
	 */
	void destroy();

	/**
	 * Set the body's user-data pointer.
	 * @param data arbitraty pointer
	 */
	void setData (Object data);
	/**
	 * Get the body's user-data pointer.
	 * @return a pointer to the user's data.
	 */
	Object getData();

	/**
	 * Set position of a body.
	 * <p>REMARK:
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setPosition (double x, double y, double z);
	/**
	 * Set position of a body.
	 * <p>REMARK:
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param p p
	 */
	void setPosition (DVector3C p);

	/**
	 * Set the orientation of a body.
	 * <p>REMARK:
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param R R
	 */
	void setRotation (DMatrix3C R);
	/**
	 * Set the orientation of a body.
	 * <p>REMARK:
	 * After setting, the outcome of the simulation is undefined
	 * if the new configuration is inconsistent with the joints/constraints
	 * that are present.
	 * @param q q
	 */
	void setQuaternion (DQuaternionC q);
	/**
	 * Set the linear velocity of a body.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setLinearVel (double x, double y, double z);
	/**
	 * Set the linear velocity of a body.
	 * @param v v
	 */
	void setLinearVel (DVector3C v);
	/**
	 * Add to the linear velocity of a body.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void addLinearVel (double x, double y, double z);
	/**
	 * Add to the linear velocity of a body.
	 * @param v v
	 */
	void addLinearVel (DVector3C v);

	/**
	 * Set the angular velocity of a body.
	 * The velocity must be provided in radians/timeunit.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setAngularVel (double x, double y, double z);

	/**
	 * Set the angular velocity of a body.
	 * The velocity must be provided in radians/timeunit.
	 * @param v v
	 */
	void setAngularVel (DVector3C v);

	/**
	 * Get the position of a body.
	 * <p>REMARK:
	 * When getting, the returned values are pointers to internal data structures,
	 * so the vectors are valid until any changes are made to the rigid body
	 * system structure.
	 * @return position vector
	 */
	DVector3C getPosition();
	/**
	 * Get the rotation of a body.
	 * @return pointer to a 4x3 rotation matrix.
	 */
	DMatrix3C getRotation();
	/**
	 * Get the rotation of a body.
	 * @return pointer to 4 scalars that represent the quaternion.
	 */
	DQuaternionC getQuaternion();
	/**
	 * Get the linear velocity of a body.
	 * @return vector
	 */
	DVector3C getLinearVel();
	/**
	 * Get the angular velocity of a body.
	 * @return vector
	 */
	DVector3C getAngularVel();

	//  void setMass (final dMass *mass)
	//  { dBodySetMass (_id,mass); }
	//void setMass (final dMass &mass)
	//  { setMass (&mass); }
	/**
	 * Set the mass of a body.
	 * @param mass mass
	 */
	void setMass (DMassC mass);
	/**
	 * Get the mass of a body.
	 * @return mas
	 */
	DMassC getMass ();


	/**
	 * Retrieves the world attached to the given body.
	 * @return World object
	 */
	DWorld getWorld();


	/**
	 * Set auto disable linear average threshold.
	 * @param threshold threshold
	 */
	void setAutoDisableLinearThreshold (double threshold);
	/**
	 * Get auto disable linear average threshold.
	 * @return the threshold
	 */
	double getAutoDisableLinearThreshold();
	/**
	 * Set auto disable angular average threshold.
	 * @param threshold threshold
	 */
	void setAutoDisableAngularThreshold (double threshold);
	/**
	 * Get auto disable angular average threshold.
	 * @return the threshold
	 */
	double getAutoDisableAngularThreshold();
	/**
	 * Set auto disable steps.
	 * @param steps the nr of steps.
	 */
	void setAutoDisableSteps (int steps);
	/**
	 * Get auto steps a body must be thought of as idle to disable
	 * @return the nr of steps
	 */
	int getAutoDisableSteps();
	/**
	 * Set auto disable time.
	 * @param time nr of seconds.
	 */
	void setAutoDisableTime (double time);
	/**
	 * Get auto disable time.
	 * @return nr of seconds
	 */
	double getAutoDisableTime();
	/**
	 * Set auto disable flag.
	 * @param do_auto_disable 0 or 1
	 */
	void setAutoDisableFlag (boolean do_auto_disable);
	/**
	 * Get auto disable flag.
	 * @return 0 or 1
	 */
	boolean getAutoDisableFlag();
	/**
	 * Get auto disable average size (samples count).
	 * @return the nr of steps/size.
	 */
	int getAutoDisableAverageSamplesCount();
	/**
	 * Set auto disable average buffer size (average steps).
	 * @param average_samples_count the nr of samples to review.
	 */
	void setAutoDisableAverageSamplesCount(int average_samples_count);
	/**
	 * Set auto disable defaults.
	 * <p>REMARK:
	 * Set the values for the body to those set as default for the world.
	 */
	void setAutoDisableDefaults();


	/**
	 * Add force at centre of mass of body in absolute coordinates.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 */
	void addForce (double fx, double fy, double fz);
	/**
	 * Add force at centre of mass of body in absolute coordinates.
	 * @param f f
	 */
	void addForce (DVector3C f);
	/**
	 * Add torque at centre of mass of body in absolute coordinates.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 */
	void addTorque (double fx, double fy, double fz);
	/**
	 * Add torque at centre of mass of body in absolute coordinates.
	 * @param t t
	 */
	void addTorque (DVector3C t);

	/**
	 * Add force at centre of mass of body in coordinates relative to body.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 */
	void addRelForce (double fx, double fy, double fz);
	/**
	 * Add force at centre of mass of body in coordinates relative to body.
	 * @param f f
	 */
	void addRelForce (DVector3C f);
	/**
	 * Add torque at centre of mass of body in coordinates relative to body.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 */
	void addRelTorque (double fx, double fy, double fz);
	/**
	 * Add torque at centre of mass of body in coordinates relative to body.
	 * @param t t
	 */
	void addRelTorque (DVector3C t);

	/**
	 * Add force at specified point in body in global coordinates.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 */
	void addForceAtPos (double fx, double fy, double fz,
			double px, double py, double pz);
	/**
	 * Add force at specified point in body in global coordinates.
	 * @param f f
	 * @param p p
	 */
	void addForceAtPos (DVector3C f, DVector3C p);

	/**
	 * Add force at specified point in body in local coordinates.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 */
	void addForceAtRelPos (double fx, double fy, double fz,
			double px, double py, double pz);
	/**
	 * Add force at specified point in body in local coordinates.
	 * @param f f
	 * @param p p
	 */
	void addForceAtRelPos (DVector3C f, DVector3C p);

	/**
	 * Add force at specified point in body in global coordinates.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 */
	void addRelForceAtPos (double fx, double fy, double fz,
			double px, double py, double pz);
	/**
	 * Add force at specified point in body in global coordinates.
	 * @param f f
	 * @param p p
	 */
	void addRelForceAtPos (DVector3C f, DVector3C p);

	/**
	 * Add force at specified point in body in local coordinates.
	 * @param fx fx
	 * @param fy fy
	 * @param fz fz
	 * @param px px
	 * @param py py
	 * @param pz pz
	 */
	void addRelForceAtRelPos (double fx, double fy, double fz,
			double px, double py, double pz);
	/**
	 * Add force at specified point in body in local coordinates.
	 * @param f f
	 * @param p p
	 */
	void addRelForceAtRelPos (DVector3C f, DVector3C p);

	/**
	 * Return the current accumulated force vector.
	 * <p>REMARK:
	 * The returned values are pointers to internal data structures, so
	 * the vectors are only valid until any changes are made to the rigid
	 * body system.
	 * @return points to an array of 3 reals.
	 */
	DVector3C getForce();
	/**
	 * Return the current accumulated torque vector.
	 * <p>REMARK:
	 * The returned values are pointers to internal data structures, so
	 * the vectors are only valid until any changes are made to the rigid
	 * body system.
	 * @return points to an array of 3 reals.
	 */
	DVector3C getTorque();
	/**
	 * Set the body force accumulation vector.
	 * <p>REMARK:
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @param x x
	 * @param y y
	 * @param z Z
	 */
	void setForce (double x, double y, double z);
	/**
	 * Set the body force accumulation vector.
	 * <p>REMARK:
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @param f f
	 */
	void setForce (DVector3C f);
	/**
	 * Set the body torque accumulation vector.
	 * <p>REMARK:
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setTorque (double x, double y, double z);
	/**
	 * Set the body torque accumulation vector.
	 * <p>REMARK:
	 * This is mostly useful to zero the force and torque for deactivated bodies
	 * before they are reactivated, in the case where the force-adding functions
	 * were called on them while they were deactivated.
	 * @param t t
	 */
	void setTorque (DVector3C t);


	/**
	 * Get world position of a relative point on body.
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void getRelPointPos (double px, double py, double pz, DVector3 result);
	/**
	 * Get world position of a relative point on body.
	 * @param p p
	 * @param result will contain the result.
	 */
	void getRelPointPos (DVector3C p, DVector3 result);

	/**
	 * Get velocity vector in global coords of a relative point on body.
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void getRelPointVel (double px, double py, double pz, DVector3 result);
	/**
	 * Get velocity vector in global coords of a relative point on body.
	 * @param p p
	 * @param result will contain the result.
	 */
	void getRelPointVel (DVector3C p, DVector3 result);

	/**
	 * Get velocity vector in global coords of a globally
	 * specified point on a body.
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void getPointVel (double px, double py, double pz, DVector3 result);
	/**
	 * Get velocity vector in global coords of a globally
	 * specified point on a body.
	 * @param p p
	 * @param result will contain the result.
	 */
	void getPointVel (DVector3C p, DVector3 result);

	/**
	 * Takes a point in global coordinates and returns
	 * the point's position in body-relative coordinates.
	 * <p>REMARK:
	 * This is the inverse of dBodyGetRelPointPos()
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void getPosRelPoint (double px, double py, double pz, DVector3 result);
	/**
	 * Takes a point in global coordinates and returns
	 * the point's position in body-relative coordinates.
	 * <p>REMARK:
	 * This is the inverse of dBodyGetRelPointPos()
	 * @param p p
	 * @param result will contain the result.
	 */
	void getPosRelPoint (DVector3C p, DVector3 result);

	/**
	 * Convert from local to world coordinates.
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void vectorToWorld (double px, double py, double pz, DVector3 result);
	/**
	 * Convert from local to world coordinates.
	 * @param p p
	 * @param result will contain the result.
	 */
	void vectorToWorld (DVector3C p, DVector3 result);

	/**
	 * Convert from world to local coordinates.
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void vectorFromWorld (double px, double py, double pz, DVector3 result);
	/**
	 * Convert from world to local coordinates.
	 * @param p p
	 * @param result will contain the result.
	 */
	void vectorFromWorld (DVector3C p, DVector3 result);

	/**
	 * Controls the way a body's orientation is updated at each timestep.
	 * 
	 * <p>REMARK:
	 * Note however that high speed rotations can result in many types of
	 * error in a simulation, and the finite mode will only fix one of those
	 * sources of error.
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
	 * speed rotations.</li>
	 * </ul>
	 */
	void setFiniteRotationMode (boolean mode);

	/**
	 * Sets the finite rotation axis for a body.
	 * 
	 * <p>REMARK:
	 * This is axis only has meaning when the finite rotation mode is set
	 * If this axis is zero (0,0,0), full finite rotations are performed on
	 * the body.
	 * If this axis is nonzero, the body is rotated by performing a partial finite
	 * rotation along the axis direction followed by an infinitesimal rotation
	 * along an orthogonal direction.
	 * 
	 * <p>REMARK:
	 * This can be useful to alleviate certain sources of error caused by quickly
	 * spinning bodies. For example, if a car wheel is rotating at high speed
	 * you can call this function with the wheel's hinge axis as the argument to
	 * try and improve its behavior.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setFiniteRotationAxis (double x, double y, double z);
	/**
	 * Sets the finite rotation axis for a body.
	 * 
	 * <p>REMARK:
	 * This is axis only has meaning when the finite rotation mode is set
	 * If this axis is zero (0,0,0), full finite rotations are performed on
	 * the body.
	 * If this axis is nonzero, the body is rotated by performing a partial finite
	 * rotation along the axis direction followed by an infinitesimal rotation
	 * along an orthogonal direction.
	 * 
	 * <p>REMARK:
	 * This can be useful to alleviate certain sources of error caused by quickly
	 * spinning bodies. For example, if a car wheel is rotating at high speed
	 * you can call this function with the wheel's hinge axis as the argument to
	 * try and improve its behavior.
	 * @param a a
	 */
	void setFiniteRotationAxis (DVector3C a);

	/**
	 * Get the way a body's orientation is updated each timestep.
	 * @return the mode 0 (infitesimal) or 1 (finite).
	 */
	boolean getFiniteRotationMode();
	/**
	 * Get the finite rotation axis.
	 * @param result will contain the axis.
	 */
	void getFiniteRotationAxis (DVector3 result);

	/**
	 * Get the number of joints that are attached to this body.
	 * @return nr of joints
	 */
	int getNumJoints();
	/**
	 * Return a joint attached to this body, given by index.
	 * @param index valid range is  0 to n-1 where n is the value returned by
	 * dBodyGetNumJoints().
	 * @return Joint object
	 */
	DJoint getJoint (int index);

	/**
	 * Set rigid body to dynamic state (default).
	 */
	void setDynamic();
	/**
	 * Set rigid body to kinematic state.
	 * When in kinematic state the body isn't simulated as a dynamic
	 * body (it's "unstoppable", doesn't respond to forces),
	 * but can still affect dynamic bodies (e.g. in joints).
	 * Kinematic bodies can be controlled by position and velocity.
	 * 
	 * <p>NOTE: A kinematic body has infinite mass. If you set its mass
	 * to something else, it loses the kinematic state and behaves
	 * as a normal dynamic body.
	 */
	void setKinematic();
	/**
	 * Check wether a body is in kinematic state.
	 * @return 1 if a body is kinematic or 0 if it is dynamic.
	 */
	boolean isKinematic();

	/**
	 * Manually enable a body.
	 */
	void enable();
	/**
	 * Manually disable a body.
	 * 
	 * <p>REMARK:
	 * A disabled body that is connected through a joint to an enabled body will
	 * be automatically re-enabled at the next simulation step.
	 */
	void disable();
	/**
	 * Check wether a body is enabled.
	 * @return 1 if a body is currently enabled or 0 if it is disabled.
	 */
	boolean isEnabled();


	/**
	 * Set whether the body is influenced by the world's gravity or not.
	 * <p>REMARK:
	 * Newly created bodies are always influenced by the world's gravity.
	 * @param mode when nonzero gravity affects this body.
	 */
	void setGravityMode (boolean mode);
	/**
	 * Get whether the body is influenced by the world's gravity or not.
	 * @return nonzero means gravity affects this body.
	 */
	boolean getGravityMode();

	boolean isConnectedTo (DBody body);

	/**
	 * Get the body's linear damping scale.
	 * @return damping value
	 */
	double getLinearDamping();
	/**
	 * Set the body's linear damping scale.
	 * 
	 * <p>REMARK: From now on the body will not use the world's linear damping
	 * scale until dBodySetDampingDefaults() is called.
	 * @param scale The linear damping scale. Should be in the interval [0, 1].
	 * @see #setDampingDefaults()
	 */
	void setLinearDamping(double scale);
	/**
	 * Get the body's angular damping scale.
	 * 
	 * <p>REMARK: If the body's angular damping scale was not set, this function
	 * returns the world's angular damping scale.
	 * @return damping value
	 */
	double getAngularDamping();
	/**
	 * Set the body's angular damping scale.
	 * 
	 * <p>REMARK: From now on the body will not use the world's angular damping
	 * scale until dBodyResetAngularDamping() is called.
	 * @param scale The angular damping scale. Should be in the interval [0, 1].
	 */
	void setAngularDamping(double scale);
	/**
	 * Convenience function to set linear and angular scales at once.
	 * @param linear_scale The linear damping scale. Should be in the interval [0, 1].
	 * @param angular_scale The angular damping scale. Should be in the interval [0, 1].
	 * @see #setLinearDamping(double)
	 * @see #setAngularDamping(double)
	 */
	void setDamping(double linear_scale, double angular_scale);
	/**
	 * Get the body's linear damping threshold.
	 * @return threshold value
	 */
	double getLinearDampingThreshold();
	/**
	 * Set the body's linear damping threshold.
	 * @param threshold The linear threshold to be used. Damping
	 *      is only applied if the linear speed is above this limit.
	 */
	void setLinearDampingThreshold(double threshold);
	/**
	 * Get the body's angular damping threshold.
	 * @return threshold value
	 */
	double getAngularDampingThreshold();
	/**
	 * Set the body's angular damping threshold.
	 * @param threshold The angular threshold to be used. Damping is
	 *      only used if the angular speed is above this limit.
	 */
	void setAngularDampingThreshold(double threshold);
	/**
	 * Resets the damping settings to the current world's settings.
	 */
	void setDampingDefaults();

	/**
	 * Get the body's maximum angular speed.
	 * @return speed limit
	 * @see DWorld#getMaxAngularSpeed()
	 */
	double getMaxAngularSpeed();
	/**
	 * Set the body's maximum angular speed.
	 * 
	 * The default value is dInfinity, but it's a good idea to limit
	 * it at less than 500 if the body has the gyroscopic term
	 * enabled.
	 * @param max_speed speed limit
	 * @see DWorld#setMaxAngularSpeed(double) 
	 */
	void setMaxAngularSpeed(double max_speed);

	/**
	 * Get the body's gyroscopic state.
	 *
	 * @return nonzero if gyroscopic term computation is enabled (default),
	 * zero otherwise.
	 */
	boolean getGyroscopicMode();
	/**
	 * Enable/disable the body's gyroscopic term.
	 *
	 * Disabling the gyroscopic term of a body usually improves
	 * stability. It also helps turning spining objects, like cars'
	 * wheels.
	 *
	 * @param enabled   nonzero (default) to enable gyroscopic term, 0
	 * to disable.
	 */
	void setGyroscopicMode(boolean enabled);


	/**
	 * Set the 'moved' callback of a body.
	 *
	 * Whenever a body has its position or rotation changed during the
	 * timestep, the callback will be called (with body as the argument).
	 * Use it to know which body may need an update in an external
	 * structure (like a 3D engine).
	 *
	 * @param callback the callback to be invoked when the body moves. Set to zero
	 * to disable.
	 */
	void setMovedCallback(BodyMoveCallBack callback);


	/**
	 * Return the first geom associated with the body.
	 *
	 * You can traverse through the geoms by repeatedly calling
	 * dBodyGetNextGeom().
	 *
	 * @return the first geom attached to this body, or null.
	 */
    DGeom getFirstGeom ();


	/**
	 * Returns the next geom associated with the same body.
	 * @param geom a geom attached to some body.
	 * @return the next geom attached to the same body, or 0.
	 * @see DBody#getFirstGeom()
	 * @deprecated May be replaced by a more Java-like API. Please use getGeomIterator() instead
	 */
    @Deprecated
    DGeom getNextGeom (DGeom geom);

	/**
	 * @return an iterator over all geoms associated with with body.
	 */
	Iterator<DGeom> getGeomIterator();

}
