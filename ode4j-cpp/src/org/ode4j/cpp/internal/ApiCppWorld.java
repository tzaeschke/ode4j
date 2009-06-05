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

import org.ode4j.math.DVector3;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.DxWorld;

/**
 * @defgroup world World
 *
 * The world object is a container for rigid bodies and joints. Objects in
 * different worlds can not interact, for example rigid bodies from two
 * different worlds can not collide.
 *
 * All the objects in a world exist at the same point in time, thus one
 * reason to use separate worlds is to simulate systems at different rates.
 * Most applications will only need one world.
 */
public abstract class ApiCppWorld extends ApiCppBody {

	/**
	 * @brief Create a new, empty world and return its ID number.
	 * @return an identifier
	 * @ingroup world
	 */
	//ODE_API
	public static DWorld dWorldCreate() {
		return DxWorld.dWorldCreate();
	}


	/**
	 * @brief Destroy a world and everything in it.
	 *
	 * This includes all bodies, and all joints that are not part of a joint
	 * group. Joints that are part of a joint group will be deactivated, and
	 * can be destroyed by calling, for example, dJointGroupEmpty().
	 * @ingroup world
	 * @param world the identifier for the world the be destroyed.
	 */
	//ODE_API
	public static void dWorldDestroy (DWorld world) {
		((DxWorld)world).dWorldDestroy();
	}


	/**
	 * @brief Set the world's global gravity vector.
	 *
	 * The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
	 * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
	 *
	 * @ingroup world
	 */
	//ODE_API 
	public static void dWorldSetGravity (DWorld w, double x, double y, double z) {
		((DxWorld)w).dWorldSetGravity(x, y, z);
	}



	/**
	 * @brief Get the gravity vector for a given world.
	 * @ingroup world
	 */
	//ODE_API 
	void dWorldGetGravity (DWorld w, DVector3 gravity) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Set the global ERP value, that controls how much error
	 * correction is performed in each time step.
	 * @ingroup world
	 * @param DWorld the identifier of the world.
	 * @param erp Typical values are in the range 0.1--0.8. The default is 0.2.
	 */
	//ODE_API 
	public static void dWorldSetERP (DWorld w, double erp) {
		((DxWorld)w).dWorldSetERP(erp);
	}


	/**
	 * @brief Get the error reduction parameter.
	 * @ingroup world
	 * @return ERP value
	 */
	//ODE_API 
	double dWorldGetERP (DWorld w) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Set the global CFM (constraint force mixing) value.
	 * @ingroup world
	 * @param cfm Typical values are in the range @m{10^{-9}} -- 1.
	 * The default is 10^-5 if single precision is being used, or 10^-10
	 * if double precision is being used.
	 */
	//ODE_API 
	public static void dWorldSetCFM (DWorld w, double cfm) {
		((DxWorld)w).dWorldSetCFM(cfm);
	}


	/**
	 * @brief Get the constraint force mixing value.
	 * @ingroup world
	 * @return CFM value
	 */
	//ODE_API 
	double dWorldGetCFM (DWorld w) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Step the world.
	 *
	 * This uses a "big matrix" method that takes time on the order of m^3
	 * and memory on the order of m^2, where m is the total number of constraint
	 * rows. For large systems this will use a lot of memory and can be very slow,
	 * but this is currently the most accurate method.
	 * @ingroup world
	 * @param stepsize The number of seconds that the simulation has to advance.
	 */
	//ODE_API 
	public static void dWorldStep (DWorld w, double stepsize) {
		((DxWorld)w).dWorldStep(stepsize);
	}



	/**
	 * @brief Converts an impulse to a force.
	 * @ingroup world
	 * @remarks
	 * If you want to apply a linear or angular impulse to a rigid body,
	 * instead of a force or a torque, then you can use this function to convert
	 * the desired impulse into a force/torque vector before calling the
	 * BodyAdd... function.
	 * The current algorithm simply scales the impulse by 1/stepsize,
	 * where stepsize is the step size for the next step that will be taken.
	 * This function is given a dWorld because, in the future, the force
	 * computation may depend on integrator parameters that are set as
	 * properties of the world.
	 */
	//ODE_API 
	void dWorldImpulseToForce
	(
			DWorld w, double stepsize,
			double ix, double iy, double iz, DVector3 force
	) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Step the world.
	 * @ingroup world
	 * @remarks
	 * This uses an iterative method that takes time on the order of m*N
	 * and memory on the order of m, where m is the total number of constraint
	 * rows N is the number of iterations.
	 * For large systems this is a lot faster than dWorldStep(),
	 * but it is less accurate.
	 * @remarks
	 * QuickStep is great for stacks of objects especially when the
	 * auto-disable feature is used as well.
	 * However, it has poor accuracy for near-singular systems.
	 * Near-singular systems can occur when using high-friction contacts, motors,
	 * or certain articulated structures. For example, a robot with multiple legs
	 * sitting on the ground may be near-singular.
	 * @remarks
	 * There are ways to help overcome QuickStep's inaccuracy problems:
	 * \li Increase CFM.
	 * \li Reduce the number of contacts in your system (e.g. use the minimum
	 *     number of contacts for the feet of a robot or creature).
	 * \li Don't use excessive friction in the contacts.
	 * \li Use contact slip if appropriate
	 * \li Avoid kinematic loops (however, kinematic loops are inevitable in
	 *     legged creatures).
	 * \li Don't use excessive motor strength.
	 * \liUse force-based motors instead of velocity-based motors.
	 *
	 * Increasing the number of QuickStep iterations may help a little bit, but
	 * it is not going to help much if your system is really near singular.
	 */
	//ODE_API 
	public static void dWorldQuickStep (DWorld w, double stepsize) {
		((DxWorld)w).dWorldQuickStep(stepsize);
	}



	/**
	 * @brief Set the number of iterations that the QuickStep method performs per
	 *        step.
	 * @ingroup world
	 * @remarks
	 * More iterations will give a more accurate solution, but will take
	 * longer to compute.
	 * @param num The default is 20 iterations.
	 */
	//ODE_API 
	public static void dWorldSetQuickStepNumIterations (DWorld w, int num) {
		((DxWorld)w).dWorldSetQuickStepNumIterations(num);
	}



	/**
	 * @brief Get the number of iterations that the QuickStep method performs per
	 *        step.
	 * @ingroup world
	 * @return nr of iterations
	 */
	//ODE_API 
	int dWorldGetQuickStepNumIterations (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the SOR over-relaxation parameter
	 * @ingroup world
	 * @param over_relaxation value to use by SOR
	 */
	//ODE_API 
	void dWorldSetQuickStepW (DWorld w, double over_relaxation) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the SOR over-relaxation parameter
	 * @ingroup world
	 * @returns the over-relaxation setting
	 */
	//ODE_API 
	double dWorldGetQuickStepW (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/* World contact parameter functions */

	/**
	 * @brief Set the maximum correcting velocity that contacts are allowed
	 * to generate.
	 * @ingroup world
	 * @param vel The default value is infinity (i.e. no limit).
	 * @remarks
	 * Reducing this value can help prevent "popping" of deeply embedded objects.
	 */
	//ODE_API 
	public static void dWorldSetContactMaxCorrectingVel (DWorld w, double vel) {
		((DxWorld)w).dWorldSetContactMaxCorrectingVel(vel);
	}


	/**
	 * @brief Get the maximum correcting velocity that contacts are allowed
	 * to generated.
	 * @ingroup world
	 */
	//ODE_API 
	double dWorldGetContactMaxCorrectingVel (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the depth of the surface layer around all geometry objects.
	 * @ingroup world
	 * @remarks
	 * Contacts are allowed to sink into the surface layer up to the given
	 * depth before coming to rest.
	 * @param depth The default value is zero.
	 * @remarks
	 * Increasing this to some small value (e.g. 0.001) can help prevent
	 * jittering problems due to contacts being repeatedly made and broken.
	 */
	//ODE_API 
	public static void dWorldSetContactSurfaceLayer (DWorld w, double depth) {
		((DxWorld)w).dWorldSetContactSurfaceLayer(depth);
	}


	/**
	 * @brief Get the depth of the surface layer around all geometry objects.
	 * @ingroup world
	 * @returns the depth
	 */
	//ODE_API 
	double dWorldGetContactSurfaceLayer (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/* StepFast1 functions */

	/**
	 * @brief Step the world using the StepFast1 algorithm.
	 * @param stepsize the nr of seconds to advance the simulation.
	 * @param maxiterations The number of iterations to perform.
	 * @ingroup world
	 */
	//ODE_API 
	void dWorldStepFast1(DWorld w, double stepsize, int maxiterations) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @defgroup disable Automatic Enabling and Disabling
	 * @ingroup world bodies
	 *
	 * Every body can be enabled or disabled. Enabled bodies participate in the
	 * simulation, while disabled bodies are turned off and do not get updated
	 * during a simulation step. New bodies are always created in the enabled state.
	 *
	 * A disabled body that is connected through a joint to an enabled body will be
	 * automatically re-enabled at the next simulation step.
	 *
	 * Disabled bodies do not consume CPU time, therefore to speed up the simulation
	 * bodies should be disabled when they come to rest. This can be done automatically
	 * with the auto-disable feature.
	 *
	 * If a body has its auto-disable flag turned on, it will automatically disable
	 * itself when
	 *   @li It has been idle for a given number of simulation steps.
	 *   @li It has also been idle for a given amount of simulation time.
	 *
	 * A body is considered to be idle when the magnitudes of both its
	 * linear average velocity and angular average velocity are below given thresholds.
	 * The sample size for the average defaults to one and can be disabled by setting
	 * to zero with
	 *
	 * Thus, every body has six auto-disable parameters: an enabled flag, a idle step
	 * count, an idle time, linear/angular average velocity thresholds, and the
	 * average samples count.
	 *
	 * Newly created bodies get these parameters from world.
	 */

	/**
	 * @brief Set the AutoEnableDepth parameter used by the StepFast1 algorithm.
	 * @ingroup disable
	 */
	//ODE_API 
	void dWorldSetAutoEnableDepthSF1(DWorld w, int autoEnableDepth) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the AutoEnableDepth parameter used by the StepFast1 algorithm.
	 * @ingroup disable
	 */
	//ODE_API 
	int dWorldGetAutoEnableDepthSF1(DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable linear threshold for newly created bodies.
	 * @ingroup disable
	 * @return the threshold
	 */
	//ODE_API 
	double dWorldGetAutoDisableLinearThreshold (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable linear threshold for newly created bodies.
	 * @param linear_threshold default is 0.01
	 * @ingroup disable
	 */
	//ODE_API 
	void  dWorldSetAutoDisableLinearThreshold (DWorld w, double linear_threshold) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable angular threshold for newly created bodies.
	 * @ingroup disable
	 * @return the threshold
	 */
	//ODE_API 
	double dWorldGetAutoDisableAngularThreshold (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable angular threshold for newly created bodies.
	 * @param linear_threshold default is 0.01
	 * @ingroup disable
	 */
	//ODE_API 
	void dWorldSetAutoDisableAngularThreshold (DWorld w, double angular_threshold) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable linear average threshold for newly created bodies.
	 * @ingroup disable
	 * @return the threshold
	 */
	//ODE_API 
	double dWorldGetAutoDisableLinearAverageThreshold (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable linear average threshold for newly created bodies.
	 * @param linear_average_threshold default is 0.01
	 * @ingroup disable
	 */
	//ODE_API 
	void  dWorldSetAutoDisableLinearAverageThreshold (DWorld w, double linear_average_threshold) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable angular average threshold for newly created bodies.
	 * @ingroup disable
	 * @return the threshold
	 */
	//ODE_API 
	double dWorldGetAutoDisableAngularAverageThreshold (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable angular average threshold for newly created bodies.
	 * @param linear_average_threshold default is 0.01
	 * @ingroup disable
	 */
	//ODE_API 
	void dWorldSetAutoDisableAngularAverageThreshold (DWorld w, double angular_average_threshold) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable sample count for newly created bodies.
	 * @ingroup disable
	 * @return number of samples used
	 */
	//ODE_API 
	int dWorldGetAutoDisableAverageSamplesCount (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable average sample count for newly created bodies.
	 * @ingroup disable
	 * @param average_samples_count Default is 1, meaning only instantaneous velocity is used.
	 * Set to zero to disable sampling and thus prevent any body from auto-disabling.
	 */
	//ODE_API 
	//	 void dWorldSetAutoDisableAverageSamplesCount (dWorld w, 
	//	 unsigned int average_samples_count ) {
	public static void dWorldSetAutoDisableAverageSamplesCount (DWorld w,  
			int average_samples_count ) {
		((DxWorld)w).dWorldSetAutoDisableAverageSamplesCount(average_samples_count);
	}


	/**
	 * @brief Get auto disable steps for newly created bodies.
	 * @ingroup disable
	 * @return nr of steps
	 */
	//ODE_API 
	int dWorldGetAutoDisableSteps (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable steps for newly created bodies.
	 * @ingroup disable
	 * @param steps default is 10
	 */
	//ODE_API 
	void dWorldSetAutoDisableSteps (DWorld w, int steps) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable time for newly created bodies.
	 * @ingroup disable
	 * @return nr of seconds
	 */
	//ODE_API 
	double dWorldGetAutoDisableTime (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable time for newly created bodies.
	 * @ingroup disable
	 * @param time default is 0 seconds
	 */
	//ODE_API 
	void dWorldSetAutoDisableTime (DWorld w, double time) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get auto disable flag for newly created bodies.
	 * @ingroup disable
	 * @return 0 or 1
	 */
	//ODE_API 
	int dWorldGetAutoDisableFlag (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set auto disable flag for newly created bodies.
	 * @ingroup disable
	 * @param do_auto_disable default is false.
	 */
	//ODE_API 
	public static void dWorldSetAutoDisableFlag (DWorld w, boolean do_auto_disable) {
		((DxWorld)w).dWorldSetAutoDisableFlag(do_auto_disable);
	}



	/**
	 * @defgroup damping Damping
	 * @ingroup bodies world
	 *
	 * Damping serves two purposes: reduce simulation instability, and to allow
	 * the bodies to come to rest (and possibly auto-disabling them).
	 *
	 * Bodies are constructed using the world's current damping parameters. Setting
	 * the scales to 0 disables the damping.
	 *
	 * Here is how it is done: after every time step linear and angular
	 * velocities are tested against the corresponding thresholds. If they
	 * are above, they are multiplied by (1 - scale). So a negative scale value
	 * will actually increase the speed, and values greater than one will
	 * make the object oscillate every step {
		throw new UnsupportedOperationException();
	}
 both can make the simulation unstable.
	 *
	 * To disable damping just set the damping scale to zero.
	 *
	 * You can also limit the maximum angular velocity. In contrast to the damping
	 * functions, the angular velocity is affected before the body is moved.
	 * This means that it will introduce errors in joints that are forcing the body
	 * to rotate too fast. Some bodies have naturally high angular velocities
	 * (like cars' wheels), so you may want to give them a very high (like the default,
	 * dInfinity) limit.
	 *
	 * @note The velocities are damped after the stepper function has moved the
	 * object. Otherwise the damping could introduce errors in joints. First the
	 * joint constraints are processed by the stepper (moving the body), then
	 * the damping is applied.
	 *
	 * @note The damping happens right after the moved callback is called {
		throw new UnsupportedOperationException();
	}
 this way
	 * it still possible use the exact velocities the body has acquired during the
	 * step. You can even use the callback to create your own customized damping.
	 */

	/**
	 * @brief Get the world's linear damping threshold.
	 * @ingroup damping
	 */
	//ODE_API 
	double dWorldGetLinearDampingThreshold (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the world's linear damping threshold.
	 * @param threshold The damping won't be applied if the linear speed is
	 *        below this threshold. Default is 0.01.
	 * @ingroup damping
	 */
	//ODE_API 
	void dWorldSetLinearDampingThreshold(DWorld w, double threshold) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the world's angular damping threshold.
	 * @ingroup damping
	 */
	//ODE_API 
	double dWorldGetAngularDampingThreshold (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the world's angular damping threshold.
	 * @param threshold The damping won't be applied if the angular speed is
	 *        below this threshold. Default is 0.01.
	 * @ingroup damping
	 */
	//ODE_API 
	void dWorldSetAngularDampingThreshold(DWorld w, double threshold) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the world's linear damping scale.
	 * @ingroup damping
	 */
	//ODE_API 
	double dWorldGetLinearDamping (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the world's linear damping scale.
	 * @param scale The linear damping scale that is to be applied to bodies.
	 * Default is 0 (no damping). Should be in the interval [0, 1].
	 * @ingroup damping
	 */
	//ODE_API 
	public static void dWorldSetLinearDamping (DWorld w, double scale) {
		((DxWorld)w).dWorldSetLinearDamping(scale);
	}


	/**
	 * @brief Get the world's angular damping scale.
	 * @ingroup damping
	 */
	//ODE_API 
	double dWorldGetAngularDamping (DWorld w) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the world's angular damping scale.
	 * @param scale The angular damping scale that is to be applied to bodies.
	 * Default is 0 (no damping). Should be in the interval [0, 1].
	 * @ingroup damping
	 */
	//ODE_API 
	public static void dWorldSetAngularDamping(DWorld w, double scale) {
		((DxWorld)w).dWorldSetAngularDamping(scale);
	}


	/**
	 * @brief Convenience function to set body linear and angular scales.
	 * @param linear_scale The linear damping scale that is to be applied to bodies.
	 * @param angular_scale The angular damping scale that is to be applied to bodies.
	 * @ingroup damping
	 */
	//ODE_API 
	void dWorldSetDamping(DWorld w,
			double linear_scale,
			double angular_scale) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the default maximum angular speed.
	 * @ingroup damping
	 * @sa dBodyGetMaxAngularSpeed()
	 */
	//ODE_API 
	double dWorldGetMaxAngularSpeed (DWorld w) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Set the default maximum angular speed for new bodies.
	 * @ingroup damping
	 * @sa dBodySetMaxAngularSpeed()
	 */
	//ODE_API 
	public static void dWorldSetMaxAngularSpeed (DWorld w, double max_speed) {
		((DxWorld)w).dWorldSetMaxAngularSpeed(max_speed);
	}


}
