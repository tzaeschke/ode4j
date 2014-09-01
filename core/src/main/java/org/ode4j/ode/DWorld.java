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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.threading.DThreadingImplementation;
import org.ode4j.ode.threading.Threading_H.DThreadingFunctionsInfo;

/**
 * The world object is a container for rigid bodies and joints. Objects in
 * different worlds can not interact, for example rigid bodies from two
 * different worlds can not collide.
 * <p>
 * All the objects in a world exist at the same point in time, thus one
 * reason to use separate worlds is to simulate systems at different rates.
 * Most applications will only need one world.
 * 
 * 
 * <p>
 * <b>Automatic Enabling and Disabling</b>
 * <p>
 * Every body can be enabled or disabled. Enabled bodies participate in the
 * simulation, while disabled bodies are turned off and do not get updated
 * during a simulation step. New bodies are always created in the enabled state.
 * <p>
 * A disabled body that is connected through a joint to an enabled body will be
 * automatically re-enabled at the next simulation step.
 * <p>
 * Disabled bodies do not consume CPU time, therefore to speed up the simulation
 * bodies should be disabled when they come to rest. This can be done automatically
 * with the auto-disable feature.
 * <p>
 * If a body has its auto-disable flag turned on, it will automatically disable
 * itself when
 *   <li> It has been idle for a given number of simulation steps.</li>
 *   <li> It has also been idle for a given amount of simulation time.</li>
 * <p>
 * A body is considered to be idle when the magnitudes of both its
 * linear average velocity and angular average velocity are below given thresholds.
 * The sample size for the average defaults to one and can be disabled by setting
 * to zero with
 * <p>
 * Thus, every body has six auto-disable parameters: an enabled flag, a idle step
 * count, an idle time, linear/angular average velocity thresholds, and the
 * average samples count.
 * <p>
 * Newly created bodies get these parameters from world.
 * <p>
 * <p>
 * 
 * <b>Damping</b><br>
 * <p>
 * Damping serves two purposes: reduce simulation instability, and to allow
 * the bodies to come to rest (and possibly auto-disabling them).
 * <p>
 * Bodies are constructed using the world's current damping parameters. Setting
 * the scales to 0 disables the damping.
 * <p>
 * Here is how it is done: after every time step linear and angular
 * velocities are tested against the corresponding thresholds. If they
 * are above, they are multiplied by (1 - scale). So a negative scale value
 * will actually increase the speed, and values greater than one will
 * make the object oscillate every step; both can make the simulation unstable.
 * <p>
 * To disable damping just set the damping scale to zero.
 * <p>
 * You can also limit the maximum angular velocity. In contrast to the damping
 * functions, the angular velocity is affected before the body is moved.
 * This means that it will introduce errors in joints that are forcing the body
 * to rotate too fast. Some bodies have naturally high angular velocities
 * (like cars' wheels), so you may want to give them a very high (like the default,
 * dInfinity) limit.
 * <p>
 * <p>NOTE: The velocities are damped after the stepper function has moved the
 * object. Otherwise the damping could introduce errors in joints. First the
 * joint constraints are processed by the stepper (moving the body), then
 * the damping is applied.
 * <p>
 * <p>NOTE: The damping happens right after the moved callback is called; this way
 * it still possible use the exact velocities the body has acquired during the
 * step. You can even use the callback to create your own customized damping.
 */

public interface DWorld {

	//~dWorld()
	void DESTRUCTOR();

	/**
	 * Set the user-data pointer.
	 * @param world the world to set the data on
	 * @param data
	 */
	void setData (Object data);


	/**
	 * Get the user-data pointer.
	 * @param world the world to set the data on
	 * @param data
	 */
	Object getData ();

	
	/**
	 * Set the world's global gravity vector.
	 *
	 * The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
	 * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
	 */
	void setGravity (double x, double y, double z);

	
	/**
	 * Set the world's global gravity vector.
	 *
	 * The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
	 * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
	 */
	void setGravity (DVector3C g);

	
	/**
	 * Get the gravity vector for a given world.
	 */
	void getGravity (DVector3 g) ;


	/**
	 * Set the global ERP value, that controls how much error
	 * correction is performed in each time step.
	 * @param erp Typical values are in the range 0.1--0.8. The default is 0.2.
	 */
	void setERP (double erp);

	
	/**
	 * Get the error reduction parameter.
	 * 
	 * @return ERP value
	 */
	double getERP() ;


	/**
	 * Set the global CFM (constraint force mixing) value.
	 * 
	 * @param cfm Typical values are in the range @m{10^{-9}} -- 1.
	 * The default is 10^-5 if single precision is being used, or 10^-10
	 * if double precision is being used.
	 */
	void setCFM (double cfm);

	
	/**
	 * Get the constraint force mixing value.
	 * @return CFM value
	 */
	double getCFM() ;

	public static int dWORLDSTEP_THREADCOUNT_UNLIMITED = 0;

	/**
	 * Set maximum threads to be used for island stepping
	 *
	 * The actual number of threads that is going to be used will be the minimum
	 * of this limit and number of threads in the threading pool. By default 
	 * there is no limit ({@code WORLDSTEP_THREADCOUNT_UNLIMITED}).
	 *
	 * <p>WARNING: Running island stepping in multiple threads requires allocating 
	 * individual stepping memory buffer for each of those threads. The size of buffers
	 * allocated is the size needed to handle the largest island in the world.
	 *
	 * <p>NOTE: Setting a limit for island stepping does not affect threading at lower
	 * levels in stepper functions. The sub-calls scheduled from them can be executed
	 * in as many threads as there are available in the pool.
	 *
	 * @param w The world affected
	 * @param count Thread count limit value for island stepping
	 * @see #getStepIslandsProcessingMaxThreadCount()
	 */
	@Deprecated
	void setStepIslandsProcessingMaxThreadCount(int count);
	
	/**
	 * Get maximum threads that are allowed to be used for island stepping.
	 *
	 * Please read commentaries to {@code setStepIslandsProcessingMaxThreadCount} for 
	 * important information regarding the value returned.
	 *
	 * @param w The world queried
	 * @return Current thread count limit value for island stepping
	 * @see #setStepIslandsProcessingMaxThreadCount(int)
	 */
	@Deprecated
	int getStepIslandsProcessingMaxThreadCount();

	/**
	 * Set the world to use shared working memory along with another world.
	 *
	 * The worlds allocate working memory internally for simulation stepping. This
	 * memory is cached among the calls to <code>dWordStep</code> and <code>dWorldQuickStep</code>. 
	 * Similarly, several worlds can be set up to share this memory caches thus 
	 * reducing overall memory usage by cost of making worlds inappropriate for 
	 * simultaneous simulation in multiple threads.
	 *
	 * If null value is passed for <code>from_world</code> parameter the world is detached from 
	 * sharing and returns to defaults for working memory, reservation policy and 
	 * memory manager as if just created. This can also be used to enable use of shared 
	 * memory for a world that has already had working memory allocated privately.
	 * Normally using shared memory after a world has its private working memory allocated
	 * is prohibited.
	 *
	 * Allocation policy used can only increase world's internal reserved memory size
	 * and never decreases it. <code>dWorldCleanupWorkingMemory</code> can be used to release 
	 * working memory for a world in case if number of objects/joint decreases 
	 * significantly in it.
	 *
	 * With sharing working memory worlds also automatically share memory reservation 
	 * policy and memory manager. Thus, these parameters need to be customized for
	 * initial world to be used as sharing source only.
	 *
	 * If worlds share working memory they must also use compatible threading implementations
	 * (i.e. it is illegal for one world to perform stepping with self-threaded implementation
	 * when the other world is assigned a multi-threaded implementation). 
	 * For more information read section about threading approaches in ODE.
	 *
	 * Failure result status means a memory allocation failure.
	 *
	 * @param from_world Null or the world the shared memory is to be used from.
	 * @return 1 for success and 0 for failure.
	 *
	 * @see #cleanupWorkingMemory()
	 * @see #setStepMemoryReservationPolicy(DWorldStepReserveInfo)
//	 * @see #setStepMemoryManager(DWorldStepMemoryFunctionsInfo)
	 */
	boolean useSharedWorkingMemory(DWorld from_world/*=NULL*/);

	/**
	 * Release internal working memory allocated for world
	 *
	 * The worlds allocate working memory internally for simulation stepping. This 
	 * function can be used to free world's internal memory cache in case if number of
	 * objects/joints in the world decreases significantly. By default, internal 
	 * allocation policy is used to only increase cache size as necessary and never 
	 * decrease it.
	 *
	 * If a world shares its working memory with other worlds the cache deletion 
	 * affects all the linked worlds. However the shared status itself remains intact.
	 *
	 * The function call does affect neither memory reservation policy nor memory manager.
	 *
	 * @see #useSharedWorkingMemory(DWorld)
	 * @see #setStepMemoryReservationPolicy(DWorldStepReserveInfo)
//	 * @see #setStepMemoryManager(DWorldStepMemoryFunctionsInfo)
	 */
	void cleanupWorkingMemory();

	public static final double dWORLDSTEP_RESERVEFACTOR_DEFAULT = 1.2f;
	public static final int dWORLDSTEP_RESERVESIZE_DEFAULT = 65536;

	/**
	 * Memory reservation policy descriptor structure for world stepping functions.
	 *
	 * <code>struct_size</code> should be assigned the size of the structure.
	 *
	 * <code>reserve_factor</code> is a quotient that is multiplied by required memory size
	 *  to allocate extra reserve whenever reallocation is needed.
	 *
	 * <code>reserve_minimum</code> is a minimum size that is checked against whenever reallocation 
	 * is needed to allocate expected working memory minimum at once without extra 
	 * reallocations as number of bodies/joints grows.
	 *
	 * @see DWorld#setStepMemoryReservationPolicy(DWorldStepReserveInfo)
	 */
	public class DWorldStepReserveInfo	{
	    public int struct_size;
	    public double reserve_factor; // Use float as precision does not matter here
	    public int reserve_minimum;
	};

	/**
	 * Set memory reservation policy for world to be used with simulation stepping functions
	 *
	 * The function allows to customize reservation policy to be used for internal
	 * memory which is allocated to aid simulation for a world. By default, values
	 * of <code>dWORLDSTEP_RESERVEFACTOR_DEFAULT</code> and <code>dWORLDSTEP_RESERVESIZE_DEFAULT</code>
	 * are used.
	 *
	 * Passing <code>policyinfo</code> argument as NULL results in reservation policy being
	 * reset to defaults as if the world has been just created. The content of 
	 * <code>policyinfo</code> structure is copied internally and does not need to remain valid
	 * after the call returns.
	 *
	 * If the world uses working memory sharing, changing memory reservation policy
	 * affects all the worlds linked together.
	 *
	 * Failure result status means a memory allocation failure.
	 *
	 * @param policyinfo Null or a pointer to policy descriptor structure.
	 * @return 1 for success and 0 for failure.
	 *
	 * @see #useSharedWorkingMemory(DWorld)
	 */
	boolean setStepMemoryReservationPolicy(final DWorldStepReserveInfo policyinfo/*=NULL*/);


	/**
	 * Assign threading implementation to be used for [quick]stepping the world.
	 *
	 * <p>WARNING: It is not recommended to assign the same threading implementation to
	 * different worlds if they are going to be called in parallel. In particular this
	 * makes resources preallocation for threaded calls to lose its sense. 
	 * Built-in threading implementation is likely to crash if misused this way.
	 * 
	 * @param w The world to change threading implementation for.
	 * @param functions_info Pointer to threading functions structure
	 * @param threading_impl ID of threading implementation object
	 * @deprecated This is not supported by ode4j.
	 */
	void setStepThreadingImplementation(final DThreadingFunctionsInfo functions_info, 
			DThreadingImplementation threading_impl);

	/**
	 * Step the world.
	 *
	 * This uses a "big matrix" method that takes time on the order of m^3
	 * and memory on the order of m^2, where m is the total number of constraint
	 * rows. For large systems this will use a lot of memory and can be very slow,
	 * but this is currently the most accurate method.
	 *
	 * Failure result status means that the memory allocation has failed for operation.
	 * In such a case all the objects remain in unchanged state and simulation can be
	 * retried as soon as more memory is available.
	 *
	 * @param stepsize The number of seconds that the simulation has to advance.
	 */
	//* @return 1 for success and 0 for failure
	void step (double stepsize);


	/**
	 * Quick-step the world.
	 * 
	 * This uses an iterative method that takes time on the order of m*N
	 * and memory on the order of m, where m is the total number of constraint
	 * rows N is the number of iterations.
	 * For large systems this is a lot faster than dWorldStep(),
	 * but it is less accurate.
	 * 
	 * QuickStep is great for stacks of objects especially when the
	 * auto-disable feature is used as well.
	 * However, it has poor accuracy for near-singular systems.
	 * Near-singular systems can occur when using high-friction contacts, motors,
	 * or certain articulated structures. For example, a robot with multiple legs
	 * sitting on the ground may be near-singular.
	 * 
	 * There are ways to help overcome QuickStep's inaccuracy problems:
	 * 
	 * <li> Increase CFM. </li>
	 * <li> Reduce the number of contacts in your system (e.g. use the minimum
	 *     number of contacts for the feet of a robot or creature). </li>
	 * <li> Don't use excessive friction in the contacts. </li>
	 * <li> Use contact slip if appropriate </li>
	 * <li> Avoid kinematic loops (however, kinematic loops are inevitable in
	 *     legged creatures). </li>
	 * <li> Don't use excessive motor strength. </li>
	 * <li> Use force-based motors instead of velocity-based motors. </li>
	 * <p>
	 * Increasing the number of QuickStep iterations may help a little bit, but
	 * it is not going to help much if your system is really near singular.
	 *
	 * Failure result status means that the memory allocation has failed for operation.
	 * In such a case all the objects remain in unchanged state and simulation can be
	 * retried as soon as more memory is available.
	 *
	 * @param stepsize The number of seconds that the simulation has to advance.
	 * @return 1 for success and 0 for failure
	 */
	boolean quickStep(double stepsize);
	
	/**
	* Converts an impulse to a force.
    *
	* If you want to apply a linear or angular impulse to a rigid body,
	* instead of a force or a torque, then you can use this function to convert
	* the desired impulse into a force/torque vector before calling the
	* BodyAdd... function.
	* The current algorithm simply scales the impulse by 1/stepsize,
	* where stepsize is the step size for the next step that will be taken.
	* This function is given a dWorldID because, in the future, the force
	* computation may depend on integrator parameters that are set as
	* properties of the world.
	*/
	void impulseToForce(double stepsize, double ix, double iy, double iz, 
	        DVector3 force);
	
	
	/**
	 * Set the number of iterations that the QuickStep method performs per
	 *        step.
	 * 
	 * <p>REMARK:
	 * More iterations will give a more accurate solution, but will take
	 * longer to compute.
	 * 
	 * @param num The default is 20 iterations.
	 */
	void setQuickStepNumIterations(int num);
	
	
	/**
	 * Get the number of iterations that the QuickStep method performs per
	 *        step.
	 * @return nr of iterations
	 */
	int getQuickStepNumIterations() ;
	
	
	/**
	 * Set the SOR over-relaxation parameter
	 * 
	 * @param over_relaxation value to use by SOR
	 */
	void setQuickStepW(double over_relaxation);
	
	
	/**
	 * Get the SOR over-relaxation parameter.
	 * @return the over-relaxation setting
	 */
	double getQuickStepW();

	
	/**
	 * Set auto disable linear threshold for newly created bodies.
	 * @param threshold default is 0.01
	 */
	void  setAutoDisableLinearThreshold (double threshold);
	
	
	/**
	 * Get auto disable linear threshold for newly created bodies.
	 * @return the threshold
	 */
	double getAutoDisableLinearThreshold();
	
	
	/**
	 * Set auto disable angular threshold for newly created bodies.
	 * @param threshold default is 0.01
	 */
	void setAutoDisableAngularThreshold (double threshold);

	
	/**
	 * Get auto disable angular threshold for newly created bodies.
	 * @return the threshold
	 */
	double getAutoDisableAngularThreshold();


//	/**
//	 * Get auto disable linear average threshold for newly created bodies.
//	 * @return the threshold
//	 * @deprecated Not implemented in ODE.
//	 */
//	double getAutoDisableLinearAverageThreshold ();
//
//
//	/**
//	 * Set auto disable linear average threshold for newly created bodies.
//	 * @param linear_average_threshold default is 0.01
//	 * @deprecated Not implemented in ODE.
//	 */
//	void setAutoDisableLinearAverageThreshold (double linear_average_threshold);
//
//
//	/**
//	 * Get auto disable angular average threshold for newly created bodies.
//	 * @return the threshold
//	 * @deprecated Not implemented in ODE.
//	 */
//	double getAutoDisableAngularAverageThreshold ();
//
//
//	/**
//	 * Set auto disable angular average threshold for newly created bodies.
//	 * @param angular_average_threshold default is 0.01
//	 * @deprecated Not implemented in ODE.
//	 */
//	void setAutoDisableAngularAverageThreshold (double angular_average_threshold);


	/**
	 * Get auto disable sample count for newly created bodies.
	 * @return number of samples used
	 */
	int getAutoDisableAverageSamplesCount ();


	/**
	 * Set auto disable average sample count for newly created bodies.
	 * @param average_samples_count Default is 1, meaning only instantaneous velocity is used.
	 * Set to zero to disable sampling and thus prevent any body from auto-disabling.
	 */
	void setAutoDisableAverageSamplesCount (int average_samples_count );
	

	/**
	 * Set the depth of the surface layer around all geometry objects.
	 * 
	 * <p>REMARK:
	 * Contacts are allowed to sink into the surface layer up to the given
	 * depth before coming to rest.
	 * 
	 * @param depth The default value is zero.
	 * <p>REMARK:
	 * Increasing this to some small value (e.g. 0.001) can help prevent
	 * jittering problems due to contacts being repeatedly made and broken.
	 */
	void setContactSurfaceLayer(double depth);
	
	
	/**
	 * Get the depth of the surface layer around all geometry objects.
	 * @return the depth
	 */
	double getContactSurfaceLayer();

	
	/**
	 * Set the maximum correcting velocity that contacts are allowed
	 * to generate.
	 * 
	 * <p>REMARK:
	 * Reducing this value can help prevent "popping" of deeply embedded objects.
	 * 
	 * @param vel The default value is infinity (i.e. no limit).
	 */
	void setContactMaxCorrectingVel (double vel);
	
	
	/**
	 * Get the maximum correcting velocity that contacts are allowed
	 * to generated.
	 */
	//ODE_API 
	double getContactMaxCorrectingVel();
	
	
	/**
	 * Destroy a world and everything in it.
	 *
	 * This includes all bodies, and all joints that are not part of a joint
	 * group. Joints that are part of a joint group will be deactivated, and
	 * can be destroyed by calling, for example, dJointGroupEmpty().
	 */
	void destroy();

	
	/**
	 * Get auto disable steps for newly created bodies.
	 * @return nr of steps
	 */
	int getAutoDisableSteps ();


	/**
	 * Set auto disable steps for newly created bodies.
	 * @param steps default is 10
	 */
	void setAutoDisableSteps (int steps);


	/**
	 * Get auto disable time for newly created bodies.
	 * @return nr of seconds
	 */
	double getAutoDisableTime ();


	/**
	 * Set auto disable time for newly created bodies.
	 * @param time default is 0 seconds
	 */
	void setAutoDisableTime (double time);


	/**
	 * Get auto disable flag for newly created bodies.
	 * @return 0 or 1
	 */
	boolean getAutoDisableFlag ();


	/**
	 * Set auto disable flag for newly created bodies.
	 * @param do_auto_disable default is false.
	 */
	void setAutoDisableFlag (boolean do_auto_disable);




	/**
	 * Get the world's linear damping threshold.
	 */
	double getLinearDampingThreshold ();


	/**
	 * Set the world's linear damping threshold.
	 * @param threshold The damping won't be applied if the linear speed is
	 *        below this threshold. Default is 0.01.
	 */
	void setLinearDampingThreshold(double threshold);


	/**
	 * Get the world's angular damping threshold.
	 */
	double getAngularDampingThreshold ();


	/**
	 * Set the world's angular damping threshold.
	 * @param threshold The damping won't be applied if the angular speed is
	 *        below this threshold. Default is 0.01.
	 */
	void setAngularDampingThreshold(double threshold);


	/**
	 * Get the world's linear damping scale.
	 */
	double getLinearDamping ();


	/**
	 * Set the world's linear damping scale.
	 * @param scale The linear damping scale that is to be applied to bodies.
	 * Default is 0 (no damping). Should be in the interval [0, 1].
	 */
	void setLinearDamping (double scale);


	/**
	 * Get the world's angular damping scale.
	 */
	double getAngularDamping ();


	/**
	 * Set the world's angular damping scale.
	 * @param scale The angular damping scale that is to be applied to bodies.
	 * Default is 0 (no damping). Should be in the interval [0, 1].
	 */
	void setAngularDamping(double scale);


	/**
	 * Convenience function to set body linear and angular scales.
	 * @param linear_scale The linear damping scale that is to be applied to bodies.
	 * @param angular_scale The angular damping scale that is to be applied to bodies.
	 */
	void setDamping(double linear_scale, double angular_scale);


	/**
	 * Get the default maximum angular speed.
	 * @see DBody#getMaxAngularSpeed()
	 */
	double getMaxAngularSpeed ();


	/**
	 * Set the default maximum angular speed for new bodies.
	 * @see DBody#setMaxAngularSpeed(double)
	 */
	void setMaxAngularSpeed (double max_speed);
}
