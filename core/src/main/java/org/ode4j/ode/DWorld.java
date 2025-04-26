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
import org.ode4j.ode.threading.task.TaskExecutor;

import java.util.concurrent.atomic.AtomicInteger;

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
 * <ul>
 *   <li> It has been idle for a given number of simulation steps.</li>
 *   <li> It has also been idle for a given amount of simulation time.</li>
 * </ul>
 * <p>
 * A body is considered to be idle when the magnitudes of both its
 * linear average velocity and angular average velocity are below given thresholds.
 * The sample size for the average defaults to one and can be disabled by setting
 * to zero with {@code #setAutoDisableAverageSamplesCount}
 * <p>
 * Thus, every body has six auto-disable parameters: an enabled flag, a idle step
 * count, an idle time, linear/angular average velocity thresholds, and the
 * average samples count.
 * <p>
 * Newly created bodies get these parameters from world.
 * 
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
 * NOTE: The velocities are damped after the stepper function has moved the
 * object. Otherwise the damping could introduce errors in joints. First the
 * joint constraints are processed by the stepper (moving the body), then
 * the damping is applied.
 * <p>
 * NOTE: The damping happens right after the moved callback is called; this way
 * it still possible use the exact velocities the body has acquired during the
 * step. You can even use the callback to create your own customized damping.
 */

public interface DWorld {

	//~dWorld()
	void DESTRUCTOR();

	/**
	 * Set the user-data pointer.
	 * @param data user data
	 */
	void setData (Object data);


	/**
	 * Get the user-data pointer.
	 * @return User data
	 */
	Object getData ();

	
	/**
	 * Set the world's global gravity vector.
	 *
	 * <p>The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
	 * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setGravity (double x, double y, double z);

	
	/**
	 * Set the world's global gravity vector.
	 *
	 * <p>The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
	 * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
	 * @param g vector
	 */
	void setGravity (DVector3C g);

	
	/**
	 * Get the gravity vector for a given world.
	 * @param result Contains the result after calling the function
	 */
	void getGravity (DVector3 result) ;


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

	// TZ: This should be in threading.h
	static int dTHREADING_THREAD_COUNT_UNLIMITED = 0;
	public static int dWORLDSTEP_THREADCOUNT_UNLIMITED = dTHREADING_THREAD_COUNT_UNLIMITED;

	enum dWSTP
	{
		/** Thread count limit for parallel island iteration */
		dWSTP_WorldIslandsIterationMaxThreads(0x001),
		/** Thread count limit for stepping each individual island being iterated */
		dWSTP_IslandSteppingMaxThreads( 0x002),
		/** Thread count limit for solving QuickStep/Step LCP */
		dWSTP_LCPSolvingMaxThreads(0x004);
		public final int v;
		dWSTP(int v) {
			this.v = v;
		}
	}

	class dWorldSteppingThreadingParameters
	{
		/* must always be defined */
		// unsigned
		public int param_set; /**< Combination of dWSTP_... flags */
		// unsigned
		public int world_islands_iteration_max_threads;
		// unsigned
		public int island_stepping_max_threads;
		// unsigned
		public int lcp_solving_max_threads;

	} // dWorldSteppingThreadingParameters;

	/**
	 * Set threading parameters to be used for world stepping
	 * <p>
	 * The function sets parameters defined by bits in {@code dWorldSteppingThreadingParameters::param_set}
	 * leaving other parameters unchanged. The actual number of threads that is going to be used will be the minimum
	 * of a specific limit and the number of threads in the thread pool.
	 * <p>
	 * By default, for {@code dWorldSteppingThreadingParameters::world_islands_iteration_max_threads}
	 * and {@code dWorldSteppingThreadingParameters::island_stepping_max_threads} there is no limit (@c dWORLDSTEP_THREADCOUNT_UNLIMITED);
	 * for {@code dWorldSteppingThreadingParameters::lcp_solving_max_threads} the default limit is 1 thread (as the single threaded variant
	 * appears to perform much better). These defaults may change in future releases.
	 * <p>
	 * The parameters act in the manner that world_islands_iteration_max_threads are used to iterate islands within the world. Then, for each island,
	 * up to island_stepping_max_threads are used to prepare inputs and distribute results while up to lcp_solving_max_threads are used for solving the LCP itself.
	 * Roughly, the total number of active threads could be up to world_islands_iteration_max_threads * MAX(island_stepping_max_threads, lcp_solving_max_threads)
	 * if there is a sufficient number of threads available in the thread pool assigned for the world.
	 *
	 * <p> NOTE: Note that in current implementation {@link #step(double)} supports single threaded LCP solving only and the lcp_solving_max_threads parameter has no effect for it.
	 *
	 * <p> NOTE: world_islands_iteration_max_threads is the parameter set with {@link #setStepIslandsProcessingMaxThreadCount(int)}
	 *
	 * <p> WARNING: 
	 * WARNING! Iterating islands in multiple threads requires allocating
	 * individual stepping memory buffer for each of those threads. The size of buffers
	 * allocated is the size needed to handle the largest island in the world.
	 *
	 * @param ptr_params Pointer to structure of type @c dWorldSteppingThreadingParameters
	 * @see #getSteppingThreadingParameters(dWorldSteppingThreadingParameters) 
	 * @see #setStepIslandsProcessingMaxThreadCount(int) 
	 */
	//	ODE_API
	//void dWorldSetSteppingThreadingParameters(dWorldID w, const dWorldSteppingThreadingParameters *ptr_params);
	void setSteppingThreadingParameters(final dWorldSteppingThreadingParameters ptr_params);

	/**
	 * Obtain threading parameters used for world stepping
	 * <p>
	 * The function retrieves parameters defined by bits in {@code dWorldSteppingThreadingParameters::param_set}.
	 * <p>
	 * NOTE: 
	 * world_islands_iteration_max_threads is the parameter returned by {@link #getStepIslandsProcessingMaxThreadCount()}
	 * <p>
	 * @param ptr_params Pointer to structure of type @c dWorldSteppingThreadingParameters; the structure must have its param_set field initialized
	 * @see #setSteppingThreadingParameters(dWorldSteppingThreadingParameters) 
	 * @see #getStepIslandsProcessingMaxThreadCount() 
	 */
	//ODE_API void dWorldGetSteppingThreadingParameters(dWorldID w, dWorldSteppingThreadingParameters *ptr_params);
	void getSteppingThreadingParameters(dWorldSteppingThreadingParameters ptr_params);


	/**
	 * Set maximum threads to be used for island stepping
	 * <p>
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
	 * @param count Thread count limit value for island stepping
	 * @see #getStepIslandsProcessingMaxThreadCount()
	 */
	@Deprecated
	void setStepIslandsProcessingMaxThreadCount(int count);
	
	/**
	 * Get maximum threads that are allowed to be used for island stepping.
	 *
	 * <p>Please read commentaries to {@code setStepIslandsProcessingMaxThreadCount} for
	 * important information regarding the value returned.
	 *
	 * @return Current thread count limit value for island stepping
	 * @see #setStepIslandsProcessingMaxThreadCount(int)
	 */
	@Deprecated
	int getStepIslandsProcessingMaxThreadCount();


	//	final double dWORLDSTEP_RESERVEFACTOR_DEFAULT = 1.2f;
	//	final int dWORLDSTEP_RESERVESIZE_DEFAULT = 65536;
	//
	//	/**
	//	 * Memory reservation policy descriptor structure for world stepping functions.
	//	 *
	//	 * <code>struct_size</code> should be assigned the size of the structure.
	//	 *
	//	 * <code>reserve_factor</code> is a quotient that is multiplied by required memory size
	//	 *  to allocate extra reserve whenever reallocation is needed.
	//	 *
	//	 * <code>reserve_minimum</code> is a minimum size that is checked against whenever reallocation
	//	 * is needed to allocate expected working memory minimum at once without extra
	//	 * reallocations as number of bodies/joints grows.
	//	 *
	//	 * See DxWorld.setStepMemoryReservationPolicy(DWorldStepReserveInfo)
	//	 */
	//	class DWorldStepReserveInfo	{
	//	    public int struct_size;
	//	    public double reserve_factor; // Use float as precision does not matter here
	//	    public int reserve_minimum;
	//	}


	/**
	 * Step the world.
	 *
	 * <p>This uses a "big matrix" method that takes time on the order of m^3
	 * and memory on the order of m^2, where m is the total number of constraint
	 * rows. For large systems this will use a lot of memory and can be very slow,
	 * but this is currently the most accurate method.
	 *
	 * <p>Failure result status means that the memory allocation has failed for operation.
	 * In such a case all the objects remain in unchanged state and simulation can be
	 * retried as soon as more memory is available.
	 *
	 * @param stepsize The number of seconds that the simulation has to advance.
	 * @deprecated Please use {@link #quickStep(double)} instead, it is faster and much more stable.
	 * Please report any problems with using quickStep() instead of step().
	 */
	//* @return 1 for success and 0 for failure
	@Deprecated
	void step (double stepsize);


	/**
	 * Quick-step the world.
	 * 
	 * <p>This uses an iterative method that takes time on the order of m*N
	 * and memory on the order of m, where m is the total number of constraint
	 * rows N is the number of iterations.
	 * For large systems this is a lot faster than dWorldStep(),
	 * but it is less accurate.
	 * 
	 * <p>QuickStep is great for stacks of objects especially when the
	 * auto-disable feature is used as well.
	 * However, it has poor accuracy for near-singular systems.
	 * Near-singular systems can occur when using high-friction contacts, motors,
	 * or certain articulated structures. For example, a robot with multiple legs
	 * sitting on the ground may be near-singular.
	 * 
	 * <p>There are ways to help overcome QuickStep's inaccuracy problems:
	 * 
	 * <ul>
	 * <li> Increase CFM. </li>
	 * <li> Reduce the number of contacts in your system (e.g. use the minimum
	 *     number of contacts for the feet of a robot or creature). </li>
	 * <li> Don't use excessive friction in the contacts. </li>
	 * <li> Use contact slip if appropriate </li>
	 * <li> Avoid kinematic loops (however, kinematic loops are inevitable in
	 *     legged creatures). </li>
	 * <li> Don't use excessive motor strength. </li>
	 * <li> Use force-based motors instead of velocity-based motors. </li>
	 * </ul>
	 * <p>
	 * Increasing the number of QuickStep iterations may help a little bit, but
	 * it is not going to help much if your system is really near singular.
	 *
	 * <p>Failure result status means that the memory allocation has failed for operation.
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
	* <p>If you want to apply a linear or angular impulse to a rigid body,
	* instead of a force or a torque, then you can use this function to convert
	* the desired impulse into a force/torque vector before calling the
	* BodyAdd... function.
	* The current algorithm simply scales the impulse by 1/stepsize,
	* where stepsize is the step size for the next step that will be taken.
	* This function is given a dWorldID because, in the future, the force
	* computation may depend on integrator parameters that are set as
	* properties of the world.
	 * @param stepsize stepsize
	 * @param ix ix
	 * @param iy iy
	 * @param iz iz
	 * @param force force 
	*/
	void impulseToForce(double stepsize, double ix, double iy, double iz, 
	        DVector3 force);
	
	// #define dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT                     20U
	int dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT = 20;

	/**
	 * Set the number of iterations that the QuickStep method performs per
	 *        step.
	 * 
	 * <p>REMARK:
	 * More iterations will give a more accurate solution, but will take
	 * longer to compute.
	 * 
	 * @param num The default is dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT iterations.
	 */
	void setQuickStepNumIterations(int num);
	
	
	/**
	 * Get the number of iterations that the QuickStep method performs per
	 *        step.
	 * @return nr of iterations
	 */
	int getQuickStepNumIterations() ;


	//	#define dWORLDQUICKSTEP_ITERATION_PREMATURE_EXIT_DELTA_DEFAULT      1e-8f
	double dWORLDQUICKSTEP_ITERATION_PREMATURE_EXIT_DELTA_DEFAULT = 1e-8f;
	//	#define dWORLDQUICKSTEP_MAXIMAL_EXTRA_ITERATION_COUNT_FACTOR_DEFAULT 1.0f
	double dWORLDQUICKSTEP_MAXIMAL_EXTRA_ITERATION_COUNT_FACTOR_DEFAULT = 1.0f;
	//	#define dWORLDQUICKSTEP_EXTRA_ITERATION_REQUIREMENT_DELTA_DEFAULT   1e-2f
	double dWORLDQUICKSTEP_EXTRA_ITERATION_REQUIREMENT_DELTA_DEFAULT = 1e-2f;

	//	/**
	//	 * Configure QuickStep method dynamic iteration count adjustment.
	//	 *
	//	 * <p> REMARK: The function controls dynamic iteration count adjustment basing on maximal contact force change
	//	 * per iteration in matrix.
	//	 *
	//	 * <p>If Premature Exit Delta is configured with {@code ptr_iteration_premature_exit_delta}
	//	 * and the maximal contact force adjustment does not exceed the value the iterations are abandoned
	//	 * prematurely and computations complete in fewer steps than it would take normally.
	//	 * Passing zero in {@code ptr_iteration_premature_exit_delta} will disable the premature exit and enforce
	//	 * unconditional execution of iteration count set by {@link #setQuickStepNumIterations(int)}.
	//	 *
	//	 * <p>If extra iterations are enabled by passing  a positive fraction in {@code ptr_max_num_extra_factor}
	//	 * and, after the normal number of iterations is executed, the maximal contact force adjustment is still
	//	 * larger than the limit set with the {@code ptr_extra_iteration_requirement_delta}, up to that fraction of
	//	 * normal iteration count is executed extra until the maximal contact force change falls below the margin.
	//	 *
	//	 * <p>At least one parameter must be not NULL for the call.
	//	 * If NULL is passed for any of the parameters the corresponding parameter will retain its previous value.
	//	 * If the standard number of iterations is changed with {@link #setQuickStepNumIterations(int)} call and
	//	 * an extra iteration count was configured with {@code ptr_max_num_extra_factor} the extra absolute value will be
	//	 * adjusted accordingly.
	//	 *
	//	 * @param ptr_iteration_premature_exit_delta A margin value such that, if contact force adjustment value maximum in an iteration
	//	 * becomes less, the method is allowed to terminate prematurely.
	//	 * @param ptr_max_num_extra_factor A non-negative coefficient that defines fraction of the standard iteration count to be executed extra
	//	 * if contact force still significantly changes after the standard iterations complete.
	//	 * @param ptr_extra_iteration_requirement_delta A margin that defines when the extra iterations are not needed or can be abandoned after
	//	 * the start.
	//	 * @see #getQuickStepDynamicIterationParameters(double[], double[], double[])
	//	 */
	//	//		ODE_API void dWorldSetQuickStepDynamicIterationParameters(dWorldID w, const dReal *ptr_iteration_premature_exit_delta/*=NULL*/,
	//	//		const dReal *ptr_max_num_extra_factor/*=NULL*/, const dReal *ptr_extra_iteration_requirement_delta/*=NULL*/);
	//	void setQuickStepDynamicIterationParameters(final double[] ptr_iteration_premature_exit_delta/*=NULL*/,
	//		final double[] ptr_max_num_extra_factor/*=NULL*/, final double[] ptr_extra_iteration_requirement_delta/*=NULL*/);


	//	/**
	//	 * Retrieve QuickStep method dynamic iteration count adjustment parameters.
	//	 *
	//	 * <p>REMARK: The function retrieves dynamic iteration count adjustment parameters.
	//	 *
	//	 * <p>See {@link #setQuickStepDynamicIterationParameters(double[], double[], double[])} for the parameters description.
	//	 *
	//	 * <p>At least one parameter must be not NULL for the call.
	//	 *
	//	 * @param out_iteration_premature_exit_delta Premature Exit Delta value (can be NULL if the value is not needed).
	//	 * @param out_max_num_extra_factor Maximum Extra Iteration Number Factor value (can be NULL if the value is not needed).
	//	 * @param out_extra_iteration_requirement_delta Extra Iteration Requirement Delta value (can be NULL if the value is not needed).
	//	 * @see #setQuickStepDynamicIterationParameters(double[], double[], double[])
	//	 */
	//	//		ODE_API void dWorldGetQuickStepDynamicIterationParameters(dWorldID w, dReal *out_iteration_premature_exit_delta/*=NULL*/,
	//	//																  dReal *out_max_num_extra_factor/*=NULL*/, dReal *out_extra_iteration_requirement_delta/*=NULL*/);
	//	void getQuickStepDynamicIterationParameters(double[] out_iteration_premature_exit_delta/*=NULL*/,
	//											    double[] out_max_num_extra_factor/*=NULL*/, double[] out_extra_iteration_requirement_delta/*=NULL*/);


	/**
	 * Statistics structure to accumulate QuickStep iteration couunt dynamic adjustment data.
	 *
//	 * @see #attachQuickStepDynamicIterationStatisticsSink
	 */
	class dWorldQuickStepIterationCount_DynamicAdjustmentStatistics
	{
		//		unsigned struct_size;         /*< to be initialized with the structure size */
		//
		//		duint32 iteration_count;      /*< number of iterations executed */
		//
		//		duint32 premature_exits;      /*< number of times solution took fewer than the regular iteration count */
		//		duint32 prolonged_execs;      /*< number of times solution took more  than the regular iteration count */
		//		duint32 full_extra_execs;     /*< number of times the assigned exit criteria were not achieved even after all extra iterations allowed */
		@Deprecated
		int struct_size;         /*< to be initialized with the structure size */

		public final AtomicInteger iteration_count = new AtomicInteger();      /*< number of iterations executed */

		public final AtomicInteger premature_exits = new AtomicInteger();      /*< number of times solution took fewer than the regular iteration count */
		public final AtomicInteger prolonged_execs = new AtomicInteger();      /*< number of times solution took more  than the regular iteration count */
		public final AtomicInteger full_extra_execs = new AtomicInteger();     /*< number of times the assigned exit criteria were not achieved even after all extra iterations allowed */

		public void set(dWorldQuickStepIterationCount_DynamicAdjustmentStatistics other) {
			struct_size = other.struct_size;
			iteration_count.set(other.iteration_count.get());
			premature_exits.set(other.premature_exits.get());
			prolonged_execs.set(other.prolonged_execs.get());
			full_extra_execs.set(other.full_extra_execs.get());
		}
	} // dWorldQuickStepIterationCount_DynamicAdjustmentStatistics;

	// ODE_PURE_INLINE
	//	void dWorldInitializeQuickStepIterationCount_DynamicAdjustmentStatistics(dWorldQuickStepIterationCount_DynamicAdjustmentStatistics *ptr_stat)
	//	{
	//		memset(ptr_stat, 0, sizeof(*ptr_stat));
	//		ptr_stat->struct_size = sizeof(*ptr_stat);
	//	}
	static void initializeQuickStepIterationCount_DynamicAdjustmentStatistics(dWorldQuickStepIterationCount_DynamicAdjustmentStatistics ptr_stat)
	{
		// memset(ptr_stat, 0, sizeof(*ptr_stat));
		ptr_stat.iteration_count.set(0);
		ptr_stat.premature_exits.set(0);
		ptr_stat.prolonged_execs.set(0);
		ptr_stat.full_extra_execs.set(0);
		ptr_stat.struct_size = 5*4 + 12;
	}


	//	/**
	//	 * Attach or remove a structure to collect QuickStep iteration count dynamic adjustment statistics.
	//	 *
	//	 * <p>REMARKS: The function can be used to attach or remove a structure instance that will be updated with iteration count dynamic adjustment statistics
	//	 * of QuickStep. To break the attachment, the function must be called with NULL for the {@code var_stats}.
	//	 *
	//	 * <p>See {@link #setQuickStepDynamicIterationParameters(double[], double[], double[])} for information on the iteration count dynamic adjustment options.
	//	 *
	//	 * <p>The caller is responsible for initializing the structure before assignment. The structure must persist in memory until unattached or
	//	 * the host world object is destroyed. The same structure instance may be shared among multiple worlds if that makes sense.
	//	 *
	//	 * <p>The assignment may fail if the feature is not configured within the library, or if the structure was not initialized properly.
	//	 *
	//	 * @param var_stats A pointer to structure instance to assigned or NULL to break the previous attachment for the world.
	//	 * @return Boolean status indicating whether the function succeeded
	//	 * @see dWorldQuickStepIterationCount_DynamicAdjustmentStatistics
	//	 */
	//	// ODE_API int dWorldAttachQuickStepDynamicIterationStatisticsSink(dWorldID w, dWorldQuickStepIterationCount_DynamicAdjustmentStatistics *var_stats/*=NULL*/);
	//	int attachQuickStepDynamicIterationStatisticsSink(dWorldQuickStepIterationCount_DynamicAdjustmentStatistics var_stats/*=NULL*/);



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
	 * Set auto disable linear average threshold for newly created bodies.
	 *
	 * @param threshold default is 0.01
	 */
	void setAutoDisableLinearThreshold(double threshold);


	/**
	 * Get auto disable linear average threshold for newly created bodies.
	 *
	 * @return the threshold
	 */
	double getAutoDisableLinearThreshold();


	/**
	 * Set auto disable angular average threshold for newly created bodies.
	 *
	 * @param threshold default is 0.01
	 */
	void setAutoDisableAngularThreshold(double threshold);


	/**
	 * Get auto disable angular average threshold for newly created bodies.
	 *
	 * @return the threshold
	 */
	double getAutoDisableAngularThreshold();


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
	 * @return maximum correcting velocity
	 */
	//ODE_API 
	double getContactMaxCorrectingVel();
	
	
	/**
	 * Destroy a world and everything in it.
	 *
	 * <p>This includes all bodies, and all joints that are not part of a joint
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
	 * @return threshold
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
	 * @return threshold
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
	 * @return damping
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
	 * @return damping
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
	 * @return max angular speed
	 * @see DBody#getMaxAngularSpeed()
	 */
	double getMaxAngularSpeed ();


	/**
	 * Set the default maximum angular speed for new bodies.
	 * @param max_speed max speed
	 * @see DBody#setMaxAngularSpeed(double)
	 */
	void setMaxAngularSpeed (double max_speed);

	void setTaskExecutor(TaskExecutor executor);
}
