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

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dNODEBUG;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.Common.dSqrt;
import static org.ode4j.ode.internal.Common.dUASSERT;
import static org.ode4j.ode.internal.ErrorHandler.dMessage;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.Objects_H.dxAutoDisable;
import org.ode4j.ode.internal.Objects_H.dxContactParameters;
import org.ode4j.ode.internal.Objects_H.dxDampingParameters;
import org.ode4j.ode.internal.Objects_H.dxQuickStepParameters;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefInt;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxIslandsProcessingCallContext;
import org.ode4j.ode.internal.processmem.DxStepWorkingMemory;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dmaxcallcountestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.internal.processmem.DxUtil.alloc_block_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.free_block_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.shrink_block_fn_t;
import org.ode4j.ode.internal.processmem.DxWorldProcessContext;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemoryManager;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemoryReserveInfo;
import org.ode4j.ode.threading.Threading;
import org.ode4j.ode.threading.task.SameThreadTaskExecutor;
import org.ode4j.ode.threading.task.Task;
import org.ode4j.ode.threading.task.TaskExecutor;
import org.ode4j.ode.threading.task.TaskGroup;

public class DxWorld extends DBase implements DWorld {

	private TaskExecutor taskExecutor = new SameThreadTaskExecutor();
	
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
	private double global_erp;		// global error reduction parameter
	double global_cfm;		// global constraint force mixing parameter
	dxAutoDisable adis;		// auto-disable parameters
	int body_flags;               // flags for new bodies
    private int islands_max_threads; // maximum threads to allocate for island processing
	public DxStepWorkingMemory wmem; // Working memory object for dWorldStep/dWorldQuickStep

	dxQuickStepParameters qs;
	public dxContactParameters contactp;
	dxDampingParameters dampingp; // damping parameters
	double max_angular_speed;      // limit the angular velocity to this magnitude

	private Object userdata;

    //dxWorld();
    //virtual ~dxWorld(); // Compilers emit warnings if a class with virtual methods does not have a virtual destructor :(

//    static boolean InitializeDefaultThreading();
//    static void FinalizeDefaultThreading();
//
//    void AssignThreadingImpl(dxThreadingFunctionsInfo [][]functions_info, dThreadingImplementationID threading_impl);
//    int GetThreadingIslandsMaxThreadsCount(int [][]out_active_thread_count_ptr=NULL);
//    dxWorldProcessContext[][] UnsafeGetWorldProcessingContext();
//
//    //private: // dxIThreadingDefaultImplProvider
//    dxThreadingFunctionsInfo RetrieveThreadingDefaultImpl(dThreadingImplementationID &out_default_impl);

    //****************************************************************************
	// world
	private DxWorld() {
		//private
		super();
		firstbody.set( null );
		firstjoint.set( null );
		nb = 0;
		nj = 0;
		global_erp = Objects_H.dWORLD_DEFAULT_GLOBAL_ERP;
		global_cfm = Objects_H.dWORLD_DEFAULT_GLOBAL_CFM;
		adis = null;
		body_flags = 0;  // everything disabled
		islands_max_threads = dWORLDSTEP_THREADCOUNT_UNLIMITED;
		wmem = null;
		qs = null;
		contactp = null;
		dampingp = null;
		max_angular_speed = dInfinity;
		userdata = 0;

	    //dSetZero (gravity, 4);
		gravity = new DVector3();
		
		adis = new dxAutoDisable();
		qs = new dxQuickStepParameters();
		contactp = new dxContactParameters();
		dampingp = new dxDampingParameters();
	}


	public static DxWorld dWorldCreate()
	{
		DxWorld w = new DxWorld();

		return w;
	}


	public void dWorldDestroy ()
	{
		// delete all bodies and joints
//		dAASSERT (w);
		DxBody nextb, b = firstbody.get();
		while (b != null) {
			nextb = (DxBody) b.getNext();
			b.dBodyDestroy(); // calling here dBodyDestroy for correct destroying! (i.e. the average buffers)
			b = nextb;
		}
		DxJoint nextj, j = firstjoint.get();
		while (j != null) {
			nextj = (DxJoint)j.getNext();
			if ( j.isFlagsInGroup() ) {
				// the joint is part of a group, so "deactivate" it instead
				j.world = null;
				j.node[0].body = null;
				j.node[0].next = null;
				j.node[1].body = null;
				j.node[1].next = null;
				dMessage (0,"warning: destroying world containing grouped joints");
			}
			else {
		        // TODO: shouldn't we call dJointDestroy()?
				//TZ size_t sz = j.size();
				j.DESTRUCTOR();
				//TZ dFree (j,sz);
			}
			j = nextj;
		}

//		delete w;
		DESTRUCTOR();
	}

	public void dWorldSetData (Object data)
	{
		userdata = data;
	}


	public Object dWorldGetData ()
	{
	    return userdata;
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


	private void dWorldGetGravity (DVector3 g)
	{
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


	private double dWorldGetERP ()
	{
		return global_erp;
	}


//	public void dWorldSetCFM (dxWorld w, double cfm)
	public void dWorldSetCFM (double cfm)
	{
		//dAASSERT (w);
		global_cfm = cfm;
	}


	private double dWorldGetCFM ()
	{
		return global_cfm;
	}

	void dWorldSetStepIslandsProcessingMaxThreadCount(int count)
	{
	    islands_max_threads = count;
	}

	int dWorldGetStepIslandsProcessingMaxThreadCount()
	{
	    return islands_max_threads;
	}

	//	/**
	//	 *
	//	 * @param policyinfo
	//	 * @return x
	//	 * @deprecated Not used anymore
	//	 */
	//	@Deprecated
	//	boolean dWorldSetStepMemoryReservationPolicy(final DWorldStepReserveInfo policyinfo)
	//	{
	//	    dUASSERT (policyinfo==null || (policyinfo.struct_size >= DxUtil.sizeof(policyinfo) && policyinfo.reserve_factor >= 1.0f), "Bad policy info");
	//
	//	    boolean result = false;
	//
	//	    //(TZ) DxStepWorkingMemory wmem = policyinfo!=null ? AllocateOnDemand(this.wmem) : this.wmem;
	//        DxStepWorkingMemory wmem;
	//        if (policyinfo!=null) {
	//            if (this.wmem == null) {
	//                this.wmem = new DxStepWorkingMemory();
	//            }
	//            wmem = this.wmem;
	//        } else {
	//            wmem = this.wmem;
	//        }
	//
	//	    if (wmem!=null)
	//	    {
	//	        if (policyinfo!=null)
	//	        {
	//	            wmem.SetMemoryReserveInfo(policyinfo.reserve_factor, policyinfo.reserve_minimum);
	//	            result = wmem.GetMemoryReserveInfo() != null;
	//	        }
	//	        else
	//	        {
	//	            wmem.ResetMemoryReserveInfoToDefault();
	//	            result = true;
	//	        }
	//	    }
	//	    else if (policyinfo==null)
	//	    {
	//	        result = true;
	//	    }
	//
	//	    return result;
	//	}


	boolean dWorldStep (double stepsize)
	{
	    dUASSERT (stepsize > 0,"stepsize must be > 0");

	    boolean result = false;

	    DxWorldProcessIslandsInfo islandsinfo = new DxWorldProcessIslandsInfo();
        if (DxWorldProcessContext.dxReallocateWorldProcessContext (this, islandsinfo, stepsize, 
        		Step.INSTANCE))//dxEstimateQuickStepMemoryRequirements))
        {
        	//if (dxProcessIslands (w, islandsinfo, stepsize, &dxStepIsland, &dxEstimateStepMaxCallCount))
            if (dxProcessIslands (islandsinfo, stepsize, Step.INSTANCE, Step.INSTANCE)) //dxEstimateStepMaxCallCount))
            {
                result = true;
            }
	    }
	    return result;
	}

	boolean dWorldQuickStep (double stepsize)
	{
	    dUASSERT (stepsize > 0,"stepsize must be > 0");

	    boolean result = false;

	    DxWorldProcessIslandsInfo islandsinfo = new DxWorldProcessIslandsInfo();
	    if (DxWorldProcessContext.dxReallocateWorldProcessContext (this, islandsinfo, stepsize, 
	            DxQuickStep.INSTANCE))//dxEstimateQuickStepMemoryRequirements))
	    {
	    	//if (dxProcessIslands (w, islandsinfo, stepsize, &dxQuickStepIsland, &dxEstimateQuickStepMaxCallCount))
	        if (dxProcessIslands (islandsinfo, stepsize, DxQuickStep.INSTANCE, Step.INSTANCE))
	        {
	        	result = true;
	        }
	    }
	    return result;
	}

	
	private void dWorldImpulseToForce (double stepsize,
			double ix, double iy, double iz,
			DVector3 force)
	{
		stepsize = dRecip(stepsize);
//		force.v[0] = stepsize * ix;
//		force.v[1] = stepsize * iy;
//		force.v[2] = stepsize * iz;
		force.set(stepsize * ix, stepsize * iy, stepsize * iz);
		// @@@ force[3] = 0;
	}


	// world auto-disable functions

	private double dWorldGetAutoDisableLinearThreshold ()
	{
		return dSqrt (adis.linear_average_threshold);
	}


	private void dWorldSetAutoDisableLinearThreshold (double linear_average_threshold)
	{
		adis.linear_average_threshold = linear_average_threshold * linear_average_threshold;
	}


	private double dWorldGetAutoDisableAngularThreshold ()
	{
		return dSqrt (adis.angular_average_threshold);
	}


	private void dWorldSetAutoDisableAngularThreshold (double angular_average_threshold)
	{
		adis.angular_average_threshold = angular_average_threshold * angular_average_threshold;
	}


	private int dWorldGetAutoDisableAverageSamplesCount ()
	{
		return adis.average_samples;
	}


	//	void dWorldSetAutoDisableAverageSamplesCount (dxWorld w, 
	//			unsigned int average_samples_count)
	public void dWorldSetAutoDisableAverageSamplesCount ( 
			int average_samples_count)
	{
		//dAASSERT(w);
		adis.average_samples = average_samples_count;
	}


	private int dWorldGetAutoDisableSteps ()
	{
		return adis.idle_steps;
	}


	private void dWorldSetAutoDisableSteps (int steps)
	{
		adis.idle_steps = steps;
	}


	private double dWorldGetAutoDisableTime ()
	{
		return adis.idle_time;
	}


	private void dWorldSetAutoDisableTime (double time)
	{
		adis.idle_time = time;
	}


	private boolean dWorldGetAutoDisableFlag ()
	{
		return (body_flags & DxBody.dxBodyAutoDisable) != 0;
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

	private double dWorldGetLinearDampingThreshold()
	{
		return dSqrt(dampingp.linear_threshold);
	}

	private void dWorldSetLinearDampingThreshold(double threshold)
	{
		dampingp.linear_threshold = threshold*threshold;
	}

	private double dWorldGetAngularDampingThreshold()
	{
		return dSqrt(dampingp.angular_threshold);
	}

	private void dWorldSetAngularDampingThreshold(double threshold)
	{
		dampingp.angular_threshold = threshold*threshold;
	}

	private double dWorldGetLinearDamping()
	{
		return dampingp.linear_scale;
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

	private void dWorldSetDamping(double linear_scale, double angular_scale)
	{
		dWorldSetLinearDamping(linear_scale);
		dWorldSetAngularDamping(angular_scale);
	}

	private double dWorldGetMaxAngularSpeed()
	{
		return max_angular_speed;
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


	private int dWorldGetQuickStepNumIterations ()
	{
		return qs.num_iterations;
	}


	private void dWorldSetQuickStepW (double param)
	{
		qs.w = param;
	}


	private double dWorldGetQuickStepW ()
	{
		return qs.w;
	}


//	void dWorldSetContactMaxCorrectingVel (dxWorld w, double vel)
	public void dWorldSetContactMaxCorrectingVel (double vel)
	{
		//dAASSERT(w);
		contactp.max_vel = vel;
	}


	private double dWorldGetContactMaxCorrectingVel ()
	{
		return contactp.max_vel;
	}


	//public void dWorldSetContactSurfaceLayer (dxWorld w, double depth)
	public void dWorldSetContactSurfaceLayer (double depth)
	{
		//dAASSERT(w);
		contactp.min_depth = depth;
	}


	private double dWorldGetContactSurfaceLayer ()
	{
		return contactp.min_depth;
	}

	//**** FROM util.cpp TZ

//	dxWorldProcessMemArena *dxAllocateTemporaryWorldProcessMemArena(
//	        size_t memreq, const dxWorldProcessMemoryManager *memmgr/*=NULL*/, const dxWorldProcessMemoryReserveInfo *reserveinfo/*=NULL*/);
//	      void dxFreeTemporaryWorldProcessMemArena(dxWorldProcessMemArena *arena);
	public static DxWorldProcessMemArena dxAllocateTemporaryWorldProcessMemArena(
	        int memreq, final DxWorldProcessMemoryManager memmgr/*=NULL*/, 
	        final DxWorldProcessMemoryReserveInfo reserveinfo/*=NULL*/) {       throw new UnsupportedOperationException();
	}
	public static void dxFreeTemporaryWorldProcessMemArena(
	        DxWorldProcessMemArena arena) {}



//	      template<class ClassType>
//	      inline ClassType *AllocateOnDemand(ClassType *&pctStorage)
	//(TZ) renamed, because this is not possible!
	public static <T> T AllocateOnDemandX(T pctStorage)
	{
	    throw new UnsupportedOperationException();
//	    T pctCurrentInstance = pctStorage;
//
//	    if (pctCurrentInstance == null)
//	    {
//	        //TODO TZ obviously this will fail...
//	        pctCurrentInstance = (T) new Object();
//	        pctStorage = pctCurrentInstance;
//	    }
//
//	    return pctCurrentInstance;
	}

	// This estimates dynamic memory requirements for dxProcessIslands
	public int EstimateIslandProcessingMemoryRequirements()
	{
	    //          int res = 0;
	    //
	    //          int islandcounts = dEFFICIENT_SIZE((size_t)(int)nb * 2 * sizeof(int));
	    //          res += islandcounts;
	    //
	    //          size_t bodiessize = dEFFICIENT_SIZE((size_t)(unsigned)world->nb * sizeof(dxBody*));
	    //          size_t jointssize = dEFFICIENT_SIZE((size_t)(unsigned)world->nj * sizeof(dxJoint*));
	    //          res += bodiessize + jointssize;
	    //
	    //          size_t sesize = (bodiessize < jointssize) ? bodiessize : jointssize;
	    //          res += sesize;
	    //
	    //          return res;
	    return -1;
	}

	
	static int EstimateIslandProcessingSimultaneousCallsMaximumCount(int activeThreadCount, 
			int islandsAllowedThreadCount, 
			int stepperAllowedThreadCount, dmaxcallcountestimate_fn_t maxCallCountEstimator)
	{
		int stepperCallsMaximum = maxCallCountEstimator.run(activeThreadCount, stepperAllowedThreadCount);
		int islandsIntermediateCallsMaximum = (1 + 2); // ThreadedProcessIslandSearch_Callback + (ThreadedProcessIslandStepper_Callback && ThreadedProcessIslandSearch_Callback)

		int result = 
				1 // ThreadedProcessGroup_Callback
				+ islandsAllowedThreadCount * DxUtil.dMAX(stepperCallsMaximum, islandsIntermediateCallsMaximum)
				+ DxUtil.dMIN(islandsAllowedThreadCount, (activeThreadCount - islandsAllowedThreadCount)) // ThreadedProcessJobStart_Callback
				/*...the end*/;
		return result;
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
	//bool dxProcessIslands (dxWorld *world, const dxWorldProcessIslandsInfo &islandsInfo, 
    //        dReal stepSize, dstepper_fn_t stepper, dmaxcallcountestimate_fn_t maxCallCountEstimator);
	boolean dxProcessIslands (final DxWorldProcessIslandsInfo islandsInfo, 
			double stepSize, dstepper_fn_t stepper, 
			dmaxcallcountestimate_fn_t maxCallCountEstimator)
	{

		final DxIslandsProcessingCallContext callContext = new DxIslandsProcessingCallContext(this, 
				islandsInfo, stepSize, stepper);

		//DxStepWorkingMemory wmem = world.wmem;
		dIASSERT(wmem != null);
		DxWorldProcessContext context = wmem.GetWorldProcessingContext(); 
		dIASSERT(context != null);

		RefInt summaryFault = new RefInt();

		RefInt activeThreadCount = new RefInt();
		final int islandsAllowedThreadCount = GetThreadingIslandsMaxThreadsCount(activeThreadCount);
		dIASSERT(islandsAllowedThreadCount != 0);
		dIASSERT(activeThreadCount.get() >= islandsAllowedThreadCount);

		// For now, set stepper allowed threads equal to island stepping threads
		int stepperAllowedThreadCount = islandsAllowedThreadCount; 

		callContext.SetStepperAllowedThreads(Threading.ENABLE_STEPPER_MULTITHREADING ? stepperAllowedThreadCount : 1);

		final TaskGroup group = taskExecutor.group("World Islands Stepping Group", new Runnable() {
			@Override
			public void run() {}
		});

		for (int i = 0; i < islandsAllowedThreadCount; i++) {
    		Task task = group.subtask("World Islands Stepping Start", new Runnable() {
				@Override
				public void run() {
					callContext.ThreadedProcessJobStart(group);
				}
			});
			task.submit();
    	}
		group.submit();
		// Wait until group completes (since jobs were the dependencies of the group the group 
		// is going to complete only after all the jobs end)
    	group.awaitCompletion();
	
		return summaryFault.get() == 0;
	}


	//****************************************************************************
	// Auto disabling

	public void dInternalHandleAutoDisabling (double stepsize)
	{
		DxBody bb;
		for ( bb=firstbody.get(); bb!=null; bb=(DxBody)bb.getNext() )
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
					double r1 = 1.0  / bb.adis.average_samples ;

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
				av_lspeed = dCalcVectorDot3( average_lvel, average_lvel );
				if ( av_lspeed > bb.adis.linear_average_threshold )
				{
					idle = false; // average linear velocity is too high for idle
				}
				else
				{
					av_aspeed = dCalcVectorDot3( average_avel, average_avel );
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
				bb.lvel.setZero();
//				bb.avel.v[0] = 0;
//				bb.avel.v[1] = 0;
//				bb.avel.v[2] = 0;
				bb.avel.setZero();
			}
		}
	}

	// ************************************************************
	// API dWorld
	// ************************************************************
	
	//~dWorld()
	@Override
	public void DESTRUCTOR()
//	{ dWorldDestroy (); super.DESTRUCTOR(); }
	{ 
		if (wmem != null)
		{
			wmem.CleanupWorldReferences(this);
			wmem = null;
		}
		super.DESTRUCTOR(); 
	}

	static boolean InitializeDefaultThreading()
	{
		return true;
	}

	static void FinalizeDefaultThreading()
	{
	}

	public int GetThreadingIslandsMaxThreadsCount(RefInt out_active_thread_count_ptr/*=NULL*/)
	{
	    int active_thread_count = taskExecutor.getThreadCount();
	    if (out_active_thread_count_ptr != null)
	    {
	        out_active_thread_count_ptr.set( active_thread_count );
	    }

	    return islands_max_threads == dWORLDSTEP_THREADCOUNT_UNLIMITED 
	        ? active_thread_count 
	        : (islands_max_threads < active_thread_count ? islands_max_threads : active_thread_count);
	}

	public DxWorldProcessContext UnsafeGetWorldProcessingContext()
	{
	    return wmem.GetWorldProcessingContext();
	}

	public void setTaskExecutor(TaskExecutor executor) {
		this.taskExecutor = executor;
	}
	@Override
	public void setGravity (double x, double y, double z)
	{ dWorldSetGravity (x,y,z); }
	@Override
	public void setGravity (DVector3C g)
	{ setGravity (g.get0(), g.get1(), g.get2()); }
	@Override
	public void getGravity (DVector3 g) 
	{ dWorldGetGravity (g); }

	@Override
	public void setERP (double erp)
	{ dWorldSetERP(erp); }
	@Override
	public double getERP() 
	{ return dWorldGetERP(); }

	@Override
	public void setCFM (double cfm)
	{ dWorldSetCFM(cfm); }
	@Override
	public double getCFM() 
	{ return dWorldGetCFM(); }

	@Override
	public void step (double stepsize)
	{ dWorldStep (stepsize); }

	@Override
	public boolean quickStep(double stepsize)
	{ return dWorldQuickStep (stepsize); }
	@Override
	public void setQuickStepNumIterations(int num)
	{ dWorldSetQuickStepNumIterations (num); }
	@Override
	public int getQuickStepNumIterations() 
	{ return dWorldGetQuickStepNumIterations (); }
	@Override
	public void setQuickStepW(double over_relaxation)
	{ dWorldSetQuickStepW (over_relaxation); }
	@Override
	public double getQuickStepW() 
	{ return dWorldGetQuickStepW (); }

	@Override
	public void  setAutoDisableLinearThreshold (double threshold) 
	{ dWorldSetAutoDisableLinearThreshold (threshold); }
	@Override
	public double getAutoDisableLinearThreshold()
	{ return dWorldGetAutoDisableLinearThreshold (); }
	@Override
	public void setAutoDisableAngularThreshold (double threshold)
	{ dWorldSetAutoDisableAngularThreshold (threshold); }
	@Override
	public double getAutoDisableAngularThreshold()
	{ return dWorldGetAutoDisableAngularThreshold (); }
	@Override
	public void setAutoDisableSteps (int steps)
	{ dWorldSetAutoDisableSteps (steps); }
	@Override
	public int getAutoDisableSteps()
	{ return dWorldGetAutoDisableSteps (); }
	@Override
	public void setAutoDisableTime (double time)
	{ dWorldSetAutoDisableTime (time); }
	@Override
	public double getAutoDisableTime() 
	{ return dWorldGetAutoDisableTime (); }
	@Override
	public void setAutoDisableFlag (boolean do_auto_disable)
	{ dWorldSetAutoDisableFlag (do_auto_disable); }
	@Override
	public boolean getAutoDisableFlag() 
	{ return dWorldGetAutoDisableFlag (); }

	@Override
	public double getLinearDampingThreshold() 
	{ return dWorldGetLinearDampingThreshold(); }
	@Override
	public void setLinearDampingThreshold(double threshold)
	{ dWorldSetLinearDampingThreshold(threshold); }
	@Override
	public double getAngularDampingThreshold() 
	{ return dWorldGetAngularDampingThreshold(); }
	@Override
	public void setAngularDampingThreshold(double threshold)
	{ dWorldSetAngularDampingThreshold(threshold); }
	@Override
	public double getLinearDamping() 
	{ return dWorldGetLinearDamping(); }
	@Override
	public void setLinearDamping(double scale)
	{ dWorldSetLinearDamping(scale); }
	@Override
	public double getAngularDamping() 
	{ return dWorldGetAngularDamping(); }
	@Override
	public void setAngularDamping(double scale)
	{ dWorldSetAngularDamping(scale); }
	@Override
	public void setDamping(double linear_scale, double angular_scale)
	{ dWorldSetDamping(linear_scale, angular_scale); }

	@Override
	public 	double getMaxAngularSpeed() 
	{ return dWorldGetMaxAngularSpeed(); }
	@Override
	public void setMaxAngularSpeed(double max_speed)
	{ dWorldSetMaxAngularSpeed(max_speed); }

	@Override
	public void setContactSurfaceLayer(double depth)
	{ dWorldSetContactSurfaceLayer (depth); }
	@Override
	public double getContactSurfaceLayer() 
	{ return dWorldGetContactSurfaceLayer (); }

	@Override
	public void impulseToForce (double stepsize, double ix, double iy, double iz,
			DVector3 force)
	{ dWorldImpulseToForce (stepsize,ix,iy,iz,force); }


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


//	@Override
//	public double getAutoDisableAngularAverageThreshold() {
//		throw new UnsupportedOperationException("Not implemented in ODE.");
//	}


	@Override
	public int getAutoDisableAverageSamplesCount() {
		return dWorldGetAutoDisableAverageSamplesCount();
	}


//	@Override
//	public double getAutoDisableLinearAverageThreshold() {
//		throw new UnsupportedOperationException("Not implemented in ODE.");
//	}


	@Override
	public double getContactMaxCorrectingVel() {
		return dWorldGetContactMaxCorrectingVel();
	}


//	@Override
//	public void setAutoDisableAngularAverageThreshold(
//			double angularAverageThreshold) {
//		throw new UnsupportedOperationException("Not implemented in ODE.");
//	}
//
//
//	@Override
//	public void setAutoDisableLinearAverageThreshold(
//			double linearAverageThreshold) {
//		throw new UnsupportedOperationException("Not implemented in ODE.");
//	}


	@Override
	public void setData(Object data) {
		dWorldSetData(data);
	}


	@Override
	public Object getData() {
		return dWorldGetData();
	}


	@Override
	@Deprecated
	public void setStepIslandsProcessingMaxThreadCount(int count) {
		dWorldSetStepIslandsProcessingMaxThreadCount(count);
	}


	@Override
	@Deprecated
	public int getStepIslandsProcessingMaxThreadCount() {
		return dWorldGetStepIslandsProcessingMaxThreadCount();
	}


//	@Override
//	public void dWorldSetStepThreadingImplementation(
//			DThreadingFunctionsInfo functions_info,
//			DThreadingImplementation threading_impl) {
//		
//	}


	//Moved from DWorld (TZ)
	
	/**
	* World stepping memory manager descriptor structure
	*
	* This structure is intended to define the functions of memory manager to be used
	* with world stepping functions.
	*
	* <code>struct_size</code> should be assigned the size of the structure
	*
	* <code>alloc_block</code> is a function to allocate memory block of given size.
	*
	* <code>shrink_block</code> is a function to shrink existing memory block to a smaller size.
	* It must preserve the contents of block head while shrinking. The new block size
	* is guaranteed to be always less than the existing one.
	*
	* <code>free_block</code> is a function to delete existing memory block.
	*
	* @deprecated Do not use ! (TZ)
	*/
	@Deprecated
    public static class DWorldStepMemoryFunctionsInfo
	{
	    // public int struct_size;
	    //TODO, already in DxUtil (TZ) -> Should not be public in Java.
	    //	  void *(*alloc_block)(size_t block_size);
	    public static final DxUtil.alloc_block_fn_t alloc_block = new alloc_block_fn_t() {
			@Override
			public BlockPointer run(int block_size) {
				return new BlockPointer(new DxWorldProcessMemArena(), 0);
			}
		};
	    //	  void *(*shrink_block)(void *block_pointer, size_t block_current_size, size_t block_smaller_size);
	    public static final DxUtil.shrink_block_fn_t shrink_block = new shrink_block_fn_t() {
			@Override
			public BlockPointer run(BlockPointer block_pointer, int block_current_size,
					int block_smaller_size) {
				//block_pointer.setSize(block_smaller_size);
				return block_pointer;
			}
		};
	    //	  void (*free_block)(void *block_pointer, size_t block_current_size);
	    public static final DxUtil.free_block_fn_t free_block = new free_block_fn_t() {
			@Override
			public void run(BlockPointer block_pointer, int block_current_size) {
				// Nothing
			}
		};

		private DWorldStepMemoryFunctionsInfo() {}
	}
}
