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
package org.ode4j.ode.threading;

import static org.ode4j.ode.internal.Common.dAASSERT;

import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefInt;
import org.ode4j.ode.internal.processmem.DxWorldProcessContext.dxProcessContextMutex;
import org.ode4j.ode.threading.ThreadingImpl_H.dxSelfThreadedThreading;
import org.ode4j.ode.threading.ThreadingTemplates.dIMutexGroup;
import org.ode4j.ode.threading.ThreadingTemplates.dxICallWait;
import org.ode4j.ode.threading.ThreadingTemplates.dxIThreadingImplementation;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;
import org.ode4j.ode.threading.Threading_H.DCallWait;
import org.ode4j.ode.threading.Threading_H.DMutexGroup;
import org.ode4j.ode.threading.Threading_H.DThreadedWaitTime;
import org.ode4j.ode.threading.Threading_H.DThreadingFunctionsInfo;
import org.ode4j.ode.threading.Threading_H.DxThreadingFunctionsInfo;
import org.ode4j.ode.threading.Threading_H.dMutexGroupAllocFunction;
import org.ode4j.ode.threading.Threading_H.dMutexGroupFreeFunction;
import org.ode4j.ode.threading.Threading_H.dMutexGroupMutexLockFunction;
import org.ode4j.ode.threading.Threading_H.dMutexGroupMutexUnlockFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallDependenciesCountAlterFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallPostFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallWaitAllocFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallWaitFreeFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallWaitFunction;
import org.ode4j.ode.threading.Threading_H.dThreadedCallWaitResetFunction;
import org.ode4j.ode.threading.Threading_H.dThreadingImplResourcesForCallsPreallocateFunction;
import org.ode4j.ode.threading.Threading_H.dThreadingImplThreadCountRetrieveFunction;

/**
 * 
 * @author Tilmann Zaeschke
 * @Deprecated Not supported in ode4j.
 */
public abstract class DxThreadingImplementation extends DThreadingImplementation {

	private static final boolean dBUILTIN_THREADING_IMPL_ENABLED = false;
	
	/*
	 *  Threading implementation header for library private functions.
	 */
	// This function has been removed from public headers as there is no need for it
	// to be accessible to outer code at this time. In future it is possible 
	// it could be published back again.
	/**
	 * Allocates built-in self-threaded threading implementation object.
	 *
	 * A self-threaded implementation is a type of implementation that performs 
	 * processing of posted calls by means of caller thread itself. This type of 
	 * implementation does not need thread pool to serve it.
	 * 
	 * The processing is arranged in a way to prevent call stack depth growth 
	 * as more and more nested calls are posted.
	 *
	 * @return ID of object allocated or NULL on failure
	 * 
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation()
	 * @see DThreadingImplementation#free()
	 */
	/*ODE_API */
//	public static DThreadingImplementation dThreadingAllocateSelfThreadedImplementation() {
//		throw new UnsupportedOperationException();
//	}

//	static void PostThreadedCall(
//		    dThreadingImplementationID impl, int *out_summary_fault/*=NULL*/, 
//		    dCallReleaseeID *out_post_releasee/*=NULL*/, ddependencycount_t dependencies_count, dCallReleaseeID dependent_releasee/*=NULL*/, 
//		    dCallWaitID call_wait/*=NULL*/, 
//		    dThreadedCallFunction *call_func, void *call_context, dcallindex_t instance_index, 
//		    const char *call_name/*=NULL*/);
//		static void AlterThreadedCallDependenciesCount(
//		    dThreadingImplementationID impl, dCallReleaseeID target_releasee, 
//		    ddependencychange_t dependencies_count_change);
//		static void WaitThreadedCall(
//		    dThreadingImplementationID impl, int *out_wait_status/*=NULL*/, 
//		    dCallWaitID call_wait, const dThreadedWaitTime *timeout_time_ptr/*=NULL*/, 
//		    const char *wait_name/*=NULL*/);
//
//		static unsigned RetrieveThreadingThreadCount(dThreadingImplementationID impl);
//		static int PreallocateResourcesForThreadedCalls(dThreadingImplementationID impl, ddependencycount_t max_simultaneous_calls_estimate);



	/*extern */public static DThreadingImplementation dThreadingAllocateSelfThreadedImplementation()
	{
		dxSelfThreadedThreading threading = new dxSelfThreadedThreading();

		if (threading != null && !threading.InitializeObject())
		{
			//delete threading;
			threading.DESTRUCTOR();
			threading = null;
		}

		dxIThreadingImplementation impl = threading;
		return impl;
	}

	/*extern */public static DThreadingImplementation dThreadingAllocateMultiThreadedImplementation()
	{
		throw new UnsupportedOperationException();
//		dxIThreadingImplementation threading = null;
//		if (dBUILTIN_THREADING_IMPL_ENABLED) {
//			//dxMultiThreadedThreading 
//			threading = new dxMultiThreadedThreading();
//
//			if (threading != null && !threading.InitializeObject())
//			{
//				//delete threading;
//				threading.DESTRUCTOR();
//				threading = null;
//			}
//			//			} else {
//			//				dxIThreadingImplementation threading = null;
//		}//endif // #if dBUILTIN_THREADING_IMPL_ENABLED
//
//		dxIThreadingImplementation impl = threading;
//		return (DThreadingImplementation)impl;
	}

	@Override
	/*extern */public DThreadingFunctionsInfo dThreadingImplementationGetFunctions()
	{
		DThreadingImplementation impl = this;
		if (dBUILTIN_THREADING_IMPL_ENABLED) {
			dAASSERT(impl != null);
		}//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED

		DThreadingFunctionsInfo functions = null;

		if (!dBUILTIN_THREADING_IMPL_ENABLED) {
			if (impl != null) {
				functions = g_builtin_threading_functions;
			}
		}//#endif // #if !dBUILTIN_THREADING_IMPL_ENABLED
		else {
			functions = g_builtin_threading_functions;
		}

		return functions;
	}

	@Override
	/*extern */void dThreadingImplementationShutdownProcessing()
	{
		DThreadingImplementation impl = this;
		if (dBUILTIN_THREADING_IMPL_ENABLED) {
			dAASSERT(impl != null);
		}//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED

		if (!dBUILTIN_THREADING_IMPL_ENABLED) {
			if (impl != null) {
				((dxIThreadingImplementation)impl).ShutdownProcessing();
			}
		}//#endif // #if !dBUILTIN_THREADING_IMPL_ENABLED
		else
		{
			((dxIThreadingImplementation)impl).ShutdownProcessing();
		}
	}

	@Override
	/*extern */public void dThreadingImplementationCleanupForRestart()
	{
		DThreadingImplementation impl = this;
		if (dBUILTIN_THREADING_IMPL_ENABLED) {
			dAASSERT(impl != null);
		}//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED

		if (!dBUILTIN_THREADING_IMPL_ENABLED) {
			if (impl != null) {
				((dxIThreadingImplementation)impl).CleanupForRestart();
			}
		}//#endif // #if !dBUILTIN_THREADING_IMPL_ENABLED
		else {
			((dxIThreadingImplementation)impl).CleanupForRestart();
		}
	}

	@Override
	/*extern */public void free()
	{
		DThreadingImplementation impl = this;
		if (impl != null)
		{
			((dxIThreadingImplementation)impl).FreeInstance();
		}
	}

	@Override
//	/*extern */void dExternalThreadingServeMultiThreadedImplementation(DThreadingImplementation impl, 
//			DThreadReadyToServeCallback readiness_callback/*=NULL*/, Object[] callback_context/*=NULL*/)
	public void dExternalThreadingServeMultiThreadedImplementation( 
			DThreadReadyToServeCallback readiness_callback/*=NULL*/, CallContext callback_context/*=NULL*/)
	{
		DThreadingImplementation impl = this;
		if (dBUILTIN_THREADING_IMPL_ENABLED) {
			dAASSERT(impl != null);
		}//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED

		if (!dBUILTIN_THREADING_IMPL_ENABLED) {
			//if (impl != null) {
				((dxIThreadingImplementation)impl).StickToJobsProcessing(readiness_callback, callback_context);
			//}
		}//#endif // #if !dBUILTIN_THREADING_IMPL_ENABLED
		else {
			((dxIThreadingImplementation)impl).StickToJobsProcessing(readiness_callback, callback_context);
		}
	}


	//////////////////////////////////////////////////////////////////////////

	private static final dMutexGroupAllocFunction
	//static DMutexGroup 
	AllocMutexGroup = new dMutexGroupAllocFunction() {
		@Override
		public DMutexGroup run(DThreadingImplementation impl,
				dxProcessContextMutex Mutex_count, String[] Mutex_names_ptr) {
			//(void)Mutex_names_ptr; // unused
			dIMutexGroup mutex_group = ((dxIThreadingImplementation)impl).AllocMutexGroup(Mutex_count);
			return (DMutexGroup)mutex_group;
		}
	};

	private static final dMutexGroupFreeFunction
	//static void 
	FreeMutexGroup = new dMutexGroupFreeFunction() {
		@Override
		public void run(DThreadingImplementation impl, DMutexGroup mutex_group) {
			((dxIThreadingImplementation)impl).FreeMutexGroup((dIMutexGroup)mutex_group);
		}
	};

	private static final
	dMutexGroupMutexLockFunction LockMutexGroupMutex = new dMutexGroupMutexLockFunction() {
		@Override
		public void run(DThreadingImplementation impl, DMutexGroup mutex_group,
				dxProcessContextMutex mutex_index) {
			((dxIThreadingImplementation)impl).LockMutexGroupMutex((dIMutexGroup)mutex_group, mutex_index);
		}
	};

	// static int TryLockMutexGroupMutex(dThreadingImplementationID impl, dMutexGroupID mutex_group, dmutexindex_t mutex_index)
	// {
	//   bool trylock_result = ((dxIThreadingImplementation *)impl).TryLockMutexGroupMutex((dIMutexGroup *)mutex_group, mutex_index);
	//   return trylock_result;
	// }

	private final static
	dMutexGroupMutexUnlockFunction UnlockMutexGroupMutex = new dMutexGroupMutexUnlockFunction() {
		@Override
		public void run(DThreadingImplementation impl, DMutexGroup mutex_group,
				dxProcessContextMutex mutex_index) {
			((dxIThreadingImplementation)impl).UnlockMutexGroupMutex((dIMutexGroup)mutex_group, mutex_index);
		}
	};

	private static final 
	dThreadedCallWaitAllocFunction AllocThreadedCallWait = new dThreadedCallWaitAllocFunction() {
		@Override
		public DCallWait run(DThreadingImplementation impl) {
			dxICallWait call_wait = ((dxIThreadingImplementation)impl).AllocACallWait();
			return call_wait;
		}
	};

	private static final 
	dThreadedCallWaitResetFunction ResetThreadedCallWait = new dThreadedCallWaitResetFunction() {
		@Override
		public void run(DThreadingImplementation impl, DCallWait call_wait) {
			((dxIThreadingImplementation)impl).ResetACallWait((dxICallWait)call_wait);
		}
	};

	private static final
	dThreadedCallWaitFreeFunction FreeThreadedCallWait = new dThreadedCallWaitFreeFunction() {
		@Override
		public void run(DThreadingImplementation impl, DCallWait call_wait) {
			((dxIThreadingImplementation)impl).FreeACallWait((dxICallWait)call_wait);
		}
	};


	private static final 
	dThreadedCallPostFunction PostThreadedCall = new dThreadedCallPostFunction() {
		@Override
		public void run(DThreadingImplementation impl, RefInt out_summary_fault,
				Ref<DCallReleasee> out_post_releasee, int dependencies_count,
				DCallReleasee dependent_releasee, DCallWait call_wait,
				dThreadedCallFunction call_func, CallContext call_context,
				int instance_index, String call_name) {
			//(void)call_name; // unused
			((dxIThreadingImplementation)impl).ScheduleNewJob(out_summary_fault, out_post_releasee, 
					dependencies_count, dependent_releasee, (dxICallWait)call_wait, call_func, call_context, instance_index);
		}
	};

	private static final 
	dThreadedCallDependenciesCountAlterFunction AlterThreadedCallDependenciesCount = new dThreadedCallDependenciesCountAlterFunction() {
		@Override
		public void run(DThreadingImplementation impl,
				DCallReleasee target_releasee, int dependencies_count_change) {
			((dxIThreadingImplementation)impl).AlterJobDependenciesCount(target_releasee, dependencies_count_change);
		}
	};

	private static final
	dThreadedCallWaitFunction WaitThreadedCall = new dThreadedCallWaitFunction() {
		@Override
		public void run(DThreadingImplementation impl, RefInt out_wait_status,
				DCallWait call_wait, DThreadedWaitTime timeout_time_ptr,
				String wait_name) {
			//(void)wait_name; // unused
			((dxIThreadingImplementation)impl).WaitJobCompletion(out_wait_status, (dxICallWait)call_wait, timeout_time_ptr);
		}
	};


	private static final
	dThreadingImplThreadCountRetrieveFunction RetrieveThreadingThreadCount = new dThreadingImplThreadCountRetrieveFunction() {
		@Override
		public int run(DThreadingImplementation impl) {
			return ((dxIThreadingImplementation)impl).RetrieveActiveThreadsCount();
		}
	};

	private static final 
	dThreadingImplResourcesForCallsPreallocateFunction PreallocateResourcesForThreadedCalls = 
	new dThreadingImplResourcesForCallsPreallocateFunction() {
		@Override
		public boolean run(DThreadingImplementation impl,
				int max_simultaneous_calls_estimate) {
			return ((dxIThreadingImplementation)impl).PreallocateJobInfos(max_simultaneous_calls_estimate);
		}
	};

	static DxThreadingFunctionsInfo g_builtin_threading_functions =
			new DxThreadingFunctionsInfo(
					//sizeof(dxThreadingFunctionsInfo), // unsigned struct_size;

					AllocMutexGroup, // dMutexGroupAllocFunction *alloc_mutex_group;
					FreeMutexGroup, // dMutexGroupFreeFunction *free_mutex_group;
					LockMutexGroupMutex, // dMutexGroupMutexLockFunction *lock_group_mutex;
					UnlockMutexGroupMutex, // dMutexGroupMutexUnlockFunction *unlock_group_mutex;

					AllocThreadedCallWait, // dThreadedCallWaitAllocFunction *alloc_call_wait;
					ResetThreadedCallWait, // dThreadedCallWaitResetFunction *reset_call_wait;
					FreeThreadedCallWait, // dThreadedCallWaitFreeFunction *free_call_wait;

					PostThreadedCall, // dThreadedCallPostFunction *post_call;
					AlterThreadedCallDependenciesCount, // dThreadedCallDependenciesCountAlterFunction *alter_call_dependencies_count;
					WaitThreadedCall, // dThreadedCallWaitFunction *wait_call;

					RetrieveThreadingThreadCount, // dThreadingImplThreadCountRetrieveFunction *retrieve_thread_count;
					PreallocateResourcesForThreadedCalls // dThreadingImplResourcesForCallsPreallocateFunction *preallocate_resources_for_calls;

					// &TryLockMutexGroupMutex, // dMutexGroupMutexTryLockFunction *trylock_group_mutex;
					);


}
