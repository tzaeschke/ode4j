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

import org.ode4j.ode.internal.Common;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefInt;
import org.ode4j.ode.internal.processmem.DxWorldProcessContext.dxProcessContextMutex;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;
import org.ode4j.ode.threading.Threading_H.DCallWait;
import org.ode4j.ode.threading.Threading_H.DMutexGroup;
import org.ode4j.ode.threading.Threading_H.DThreadedWaitTime;
import org.ode4j.ode.threading.Threading_H.DxThreadingFunctionsInfo;
import org.ode4j.ode.threading.Threading_H.dThreadedCallFunction;

public class DxThreadingBase {
	
	public interface DxIThreadingDefaultImplProvider
	{
	//public:
	    //virtual const dxThreadingFunctionsInfo *RetrieveThreadingDefaultImpl(dThreadingImplementationID &out_default_impl) = 0;
		DxThreadingFunctionsInfo RetrieveThreadingDefaultImpl(Ref<DThreadingImplementation> out_default_impl);
	};


//	class dxThreadingBase
//	{
		//protected:
		public DxThreadingBase() {
			m_default_impl_provider = null;//(NULL),
			m_functions_info = null;//(NULL), 
			m_threading_impl = null;//(NULL)
		}

		// This ought to be done via constructor, but passing 'this' in base class initializer emits a warning in MSVC :(
		public void SetThreadingDefaultImplProvider(DxIThreadingDefaultImplProvider default_impl_provider) { 
			m_default_impl_provider = default_impl_provider; 
		}

		//public:
	    //public 
	    public void AssignThreadingImpl(final DxThreadingFunctionsInfo functions_info, 
	    		DThreadingImplementation threading_impl)
	    {
	        Common.dAASSERT((functions_info == null) == (threading_impl == null));

	        m_functions_info = functions_info;
	        m_threading_impl = threading_impl;
	    }

	    //public:
	    //public 
	    public DMutexGroup AllocMutexGroup(dxProcessContextMutex /*dmutexindex_t*/ Mutex_count, String[] Mutex_names_ptr/*=NULL*/)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        return functions.alloc_mutex_group.run(impl.get(), Mutex_count, Mutex_names_ptr);
	    }

	    //public 
	    public void FreeMutexGroup(DMutexGroup mutex_group) 
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.free_mutex_group.run(impl.get(), mutex_group);
	    }

	    //public 
	    public void LockMutexGroupMutex(DMutexGroup mutex_group, 
	    		//int /*dmutexindex_t*/ mutex_index)
	    		dxProcessContextMutex mutex_index)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.lock_group_mutex.run(impl.get(), mutex_group, mutex_index);
	    }

//	     bool TryLockMutexGroupMutex(dMutexGroupID mutex_group, dmutexindex_t mutex_index) const
//	     {
//	         dThreadingImplementationID impl;
//	         const dxThreadingFunctionsInfo *functions = FindThreadingImpl(impl);
//	         return functions->trylock_group_mutex(impl, mutex_group, mutex_index) != 0;
//	     }

	    public void UnlockMutexGroupMutex(DMutexGroup mutex_group, 
	    		//int /*dmutexindex_t*/ mutex_index)
	    		dxProcessContextMutex mutex_index)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.unlock_group_mutex.run(impl.get(), mutex_group, mutex_index);
	    }

	    public DCallWait AllocThreadedCallWait() 
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        return functions.alloc_call_wait.run(impl.get());
	    }

	    void ResetThreadedCallWait(DCallWait call_wait)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.reset_call_wait.run(impl.get(), call_wait);
	    }

	    public void FreeThreadedCallWait(DCallWait call_wait)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.free_call_wait.run(impl.get(), call_wait);
	    }

//	    void PostThreadedCall(int *out_summary_fault/*=NULL*/, 
//	        dCallReleaseeID *out_post_releasee/*=NULL*/, 
//	        ddependencycount_t dependencies_count, 
//	        dCallReleaseeID dependent_releasee/*=NULL*/, 
//	        dCallWaitID call_wait/*=NULL*/, 
//	        dThreadedCallFunction *call_func, void *call_context, dcallindex_t instance_index, 
//	        const char *call_name/*=NULL*/) const
	    public void PostThreadedCall(RefInt out_summary_fault/*=NULL*/, 
		        Ref<DCallReleasee> out_post_releasee/*=NULL*/, 
		        int /*ddependencycount_t*/ dependencies_count, 
		        DCallReleasee dependent_releasee/*=NULL*/, 
		        DCallWait call_wait/*=NULL*/, 
		        dThreadedCallFunction call_func, 
		        CallContext call_context, 
		        int /*dcallindex_t*/ instance_index, 
		        String call_name/*=NULL*/)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.post_call.run(impl.get(), out_summary_fault, out_post_releasee, dependencies_count, 
	        		dependent_releasee, call_wait, call_func, call_context, instance_index, 
	        		call_name);
	    }

	    public void AlterThreadedCallDependenciesCount(DCallReleasee target_releasee, 
	        int /*ddependencychange_t*/ dependencies_count_change) 
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.alter_call_dependencies_count.run(impl.get(), target_releasee, dependencies_count_change);
	    }

//	    void WaitThreadedCallExclusively(int *out_wait_status/*=NULL*/, 
//	        dCallWaitID call_wait, const dThreadedWaitTime *timeout_time_ptr/*=NULL*/, 
//	        const char *wait_name/*=NULL*/) 
	    public void WaitThreadedCallExclusively(RefInt out_wait_status/*=NULL*/, 
		        DCallWait call_wait, DThreadedWaitTime timeout_time_ptr/*=NULL*/, 
		        String wait_name/*=NULL*/) 
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.wait_call.run(impl.get(), out_wait_status, call_wait, timeout_time_ptr, wait_name);
	        functions.reset_call_wait.run(impl.get(), call_wait);
	    }

//	    void WaitThreadedCallCollectively(int *out_wait_status/*=NULL*/, 
//	        dCallWaitID call_wait, const dThreadedWaitTime *timeout_time_ptr/*=NULL*/, 
//	        const char *wait_name/*=NULL*/) 
	    void WaitThreadedCallCollectively(RefInt out_wait_status/*=NULL*/, 
		        DCallWait call_wait, DThreadedWaitTime timeout_time_ptr/*=NULL*/, 
		        String wait_name/*=NULL*/) 
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        functions.wait_call.run(impl.get(), out_wait_status, call_wait, timeout_time_ptr, wait_name);
	    }

	    public int RetrieveThreadingThreadCount() 
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        return functions.retrieve_thread_count.run(impl.get());
	    }

	    public boolean PreallocateResourcesForThreadedCalls(int max_simultaneous_calls_estimate)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	        DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);
	        return functions.preallocate_resources_for_calls.run(impl.get(), max_simultaneous_calls_estimate);// != 0;
	    }

	//public:
//	    void PostThreadedCallsGroup(int *out_summary_fault/*=NULL*/, 
//	        ddependencycount_t member_count, dCallReleaseeID dependent_releasee/*=NULL*/, 
//	        dThreadedCallFunction *call_func, void *call_context, 
//	        const char *call_name/*=NULL*/) const;
//	    void PostThreadedCallForUnawareReleasee(int *out_summary_fault/*=NULL*/, 
//	        dCallReleaseeID *out_post_releasee/*=NULL*/, ddependencycount_t dependencies_count, dCallReleaseeID dependent_releasee/*=NULL*/, 
//	        dCallWaitID call_wait/*=NULL*/, 
//	        dThreadedCallFunction *call_func, void *call_context, dcallindex_t instance_index, 
//	        const char *call_name/*=NULL*/) const;

	//protected:
	    //const dxThreadingFunctionsInfo *FindThreadingImpl(dThreadingImplementationID &out_impl_found) const;

	//private:
	    //const dxThreadingFunctionsInfo *GetFunctionsInfo() const { return m_functions_info; }
	    DxThreadingFunctionsInfo GetFunctionsInfo() { return m_functions_info; }
	    //dThreadingImplementationID GetThreadingImpl() const { return m_threading_impl; }
	    DThreadingImplementation GetThreadingImpl() { return m_threading_impl; }

	//private:
	    private DxIThreadingDefaultImplProvider  	m_default_impl_provider;
	    private DxThreadingFunctionsInfo    		m_functions_info;
	    private DThreadingImplementation        	m_threading_impl;
	    
	    
	    
//	    void PostThreadedCallsGroup(
//	    	    int *out_summary_fault/*=NULL*/, 
//	    	    ddependencycount_t member_count, dCallReleaseeID dependent_releasee/*=NULL*/, 
//	    	    dThreadedCallFunction *call_func, void *call_context, 
//	    	    const char *call_name/*=NULL*/) const
	    public void PostThreadedCallsGroup(
	    		RefInt out_summary_fault/*=NULL*/, 
	    		int /*ddependencycount_t*/ member_count, 
	    		DCallReleasee dependent_releasee/*=NULL*/, 
	    		dThreadedCallFunction call_func, CallContext call_context, 
	    		String call_name/*=NULL*/)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	    	DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);

	    	for (int member_index = 0; member_index != member_count; ++member_index) {
	    		// Post individual group member jobs
	    		functions.post_call.run(impl.get(), out_summary_fault, null, 0, dependent_releasee, null, 
	    				call_func, call_context, member_index, call_name);
	    	}
	    }

//	    	void dxThreadingBase::PostThreadedCallForUnawareReleasee(
//	    	    int *out_summary_fault/*=NULL*/, 
//	    	    dCallReleaseeID *out_post_releasee/*=NULL*/, ddependencycount_t dependencies_count, 
//	    		dCallReleaseeID dependent_releasee/*=NULL*/, 
//	    	    dCallWaitID call_wait/*=NULL*/, 
//	    	    dThreadedCallFunction *call_func, void *call_context, dcallindex_t instance_index, 
//	    	    const char *call_name/*=NULL*/) const
	    public void PostThreadedCallForUnawareReleasee(
	    		RefInt out_summary_fault/*=NULL*/, 
	    		Ref<DCallReleasee> out_post_releasee/*=NULL*/, 
	    		int /*ddependencycount_t*/ dependencies_count, 
	    		DCallReleasee dependent_releasee/*=NULL*/, 
	    		DCallWait call_wait/*=NULL*/, 
	    		dThreadedCallFunction call_func, 
	    		CallContext call_context, 
	    		int /*dcallindex_t*/ instance_index, 
	    		String call_name/*=NULL*/)
	    {
	        Ref<DThreadingImplementation> impl = new Ref<DThreadingImplementation>();
	    	DxThreadingFunctionsInfo functions = FindThreadingImpl(impl);

	    	functions.alter_call_dependencies_count.run(impl.get(), dependent_releasee, 1);
	    	functions.post_call.run(impl.get(), out_summary_fault, out_post_releasee, dependencies_count, dependent_releasee, call_wait, call_func, call_context, instance_index, call_name);
	    }

//	    	const dxThreadingFunctionsInfo *dxThreadingBase::FindThreadingImpl(dThreadingImplementationID &out_impl_found) const
	    DxThreadingFunctionsInfo FindThreadingImpl(Ref<DThreadingImplementation> out_impl_found)
	    {
	    	DxThreadingFunctionsInfo functions_found = GetFunctionsInfo();

	    	if (functions_found != null)
	    	{
	    		out_impl_found.set( GetThreadingImpl() );
	    	}
	    	else
	    	{
	    		functions_found = m_default_impl_provider.RetrieveThreadingDefaultImpl(out_impl_found);
	    	}

	    	return functions_found;
	    }
	    
//	}  //End of dxThreadingBase

	public static class DxMutexGroupLockHelper
	{
	//public:
		public DxMutexGroupLockHelper(DxThreadingBase threading_base, DMutexGroup mutex_group, 
				//int /*dmutexindex_t*/ mutex_index) {
				dxProcessContextMutex mutex_index) {
			m_threading_base = threading_base;
			m_mutex_group = mutex_group;
			m_mutex_index = mutex_index;
			m_mutex_locked = true;
			threading_base.LockMutexGroupMutex(mutex_group, mutex_index);
		}

	    //~dxMutexGroupLockHelper()
		public void DESTRUCTOR()
	    {
	        if (m_mutex_locked)
	        {
	            m_threading_base.UnlockMutexGroupMutex(m_mutex_group, m_mutex_index);
	        }
	    }

	    public void UnlockMutex()
	    {
	    	Common.dIASSERT(m_mutex_locked);

	        m_threading_base.UnlockMutexGroupMutex(m_mutex_group, m_mutex_index);
	        m_mutex_locked = false;
	    }

	    void RelockMutex()
	    {
	        Common.dIASSERT(!m_mutex_locked);

	        m_threading_base.LockMutexGroupMutex(m_mutex_group, m_mutex_index);
	        m_mutex_locked = true;
	    }

	    //private:
	    private DxThreadingBase          	m_threading_base;
	    private DMutexGroup                 m_mutex_group;
	    //private int /*dmutexindex_t*/               m_mutex_index;
	    private dxProcessContextMutex		m_mutex_index;
	    private boolean                     m_mutex_locked;
	};

}
