/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * Threading implementation templates file.                              *
 * Copyright (C) 2011-2012 Oleh Derevenko. All rights reserved.          *
 * e-mail: odar@eleks.com (change all "a" to "e")                        *
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

package org.ode4j.ode.threading;

import static org.ode4j.ode.internal.Common.dAASSERT;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.threading.ThreadingAtomics.dxFakeAtomicsProvider.CompareExchangeTargetPtr;
import static org.ode4j.ode.threading.ThreadingImpl_H.THREAD_FACTORY;

import java.util.concurrent.atomic.AtomicReference;

import org.ode4j.ode.internal.DBase;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.cpp4j.java.RefInt;
import org.ode4j.ode.internal.processmem.DxWorldProcessContext.dxProcessContextMutex;
import org.ode4j.ode.threading.DThreadingImplementation.DThreadReadyToServeCallback;
import org.ode4j.ode.threading.ThreadingTemplates.dxtemplateJobListContainer.dWaitSignallingFunction;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;
import org.ode4j.ode.threading.Threading_H.DCallWait;
import org.ode4j.ode.threading.Threading_H.DMutexGroup;
import org.ode4j.ode.threading.Threading_H.DThreadedWaitTime;
import org.ode4j.ode.threading.Threading_H.dThreadedCallFunction;

/**
 *  Job list and Mutex group implementation templates for built-in threading 
 *  support provider.
 */
class ThreadingTemplates { 

	//TZ
	public static class dxCallWait extends dxICallWait {} //<tThreadWakeup> {};

	//TZ
	public static interface tThreadMutex {
		void LockMutex();
		boolean TryLockMutex();
		void UnlockMutex();
		boolean InitializeObject();
		void DESTRUCTOR();
	}

	//TZ
	public static interface tThreadWakeup {
		boolean InitializeObject();
		void ResetWakeup();
		void WakeupAllThreads();
		boolean WaitWakeup(DThreadedWaitTime timeout_time_ptr);
	}

	//TZ
	public static interface tJobListHandler {
		boolean InitializeObject();
		void ProcessActiveJobAddition();
		void PrepareForWaitingAJobCompletion();
		int RetrieveActiveThreadsCount();
		void ShutdownProcessing();
		void CleanupForRestart();
		void StickToJobsProcessing(
				DThreadReadyToServeCallback readiness_callback,
				CallContext callback_context);
	}

	//TZ
	public static interface tJobListContainer {
		boolean InitializeObject();
		boolean EnsureNumberOfJobInfosIsPreallocated(
				int max_simultaneous_calls_estimate);
		dxThreadedJobInfo AllocateJobInfoFromPool();
		void QueueJobForProcessing(dxThreadedJobInfo new_job);
		void AlterJobProcessingDependencies(dxThreadedJobInfo job_instance,
				int dependencies_count_change, RefBoolean job_has_become_ready);
		dxThreadedJobInfo ReleaseAJobAndPickNextPendingOne(
				dxThreadedJobInfo current_job, boolean job_result,
				dWaitSignallingFunction abstractSignalTheWait,
				RefBoolean dummy_last_job_flag);
	}

	//TZ
	public static interface tThreadLull {
		void RegisterToLull();
		void WaitForLullAlarm();
		void UnregisterFromLull();
		void SignalLullAlarmIfAnyRegistrants();
		boolean InitializeObject();
	}

	private static DCallReleasee dMAKE_JOBINSTANCE_RELEASEE(dxThreadedJobInfo job_instance) {
		return job_instance;
	}

	private static final dxThreadedJobInfo dMAKE_RELEASEE_JOBINSTANCE(DCallReleasee releasee) {
		return (dxThreadedJobInfo)releasee;
	}


	//template <class tThreadMutex>
	static class dxtemplateMutexGroup implements dIMutexGroup, DMutexGroup
	{
		//private:
		private dxtemplateMutexGroup(int size) {
			//TZ
			m_Mutex_array = new tThreadMutex[size];
		}
		//~dxtemplateMutexGroup() {}
		//private void DESTRUCTOR() {};

		//public:
		//public static dxtemplateMutexGroup<tThreadMutex>[] AllocateInstance(dmutexindex_t Mutex_count);
		//public static void FreeInstance(dxtemplateMutexGroup<tThreadMutex>[] mutex_group);

		//private:
		//private boolean InitializeMutexArray(dmutexindex_t Mutex_count);
		//private void FinalizeMutexArray(dmutexindex_t Mutex_count);

		//public:
		public void LockMutex(dxProcessContextMutex /*dmutexindex_t*/ mutex_index) { 
			dIASSERT(mutex_index.ordinal() < /*m_un.*/m_mutex_count.ordinal()); 
			m_Mutex_array[mutex_index.ordinal()].LockMutex(); 
		}
		public boolean TryLockMutex(dxProcessContextMutex /*dmutexindex_t*/ mutex_index) { 
			dIASSERT(mutex_index.ordinal() < /*m_un.*/m_mutex_count.ordinal()); 
			return m_Mutex_array[mutex_index.ordinal()].TryLockMutex(); 
		}
		public void UnlockMutex(dxProcessContextMutex /*dmutexindex_t*/ mutex_index) { 
			dIASSERT(mutex_index.ordinal() < /*m_un.*/m_mutex_count.ordinal()); 
			m_Mutex_array[mutex_index.ordinal()].UnlockMutex(); 
		}

		//private:
		//union
		//		private class m_un
		//		{
		dxProcessContextMutex /*dmutexindex_t*/     m_mutex_count;
		//(TZ)			unsigned long     m_reserved_for_allignment[2];
		//
		//		};// m_un;

		final tThreadMutex[]  m_Mutex_array;
		/************************************************************************/
		/* Implementation of dxtemplateMutexGroup                               */
		/************************************************************************/

		//template<class tThreadMutex>
		// /*static */dxtemplateMutexGroup<tThreadMutex> *dxtemplateMutexGroup<tThreadMutex>::
		public static dxtemplateMutexGroup AllocateInstance(
				dxProcessContextMutex /*dmutexindex_t*/ Mutex_count)
		{
			dAASSERT(Mutex_count != null);

			//TZ const dxtemplateMutexGroup<tThreadMutex> *const dummy_group = (dxtemplateMutexGroup<tThreadMutex> *)(size_t)8;
			//TZ final int size_requited = ((size_t)(&dummy_group.m_Mutex_array) - (size_t)dummy_group) + Mutex_count * sizeof(tThreadMutex);
			//dxtemplateMutexGroup<tThreadMutex>[] mutex_group = (dxtemplateMutexGroup<tThreadMutex> *)dAlloc(size_requited);
			dxtemplateMutexGroup mutex_group = new dxtemplateMutexGroup(Mutex_count.ordinal());

			if (mutex_group != null)
			{
				mutex_group./*m_un.*/m_mutex_count = Mutex_count;

				if (!mutex_group.InitializeMutexArray(Mutex_count))
				{
					//dFree((void *)mutex_group, size_requited);
					mutex_group = null;
				}
			}

			return mutex_group;
		}

		//template<class tThreadMutex>
		// /*static */void dxtemplateMutexGroup<tThreadMutex>::
		public static void FreeInstance(dxtemplateMutexGroup mutex_group)
		{
			if (mutex_group != null)
			{
				dxProcessContextMutex /*dmutexindex_t*/ Mutex_count = mutex_group./*m_un.*/m_mutex_count;
				mutex_group.FinalizeMutexArray(Mutex_count);

				//final int anyting_not_zero = 2 * sizeof(size_t);
				//const dxtemplateMutexGroup<tThreadMutex> *const dummy_group = (dxtemplateMutexGroup<tThreadMutex> *)anyting_not_zero;
				//const size_t size_requited = ((size_t)(&dummy_group.m_Mutex_array) - (size_t)dummy_group) + Mutex_count * sizeof(tThreadMutex);
				//dFree((void *)mutex_group, size_requited);
			}
		}

		//template<class tThreadMutex>
		private boolean InitializeMutexArray(dxProcessContextMutex /*dmutexindex_t*/ Mutex_count)
		{
			boolean any_fault = false;

			dxProcessContextMutex /*dmutexindex_t*/ mutex_index = dxProcessContextMutex.values()[0];
			//			for (; mutex_index != Mutex_count; ++mutex_index)
			//			{
			//				tThreadMutex *mutex_storage = m_Mutex_array + mutex_index;
			//
			//				new(mutex_storage) tThreadMutex;
			//
			//				if (!mutex_storage.InitializeObject())
			//				{
			//					mutex_storage.tThreadMutex::~tThreadMutex();
			//
			//					any_fault = true;
			//					break;
			//				}
			//			}
			for (int i = 0; i < m_Mutex_array.length; ++i) {
				tThreadMutex mutex_storage = THREAD_FACTORY.createThreadMutex();//new tThreadMutex();
				m_Mutex_array[i] = mutex_storage;
				if (!mutex_storage.InitializeObject())
				{
					mutex_storage.DESTRUCTOR();

					any_fault = true;
					break;
				}
			}


			if (any_fault)
			{
				FinalizeMutexArray(mutex_index);
			}

			boolean init_result = !any_fault;
			return init_result;
		}

		//template<class tThreadMutex>
		//void dxtemplateMutexGroup<tThreadMutex>::
		private void FinalizeMutexArray(dxProcessContextMutex /*dmutexindex_t*/ Mutex_count)
		{
			//			for (dmutexindex_t mutex_index = 0; mutex_index != Mutex_count; ++mutex_index)
			//			{
			//				tThreadMutex *mutex_storage = m_Mutex_array + mutex_index;
			//
			//				mutex_storage.tThreadMutex::~tThreadMutex();
			//			}
			for (int i = 0; i < Mutex_count.ordinal(); i++) {
				m_Mutex_array[i].DESTRUCTOR();
			}
		}

	};

	//template<class tThreadWakeup>
	static class dxtemplateCallWait
	extends DBase
	implements DCallWait
	{
		//public:
		public dxtemplateCallWait() {
			//TZ
			m_wait_wakeup = THREAD_FACTORY.createThreadWakeup();
		}
		//~dxtemplateCallWait() { DoFinalizeObject(); }
		@Override
		public void DESTRUCTOR() { DoFinalizeObject(); }

		public boolean InitializeObject() { return DoInitializeObject(); }

		//private:
		private boolean DoInitializeObject() { return m_wait_wakeup.InitializeObject(); }
		private void DoFinalizeObject() { /* Do nothing */ }

		//public:
		//typedef dxtemplateCallWait<tThreadWakeup> dxCallWait;
		//TZ see top of class 

		//public:
		public void ResetTheWait() { m_wait_wakeup.ResetWakeup(); }
		public void SignalTheWait() { m_wait_wakeup.WakeupAllThreads(); }
		public boolean PerformWaiting(final DThreadedWaitTime timeout_time_ptr) { 
			return m_wait_wakeup.WaitWakeup(timeout_time_ptr); 
		}

		//public:
		public static dWaitSignallingFunction AbstractSignalTheWait = new dWaitSignallingFunction() {
			@Override
			public void run(dxCallWait job_call_wait) {
				job_call_wait.SignalTheWait(); 
			}
		};

		//private:
		private tThreadWakeup           m_wait_wakeup;
	};


	//TODO TZ implement for multi-threading
	//#if dBUILTIN_THREADING_IMPL_ENABLED
	//
	//template<class tThreadWakeup, class tAtomicsProvider, const bool tatomic_test_required>
	//class dxtemplateThreadedLull
	//{
	//public:
	//    dxtemplateThreadedLull(): m_registrant_count(0), m_alarm_wakeup() {}
	//    ~dxtemplateThreadedLull() { dIASSERT(m_registrant_count == 0); DoFinalizeObject(); }
	//
	//    bool InitializeObject() { return DoInitializeObject(); }
	//
	//private:
	//    bool DoInitializeObject() { return m_alarm_wakeup.InitializeObject(); }
	//    void DoFinalizeObject() { /* Do nothing */ }
	//
	//private:
	//    typedef typename tAtomicsProvider::atomicord_t atomicord_t;
	//
	//public:
	//    void RegisterToLull() { tAtomicsProvider::IncrementTargetNoRet(&m_registrant_count); }
	//    void WaitForLullAlarm() { dIASSERT(m_registrant_count != 0); m_alarm_wakeup.WaitWakeup(NULL); }
	//    void UnregisterFromLull() { tAtomicsProvider::DecrementTargetNoRet(&m_registrant_count); }
	//
	//    void SignalLullAlarmIfAnyRegistrants()
	//    {
	//        if (tatomic_test_required ? (tAtomicsProvider::QueryTargetValue(&m_registrant_count) != 0) : (m_registrant_count != 0))
	//        {
	//            m_alarm_wakeup.WakeupAThread();
	//        }
	//    }
	//
	//private:
	//    atomicord_t             m_registrant_count;
	//    tThreadWakeup           m_alarm_wakeup;
	//};
	//
	//
	//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED


	static class dxThreadedJobInfo extends
	DBase implements DCallReleasee
	{
		dxThreadedJobInfo() {}
		//explicit dxThreadedJobInfo(void *): m_next_job(NULL) {}
		dxThreadedJobInfo(Object[] noName) {
			m_next_job = null;
		}
		@Override
		public void DESTRUCTOR() {};

		void AssignJobData(int /*ddependencycount_t*/ dependencies_count, 
				dxThreadedJobInfo dxThreadedJobInfo,
				dxCallWait call_wait, 
				RefInt fault_accumulator_ptr, 
				dThreadedCallFunction call_function, 
				CallContext call_context, int/*dcallindex_t*/ call_index)
		{
			m_dependencies_count = dependencies_count;
			m_dependent_job = dxThreadedJobInfo;
			m_call_wait = call_wait;
			m_fault_accumulator_ptr = fault_accumulator_ptr;

			m_call_fault = 0;
			m_call_function = call_function;
			m_call_context = call_context;
			m_call_index = call_index;
		}

		boolean InvokeCallFunction()
		{
			boolean call_result = m_call_function.run(m_call_context, m_call_index, dMAKE_JOBINSTANCE_RELEASEE(this));
			return call_result;// != 0;
		}

		dxThreadedJobInfo       m_next_job;
		Ref<dxThreadedJobInfo>   m_prev_job_next_ptr;

		int/*ddependencycount_t*/ m_dependencies_count;
		dxThreadedJobInfo       m_dependent_job;
		dxCallWait              m_call_wait;
		RefInt                  m_fault_accumulator_ptr = new RefInt();

		int                     m_call_fault;
		dThreadedCallFunction   m_call_function;
		CallContext             m_call_context;
		int /*dcallindex_t*/            m_call_index;
	};


	//template<class tThreadMutex>
	static class dxtemplateThreadingLockHelper
	{
		//public:
		public dxtemplateThreadingLockHelper(tThreadMutex /* & */ mutex_instance){
			m_mutex_instance = mutex_instance;
			m_lock_indicator_flag = false;
			LockMutex(); 
		}
		//~dxtemplateThreadingLockHelper() { if (m_lock_indicator_flag) { UnlockMutex(); } }
		public void DESTRUCTOR() { 
			if (m_lock_indicator_flag) { UnlockMutex(); } 
		}

		void LockMutex() { 
			dIASSERT(!m_lock_indicator_flag); 
			m_mutex_instance.LockMutex(); 
			m_lock_indicator_flag = true; 
		}
		void UnlockMutex() { 
			dIASSERT(m_lock_indicator_flag); 
			m_mutex_instance.UnlockMutex(); 
			m_lock_indicator_flag = false;
		}

		//private:
		private tThreadMutex       /* & */     m_mutex_instance;
		private boolean                 m_lock_indicator_flag;
	};

	//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
	static class dxtemplateJobListContainer//<tThreadLull>//, tThreadMutex>//, tAtomicsProvider>
	implements tJobListContainer
	{
		//public:
		public dxtemplateJobListContainer() {
			m_job_list.set(null);//(NULL),
			//m_info_pool = new ((atomicptr_t)NULL);
			m_pool_access_lock = THREAD_FACTORY.createThreadMutex();//();
			m_list_access_lock = THREAD_FACTORY.createThreadMutex();//();
			m_info_wait_lull = THREAD_FACTORY.createThreadLull();//();
			m_info_count_known_to_be_preallocated = 0;//(0)
		}

		//~dxtemplateJobListContainer()
		public void DESTRUCTOR()
		{
			dIASSERT(m_job_list.get() == null); // Would not it be nice to wait for jobs to complete before deleting the list?

			FreeJobInfoPoolInfos();
			DoFinalizeObject();
		}

		@Override
		public boolean InitializeObject() { return DoInitializeObject(); }

		//private:
		private boolean DoInitializeObject() { 
			return m_pool_access_lock.InitializeObject() && m_list_access_lock.InitializeObject() && m_info_wait_lull.InitializeObject();
		}
		private void DoFinalizeObject() { /* Do nothing */ }

		//public:
		//    typedef tAtomicsProvider dxAtomicsProvider;
		//    typedef typename tAtomicsProvider::atomicord_t atomicord_t;
		//    typedef typename tAtomicsProvider::atomicptr_t atomicptr_t;
		//    typedef tThreadMutex dxThreadMutex;
		//    typedef dxtemplateThreadingLockHelper<tThreadMutex> dxMutexLockHelper;
		//    typedef void dWaitSignallingFunction(void *job_call_wait);
		public static interface dWaitSignallingFunction{

			void run(dxCallWait job_call_wait);}

		//public:
		//public dxThreadedJobInfo[] ReleaseAJobAndPickNextPendingOne(
		//		dxThreadedJobInfo[] job_to_release, boolean job_result, dWaitSignallingFunction[] wait_signal_proc_ptr, 
		//		RefBool out_last_job_flag);

		//private:
		//private dxThreadedJobInfo[] PickNextPendingJob(RefBool out_last_job_flag);
		//private void ReleaseAJob(dxThreadedJobInfo[] job_instance, boolean job_result, dWaitSignallingFunction[] wait_signal_proc_ptr);

		//public:
		//public final dxThreadedJobInfo[] AllocateJobInfoFromPool();
		//public void QueueJobForProcessing(dxThreadedJobInfo[] job_instance);

		//public void AlterJobProcessingDependencies(dxThreadedJobInfo[] job_instance, int /*ddependencychange_t*/ dependencies_count_change, 
		//		RefBool out_job_has_become_ready);

		//private:
		//private final int /*ddependencycount_t*/ SmartAddJobDependenciesCount(dxThreadedJobInfo[] job_instance, 
		//		int /*ddependencychange_t*/ dependencies_count_change);

		//private final void InsertJobInfoIntoListHead(dxThreadedJobInfo[] job_instance);
		//private final void RemoveJobInfoFromList(dxThreadedJobInfo[] job_instance);

		//private dxThreadedJobInfo[] ExtractJobInfoFromPoolOrAllocate();
		//private final void ReleaseJobInfoIntoPool(dxThreadedJobInfo[] job_instance);

		//private:
		//private void FreeJobInfoPoolInfos();

		//public:
		//public boolean EnsureNumberOfJobInfosIsPreallocated(int /*ddependencycount_t*/ required_info_count);

		//private:
		//private boolean DoPreallocateJobInfos(int /*ddependencycount_t*/ required_info_count);

		//public:
		public boolean IsJobListReadyForShutdown() { return m_job_list.get() == null; }

		//private:
		private final Ref<dxThreadedJobInfo>       m_job_list = new Ref<dxThreadedJobInfo>();
		//private volatile atomicptr_t    m_info_pool; // dxThreadedJobInfo *
		private final AtomicReference<dxThreadedJobInfo> m_info_pool = 
				new AtomicReference<dxThreadedJobInfo>(); // dxThreadedJobInfo *
		private tThreadMutex            m_pool_access_lock;
		private tThreadMutex            m_list_access_lock;
		private tThreadLull             m_info_wait_lull;
		private int /*ddependencycount_t*/      m_info_count_known_to_be_preallocated;

		/************************************************************************/
		/* Implementation of dxtemplateJobListContainer                         */
		/************************************************************************/


		@Override
//		public dxThreadedJobInfo ReleaseAJobAndPickNextPendingOne(
//				dxThreadedJobInfo current_job, boolean job_result,
//				AbstractSignalTheWait_class abstractSignalTheWait,
//				RefBoolean dummy_last_job_flag) {
//			// TODO Auto-generated method stub
//			throw new UnsupportedOperationException();
//			//return null;
//		}
		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//dxThreadedJobInfo *dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		public dxThreadedJobInfo ReleaseAJobAndPickNextPendingOne(
				dxThreadedJobInfo job_to_release, boolean job_result, 
				dWaitSignallingFunction wait_signal_proc_ptr, RefBoolean out_last_job_flag)
		{
			if (job_to_release != null)
			{
				ReleaseAJob(job_to_release, job_result, wait_signal_proc_ptr);
			}

			//dxMutexLockHelper list_access(m_list_access_lock);
			dxtemplateThreadingLockHelper list_access = new dxtemplateThreadingLockHelper(m_list_access_lock);

			dxThreadedJobInfo picked_job = PickNextPendingJob(out_last_job_flag);
			return picked_job;
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//dxThreadedJobInfo *dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private dxThreadedJobInfo PickNextPendingJob(
				RefBoolean out_last_job_flag)
		{
			dxThreadedJobInfo current_job = m_job_list.get();
			boolean last_job_flag = false;

			while (current_job != null)
			{
				if (current_job.m_dependencies_count == 0)
				{
					// It is OK to assign in unsafe manner - dependencies count should not be changed
					// after the job has become ready for execution
					current_job.m_dependencies_count = 1;
					last_job_flag = current_job.m_next_job == null;

					RemoveJobInfoFromList(current_job);
					break;
				}

				current_job = current_job.m_next_job;
			}

			out_last_job_flag.set(last_job_flag);
			return current_job;
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private void ReleaseAJob(
				dxThreadedJobInfo job_instance, boolean job_result, 
				dWaitSignallingFunction wait_signal_proc_ptr)
		{
			dxThreadedJobInfo current_job = job_instance;

			if (!job_result)
			{
				// Accumulate call fault (be careful to not reset it!!!)
				current_job.m_call_fault = 1;
			}

			boolean job_dequeued = true;
			dIASSERT(current_job.m_prev_job_next_ptr == null);

			while (true)
			{
				dIASSERT(current_job.m_dependencies_count != 0);

				int /*ddependencycount_t*/ new_dependencies_count = SmartAddJobDependenciesCount(current_job, -1);

				if (new_dependencies_count != 0 || !job_dequeued)
				{
					break;
				}

				dxCallWait job_call_wait = current_job.m_call_wait;

				if (job_call_wait != null)
				{
					wait_signal_proc_ptr.run(job_call_wait);
				}

				int call_fault = current_job.m_call_fault;

				if (current_job.m_fault_accumulator_ptr != null)
				{
					current_job.m_fault_accumulator_ptr.set( call_fault );
				}

				dxThreadedJobInfo dependent_job = current_job.m_dependent_job;
				ReleaseJobInfoIntoPool(current_job);

				if (dependent_job == null)
				{
					break;
				}

				if (call_fault != 0)
				{
					// Accumulate call fault (be careful to not reset it!!!)
					dependent_job.m_call_fault = 1;
				}

				current_job = dependent_job;
				job_dequeued = dependent_job.m_prev_job_next_ptr == null;
			}
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//dxThreadedJobInfo *dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		@Override
		public dxThreadedJobInfo AllocateJobInfoFromPool()
		{
			// No locking is necessary
			dxThreadedJobInfo job_instance = ExtractJobInfoFromPoolOrAllocate();
			return job_instance;
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		@Override
		public void QueueJobForProcessing(dxThreadedJobInfo job_instance)
		{
			//dxMutexLockHelper list_access(m_list_access_lock);
			dxtemplateThreadingLockHelper list_access = new dxtemplateThreadingLockHelper(m_list_access_lock);

			InsertJobInfoIntoListHead(job_instance);
		}


		@Override
		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		public void AlterJobProcessingDependencies(dxThreadedJobInfo job_instance, 
				int /*ddependencychange_t*/ dependencies_count_change, 
				RefBoolean out_job_has_become_ready)
		{
			// Dependencies should not be changed when job has already become ready for execution
			dIASSERT(job_instance.m_dependencies_count != 0);
			// It's OK that access is not atomic - that is to be handled by external logic
			//dIASSERT(dependencies_count_change < 0 ? (job_instance.m_dependencies_count >= (ddependencycount_t)(-dependencies_count_change)) : ((ddependencycount_t)(-(ddependencychange_t)job_instance.m_dependencies_count) > (ddependencycount_t)dependencies_count_change));
			//TZ: ddependencycount_t is UN-signed <--> dependencies_count_change is SIGNED --> wtf?!?!?
			// -->   job_instance.m_dependencies_count = dependencies_count_change = 1 fails in Java but passes in C/C++
			//dIASSERT(dependencies_count_change < 0 ? (job_instance.m_dependencies_count >= (-dependencies_count_change)) : ((-job_instance.m_dependencies_count) > (int)dependencies_count_change));

			int /*ddependencycount_t*/ new_dependencies_count = SmartAddJobDependenciesCount(job_instance, dependencies_count_change);
			out_job_has_become_ready.set( new_dependencies_count == 0 );
		}


		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//ddependencycount_t dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private int SmartAddJobDependenciesCount(
				dxThreadedJobInfo job_instance, int /*ddependencychange_t*/ dependencies_count_change)
		{
			//			ddependencycount_t new_dependencies_count = 
			//					tAtomicsProvider::template AddValueToTarget<sizeof(ddependencycount_t)>((volatile void *)&job_instance.m_dependencies_count, dependencies_count_change) 
			//					+ dependencies_count_change;
//			int new_dependencies_count = ThreadingAtomics.dxFakeAtomicsProvider.
//					AddValueToTarget1(job_instance.m_dependencies_count, dependencies_count_change) 
//					+ dependencies_count_change;
			//TODO TZ keep in mind for future changes:
			//TZ no atomic needed, update occurs only here
			synchronized (job_instance) {
				job_instance.m_dependencies_count += dependencies_count_change;
				return job_instance.m_dependencies_count;
			}
			//return new_dependencies_count;
		}


		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private void InsertJobInfoIntoListHead(
				dxThreadedJobInfo job_instance)
		{
			dxThreadedJobInfo job_list_head = m_job_list.get();
			job_instance.m_next_job = job_list_head;

			if (job_list_head != null)
			{
				job_list_head.m_prev_job_next_ptr = 
						new Ref<dxThreadedJobInfo>(job_instance.m_next_job);
			}

			job_instance.m_prev_job_next_ptr = m_job_list;
			m_job_list.set( job_instance );
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private void RemoveJobInfoFromList(
				dxThreadedJobInfo job_instance)
		{
			if (job_instance.m_next_job != null)
			{ 
				job_instance.m_next_job.m_prev_job_next_ptr = job_instance.m_prev_job_next_ptr;
			}

			job_instance.m_prev_job_next_ptr.set(job_instance.m_next_job);
			// Assign NULL to m_prev_job_next_ptr as an indicator that instance has been dequeued
			//TZ this makes sense if the pointer equals the m_job_list REF.
			job_instance.m_prev_job_next_ptr = null;
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//dxThreadedJobInfo *dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private dxThreadedJobInfo ExtractJobInfoFromPoolOrAllocate()
		{
			dxThreadedJobInfo result_info;

			boolean waited_lull = false;
			m_info_wait_lull.RegisterToLull();

			while (true)
			{
				dxThreadedJobInfo raw_head_info = m_info_pool.get();

				if (raw_head_info == null)
				{
					result_info = new dxThreadedJobInfo();

					if (result_info != null)
					{
						break;
					}

					m_info_wait_lull.WaitForLullAlarm();
					waited_lull = true;
				}

				// Extraction must be locked so that other thread does not "steal" head info,
				// use it and then reinsert back with a different "next"
				//dxMutexLockHelper pool_access(m_pool_access_lock);
				dxtemplateThreadingLockHelper pool_access = new dxtemplateThreadingLockHelper(m_pool_access_lock);

				dxThreadedJobInfo head_info = m_info_pool.get(); // Head info must be re-read after mutex had been locked

				if (head_info != null)
				{
					dxThreadedJobInfo next_info = head_info.m_next_job;
					//if (tAtomicsProvider::CompareExchangeTargetPtr(&m_info_pool, (atomicptr_t)head_info, (atomicptr_t)next_info))
					if (CompareExchangeTargetPtr(m_info_pool, head_info, next_info))
					{
						result_info = head_info;
						break;
					}
				}
			}

			m_info_wait_lull.UnregisterFromLull();

			if (waited_lull)
			{
				// It is necessary to re-signal lull alarm if current thread was waiting as
				// there might be other threads waiting which might have not received alarm signal.
				m_info_wait_lull.SignalLullAlarmIfAnyRegistrants();
			}

			return result_info;
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private void ReleaseJobInfoIntoPool(
				dxThreadedJobInfo job_instance)
		{
			while (true)
			{
				dxThreadedJobInfo next_info = m_info_pool.get();
				job_instance.m_next_job = next_info;

				if (CompareExchangeTargetPtr(m_info_pool, next_info, job_instance))
				{
					break;
				}
			}

			m_info_wait_lull.SignalLullAlarmIfAnyRegistrants();
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//void dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private void FreeJobInfoPoolInfos()
		{
			dxThreadedJobInfo current_info = m_info_pool.get();

			while (current_info != null)
			{
				dxThreadedJobInfo info_save = current_info;
				current_info = current_info.m_next_job;

				//delete info_save;
				info_save.DESTRUCTOR();
			}

			m_info_pool.set(null);
		}

		@Override
		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//bool dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		public boolean EnsureNumberOfJobInfosIsPreallocated(int /*ddependencycount_t*/ required_info_count)
		{
			boolean result = required_info_count <= m_info_count_known_to_be_preallocated 
					|| DoPreallocateJobInfos(required_info_count);
			return result;
		}

		//template<class tThreadLull, class tThreadMutex, class tAtomicsProvider>
		//bool dxtemplateJobListContainer<tThreadLull, tThreadMutex, tAtomicsProvider>::
		private boolean DoPreallocateJobInfos(int /*ddependencycount_t*/ required_info_count)
		{
			dIASSERT(required_info_count > m_info_count_known_to_be_preallocated); // Also ensures required_info_count > 0

			boolean allocation_failure = false;

			final dxThreadedJobInfo info_pool = m_info_pool.get();

			int /*ddependencycount_t*/ info_index = 0;
			//for (dxThreadedJobInfo **current_info_ptr = &info_pool; ; )
			//TZ
			dxThreadedJobInfo previous_info = null;
			for (dxThreadedJobInfo current_info_ptr = info_pool; ; )
			{
				dxThreadedJobInfo current_info = current_info_ptr;

				if (current_info == null)
				{
					current_info = new dxThreadedJobInfo(null);

					if (current_info == null)
					{
						allocation_failure = true;
						break;
					}

					current_info_ptr = current_info;
					//TZ
					if (previous_info == null) {
						//We are in the first loop round
						m_info_pool.set(current_info);
					} else {
						//for subsequent loop rounds
						previous_info.m_next_job = current_info;
					}
				}

				if (++info_index == required_info_count)
				{
					m_info_count_known_to_be_preallocated = info_index;
					break;
				}

				//current_info_ptr = &current_info.m_next_job;
				previous_info = current_info_ptr;
				current_info_ptr = current_info.m_next_job;
			}

			// Make sure m_info_pool was not changed
//TZ TODO?			dIASSERT(m_info_pool == null || m_info_pool.get() == info_pool);

//TZ TODO?			m_info_pool.set(info_pool);

			boolean result = !allocation_failure;
			return result;
		}

	};


	//typedef void (dxThreadReadyToServeCallback)(void *callback_context);
	/** A function that takes a callback context. */
	//see DThreadReadyToServeCallback
	//public static interface dxThreadReadyToServeCallback extends DThreadReadyToServeCallback {
	//	//void run(Object[] callback_context);
	//}


	//TODO TZ implement for multi-threading
	//#if dBUILTIN_THREADING_IMPL_ENABLED
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//class dxtemplateJobListThreadedHandler
	//{
	//public:
	//    dxtemplateJobListThreadedHandler(tJobListContainer *list_container_ptr):
	//        m_job_list_ptr(list_container_ptr),
	//        m_processing_wakeup(),
	//        m_active_thread_count(0),
	//        m_shutdown_requested(0)
	//    {
	//    }
	//
	//    ~dxtemplateJobListThreadedHandler()
	//    {
	//        dIASSERT(m_active_thread_count == 0);
	//
	//        DoFinalizeObject();
	//    }
	//
	//    bool InitializeObject() { return DoInitializeObject(); }
	//
	//private:
	//    bool DoInitializeObject() { return m_processing_wakeup.InitializeObject(); }
	//    void DoFinalizeObject() { /* Do nothing */ }
	//
	//public:
	//    typedef dxtemplateCallWait<tThreadWakeup> dxCallWait;
	//
	//public:
	//    inline void ProcessActiveJobAddition();
	//    inline void PrepareForWaitingAJobCompletion();
	//
	//public:
	//    inline unsigned RetrieveActiveThreadsCount();
	//    inline void StickToJobsProcessing(dxThreadReadyToServeCallback *readiness_callback/*=NULL*/, void *callback_context/*=NULL*/);
	//
	//private:
	//    void PerformJobProcessingUntilShutdown();
	//    void PerformJobProcessingSession();
	//
	//    void BlockAsIdleThread();
	//    void ActivateAnIdleThread();
	//
	//public:
	//    inline void ShutdownProcessing();
	//    inline void CleanupForRestart();
	//
	//private:
	//    bool IsShutdownRequested() const { return m_shutdown_requested != 0; }
	//
	//private:
	//    typedef typename tJobListContainer::dxAtomicsProvider dxAtomicsProvider;
	//    typedef typename tJobListContainer::atomicord_t atomicord_t;
	//
	//    atomicord_t GetActiveThreadsCount() const { return m_active_thread_count; }
	//    void RegisterAsActiveThread() { dxAtomicsProvider::template AddValueToTarget<sizeof(atomicord_t)>((volatile void *)&m_active_thread_count, 1); }
	//    void UnregisterAsActiveThread() { dxAtomicsProvider::template AddValueToTarget<sizeof(atomicord_t)>((volatile void *)&m_active_thread_count, -1); }
	//
	//private:
	//    tJobListContainer       *m_job_list_ptr;
	//    tThreadWakeup           m_processing_wakeup;
	//    volatile atomicord_t    m_active_thread_count;
	//    int                     m_shutdown_requested;
	//};
	//
	//
	//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED


	//template<class tThreadWakeup, class tJobListContainer>
	static class dxtemplateJobListSelfHandlertemplate//<tThreadWakeup, tJobListContainer>
	implements tJobListHandler
	{
		//TZ
		public dxtemplateJobListSelfHandlertemplate() {
			// nothing
		}
		
		//public:
		public dxtemplateJobListSelfHandlertemplate(tJobListContainer list_container_ptr)
		{
			m_job_list_ptr = list_container_ptr;
		}

		public void DESTRUCTOR()//~dxtemplateJobListSelfHandler()
		{
			// Do nothing
		}

		@Override
		public boolean InitializeObject() { return true; }

		//public:
		//typedef dxtemplateCallWait<tThreadWakeup> dxCallWait;

		//public:
		//public final void ProcessActiveJobAddition();
		//public final void PrepareForWaitingAJobCompletion();

		//public:
		//public final int RetrieveActiveThreadsCount();
		//public final void StickToJobsProcessing(
		//		dxThreadReadyToServeCallback[] readiness_callback/*=NULL*/, 
		//		Object[] callback_context/*=NULL*/);

		//private:
		//private void PerformJobProcessingUntilExhaustion();
		//private void PerformJobProcessingSession();

		//public:
		//public final void ShutdownProcessing();
		//public final void CleanupForRestart();

		//private:
		private tJobListContainer       m_job_list_ptr;

		/************************************************************************/
		/* Implementation of dxtemplateJobListSelfHandler                       */
		/************************************************************************/

		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		@Override
		public void ProcessActiveJobAddition()
		{
			// Do nothing
		}

		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		@Override
		public void PrepareForWaitingAJobCompletion()
		{
			PerformJobProcessingUntilExhaustion();
		}


		//template<class tThreadWakeup, class tJobListContainer>
		//unsigned dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		@Override
		public int RetrieveActiveThreadsCount()
		{
			return 1;//U; // Self-Handling is always performed by a single thread
		}

		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		@Override
		public void StickToJobsProcessing(DThreadReadyToServeCallback readiness_callback/*=NULL*/, 
				CallContext callback_context/*=NULL*/)
		{
			//(void)readiness_callback; // unused
			//(void)callback_context; // unused
			dIASSERT(false); // This method is not expected to be called for Self-Handler
		}


		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		private void PerformJobProcessingUntilExhaustion()
		{
			PerformJobProcessingSession();
		}

		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		private void PerformJobProcessingSession()
		{
			dxThreadedJobInfo current_job = null;
			boolean job_result = false;

			while (true)
			{
				RefBoolean dummy_last_job_flag = new RefBoolean(false);
				current_job = m_job_list_ptr.ReleaseAJobAndPickNextPendingOne(
						current_job, job_result, dxCallWait.AbstractSignalTheWait, dummy_last_job_flag);

				if (current_job == null)
				{
					break;
				}

				job_result = current_job.InvokeCallFunction();
			}
		}

		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		@Override
		public void ShutdownProcessing()
		{
			// Do nothing
		}

		//template<class tThreadWakeup, class tJobListContainer>
		//void dxtemplateJobListSelfHandler<tThreadWakeup, tJobListContainer>::
		@Override
		public void CleanupForRestart()
		{
			// Do nothing
		}
	};


	//struct dIMutexGroup;
	public static interface dIMutexGroup {}
	//struct dxICallWait;
	public static class dxICallWait extends dxtemplateCallWait {}

	abstract static class dxIThreadingImplementation extends DxThreadingImplementation
	{
		//public:
		public abstract void FreeInstance();// = 0;

		//public:
		public abstract dIMutexGroup AllocMutexGroup(dxProcessContextMutex /*dmutexindex_t*/ Mutex_count);// = 0;
		public abstract void FreeMutexGroup(dIMutexGroup mutex_group);// = 0;
		public abstract void LockMutexGroupMutex(dIMutexGroup mutex_group, dxProcessContextMutex /*dmutexindex_t*/ mutex_index);// = 0;
		// virtual bool TryLockMutexGroupMutex(dIMutexGroup *mutex_group, dmutexindex_t mutex_index) = 0;
		public abstract void UnlockMutexGroupMutex(dIMutexGroup mutex_group, dxProcessContextMutex /*dmutexindex_t*/ mutex_index);// = 0;

		//public:
		public abstract dxICallWait AllocACallWait();// = 0;
		public abstract void ResetACallWait(dxICallWait call_wait);// = 0;
		public abstract void FreeACallWait(dxICallWait call_wait);// = 0;

		//public:
		public abstract boolean PreallocateJobInfos(int /*ddependencycount_t*/ max_simultaneous_calls_estimate);// = 0;
		public abstract void ScheduleNewJob(RefInt fault_accumulator_ptr/*=NULL*/, 
				Ref<DCallReleasee> out_post_releasee_ptr/*=NULL*/, int /*ddependencycount_t*/ dependencies_count, 
				DCallReleasee dependent_releasee/*=NULL*/, 
				dxICallWait call_wait/*=NULL*/, 
				dThreadedCallFunction call_func, CallContext call_context, 
				int /*dcallindex_t*/ instance_index);// = 0;
		public abstract void AlterJobDependenciesCount(DCallReleasee target_releasee, 
				int /*ddependencychange_t*/ dependencies_count_change);// = 0;
		public abstract void WaitJobCompletion(RefInt out_wait_status_ptr/*=NULL*/, 
				dxICallWait call_wait, final DThreadedWaitTime timeout_time_ptr/*=NULL*/);// = 0;

		//public:
		public abstract int RetrieveActiveThreadsCount();// = 0;
		public abstract void StickToJobsProcessing(DThreadReadyToServeCallback readiness_callback/*=NULL*/, 
				CallContext callback_context/*=NULL*/);// = 0;
		public abstract void ShutdownProcessing();// = 0;
		public abstract void CleanupForRestart();// = 0;
	};


	//template<class tJobListContainer, class tJobListHandler>
	static abstract class dxtemplateThreadingImplementation extends //<tJobListContainer, tJobListHandler> extends
	//DBase,  --> TZ: not really required...
	dxIThreadingImplementation
	{
		//public:
		public dxtemplateThreadingImplementation() {
			super();//dBase();
			m_list_container = THREAD_FACTORY.createJobListContainer();//();
			m_list_handler = new dxtemplateJobListSelfHandlertemplate(m_list_container);//(&m_list_container);
		}

		//virtual ~dxtemplateThreadingImplementation()
		public void DESTRUCTOR()
		{
			DoFinalizeObject();
		}

		boolean InitializeObject() { return DoInitializeObject(); }

		//private:
		private boolean DoInitializeObject() { 
			return m_list_container.InitializeObject() && m_list_handler.InitializeObject(); }
		private void DoFinalizeObject() { /* Do nothing */ }

		//protected:
		//protected abstract void FreeInstance();

		//private:
		//private typedef dxtemplateMutexGroup<typename tJobListContainer::dxThreadMutex> dxMutexGroup;
		//private typedef typename tJobListHandler::dxCallWait dxCallWait;

		//protected:
		//protected abstract dIMutexGroup[] AllocMutexGroup(dmutexindex_t Mutex_count);
		//protected abstract void FreeMutexGroup(dIMutexGroup[] mutex_group);
		//protected abstract void LockMutexGroupMutex(dIMutexGroup[] mutex_group, dmutexindex_t mutex_index);
		// virtual bool TryLockMutexGroupMutex(dIMutexGroup *mutex_group, dmutexindex_t mutex_index);
		//protected abstract void UnlockMutexGroupMutex(dIMutexGroup[] mutex_group, dmutexindex_t mutex_index);

		//protected:
		//protected abstract dxICallWait[] AllocACallWait();
		//protected abstract void ResetACallWait(dxICallWait[] call_wait);
		//protected abstract void FreeACallWait(dxICallWait[] call_wait);

		//protected:
		//protected abstract boolean PreallocateJobInfos(int /*ddependencycount_t*/ max_simultaneous_calls_estimate);
		//protected abstract void ScheduleNewJob(int[] fault_accumulator_ptr/*=NULL*/, 
		//		DCallReleasee[] out_post_releasee_ptr/*=NULL*/, int /*ddependencycount_t*/ dependencies_count, DCallReleasee dependent_releasee/*=NULL*/, 
		//		dxICallWait[] call_wait/*=NULL*/, 
		//		dThreadedCallFunction[] call_func, Object[] call_context, dcallindex_t instance_index);
		//protected abstract void AlterJobDependenciesCount(DCallReleasee target_releasee, int /*ddependencychange_t*/ dependencies_count_change);
		//protected abstract void WaitJobCompletion(int[] out_wait_status_ptr/*=NULL*/, 
		//		dxICallWait[] call_wait, final dThreadedWaitTime[] timeout_time_ptr/*=NULL*/);

		//protected:
		//protected abstract int RetrieveActiveThreadsCount();
		//protected abstract void StickToJobsProcessing(dxThreadReadyToServeCallback[] readiness_callback/*=NULL*/, 
		//		Object[] callback_context/*=NULL*/);
		//protected abstract void ShutdownProcessing();
		//protected abstract void CleanupForRestart();

		//private:
		//TZ final to ensure that simplified constructor is correct
		//TZ TOD why on earth are there two separate fields?
		private final tJobListContainer     m_list_container;
		private final tJobListHandler       m_list_handler;

		/************************************************************************/
		/* Implementation of dxtemplateThreadingImplementation                          */
		/************************************************************************/

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void FreeInstance()
		{
			//delete this;
			DESTRUCTOR();
		}


		//template<class tJobListContainer, class tJobListHandler>
		//dIMutexGroup *dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public dIMutexGroup AllocMutexGroup(dxProcessContextMutex /*dmutexindex_t*/ Mutex_count)
		{
			dxtemplateMutexGroup mutex_group = dxtemplateMutexGroup.AllocateInstance(Mutex_count);
			return mutex_group;
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void FreeMutexGroup(dIMutexGroup mutex_group)
		{
			//dxMutexGroup::FreeInstance((dxMutexGroup *)mutex_group);
			dxtemplateMutexGroup.FreeInstance((dxtemplateMutexGroup) mutex_group);
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void LockMutexGroupMutex(dIMutexGroup mutex_group, 
				dxProcessContextMutex /*dmutexindex_t*/ mutex_index)
		{
			//((dxMutexGroup *)mutex_group).LockMutex(mutex_index);
			((dxtemplateMutexGroup)mutex_group).LockMutex(mutex_index);
		}

		// template<class tJobListContainer, class tJobListHandler>
		// bool dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::TryLockMutexGroupMutex(dIMutexGroup *mutex_group, dmutexindex_t mutex_index)
		// {
		//   return ((dxMutexGroup *)mutex_group)->TryLockMutex(mutex_index);
		// }

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void UnlockMutexGroupMutex(dIMutexGroup mutex_group, 
				dxProcessContextMutex /*dmutexindex_t*/ mutex_index)
		{
			//((dxMutexGroup *)mutex_group).UnlockMutex(mutex_index);
			((dxtemplateMutexGroup)mutex_group).UnlockMutex(mutex_index);
		}


		//template<class tJobListContainer, class tJobListHandler>
		//dxICallWait *dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public dxICallWait AllocACallWait()
		{
			dxCallWait call_wait = new dxCallWait();

			if (call_wait != null && !call_wait.InitializeObject())
			{
				//delete call_wait;
				call_wait.DESTRUCTOR();
				call_wait = null;
			}

			return call_wait;
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void ResetACallWait(dxICallWait call_wait)
		{
			((dxCallWait)call_wait).ResetTheWait();
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void FreeACallWait(dxICallWait call_wait)
		{
			//delete ((dxCallWait *)call_wait);
			call_wait.DESTRUCTOR();
		}


		//template<class tJobListContainer, class tJobListHandler>
		//bool dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public boolean PreallocateJobInfos(int /*ddependencycount_t*/ max_simultaneous_calls_estimate)
		{
			// No multithreading protection here!
			// Resources are to be preallocated before jobs start to be scheduled
			// as otherwise there is no way to implement the preallocation.
			boolean result = m_list_container.EnsureNumberOfJobInfosIsPreallocated(max_simultaneous_calls_estimate);
			return result;
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void ScheduleNewJob(
				RefInt fault_accumulator_ptr/*=NULL*/, 
				final Ref<DCallReleasee> out_post_releasee_ptr/*=NULL*/, 
				int /*ddependencycount_t*/ dependencies_count, 
				DCallReleasee dependent_releasee/*=NULL*/, 
				dxICallWait call_wait/*=NULL*/, 
				dThreadedCallFunction call_func, CallContext call_context, 
				int /*dcallindex_t*/ instance_index)
		{
			dxThreadedJobInfo new_job = m_list_container.AllocateJobInfoFromPool();
			dIASSERT(new_job != null);

			new_job.AssignJobData(dependencies_count, dMAKE_RELEASEE_JOBINSTANCE(dependent_releasee), 
					(dxCallWait)call_wait, fault_accumulator_ptr, call_func, call_context, instance_index);

			if (out_post_releasee_ptr != null)
			{
				out_post_releasee_ptr.set(dMAKE_JOBINSTANCE_RELEASEE(new_job));
			}

			m_list_container.QueueJobForProcessing(new_job);

			if (dependencies_count == 0)
			{
				m_list_handler.ProcessActiveJobAddition();
			}
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void AlterJobDependenciesCount(
				DCallReleasee target_releasee, int /*ddependencychange_t*/ dependencies_count_change)
		{
			dIASSERT(dependencies_count_change != 0);

			dxThreadedJobInfo job_instance = dMAKE_RELEASEE_JOBINSTANCE(target_releasee);

			RefBoolean job_has_become_ready = new RefBoolean(false);
			m_list_container.AlterJobProcessingDependencies(job_instance, dependencies_count_change, job_has_become_ready);

			if (job_has_become_ready.get())
			{
				m_list_handler.ProcessActiveJobAddition();
			}
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void WaitJobCompletion(
				RefInt out_wait_status_ptr/*=NULL*/, 
				dxICallWait call_wait, final DThreadedWaitTime timeout_time_ptr/*=NULL*/)
		{
			dIASSERT(call_wait != null);

			m_list_handler.PrepareForWaitingAJobCompletion();

			boolean wait_status = ((dxCallWait)call_wait).PerformWaiting(timeout_time_ptr);
			dIASSERT(timeout_time_ptr != null || wait_status);

			if (out_wait_status_ptr != null)
			{
				out_wait_status_ptr.set( wait_status ? 1 : 0 );
			}
		}


		//template<class tJobListContainer, class tJobListHandler>
		//unsigned dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public int RetrieveActiveThreadsCount()
		{
			return m_list_handler.RetrieveActiveThreadsCount();
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void StickToJobsProcessing(DThreadReadyToServeCallback readiness_callback/*=NULL*/, 
				CallContext callback_context/*=NULL*/)
		{
			m_list_handler.StickToJobsProcessing(readiness_callback, callback_context);
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void ShutdownProcessing()
		{
			m_list_handler.ShutdownProcessing();
		}

		//template<class tJobListContainer, class tJobListHandler>
		//void dxtemplateThreadingImplementation<tJobListContainer, tJobListHandler>::
		@Override
		public void CleanupForRestart()
		{
			m_list_handler.CleanupForRestart();
		}


	};





	//TODO TZ implement for multi-threading
	//#if dBUILTIN_THREADING_IMPL_ENABLED
	//
	///************************************************************************/
	///* Implementation of dxtemplateJobListThreadedHandler                   */
	///************************************************************************/
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::ProcessActiveJobAddition()
	//{
	//    ActivateAnIdleThread();
	//}
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::PrepareForWaitingAJobCompletion()
	//{
	//    // Do nothing
	//}
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//unsigned dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::RetrieveActiveThreadsCount()
	//{
	//    return GetActiveThreadsCount();
	//}
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::StickToJobsProcessing(dxThreadReadyToServeCallback *readiness_callback/*=NULL*/, void *callback_context/*=NULL*/)
	//{
	//    RegisterAsActiveThread();
	//
	//    if (readiness_callback != NULL)
	//    {
	//        (*readiness_callback)(callback_context);
	//    }
	//
	//    PerformJobProcessingUntilShutdown();
	//
	//    UnregisterAsActiveThread();
	//}
	//
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::PerformJobProcessingUntilShutdown()
	//{
	//    while (true)
	//    {
	//        // It is expected that new jobs will not be queued any longer after shutdown had been requested
	//        if (IsShutdownRequested() && m_job_list_ptr->IsJobListReadyForShutdown())
	//        {
	//            break;
	//        }
	//
	//        PerformJobProcessingSession();
	//
	//        // It is expected that new jobs will not be queued any longer after shutdown had been requested
	//        if (IsShutdownRequested() && m_job_list_ptr->IsJobListReadyForShutdown())
	//        {
	//            break;
	//        }
	//
	//        BlockAsIdleThread();
	//    }
	//}
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::PerformJobProcessingSession()
	//{
	//    dxThreadedJobInfo *current_job = NULL;
	//    bool job_result = false;
	//
	//    while (true)
	//    {
	//        bool last_job_flag;
	//        current_job = m_job_list_ptr->ReleaseAJobAndPickNextPendingOne(current_job, job_result, &dxCallWait::AbstractSignalTheWait, last_job_flag);
	//
	//        if (!current_job)
	//        {
	//            break;
	//        }
	//
	//        if (!last_job_flag)
	//        {
	//            ActivateAnIdleThread();
	//        }
	//
	//        job_result = current_job->InvokeCallFunction();
	//    }
	//}
	//
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::BlockAsIdleThread()
	//{
	//    m_processing_wakeup.WaitWakeup(NULL);
	//}
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::ActivateAnIdleThread()
	//{
	//    m_processing_wakeup.WakeupAThread();
	//}
	//
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::ShutdownProcessing()
	//{
	//    m_shutdown_requested = true;
	//    m_processing_wakeup.WakeupAllThreads();
	//}
	//
	//template<class tThreadWakeup, class tJobListContainer>
	//void dxtemplateJobListThreadedHandler<tThreadWakeup, tJobListContainer>::CleanupForRestart()
	//{
	//    m_shutdown_requested = false;
	//    m_processing_wakeup.ResetWakeup();
	//}
	//
	//
	//#endif // #if dBUILTIN_THREADING_IMPL_ENABLED




}
//#endif // #ifndef _ODE_THREADING_IMPL_TEMPLATES_H_
