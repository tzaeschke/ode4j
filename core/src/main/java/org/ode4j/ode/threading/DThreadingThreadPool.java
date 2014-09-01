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


/**
 * 
 * @author Tilmann Zäschke
 * @Deprecated Not supported in ode4j.
 */
public abstract class DThreadingThreadPool {

	/**
	 * Creates an instance of built-in thread pool object that can be used to serve
	 * multi-threaded threading implementations.
	 *
	 * The threads allocated inherit priority of caller thread. Their affinity is not
	 * explicitly adjusted and gets the value the system assigns by default. Threads 
	 * have their stack memory fully committed immediately on start. On POSIX platforms 
	 * threads are started with all the possible signals blocked. Threads execute 
	 * calls to {@code dAllocateODEDataForThread} with {@code ode_data_allocate_flags} 
	 * on initialization.
	 *
	 * On POSIX platforms this function must be called with signals masked 
	 * or other measures must be taken to prevent reception of signals by calling thread 
	 * for the duration of the call.
	 * 
	 * @param thread_count Number of threads to start in pool
	 * @param stack_size Size of stack to be used for every thread or 0 for system default value
	 * @param ode_data_allocate_flags Flags to be passed to @c dAllocateODEDataForThread on behalf of each thread
	 * @return ID of object allocated or NULL on failure
	 *
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 * @see DThreadingImplementation#shutdownProcessing()
	 * @see #freeThreadPool()
	 */
	public static DThreadingThreadPool allocateThreadPool(int thread_count, 
	  int stack_size, int ode_data_allocate_flags, Object[][] reserved/*=NULL*/) {
		throw new UnsupportedOperationException();
	}

	/**
	 * Commands an instance of built-in thread pool to serve a built-in multi-threaded 
	 * threading implementation.
	 *
	 * A pool can only serve one threading implementation at a time. 
	 * Call {@code dThreadingImplementationShutdownProcessing} to release pool threads 
	 * from implementation serving and make them idle. Pool threads must be released 
	 * from any implementations before pool is attempted to be deleted.
	 *
	 * This function waits for threads to register within implementation before returning.
	 * So, after the function call exits the implementation can be used immediately.
	 * 
	 * @param impl Implementation ID of implementation to be served
	 *
	 * @see #allocateThreadPool(int, int, int, Object[][])
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 * @see DThreadingImplementation#shutdownProcessing()
	 */
	public abstract void serveMultiThreadedImplementation(DThreadingImplementation impl);

	/**
	 * Waits until all pool threads are released from threading implementation 
	 * they might be serving.
	 *
	 * The function can be used after a call to {@code dThreadingImplementationShutdownProcessing}
	 * to make sure all the threads have already been released by threading implementation 
	 * and it can be deleted or it can be cleaned up for restart and served by another pool
	 * or this pool's threads can be used to serve another threading implementation.
	 *
	 * Note that is it not necessary to call this function before pool destruction
	 * since {@code dThreadingFreeThreadPool} performs similar wait operation implicitly on its own.
	 * 
	 * It is OK to call this function even if pool was not serving any threading implementation
	 * in which case the call exits immediately with minimal delay.
	 * 
	 * @see #allocateThreadPool(int, int, int, Object[][])
	 * @see DThreadingImplementation#shutdownProcessing()
	 * @see #freeThreadPool()
	 */
	public abstract void waitIdleState();

	/**
	 * Deletes a built-in thread pool instance.
	 *
	 * The pool threads must be released from any implementations they might be serving
	 * before this function is called. Otherwise the call is going to block 
	 * and wait until pool's threads return.
	 * 
	 * @see #allocateThreadPool(int, int, int, Object[][])
	 * @see DThreadingImplementation#shutdownProcessing()
	 */
	public abstract void freeThreadPool();
	

}
