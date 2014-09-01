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

import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DThreadingFunctionsInfo;

/**
 * 
 * @author Tilmann Zäschke
 * @Deprecated Not supported in ode4j.
 */
public abstract class DThreadingImplementation {
	
	/**
	 * Allocates built-in multi-threaded threading implementation object.
	 *
	 * A multi-threaded implementation is a type of implementation that has to be 
	 * served with a thread pool. The thread pool can be either the built-in ODE object
	 * or set of external threads that dedicate themselves to this purpose and stay
	 * in ODE until implementation releases them.
	 * 
	 * @return ID of object allocated or NULL on failure
	 * 
	 * @see dThreadingThreadPoolServeMultiThreadedImplementation
	 * @see dExternalThreadingServeMultiThreadedImplementation
	 * @see dThreadingFreeImplementation
	 */
//	public static DThreadingImplementation dThreadingAllocateSelfThreadedImplementation() {
//		throw new UnsupportedOperationException();
//	};
	//TZ see DxThreadingImplementation

	/**
	 * Retrieves the functions record of a built-in threading implementation.
	 *
	 * The implementation can be the one allocated by ODE (from @c dThreadingAllocateMultiThreadedImplementation). 
	 * Do not use this function with self-made custom implementations - 
	 * they should be bundled with their own set of functions.
	 * 
	 * @return Pointer to associated functions structure
	 * 
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 */
	public abstract DThreadingFunctionsInfo dThreadingImplementationGetFunctions();

	/**
	 * Requests a built-in implementation to release threads serving it.
	 *
	 * The function unblocks threads employed in implementation serving and lets them 
	 * return to from where they originate. It's the responsibility of external code 
	 * to make sure all the calls to ODE that might be dependent on given threading 
	 * implementation object had already returned before this call is made. If threading 
	 * implementation is still processing some posted calls while this function is 
	 * invoked the behavior is implementation dependent.
	 *
	 * This call is to be used to request the threads to be released before waiting 
	 * for them in host pool or before waiting for them to exit. Implementation object 
	 * must not be destroyed before it is known that all the serving threads have already 
	 * returned from it. If implementation needs to be reused after this function is called
	 * and all the threads have exited from it a call to @c dThreadingImplementationCleanupForRestart
	 * must be made to restore internal state of the object.
	 *
	 * If this function is called for self-threaded built-in threading implementation
	 * the call has no effect.
	 * 
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 * @see #dThreadingImplementationCleanupForRestart
	 */
	public void shutdownProcessing() {
		dThreadingImplementationShutdownProcessing();
	}
	abstract void dThreadingImplementationShutdownProcessing();


	/**
	 * Restores built-in implementation's state to let it be reused after shutdown.
	 *
	 * If a multi-threaded built-in implementation needs to be reused after a call
	 * to {@code dThreadingImplementationShutdownProcessing} this call is to be made to 
	 * restore object's internal state. After that the implementation can be served again.
	 *
	 * If this function is called for self-threaded built-in threading implementation
	 * the call has no effect.
	 * 
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 * @see #shutdownProcessing()
	 */
	public abstract void dThreadingImplementationCleanupForRestart();

	/**
	 * Deletes an instance of built-in threading implementation.
	 *
	 * WARNING: A care must be taken to make sure the implementation is unassigned
	 * from all the objects it was assigned to and that there are no more threads 
	 * serving it before attempting to call this function.
	 *
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 */
	public abstract void free();
	//public abstract void dThreadingFreeImplementation();


	//typedef void (dThreadReadyToServeCallback)(void *callback_context);
	public interface DThreadReadyToServeCallback {
		
	}

	/**
	 * An entry point for external threads that would like to serve a built-in 
	 * threading implementation object.
	 *
	 * A thread that calls this function remains blocked in ODE and serves implementation
	 * object {@code impl} until being released with 
	 * {@code dThreadingImplementationShutdownProcessing} call.
	 * This function can be used to provide external threads instead of ODE's built-in
	 * thread pools.
	 *
	 * The optional callback {@code readiness_callback} is called after the thread has reached 
	 * and has registered within the implementation. The implementation should not 
	 * be used until all dedicated threads register within it as otherwise it will not
	 * have accurate view of the execution resources available.
	 *
	 * @param readiness_callback Optional readiness callback to be called after thread enters the implementation
	 * @param callback_context A value to be passed as parameter to readiness callback
	 * 
	 * @see DxThreadingImplementation#dThreadingAllocateMultiThreadedImplementation
	 * @see #shutdownProcessing()
	 */
	public abstract void dExternalThreadingServeMultiThreadedImplementation( 
	  DThreadReadyToServeCallback readiness_callback/*=NULL*/, CallContext callback_context/*=NULL*/);

}
