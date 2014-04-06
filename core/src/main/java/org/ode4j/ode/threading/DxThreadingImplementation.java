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
public abstract class DxThreadingImplementation extends DThreadingImplementation {

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
	 * @returns ID of object allocated or NULL on failure
	 * 
	 * @see DThreadingImplementation#allocateMultiThreadedImplementation()
	 * @see DThreadingImplementation#freeImplementation()
	 */
	/*ODE_API */
	public static DThreadingImplementation allocateSelfThreadedImplementation() {
		throw new UnsupportedOperationException();
	}


}
