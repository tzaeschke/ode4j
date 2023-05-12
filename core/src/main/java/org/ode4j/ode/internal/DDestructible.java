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


/**
 * This base class should be extended by all classes that implement
 * a C++ destructor.
 * To implement the deterministic behaviour of C++ constructors,
 * each sub-class should migrate its destructor as follows:
 * <br>
 * <code>
 * //~myClass() <br>
 * \@Override   <br>
 * protected void DESTRUCTOR() {   <br>
 *   //Do anything the desctructor used to do  <br>
 *   super.DESTRUCTOR(); <br>
 * } <br>
 * </code>
 *
 * @author Tilmann Zaeschke
 *
 */
public abstract class DDestructible {
	
//	private static final boolean DEBUG_FINALIZE = false; 
	private volatile boolean isDestructed = false;  //TZ to assure timely destruction
	
	//TODO use this for debugging:
	private static volatile long counter = 0;
	private long id = ++counter;
//	private RuntimeException creator;
	protected DDestructible() {
//		creator = new RuntimeException();
//		System.err.println("Creating object: " + _id + " " +
//				getClass()
//				);
	}
	
	/** 
	 * (TZ) Replacement for the C++ ~ destructor construct. This
	 * should be called 
	 */
	protected void DESTRUCTOR() {
		if (isDestructed) {
			System.err.println("WARNING Object was already destructed.");
			new RuntimeException().printStackTrace();
		}
		isDestructed = true;
		//System.err.println("CCC");
	}
	
	
//	@Override
//	protected void finalize() throws Throwable {
//		if (DEBUG_FINALIZE) { 
//			if (!_isDestructed) {
//				_isDestructed = true;
//	//			System.err.println("WARNING Object has not been destructed! " +
//	//					this.getClass()
//	//					);
//				System.err.println("W--" + _id
//						);
//			}
//			super.finalize();
//		}
//	}
	
	@Override
	public String toString() {
		return "ID=" + id + " (" + getClass().getName() + ")";
	}
}
