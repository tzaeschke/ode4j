/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
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

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;
import java.util.concurrent.atomic.AtomicReference;

public class Atomics {

	public static boolean ThrsafeCompareExchange(AtomicInteger paoDestination, int aoComparand, int aoExchange)
	{
	    return paoDestination.compareAndSet(aoComparand, aoExchange);
	}

	public static boolean ThrsafeCompareExchange(AtomicIntegerArray paoDestination, int arrayOffset, int aoComparand, int aoExchange)
	{
		return paoDestination.compareAndSet(arrayOffset, aoComparand, aoExchange);
	}

	public static int ThrsafeExchange(AtomicInteger paoDestination, int aoExchange)
	{
		return paoDestination.getAndSet(aoExchange);
	}

	public static <T> boolean ThrsafeCompareExchangePointer(AtomicReference<T> papDestination, T apComparand, T apExchange)
	{
		return papDestination.compareAndSet(apComparand, apExchange);
	}

	static <T> T ThrsafeExchangePointer(AtomicReference<T> papDestination, T apExchange)
	{
		return papDestination.getAndSet(apExchange);
	}

	public static int ThrsafeIncrementIntUpToLimit(AtomicInteger storagePointer, int limitValue)
	{
	    int resultValue;
	    while (true) {
	        resultValue = storagePointer.get();
			// The ">=" comparison is used here to allow continuing incrementing the destination
			// without waiting for all the threads to pass the barrier of checking its value
			if (resultValue >= limitValue) {
				resultValue = limitValue;
	            break;
	        }
	        if (ThrsafeCompareExchange(storagePointer, resultValue, resultValue + 1)) {
	            break;
	        }
	    }
	    return resultValue;
	}

	public static int ThrsafeIncrementSizeUpToLimit(AtomicInteger storagePointer, int limitValue)
	{
	    int resultValue;
	    while (true) {
	        resultValue = storagePointer.get();
			// The ">=" comparison is not required here at present ("==" could be used).
			// It is just used this way to match the other function above.
			if (resultValue >= limitValue) {
				resultValue = limitValue;
	            break;
	        }
	        //if (ThrsafeCompareExchangePointer((volatile atomicptr *)storagePointer, (atomicptr)resultValue, (atomicptr)(resultValue + 1))) {
	        //TODO (TZ)
	        //System.out.println("Is this right? Using Int instead of Pointer?");
	        if (ThrsafeCompareExchange(storagePointer, resultValue, (resultValue + 1))) {
	        	break;
	        }
	    }
	    return resultValue;
	}


	public static int ThrsafeAdd(AtomicInteger paoDestination, int aoAddend){
		return paoDestination.addAndGet(aoAddend);
	}

    public static int ThrsafeExchangeAdd(AtomicInteger paoDestination, int aoAddend) {
        return paoDestination.getAndAdd(aoAddend);
    }

	private Atomics() {}
}
