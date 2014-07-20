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
import java.util.concurrent.atomic.AtomicReference;

public class ThreadingUtils {
	//#if !dTHREADING_INTF_DISABLED

//	static inline 
//	bool ThrsafeCompareExchange(volatile atomicord32 *paoDestination, atomicord32 aoComparand, atomicord32 aoExchange)
//	{
//	    return AtomicCompareExchange(paoDestination, aoComparand, aoExchange);
//	}
//
//	static inline 
//	atomicord32 ThrsafeExchange(volatile atomicord32 *paoDestination, atomicord32 aoExchange)
//	{
//	    return AtomicExchange(paoDestination, aoExchange);
//	}
//
//	static inline 
//	bool ThrsafeCompareExchangePointer(volatile atomicptr *papDestination, atomicptr apComparand, atomicptr apExchange)
//	{
//	    return AtomicCompareExchangePointer(papDestination, apComparand, apExchange);
//	}
//
//	static inline 
//	atomicptr ThrsafeExchangePointer(volatile atomicptr *papDestination, atomicptr apExchange)
//	{
//	    return AtomicExchangePointer(papDestination, apExchange);
//	}
//
//
//	#else // #if dTHREADING_INTF_DISABLED

//	static inline 
//	bool ThrsafeCompareExchange(volatile atomicord32 *paoDestination, atomicord32 aoComparand, atomicord32 aoExchange)
	static 
	boolean ThrsafeCompareExchange(AtomicInteger paoDestination, int aoComparand, int aoExchange)
	{
		//return (*paoDestination == aoComparand) ? ((*paoDestination = aoExchange), true) : false;
	    return paoDestination.compareAndSet(aoComparand, aoExchange);
	}

	//static inline 
	//atomicord32 ThrsafeExchange(volatile atomicord32 *paoDestination, atomicord32 aoExchange)
	public static  
	int ThrsafeExchange(AtomicInteger paoDestination, int aoExchange)
	{
//		atomicord32 aoDestinationValue = *paoDestination;
//		*paoDestination = aoExchange;
//		return aoDestinationValue;
		return paoDestination.getAndSet(aoExchange);
	}

//	static inline 
//	bool ThrsafeCompareExchangePointer(volatile atomicptr *papDestination, atomicptr apComparand, atomicptr apExchange)
	public static <T>  
	boolean ThrsafeCompareExchangePointer(AtomicReference<T> papDestination, T apComparand, T apExchange)
	{
		//return (*papDestination == apComparand) ? ((*papDestination = apExchange), true) : false;
	    //return (papDestination == apComparand) ? ((papDestination = apExchange), true) : false;
		return papDestination.compareAndSet(apComparand, apExchange);
	}

//	static inline 
//	atomicptr ThrsafeExchangePointer(volatile atomicptr *papDestination, atomicptr apExchange)
	static 
	<T> T ThrsafeExchangePointer(AtomicReference<T> papDestination, T apExchange)
	{
//	    atomicptr apDestinationValue = *papDestination;
//	    *papDestination = apExchange;
//	    return apDestinationValue;
		return papDestination.getAndSet(apExchange);
	}


	//#endif // #if dTHREADING_INTF_DISABLED


//	static inline 
//	unsigned int ThrsafeIncrementIntUpToLimit(volatile unsigned int *storagePointer, unsigned int limitValue)
	public static 
	int ThrsafeIncrementIntUpToLimit(AtomicInteger storagePointer, int limitValue)
	{
//	    unsigned int resultValue;
//	    while (true) {
//	        resultValue = *storagePointer;
//	        if (resultValue == limitValue) {
//	            break;
//	        }
//	        if (ThrsafeCompareExchange((volatile atomicord32 *)storagePointer, (atomicord32)resultValue, (atomicord32)(resultValue + 1))) {
//	            break;
//	        }
//	    }
//	    return resultValue;
	    int resultValue;
	    while (true) {
	        resultValue = storagePointer.get();
	        if (resultValue == limitValue) {
	            break;
	        }
	        if (ThrsafeCompareExchange(storagePointer, resultValue, resultValue + 1)) {
	            break;
	        }
	    }
	    return resultValue;
	}

//	static inline 
//	size_t ThrsafeIncrementSizeUpToLimit(volatile size_t *storagePointer, size_t limitValue)
	public static 
	int ThrsafeIncrementSizeUpToLimit(AtomicInteger storagePointer, int limitValue)
	{
//	    size_t resultValue;
//	    while (true) {
//	        resultValue = *storagePointer;
//	        if (resultValue == limitValue) {
//	            break;
//	        }
//	        if (ThrsafeCompareExchangePointer((volatile atomicptr *)storagePointer, (atomicptr)resultValue, (atomicptr)(resultValue + 1))) {
//	            break;
//	        }
//	    }
//	    return resultValue;
	    int resultValue;
	    while (true) {
	        resultValue = storagePointer.get();
	        if (resultValue == limitValue) {
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



//	#endif // _ODE_THREADINGUTILS_H_

}
