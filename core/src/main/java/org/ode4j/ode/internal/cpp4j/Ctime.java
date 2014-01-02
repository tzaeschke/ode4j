/*************************************************************************
 *                                                                       *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal.cpp4j;

/**
 * Emulator for time.
 *
 * @author Tilmann Zaeschke
 */
public class Ctime extends Ctype {
	
	/**
	 * Time type.
	 * 
	 * Type capable of representing times and support arithmetical operations.
	 * This type is returned by the time function and is used as parameter by 
	 * some other functions of the <ctime> header.
	 * 
	 * It is almost universally expected to be an integral value representing 
	 * the number of seconds elapsed since 00:00 hours, Jan 1, 1970 UTC. This 
	 * is due to historical reasons, since it corresponds to a unix timestamp, 
	 * but is widely implemented in C libraries across all platforms.
	 *
	 * @author Tilmann Zaeschke
	 * @deprecated In Java, simply use 'long'.
	 */
	public static class time_t {
		/** Current time */
		public long seconds;

		/**
		 * Set time.
		 * @param n
		 */
		public time_t(int n) {
			seconds = n;
		}
	}
	
	/**
	 * Get current time.
	 * Get the current calendar time as a time_t object.
	 * The function returns this value, and if the argument is not a null
	 * pointer, the value is also set to the object pointed by timer.
	 * 
	 * @param timer Pointer to an object of type time_t, where the time 
	 * value is stored.
	 * Alternatively, this parameter can be a null pointer, in which case 
	 * the parameter is not used, but a time_t object is still returned by 
	 * the function.
	 * @return The current calendar time as a time_t object.
	 * If the argument is not a null pointer, the return value is the same 
	 * as the one stored in the location pointed by the argument.
	 * If the function could not retrieve the calendar time, it returns 
	 * a -1 value.
	 */
	public static time_t time(time_t timer) {
		int n = (int)( System.currentTimeMillis()/1000. );
		if (timer != null) {
			timer.seconds = n;
			return timer; 
		}
		return new time_t(n);
	}
}
