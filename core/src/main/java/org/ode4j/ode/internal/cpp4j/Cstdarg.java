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
 * Emulator for stdio.
 *
 * @author Tilmann Zaeschke
 */
public class Cstdarg extends Cstdio {

	/**
	 */
	public static class va_list {
		final Object[] l;
		/**
		 * @param varArgsOfCallingMethod
		 */
		public va_list(Object[] varArgsOfCallingMethod) {
			l = varArgsOfCallingMethod;
		}
	}
	
	/**
	 * This method does nothing.
	 * @param ap
	 * @param argToStartAfter_isIgnored
	 */
	public static void va_start(va_list ap, Object argToStartAfter_isIgnored ) {
		
	}
	
	/**
	 * This method does nothing.
	 * @param ap
	 */
	public static void va_end(va_list ap) {
		
	}
}
