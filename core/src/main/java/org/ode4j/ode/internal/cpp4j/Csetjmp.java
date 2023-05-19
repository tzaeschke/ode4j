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

import org.ode4j.ode.internal.cpp4j.java.CppLongJump;

/**
 * Emulator for long jumps (setjmp).
 *
 * @author Tilmann Zaeschke
 */
public class Csetjmp extends Cstdio {

	/**
	 *
	 */
	public static class jmp_buf {
		int _ret = 0;
		public jmp_buf() {}
	}
	
	/**
	 * @param jump_buffer jump buffer
	 * @param i i
	 */
	public static void longjmp(jmp_buf jump_buffer, int i) {
		throw new CppLongJump("" + i);
//		NIW();
//		jump_buffer._ret = i;
	}

	/**
	 * @param jump_buffer jump buffer
	 * @return jump parameter i.
	 */
	public static int setjmp(jmp_buf jump_buffer) {
		NIW();
		return 0;//jump_buffer._ret;
	}
	
	private static void NIW() {
		String f = new Exception().getStackTrace()[1].toString();
		System.out.println("WARNING: Not implemented: " + f);
	}

	protected Csetjmp() {}
}
