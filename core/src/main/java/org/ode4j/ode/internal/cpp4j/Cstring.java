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

public class Cstring extends Ctype {

	public static void memcpy(double[] to, double[] from, int count) {
		System.arraycopy(from, 0, to, 0, count);
	}
	
	public static void memcpy(double[] to, int i, double[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}
	
	public static void memmove(double[] to, double[] from, int count) {
		System.arraycopy(from, 0, to, 0, count);
	}

	public static void memmove(double[] to, int i, double[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}

	public static void memmove(int[] to, int i, int[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}

	/**
	 * 
	 * @param data data
	 * @param c value to set
	 * @param l number of values to set
	 * @deprecated Do not user for c=0
	 */
	@Deprecated
    public static void memset(int[] data, int c, int l) {
		for (int i = 0; i < l; i++) {
			data[i] = c;
		}
	}
	
	public static int strcmp(char[] s, String string) {
		return new String(s).compareTo(string);
	}

	public static int strcmp(String s, String string) {
		return s.compareTo(string);
	}

	/**
	 * Returns the string length. The null terminator is not counted.
	 * @param s String to get length of
	 * @return length
	 */
	public static int strlen(char[] s) {
		for (int i = 0; i < s.length; i++) {
			if (s[i] == '\0') {
				return i; 
			}
		}
		return s.length;
//		throw new IllegalStateException("String has no delimitor (l=" + 
//				s.length + "): \"" + new String(s) + "\"");
	}

	/**
	 * @param s s
	 * @return length of the string
	 */
	public static int strlen(String s) {
		return strlen(s.toCharArray());
	}

	protected Cstring() {}
}
