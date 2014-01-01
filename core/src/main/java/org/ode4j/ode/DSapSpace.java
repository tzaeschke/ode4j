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
package org.ode4j.ode;

/**
 * Sweep and prune space.
 *
 * @author Tilmann Zaeschke
 *
 */
public interface DSapSpace extends DSpace {

	/** Order XZY or ZXY usually works best, if your Y is up. */
	public enum  AXES {
		/**  0,1,2 -> 100100 -> 36  */
		XYZ ((0)|(1<<2)|(2<<4)),
		/**  0,2,1 ->  11000 -> 24  */
		XZY ((0)|(2<<2)|(1<<4)),
		/**  1,0,2 -> 100001 -> 33  */
		YXZ ((1)|(0<<2)|(2<<4)),
		/**  1,2,0 ->   1001 ->  9  */
		YZX ((1)|(2<<2)|(0<<4)),
		/**  2,0,1 ->  10010 -> 18  */
		ZXY ((2)|(0<<2)|(1<<4)),
		/**  2,1,0 ->    110 ->  6  */
		ZYX ((2)|(1<<2)|(0<<4));
		private final int _code;
		AXES(int i) {
			_code = i;
		}
		/**
		 * @return axis code
		 */
		public int getCode() {
			return _code;
		}
	}
}
