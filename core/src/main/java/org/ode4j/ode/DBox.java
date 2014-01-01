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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

public interface DBox extends DGeom {

	
	/**
	 * Set the side lengths of the given box.
	 *
	 * @param lx      the length of the box along the X axis
	 * @param ly      the length of the box along the Y axis
	 * @param lz      the length of the box along the Z axis
	 *
	 * @see #getLengths()
	 */
	void setLengths (double lx, double ly, double lz);
	

	/**
	 * Get the side lengths of a box.
	 *
	 * @param result  the returned side lengths
	 *
	 * @see #setLengths(DVector3C)
	 */
	void getLengths (DVector3 result);
	

	/**
	 * Set the side lengths of the given box.
	 *
	 * @param sides   the lengths of the box along the X, Y and Z axes
	 *
	 * @see #getLengths()
	 */
	void setLengths (DVector3C sides);
	
	
	/**
	 * Get the side lengths of a box.
	 *
	 * @return The returned side lengths.
	 *
	 * @see #setLengths(DVector3C)
	 */
	DVector3C getLengths ();

	
	/**
	 * Return the depth of a point in a box.
	 *
	 * @param p    the X, Y and Z coordinates of the point to test.
	 *
	 * @return The depth of the point. Points inside the box will have a
	 * positive depth, points outside it will have a negative depth, and points
	 * on the surface will have a depth of zero.
	 */
	double getPointDepth(DVector3C p);

}
