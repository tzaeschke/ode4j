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

import org.ode4j.math.DVector3C;

public interface DSphere extends DGeom {

	
	/**
	 * Set the radius of a sphere geom.
	 *
	 * @param radius  the new radius.
	 *
	 * @see #getRadius
	 */
	void setRadius (double radius);
	
	
	/**
	 * Retrieves the radius of a sphere geom.
	 *
	 * @see #setRadius(double)
	 */
	double getRadius();

	
	/**
	 * Calculate the depth of the a given point within a sphere.
	 *
	 * @param p       the X, Y and Z coordinate of the point.
	 *
	 * @return The depth of the point. Points inside the sphere will have a
	 * positive depth, points outside it will have a negative depth, and points
	 * on the surface will have a depth of zero.
	 */
	double getPointDepth(DVector3C p);
}
