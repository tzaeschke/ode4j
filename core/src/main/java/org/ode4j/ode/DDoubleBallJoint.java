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

public interface DDoubleBallJoint extends DJoint {

	/**
	 * Set anchor1 for double ball joint.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setAnchor1(double x, double y, double z);

	/**
	 * Set anchor2 for double ball joint.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setAnchor2(double x, double y, double z);

	/**
	 * Set anchor1 for double ball joint.
	 * @param xyz xyz
	 */
	void setAnchor1(DVector3C xyz);

	/**
	 * Set anchor2 for double ball joint.
	 * @param xyz xyz
	 */
	void setAnchor2(DVector3C xyz);

	/**
	 * Get anchor1 from double ball joint.
	 * @param result Vector containing the result
	 */
	void getAnchor1(DVector3 result);

	/**
	 * Get anchor2 from double ball joint.
	 * @param result Vector containing the result
	 */
	void getAnchor2(DVector3 result);

	/**
	 * Get the target distance from double ball joint.
	 * @return distance
	 */
	double getDistance();

	/**
	 * Set the target distance for the double ball joint.
	 * @param distance distance
	 */
	void setDistance(double distance);

	/**
	 * Set double ball joint parameter.
	 */
	@Override
	void setParam(PARAM_N parameter, double value);

	/**
	 * Get double ball joint parameter.
	 */
	@Override
	double getParam (PARAM_N parameter);
}
