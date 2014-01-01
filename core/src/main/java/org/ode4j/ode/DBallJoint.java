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

public interface DBallJoint extends DJoint {

	/**
	 * Set the joint anchor point.
	 * <p>
	 * The joint will try to keep this point on each body
	 * together. The input is specified in world coordinates.
	 */
	void setAnchor (double x, double y, double z);
	
	
	/**
	 * Set the joint anchor point.
	 */
	void setAnchor2(double x, double y, double z);
	
	
	/**
	 * Set the joint anchor point.
	 * <p>
	 * The joint will try to keep this point on each body
	 * together. The input is specified in world coordinates.
	 */
	void setAnchor (DVector3C a);
	
	
	/**
	 * Set the joint anchor point on body 2.
	 */
	void setAnchor2 (DVector3C a);
	
	
	/**
	 * Get the joint anchor point, in world coordinates.
	 * <p>
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 */
	void getAnchor (DVector3 result);
	
	
	/**
	 * Get the joint anchor point, in world coordinates.
	 * <p>
	 * This returns the point on body 2. You can think of a ball and socket
	 * joint as trying to keep the result of {@code dJointGetBallAnchor()} and
	 * {@code dJointGetBallAnchor2()} the same.  If the joint is perfectly satisfied,
	 * this function will return the same value as {@code dJointGetBallAnchor()} to
	 * within roundoff errors. {@code dJointGetBallAnchor2()} can be used, along with
	 * {@code dJointGetBallAnchor()}, to see how far the joint has come apart.
	 */
	void getAnchor2 (DVector3 result);


	/**
	 * Param setting for Ball joints
	 */
	@Override
	void setParam(PARAM_N parameter, double value);

	
	/**
	 * Get joint parameter.
	 */
	@Override
	double getParam (PARAM_N parameter);
}
