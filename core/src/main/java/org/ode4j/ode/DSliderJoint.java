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

public interface DSliderJoint extends DJoint {

	/**
	 * Set the joint axis.
	 */
	void setAxis (double x, double y, double z);
	
	
	/**
	 * Set the joint axis.
	 */
	void setAxis (DVector3C a);

	
	/**
	 * Get the slider axis.
	 */
	void getAxis (DVector3 result);

	
	/**
	 * Get the slider linear position (i.e. the slider's extension).
	 * <p>
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * <p>
	 * The position is the distance, with respect to the zero position,
	 * along the slider axis of body 1 with respect to
	 * body 2. (A NULL body is replaced by the world).
	 */
	double getPosition();
	
	
	/**
	 * Get the slider linear position's time derivative.
	 */
	double getPositionRate();

	
	/**
	 * Applies the given force in the slider's direction.
	 * <p>
	 * That is, it applies a force with specified magnitude, in the direction of
	 * slider's axis, to body1, and with the same magnitude but opposite
	 * direction to body2.  This function is just a wrapper for dBodyAddForce().
	 */
	void addForce (double force);
	void setParamFMax(double d);
	void setParamLoStop(double d);
	void setParamHiStop(double d);
	void setParamVel(double d);
	void setParamBounce(double d);
	
	
	/**
	 */
	void setAxisDelta(double x, double y, double z, 
			double dx, double dy, double dz);

	/**
	 * Set joint parameter.
	 */
	@Override
	void setParam (PARAM_N parameter, double value);

	
	/**
	 * Get joint parameter.
	 */
	@Override
	double getParam (PARAM_N parameter);

}
