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

/**
 * ****************************************************************************<br>
 * Piston                                                                      <br>
 *                                                                             <br>
 * ****************************************************************************<br>
 * Component of a Piston joint
 * <PRE>
 *                              |- Anchor point
 *      Body_1                  |                       Body_2
 *      +---------------+       V                       +------------------+
 *     /               /|                             /                  /|
 *    /               / +       |--      ______      /                  / +
 *   /      x        /./........x.......(_____()..../         x        /.......> axis
 *  +---------------+ /         |--                +------------------+ /
 *  |               |/                             |                  |/
 *  +---------------+                              +------------------+
 *          |                                                 |
 *          |                                                 |
 *          |------------------> <----------------------------|
 *              anchor1                  anchor2
 *
 *
 * </PRE>
 *
 * When the prismatic joint as been elongated (i.e. dJointGetPistonPosition)
 * return a value >  0
 * <PRE>
 *                                   |- Anchor point
 *      Body_1                       |                       Body_2
 *      +---------------+            V                       +------------------+
 *     /               /|                                  /                  /|
 *    /               / +            |--      ______      /                  / +
 *   /      x        /./........_____x.......(_____()..../         x        /.......> axis
 *  +---------------+ /              |--                +------------------+ /
 *  |               |/                                  |                  |/
 *  +---------------+                                   +------------------+
 *          |                                                      |
 *          |                                                      |
 *          |----------------.>      <----------------------------|
 *              anchor1         |----|         anchor2
 *                                ^
 *                                |-- This is what dJointGetPistonPosition will
 *                                    return
 * </PRE>
 * ****************************************************************************
 */
public interface DPistonJoint extends DJoint {

	/**
	 * Set the joint anchor.
	 */
	void setAnchor (double x, double y, double z);

	
	/**
	 * Set the joint anchor.
	 */
	void setAnchor (DVector3C a);

	
	/**
	 * Get the joint anchor.
	 * <p>
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2 in direction perpendicular
	 * to the prismatic axis.
	 */
	void getAnchor (DVector3 result);
	
	
	/**
	 * Get the joint anchor wrt body 2.
	 * <p>
	 * This returns the point on body 2. You can think of a Piston
	 * joint as trying to keep the result of dJointGetPistonAnchor() and
	 * dJointGetPistonAnchor2() the same in the direction perpendicular to the
	 * prismatic axis. If the joint is perfectly satisfied,
	 * this function will return the same value as dJointGetPistonAnchor() to
	 * within roundoff errors. dJointGetPistonAnchor2() can be used, along with
	 * dJointGetPistonAnchor(), to see how far the joint has come apart.
	 */
	void getAnchor2 (DVector3 result);

	
	/**
	 * Set the Piston anchor as if the 2 bodies were already at [dx,dy, dz] apart.
	 * <p>
	 * This function initialize the anchor and the relative position of each body
	 * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
	 * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
	 * axis is set).
	 * Ex:  <br>
	 * <code>
	 * double offset = 3;  <br>
	 * dVector3 axis;  <br>
	 * dJointGetPistonAxis(jId, axis);  <br>
	 * dJointSetPistonAnchor(jId, 0, 0, 0);  <br>
	 * // If you request the position you will have: dJointGetPistonPosition(jId) == 0  <br>
	 * dJointSetPistonAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);  <br>
	 * // If you request the position you will have: dJointGetPistonPosition(jId) == offset  <br>
	 * </code>
	 * 
	 * @param xyz The X, Y and Z position of the anchor point in world frame
	 * @param dx A delta to be substracted to the X position as if the anchor was set
	 *           when body1 was at current_position[X] - dx
	 * @param dy A delta to be substracted to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] - dy
	 * @param dz A delta to be substracted to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] - dz
	 */
	void setAnchorOffset(DVector3C xyz, double dx, double dy, double dz);

	/**
	 * Set the joint axis.
	 */
	void setAxis (double x, double y, double z);

	
	/**
	 * Set the joint axis.
	 */
	void setAxis (DVector3C a);

	
	/**
	 * Get the prismatic axis (This is also the rotoide axis.
	 */
	void getAxis (DVector3 result);


	/**
	 * Get the Piston linear position (i.e. the piston's extension).
	 * <p>
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 */
	double getPosition();

	
	/**
	 * Get the piston linear position's time derivative.
	 */
	double getPositionRate();

	/**
	 * Applies the given force in the slider's direction.
	 * <p>
	 * That is, it applies a force with specified magnitude, in the direction of
	 * prismatic's axis, to body1, and with the same magnitude but opposite
	 * direction to body2.  This function is just a wrapper for dBodyAddForce().
	 */
	void addForce (double force);
	
	double getParamLoStop2();
	double getParamHiStop2();
	void setParamLoStop2(double d);
	void setParamHiStop2(double d);

	
	/**
	 * Get the Piston angular position (i.e. the  twist between the 2 bodies).
	 * <p>
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 */
	double getAngle();

	
	/**
	 * Get the piston angular position's time derivative.
	 */
	double getAngleRate();


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
