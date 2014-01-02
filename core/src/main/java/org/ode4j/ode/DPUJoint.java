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

public interface DPUJoint extends DJoint {

	/**
	 * Set anchor.
	 */
	void setAnchor (double x, double y, double z);


	/**
	 * Set anchor.
	 */
	void setAnchor (DVector3C a);

	
	/**
	 * Set the axis for the first axis or the universal articulation.
	 */
	void setAxis1 (double x, double y, double z);

	
	/**
	 * Set the axis for the first axis or the universal articulation.
	 */
	void setAxis1 (DVector3C a);

	
	/**
	 * Set the axis for the second axis or the universal articulation.
	 */
	void setAxis2 (double x, double y, double z);

	
	/**
	 * Set the axis for the second axis or the universal articulation.
	 */
	void setAxis2 (DVector3C a);
	
	
	/**
	 * Set the axis for the prismatic articulation.
	 */
	void setAxis3 (double x, double y, double z);

	
	/**
	 * Set the axis for the prismatic articulation.
	 */
	void setAxis3 (DVector3C a);
	
	
	/**
	 * Set the axis for the prismatic articulation.
	 * 
	 * <p>NOTE: This function was added for convenience it is the same as
	 *       dJointSetPUAxis3
	 */
	void setAxisP (double x, double y, double z);


	/**
	 * Set the axis for the prismatic articulation.
	 * 
	 * <p>NOTE: This function was added for convenience it is the same as
	 *       dJointSetPUAxis3
	 */
	void setAxisP (DVector3C a);


	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 */
	void getAnchor (DVector3 result);


	/**
	 * Get the first axis of the universal component of the joint.
	 */
	void getAxis1 (DVector3 result);


	/**
	 * Get the second axis of the Universal component of the joint.
	 */
	void getAxis2 (DVector3 result);


	/**
	 * Get the prismatic axis.
	 */
	void getAxis3 (DVector3 result);


	/**
	 * Get the prismatic axis.
	 *
	 * <p>NOTE: This function was added for convenience it is the same as
	 *       dJointGetPUAxis3
	 */
	void getAxisP (DVector3 result);


//	/**
//	 * Get both angles at the same time.
//	 *
//	 * <p>NOTE: This function combine dJointGetPUAngle1 and dJointGetPUAngle2 together
//	 *       and try to avoid redundant calculation
//	 *
//	 * @param joint   The Prismatic universal joint for which we want to calculate the angles
//	 * @param angle1  The angle between the body1 and the axis 1
//	 * @param angle2  The angle between the body2 and the axis 2
//	 */
	/**
	 * Get angle between the body1 and the axis 1.
	 */
	double getAngle1();

	
	/**
	 * Get time derivative of angle1.
	 */
	double getAngle1Rate();
	
	
	/**
	 * Get angle between the body2 and the axis 2.
	 */
	double getAngle2();
	
	
	/**
	 * Get time derivative of angle2.
	 */
	double getAngle2Rate();


	/**
	 * Get the PU linear position (i.e. the prismatic's extension).
	 * <p>
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * <p>
	 * The position is the "oriented" length between the
	 * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
	 */
	double getPosition();

	
	/**
	 * Get the PR linear position's time derivative.
	 */
	double getPositionRate();

//	/**
//	 * Applies the torque about the rotoide axis of the PU joint.
//	 *
//	 * That is, it applies a torque with specified magnitude in the direction
//	 * of the rotoide axis, to body 1, and with the same magnitude but in opposite
//	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
//	 */
//  TODO there is no implementation. Use dBodyAddTorque???? Or leave unimplemented?
//	void addTorque (double torque);


	/**
	 * Set the PU anchor as if the 2 bodies were already at [dx, dy, dz] apart.
	 * <p>
	 * This function initialize the anchor and the relative position of each body
	 * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
	 * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
	 * axis is set).
	 * Ex: <br>
	 * <code> <br>
	 * dReal offset = 3; <br>
	 * dVector3 axis; <br>
	 * dJointGetPUAxis(jId, axis); <br>
	 * dJointSetPUAnchor(jId, 0, 0, 0); <br>
	 * // If you request the position you will have: dJointGetPUPosition(jId) == 0 <br>
	 * dJointSetPUAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset); <br>
	 * // If you request the position you will have: dJointGetPUPosition(jId) == offset <br>
	 * </code>
	 * 
	 * @param x The X position of the anchor point in world frame
	 * @param y The Y position of the anchor point in world frame
	 * @param z The Z position of the anchor point in world frame
	 * @param dx A delta to be subtracted to the X position as if the anchor was set
	 *           when body1 was at current_position[X] - dx
	 * @param dy A delta to be subtracted to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] - dy
	 * @param dz A delta to be subtracted to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] - dz
	 */
	void setAnchorOffset(double x, double y, double z, double dx, double dy,
			double dz);


	/**
	 * Set joint parameter.
	 *
	 * <p>NOTE: parameterX where X equal 2 refer to parameter for second axis of the
	 *       universal articulation.
	 * <p> NOTE: parameterX where X equal 3 refer to parameter for prismatic
	 *       articulation.
	 */
	@Override
	void setParam (PARAM_N parameter, double value);

	
	/**
	 * Get joint parameter.
	 */
	@Override
	double getParam (PARAM_N parameter);

}
