/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

public interface DHingeJoint extends DJoint {

	/**
	 * @brief Set hinge anchor parameter.
	 * @ingroup joints
	 */
	void setAnchor (double x, double y, double z);
	
	
	/**
	 * @brief Set hinge anchor parameter.
	 * @ingroup joints
	 */
	void setAnchor (DVector3C a);
	
	
	/**
	 * @brief Get the hinge anchor point, in world coordinates.
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	void getAnchor (DVector3 result);

	
	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return The point on body 2. If the joint is perfectly satisfied,
	 * this will return the same value as dJointGetHingeAnchor().
	 * If not, this value will be slightly different.
	 * This can be used, for example, to see how far the joint has come apart.
	 * @ingroup joints
	 */
	void getAnchor2 (DVector3 result);

	
	/**
	 * @brief Set hinge axis.
	 * @ingroup joints
	 */
	void setAxis (double x, double y, double z);
	
	
	/**
	 * @brief Set hinge axis.
	 * @ingroup joints
	 */
	void setAxis (DVector3C a);
	
	
	/**
	 * @brief get axis
	 * @ingroup joints
	 */
	void getAxis (DVector3 result);
	
	
	/**
	 * @brief Set the Hinge axis as if the 2 bodies were already at angle appart.
	 * @ingroup joints
	 * <p>
	 * This function initialize the Axis and the relative orientation of each body
	 * as if body1 was rotated around the axis by the angle value. <br>
	 * Ex:<br>
	 * <code>
	 * dJointSetHingeAxis(jId, 1, 0, 0); <br>
	 * // If you request the position you will have: dJointGetHingeAngle(jId) == 0 <br>
	 * dJointSetHingeAxisDelta(jId, 1, 0, 0, 0.23); <br>
	 * // If you request the position you will have: dJointGetHingeAngle(jId) == 0.23 <br>
	 * </code>

	 * @param j The Hinge joint ID for which the axis will be set
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param angle The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 *
	 * @note Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b angle appart.
	 * @warning Calling dJointSetHingeAnchor or dJointSetHingeAxis will reset the "zero"
	 *          angle position.
	 */
	void setAxisOffset(double x, double y, double z, double angle);


	/**
	 * @brief Get the hinge angle.
	 * <p>
	 * The angle is measured between the two bodies, or between the body and
	 * the static environment.
	 * The angle will be between -pi..pi.
	 * Give the relative rotation with respect to the Hinge axis of Body 1 with
	 * respect to Body 2.
	 * When the hinge anchor or axis is set, the current position of the attached
	 * bodies is examined and that position will be the zero angle.
	 * @ingroup joints
	 */
	double getAngle();
	
	
	/**
	 * @brief Get the hinge angle time derivative.
	 * @ingroup joints
	 */
	double getAngleRate();

	
	/**
	 * @brief Applies the torque about the hinge axis.
	 * <p>
	 * That is, it applies a torque with specified magnitude in the direction
	 * of the hinge axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 * @ingroup joints
	 */
	void addTorque (double torque);
	
	
	void setParamFMax(double d);
	void setParamVel(double cos);
	void setParamLoStop(double d);
	void setParamHiStop(double d);
	void setParamBounce(double d);

	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	@Override
	void setParam (PARAM_N parameter, double value);
	
	
	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	@Override
	double getParam (PARAM_N parameter);

}
