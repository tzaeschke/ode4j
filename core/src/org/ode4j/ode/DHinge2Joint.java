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

public interface DHinge2Joint extends DJoint {

	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	void setAnchor (double x, double y, double z);
	
	
	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	void setAnchor (DVector3C a);
	
	
	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	void setAxis1 (double x, double y, double z);
	
	
	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	void setAxis1 (DVector3C a);
	
	
	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	void setAxis2 (double x, double y, double z);
	
	
	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	void setAxis2 (DVector3C a);

	
	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * <p>
	 * Return the point on body 1.  If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	void getAnchor (DVector3 result);

	
	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * <p>
	 * This returns the point on body 2. If the joint is perfectly satisfied,
	 * this will return the same value as dJointGetHinge2Anchor.
	 * If not, this value will be slightly different.
	 * This can be used, for example, to see how far the joint has come apart.
	 * @ingroup joints
	 */
	void getAnchor2 (DVector3 result);

	
	/**
	 * @brief Get joint axis
	 * @ingroup joints
	 */
	void getAxis1 (DVector3 result);

	
	/**
	 * @brief Get joint axis
	 * @ingroup joints
	 */
	void getAxis2 (DVector3 result);


	/**
	 * @brief Get angle
	 * @ingroup joints
	 */
	double getAngle1();

	
	/**
	 * @brief Get time derivative of angle
	 * @ingroup joints
	 */
	double getAngle1Rate();

	
	/**
	 * @brief Get time derivative of angle
	 * @ingroup joints
	 */
	double getAngle2Rate();

	
	/**
	 * @brief Applies torque1 about the hinge2's axis 1, torque2 about the
	 * hinge2's axis 2.
	 * @remarks  This function is just a wrapper for dBodyAddTorque().
	 * @ingroup joints
	 */
	void addTorques(double torque1, double torque2);
	void setParamVel2(double d);
	void setParamFMax2(double d);
	void setParamVel(double d);
	void setParamFMax(double d);
	void setParamLoStop(double d);
	void setParamHiStop(double d);
	void setParamFudgeFactor(double d);
	void setParamSuspensionERP(double d);
	void setParamSuspensionCFM(double d);

	
	
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
