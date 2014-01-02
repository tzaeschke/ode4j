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

public interface DUniversalJoint extends DJoint {

	/**
	 * Set anchor.
	 */
	void setAnchor (double x, double y, double z);
	
	
	/**
	 * Set anchor.
	 */
	void setAnchor (DVector3C a);
	
	
	/**
	 * Set axis.
	 */
	void setAxis1 (double x, double y, double z);
	
	
	/**
	 * Set axis.
	 */
	void setAxis1 (DVector3C a);
	
	
	/**
	 * Set axis.
	 */
	void setAxis2 (double x, double y, double z);
	
	
	/**
	 * Set axis.
	 */
	void setAxis2 (DVector3C a);


	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 */
	void getAnchor (DVector3 result);

	
	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 2.
	 * 
	 * <p>REMARK:
	 * You can think of the ball and socket part of a universal joint as
	 * trying to keep the result of dJointGetBallAnchor() and
	 * dJointGetBallAnchor2() the same. If the joint is
	 * perfectly satisfied, this function will return the same value
	 * as dJointGetUniversalAnchor() to within roundoff errors.
	 * dJointGetUniversalAnchor2() can be used, along with
	 * dJointGetUniversalAnchor(), to see how far the joint has come apart.
	 */
	void getAnchor2 (DVector3 result);

	
	/**
	 * Get axis.
	 */
	void getAxis1 (DVector3 result);

	
	/**
	 * Get axis.
	 */
	void getAxis2 (DVector3 result);


//	/**
//	 * Get both angles at the same time.
//	 *
//	 * <p>NOTE: This function combine getUniversalAngle1 and getUniversalAngle2 together
//	 *       and try to avoid redundant calculation
//	 *
//	 * @param joint   The universal joint for which we want to calculate the angles
//	 * @param angle1  The angle between the body1 and the axis 1
//	 * @param angle2  The angle between the body2 and the axis 2
//	 */
	/**
	 * Get angle between body the 1 and the axis 1.
	 */
	double getAngle1();

	
	/**
	 * Get angle between body the 2 and the axis 2.
	 */
	double getAngle2();
	
	
	/**
	 * Get time derivative of angle.
	 */
	double getAngle1Rate();

	
	/**
	 * Get time derivative of angle.
	 */
	double getAngle2Rate();

	/**
	 * Applies torque1 about the universal's axis 1, torque2 about the
	 * universal's axis 2.
	 * 
	 * <p>REMARK: This function is just a wrapper for dBodyAddTorque().
	 */
	void addTorques (double torque1, double torque2);
	
	
	/**
	 * Set the Universal axis1 as if the 2 bodies were already at 
	 *        offset1 and offset2 appart with respect to axis1 and axis2.
	 * <p>
	 * This function initialize the axis1 and the relative orientation of 
	 * each body as if body1 was rotated around the new axis1 by the offset1 
	 * value and as if body2 was rotated around the axis2 by offset2. <p>
	 * Ex: <br>
	 * <code>
	 * dJointSetHuniversalAxis1(jId, 1, 0, 0); <br>
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0 <br>
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0 <br>
	 * dJointSetHuniversalAxis1Offset(jId, 1, 0, 0, 0.2, 0.17); <br>
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2 <br>
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17 <br>
	 * </code>
	 *
	 * <p>NOTE: Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b offsets appart.
	 *
	 * <p>NOTE: Any previous offsets are erased.
	 *
	 * <p>WARNING: Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
	 *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
	 *          will reset the "zero" angle position.
	 *          
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param offset1 The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 * @param offset2
	 */
	void setAxis1Offset(double x, double y, double z, double offset1,
			double offset2);
	
	
	/**
	 * Set the Universal axis2 as if the 2 bodies were already at 
	 *        offset1 and offset2 appart with respect to axis1 and axis2.
	 * <p>
	 * This function initialize the axis2 and the relative orientation of 
	 * each body as if body1 was rotated around the axis1 by the offset1 
	 * value and as if body2 was rotated around the new axis2 by offset2. <p>
	 * Ex: <br>
	 * <code>
	 * dJointSetHuniversalAxis2(jId, 0, 1, 0); <br>
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0 <br>
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0 <br>
	 * dJointSetHuniversalAxis2Offset(jId, 0, 1, 0, 0.2, 0.17); <br>
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2 <br>
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17 <br>
	 * </code>
	 * 
 	 * <p>NOTE: Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b offsets apart.
	 *
	 * <p>NOTE: Any previous offsets are erased.
	 *
	 * <p>WARNING: Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
	 *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
	 *          will reset the "zero" angle position.
	 * 
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param offset1 The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 * @param offset2
	 */
	void setAxis2Offset(double x, double y, double z, double offset1,
			double offset2);
	
	
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
