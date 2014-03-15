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

public interface DTransmissionJoint extends DJoint {

	/**
	 * Get the contact point of the first wheel of the Transmission joint.
	 */
	void getContactPoint1(DVector3 result);

	/**
	 * Get contact point of the second wheel of the Transmission joint.
	 */
	void getContactPoint2(DVector3 result);
	 
	/**
	 * Set the first axis for the Transmission joint.
	 * <p>REMARK: This is the axis around which the first body is allowed to
	 * revolve and is attached to it.  It is given in global coordinates
	 * and can only be set explicitly in intersecting-axes mode.  For the
	 * parallel-axes and chain modes which share one common axis of
	 * revolution for both gears setAxis should be used.
	 */
	void setAxis1(DVector3C xyz);
	/**
	 * @see #setAxis1(DVector3C)
	 */
	void setAxis1(double x, double y, double z);

	/**
	 * Get first axis for the Transmission joint.
	 * <p>REMARK: In parallel-axes and chain mode the common axis with
	 * respect to the first body is returned.  If the joint constraint is
	 * satisfied it should be the same as the axis return with
	 * getAxis2 or getAxis.
	 */
	void getAxis1(DVector3 result);
	 
	/**
	 * Set second axis for the Transmission joint.
	 * <p>REMARK: This is the axis around which the second body is allowed
	 * to revolve and is attached to it.  It is given in global
	 * coordinates and can only be set explicitly in intersecting-axes
	 * mode.  For the parallel-axes and chain modes which share one common
	 * axis of revolution for both gears setAxis should
	 * be used.
	 */
	void setAxis2(DVector3C xyz);
	/**
	 * @see #setAxis2(DVector3C)
	 */
	void setAxis2(double x, double y, double z);

	/**
	 * Get second axis for the Transmission joint.
	 * <p>REMARK: In parallel-axes and chain mode the common axis with
	 * respect to the second body is returned.  If the joint constraint is
	 * satisfied it should be the same as the axis return with
	 * getAxis1 or getAxis.
	 */
	void getAxis2(DVector3 result);
	 
	/**
	 * Set the first anchor for the Transmission joint.
	 * <p>REMARK: This is the point of attachment of the wheel on the
	 * first body.  It is given in global coordinates.
	 */
	void setAnchor1(DVector3C xyz);
	/**
	 * @see #setAnchor1(DVector3C)
	 */
	void setAnchor1(double x, double y, double z);

	/**
	 * Get the first anchor of the Transmission joint.
	 */
	void getAnchor1(DVector3 result);
	 
	/**
	 * Set the second anchor for the Transmission joint.
	 * <p>REMARK: This is the point of attachment of the wheel on the
	 * second body.  It is given in global coordinates.
	 */
	void setAnchor2(DVector3C xyz);
	/**
	 * @see #setAnchor2(DVector3C)
	 */
	void setAnchor2(double x, double y, double z);

	/**
	 * Get the second anchor for the Transmission joint.
	 */
	void getAnchor2(DVector3 result);

	/**
	 * Set a Transmission joint parameter.
	 */
	@Override
	void setParam(PARAM_N parameter, double value);

	
	/**
	 * Get a Transmission joint parameter.
	 */
	@Override
	double getParam (PARAM_N parameter);

	/**
	 * Set the Transmission joint mode.
	 * <p>REMARK: The mode can be one of dTransmissionParallelAxes,
	 * dTransmissionIntersectingAxes and dTransmissionChainDrive simulating a
	 * set of parallel-axes gears, intersecting-axes beveled gears or
	 * chain and sprockets respectively.
	 */
	void setMode(TRANSMISSION mode );

	/**
	 * Get the Transmission joint mode.
	 */
	TRANSMISSION getMode();

	/**
	 * Set the Transmission ratio.
	 * <p>REMARK: This is the ratio of the angular speed of the first gear
	 * to that of the second gear.  It can only be set explicitly in
	 * parallel-axes mode.  In intersecting-axes mode the ratio is defined
	 * implicitly by the initial configuration of the wheels and in chain
	 * mode it is defined implicitly be the wheel radii.
	 */
	void setRatio(double ratio );

	/**
	 * Get the Transmission joint ratio.
	 */
	double getRatio();

	/**
	 * Set the common axis for both wheels of the Transmission joint.
	 * <p>REMARK: This sets the common axis of revolution for both wheels
	 * and should only be used in parallel-axes or chain mode.  For
	 * intersecting-axes mode where each wheel axis needs to be specified
	 * individually dJointSetTransmissionAxis1 and
	 * dJointSetTransmissionAxis2 should be used.  The axis is given in
	 * global coordinates
	 */
	void setAxis(DVector3C xyz);
	/**
	 * @see #setAxis(DVector3C)
	 */
	void setAxis(double x, double y, double z );

	/**
	 * Get the common axis for both wheels of the Transmission joint.
	 */
	void getAxis(DVector3 result );

	/**
	 * Get the phase, that is the traversed angle for the first
	 * wheel of the Transmission joint.
	 */
	double getAngle1();

	/**
	 * Get the phase, that is the traversed angle for the second
	 * wheel of the Transmission joint.
	 */
	double getAngle2();

	/**
	 * Get the radius of the first wheel of the Transmission joint.
	 */
	double getRadius1();

	/**
	 * Get the radius of the second wheel of the Transmission joint.
	 */
	double getRadius2();

	/**
	 * Set the radius of the first wheel of the Transmission joint.
	 * <p>REMARK: The wheel radii can only be set explicitly in chain mode.
	 * In the other modes they're defined implicitly by the initial
	 * configuration and ratio of the wheels.
	 */
	void setRadius1(double radius );

	/**
	 * Set the radius of the second wheel of the Transmission joint.
	 * <p>REMARK: The wheel radii can only be set explicitly in chain mode.
	 * In the other modes they're defined implicitly by the initial
	 * configuration and ratio of the wheels.
	 */
	void setRadius2(double radius );

	/**
	 * Get the backlash of the Transmission joint.
	 */
	double getBacklash();

	/**
	 * Set the backlash of the Transmission joint.
	 * <p>REMARK: Backlash is the clearance in the mesh of the wheels of the
	 * transmission and is defined as the maximum distance that the
	 * geometric contact point can travel without any actual contact or
	 * transfer of power between the wheels.  This can be converted in
	 * degrees of revolution for each wheel by dividing by the wheel's
	 * radius.  To further illustrate this consider the situation where a
	 * wheel of radius r_1 is driving another wheel of radius r_2 and
	 * there is an amount of backlash equal to b in their mesh.  If the
	 * driving wheel were to instantaneously stop there would be no
	 * contact and hence the driven wheel would continue to turn for
	 * another b / r_2 radians until all the backlash in the mesh was take
	 * up and contact restored with the relationship of driving and driven
	 * wheel reversed.  The backlash is therefore given in untis of
	 * length.
	 */
	void setBacklash(double backlash );

}
