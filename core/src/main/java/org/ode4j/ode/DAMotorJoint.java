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

public interface DAMotorJoint extends DJoint {

	/** angular motor mode numbers */
	public enum AMotorMode {
		dAMotorUser,
		dAMotorEuler;

		public static AMotorMode from(int mode) {
			switch (mode) {
			case 0: return dAMotorUser;
			case 1: return dAMotorEuler;
			default: throw new IllegalArgumentException("mode=" + mode);
			}
		}
	}


	/**
	 * Set mode.
	 * @param mode mode
	 */
	void setMode (AMotorMode mode);

	
	/**
	 * Get the angular motor mode.
	 * Mode must be one of the following constants:
	 * <ul>
	 * <li> dAMotorUser The AMotor axes and joint angle settings are entirely
	 * controlled by the user.  This is the default mode.</li>
	 * <li> dAMotorEuler Euler angles are automatically computed.
	 * The axis a1 is also automatically computed.
	 * The AMotor axes must be set correctly when in this mode,
	 * as described below.</li>
	 * </ul>
	 * When this mode is initially set the current relative orientations
	 * of the bodies will correspond to all euler angles at zero.
	 * @return mode
	 */
	AMotorMode getMode();

	
	/**
	 * Set the nr of axes.
	 * @param num 0..3
	 */
	void setNumAxes (int num);

	
	/**
	 * Get the number of angular axes that will be controlled by the
	 * AMotor. <p>
	 * Num can range from 0 (which effectively deactivates the joint) to 3.
	 * This is automatically set to 3 in dAMotorEuler mode.
	 * @return number of axes
	 */
	int getNumAxes();

	/**
	 * Set axis.
	 * @param anum anum
	 * @param rel rel. See ::getAxisRel
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	void setAxis (int anum, int rel, double x, double y, double z);

	
	/**
	 * Set axis.
	 * @param anum anum
	 * @param rel  rel. See ::getAxisRel
	 * @param a a
	 */
	void setAxis (int anum, int rel, DVector3C a);

	
	/**
	 * Get the AMotor axes.
	 * @param anum selects the axis to change (0,1 or 2).
	 * <ul>
	 * <li> 0: The axis is anchored to the global frame. </li>
	 * <li> 1: The axis is anchored to the first body. </li>
	 * <li> 2: The axis is anchored to the second body. </li>
	 * </ul>
	 * @param result Each axis can have one of three ``relative orientation'' modes.
	 */
	void getAxis (int anum, DVector3 result);

	
	/**
	 * Get axis.
	 * <p>
	 * The axis vector is always specified in global coordinates regardless
	 * of the setting of rel. <br>
	 * There are two GetAMotorAxis functions, one to return the axis and one to
	 * return the relative mode.
	 * <p>
	 * For dAMotorEuler mode:
	 * <ul>
	 * <li>	Only axes 0 and 2 need to be set. Axis 1 will be determined
		automatically at each time step. </li>
	 * <li>	Axes 0 and 2 must be perpendicular to each other. </li>
	 * <li>	Axis 0 must be anchored to the first body, axis 2 must be anchored
		to the second body. </li>
		</ul>
	 * @param anum axis number
	 * @return rel rel
	 */
	int getAxisRel (int anum);

	/**
	 * Tell the AMotor what the current angle is along axis anum.
	 * <p>
	 * This function should only be called in dAMotorUser mode, because in this
	 * mode the AMotor has no other way of knowing the joint angles.
	 * The angle information is needed if stops have been set along the axis,
	 * but it is not needed for axis motors.
	 * @param anum axis number
	 * @param angle angle
	 */
	void setAngle (int anum, double angle);

	
	/**
	 * Get the current angle for axis.
	 * <p>
	 * In dAMotorUser mode this is simply the value that was set with
	 * dJointSetAMotorAngle().
	 * In dAMotorEuler mode this is the corresponding euler angle.
	 * @param anum axis number
	 * @return angle
	 */
	double getAngle (int anum);
	
	
	/**
	 * Get the current angle rate for axis anum.
	 * <p>
	 * In dAMotorUser mode this is always zero, as not enough information is
	 * available.
	 * In dAMotorEuler mode this is the corresponding euler angle rate.
	 * @param anum axis number
	 * @return rate
	 */
	double getAngleRate (int anum);


	/**
	 * Applies torque0 about the AMotor's axis 0, torque1 about the
	 * AMotor's axis 1, and torque2 about the AMotor's axis 2.
	 * <p>
	 * If the motor has fewer than three axes, the higher torques are ignored.
	 * This function is just a wrapper for dBodyAddTorque().
	 * @param torque1 torque 1
	 * @param torque2 torque 2
	 * @param torque3 torque 3
	 */
	void addTorques(double torque1, double torque2, double torque3);
	
	double getParamFMax();
	double getParamFMax2();
	double getParamFMax3();
	double getParamVel();
	double getParamVel2();
	double getParamVel3();
	void setParamFMax(double d);
	void setParamFMax2(double d);
	void setParamFMax3(double d);
	void setParamLoStop(double d);
	void setParamLoStop2(double d);
	void setParamLoStop3(double d);
	void setParamHiStop(double d);
	void setParamHiStop2(double d);
	void setParamHiStop3(double d);
	void setParamVel(double d);
	void setParamVel2(double d);
	void setParamVel3(double d);


	/**
	 * Set joint parameter.
	 * @param parameter parameter indicator
	 * @param value value
	 */
	@Override
	void setParam (PARAM_N parameter, double value);


	/**
	 * Get joint parameter.
	 * @return parameter value
	 */
	@Override
	double getParam (PARAM_N parameter);
}
