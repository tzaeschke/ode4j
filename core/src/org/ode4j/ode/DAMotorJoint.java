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
	 * @brief set mode
	 * @ingroup joints
	 */
	void setMode (AMotorMode mode);

	
	/**
	 * @brief Get the angular motor mode.
	 * Mode must be one of the following constants:
	 * <li> dAMotorUser The AMotor axes and joint angle settings are entirely
	 * controlled by the user.  This is the default mode.</li>
	 * <li> dAMotorEuler Euler angles are automatically computed.
	 * The axis a1 is also automatically computed.
	 * The AMotor axes must be set correctly when in this mode,
	 * as described below.</li>
	 * When this mode is initially set the current relative orientations
	 * of the bodies will correspond to all euler angles at zero.
	 * @ingroup joints
	 */
	AMotorMode getMode();

	
	/**
	 * @brief set the nr of axes
	 * @param num 0..3
	 * @ingroup joints
	 */
	void setNumAxes (int num);

	
	/**
	 * @brief Get the number of angular axes that will be controlled by the
	 * AMotor. <p>
	 * Num can range from 0 (which effectively deactivates the joint) to 3.
	 * This is automatically set to 3 in dAMotorEuler mode.
	 * @ingroup joints
	 */
	int getNumAxes();

	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	void setAxis (int anum, int rel, double x, double y, double z);

	
	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	void setAxis (int anum, int rel, DVector3C a);

	
	/**
	 * @brief Get the AMotor axes.
	 * @param anum selects the axis to change (0,1 or 2).
	 * <li> 0: The axis is anchored to the global frame. </li>
	 * <li> 1: The axis is anchored to the first body. </li>
	 * <li> 2: The axis is anchored to the second body. </li>
	 * @param result Each axis can have one of three ``relative orientation'' modes.
	 * @ingroup joints
	 */
	void getAxis (int anum, DVector3 result);

	
	/**
	 * @brief Get axis
	 * @remarks
	 * The axis vector is always specified in global coordinates regardless
	 * of the setting of rel. <br>
	 * There are two GetAMotorAxis functions, one to return the axis and one to
	 * return the relative mode.
	 * <p>
	 * For dAMotorEuler mode:
	 * <li>	Only axes 0 and 2 need to be set. Axis 1 will be determined
		automatically at each time step. </li>
	 * <li>	Axes 0 and 2 must be perpendicular to each other. </li>
	 * <li>	Axis 0 must be anchored to the first body, axis 2 must be anchored
		to the second body. </li>
	 * @ingroup joints
	 */
	int getAxisRel (int anum);

	/**
	 * @brief Tell the AMotor what the current angle is along axis anum.
	 * <p>
	 * This function should only be called in dAMotorUser mode, because in this
	 * mode the AMotor has no other way of knowing the joint angles.
	 * The angle information is needed if stops have been set along the axis,
	 * but it is not needed for axis motors.
	 * @ingroup joints
	 */
	void setAngle (int anum, double angle);

	
	/**
	 * @brief Get the current angle for axis.
	 * @remarks
	 * In dAMotorUser mode this is simply the value that was set with
	 * dJointSetAMotorAngle().
	 * In dAMotorEuler mode this is the corresponding euler angle.
	 * @ingroup joints
	 */
	double getAngle (int anum);
	
	
	/**
	 * @brief Get the current angle rate for axis anum.
	 * @remarks
	 * In dAMotorUser mode this is always zero, as not enough information is
	 * available.
	 * In dAMotorEuler mode this is the corresponding euler angle rate.
	 * @ingroup joints
	 */
	double getAngleRate (int anum);


	/**
	 * @brief Applies torque0 about the AMotor's axis 0, torque1 about the
	 * AMotor's axis 1, and torque2 about the AMotor's axis 2.
	 * @remarks
	 * If the motor has fewer than three axes, the higher torques are ignored.
	 * This function is just a wrapper for dBodyAddTorque().
	 * @ingroup joints
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
	void setParamVel(double d);
	void setParamVel2(double d);
	void setParamVel3(double d);


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
