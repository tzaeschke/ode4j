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
import org.ode4j.ode.internal.Common.D_PARAM_NAMES_N;

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


	void setMode (AMotorMode mode);
	AMotorMode getMode();

	void setNumAxes (int num);
	int getNumAxes();

	void setAxis (int anum, int rel, double x, double y, double z);
	void setAxis (int anum, int rel, DVector3C a);
	void getAxis (int anum, DVector3 result);
	int getAxisRel (int anum);

	void setAngle (int anum, double angle);
	double getAngle (int anum);
	double getAngleRate (int anum);

	void setParam (D_PARAM_NAMES_N parameter, double value);
	double getParam (D_PARAM_NAMES_N parameter);

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


	//	  // intentionally undefined, don't use these
	//	  dAMotorJoint (const dAMotorJoint &);
	//	  void operator = (const dAMotorJoint &);
	//
	//	public:
	//	  dAMotorJoint() { }
	//	  dAMotorJoint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreateAMotor (world, group); }
	//	  dAMotorJoint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreateAMotor (world.id(), group); }
	//
	//	  void create (dWorld world, dJointGroup group=0) {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreateAMotor (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void setMode (int mode)
	//	    { dJointSetAMotorMode (_id, mode); }
	//	  int getMode() const
	//	    { return dJointGetAMotorMode (_id); }
	//
	//	  void setNumAxes (int num)
	//	    { dJointSetAMotorNumAxes (_id, num); }
	//	  int getNumAxes() const
	//	    { return dJointGetAMotorNumAxes (_id); }
	//
	//	  void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
	//	    { dJointSetAMotorAxis (_id, anum, rel, x, y, z); }
	//	  void setAxis (int anum, int rel, const dVector3 a)
	//	    { setAxis(anum, rel, a[0], a[1], a[2]); }
	//	  void getAxis (int anum, dVector3 result) const
	//	    { dJointGetAMotorAxis (_id, anum, result); }
	//	  int getAxisRel (int anum) const
	//	    { return dJointGetAMotorAxisRel (_id, anum); }
	//
	//	  void setAngle (int anum, dReal angle)
	//	    { dJointSetAMotorAngle (_id, anum, angle); }
	//	  dReal getAngle (int anum) const
	//	    { return dJointGetAMotorAngle (_id, anum); }
	//	  dReal getAngleRate (int anum)
	//	    { return dJointGetAMotorAngleRate (_id,anum); }
	//
	//	  void setParam (int parameter, dReal value)
	//	    { dJointSetAMotorParam (_id, parameter, value); }
	//	  dReal getParam (int parameter) const
	//	    { return dJointGetAMotorParam (_id, parameter); }
	//	  // TODO: expose params through methods
	//
	//	  void addTorques(dReal torque1, dReal torque2, dReal torque3)
	//		{ dJointAddAMotorTorques(_id, torque1, torque2, torque3); }
}
