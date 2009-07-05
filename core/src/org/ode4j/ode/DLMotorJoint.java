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

public interface DLMotorJoint extends DJoint {

	void setNumAxes (int num);
	int getNumAxes();

	void setAxis (int anum, int rel, double x, double y, double z);
	void setAxis (int anum, int rel, DVector3C a);
	void getAxis (int anum, DVector3 result);

	void setParam (PARAM_N parameter, double value);
	double getParam (PARAM_N parameter);
	
	double getParamVel();
	double getParamVel2();
	double getParamVel3();
	double getParamFMax();
	double getParamFMax2();
	double getParamFMax3();
	void setParamVel(double d);
	void setParamVel2(double d);
	void setParamVel3(double d);
	void setParamFMax(double d);
	void setParamFMax2(double d);
	void setParamFMax3(double d);


	//	  // intentionally undefined, don't use these
	//	  dLMotorJoint (const dLMotorJoint &);
	//	  void operator = (const dLMotorJoint &);
	//
	//	public:
	//	  dLMotorJoint() { }
	//	  dLMotorJoint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreateLMotor (world, group); }
	//	  dLMotorJoint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreateLMotor (world.id(), group); }
	//
	//	  void create (dWorld world, dJointGroup group=0) {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreateLMotor (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void setNumAxes (int num)
	//	    { dJointSetLMotorNumAxes (_id, num); }
	//	  int getNumAxes() const
	//	    { return dJointGetLMotorNumAxes (_id); }
	//
	//	  void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
	//	    { dJointSetLMotorAxis (_id, anum, rel, x, y, z); }
	//	  void setAxis (int anum, int rel, const dVector3 a)
	//	    { setAxis(anum, rel, a[0], a[1], a[2]); }
	//	  void getAxis (int anum, dVector3 result) const
	//	    { dJointGetLMotorAxis (_id, anum, result); }
	//
	//	  void setParam (int parameter, dReal value)
	//	    { dJointSetLMotorParam (_id, parameter, value); }
	//	  dReal getParam (int parameter) const
	//	    { return dJointGetLMotorParam (_id, parameter); }
	//	  // TODO: expose params through methods
}
