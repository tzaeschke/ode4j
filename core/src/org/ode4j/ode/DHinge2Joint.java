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

	void setAnchor (double x, double y, double z);
	void setAnchor (DVector3C a);
	void setAxis1 (double x, double y, double z);
	void setAxis1 (DVector3C a);
	void setAxis2 (double x, double y, double z);
	void setAxis2 (DVector3C a);

	void getAnchor (DVector3 result);
	void getAnchor2 (DVector3 result);
	void getAxis1 (DVector3 result);
	void getAxis2 (DVector3 result);

	double getAngle1();
	double getAngle1Rate();
	double getAngle2Rate();

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


	//	  // intentionally undefined, don't use these
	//	  dHinge2Joint (const dHinge2Joint &);
	//	  void operator = (const dHinge2Joint &);
	//
	//	public:
	//	  dHinge2Joint() { }
	//	  dHinge2Joint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreateHinge2 (world, group); }
	//	  dHinge2Joint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreateHinge2 (world.id(), group); }
	//
	//	  void create (dWorld world, dJointGroup group=0) {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreateHinge2 (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void setAnchor (dReal x, dReal y, dReal z)
	//	    { dJointSetHinge2Anchor (_id, x, y, z); }
	//	  void setAnchor (const dVector3 a)
	//	    { setAnchor(a[0], a[1], a[2]); }
	//	  void setAxis1 (dReal x, dReal y, dReal z)
	//	    { dJointSetHinge2Axis1 (_id, x, y, z); }
	//	  void setAxis1 (const dVector3 a)
	//	    { setAxis1 (a[0], a[1], a[2]); }
	//	  void setAxis2 (dReal x, dReal y, dReal z)
	//	    { dJointSetHinge2Axis2 (_id, x, y, z); }
	//	  void setAxis2 (const dVector3 a)
	//	    { setAxis2 (a[0], a[1], a[2]); }
	//
	//	  void getAnchor (dVector3 result) const
	//	    { dJointGetHinge2Anchor (_id, result); }
	//	  void getAnchor2 (dVector3 result) const
	//	    { dJointGetHinge2Anchor2 (_id, result); }
	//	  void getAxis1 (dVector3 result) const
	//	    { dJointGetHinge2Axis1 (_id, result); }
	//	  void getAxis2 (dVector3 result) const
	//	    { dJointGetHinge2Axis2 (_id, result); }
	//
	//	  dReal getAngle1() const
	//	    { return dJointGetHinge2Angle1 (_id); }
	//	  dReal getAngle1Rate() const
	//	    { return dJointGetHinge2Angle1Rate (_id); }
	//	  dReal getAngle2Rate() const
	//	    { return dJointGetHinge2Angle2Rate (_id); }
	//
	//	  virtual void setParam (int parameter, dReal value)
	//	    { dJointSetHinge2Param (_id, parameter, value); }
	//	  virtual dReal getParam (int parameter) const
	//	    { return dJointGetHinge2Param (_id, parameter); }
	//	  // TODO: expose params through methods
	//
	//	  void addTorques(dReal torque1, dReal torque2)
	//		{ dJointAddHinge2Torques(_id, torque1, torque2); }

}
