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

	void setAnchor (double x, double y, double z);
	void setAnchor (DVector3C a);
	void getAnchor (DVector3 result);
	void getAnchor2 (DVector3 result);

	void setAxis (double x, double y, double z);
	void setAxis (DVector3C a);
	void getAxis (DVector3 result);
	void setAxisOffset(double x, double y, double z, double angle);

	double getAngle();
	double getAngleRate();

	void addTorque (double torque);
	void setParamFMax(double d);
	void setParamVel(double cos);
	void setParamLoStop(double d);
	void setParamHiStop(double d);
	void setParamBounce(double d);


	//	  // intentionally undefined, don't use these
	//	  dHingeJoint (const dHingeJoint &);
	//	  void operator = (const dHingeJoint &);
	//
	//	public:
	//	  dHingeJoint() { }
	//	  dHingeJoint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreateHinge (world, group); }
	//	  dHingeJoint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreateHinge (world.id(), group); }
	//
	//	  void create (dWorld world, dJointGroup group=0) {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreateHinge (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void setAnchor (dReal x, dReal y, dReal z)
	//	    { dJointSetHingeAnchor (_id, x, y, z); }
	//	  void setAnchor (const dVector3 a)
	//	    { setAnchor (a[0], a[1], a[2]); }
	//	  void getAnchor (dVector3 result) const
	//	    { dJointGetHingeAnchor (_id, result); }
	//	  void getAnchor2 (dVector3 result) const
	//	    { dJointGetHingeAnchor2 (_id, result); }
	//
	//	  void setAxis (dReal x, dReal y, dReal z)
	//	    { dJointSetHingeAxis (_id, x, y, z); }
	//	  void setAxis (const dVector3 a)
	//	    { setAxis(a[0], a[1], a[2]); }
	//	  void getAxis (dVector3 result) const
	//	    { dJointGetHingeAxis (_id, result); }
	//
	//	  dReal getAngle() const
	//	    { return dJointGetHingeAngle (_id); }
	//	  dReal getAngleRate() const
	//	    { return dJointGetHingeAngleRate (_id); }
	//
	//	  virtual void setParam (int parameter, dReal value)
	//	    { dJointSetHingeParam (_id, parameter, value); }
	//	  virtual dReal getParam (int parameter) const
	//	    { return dJointGetHingeParam (_id, parameter); }
	//	  // TODO: expose params through methods
	//
	//	  void addTorque (dReal torque)
	//		{ dJointAddHingeTorque(_id, torque); }

}
