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

public interface DUniversalJoint extends DJoint {
	
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

	  //TZ removed to avoid RefDouble usage
	  //void getAngles(double *angle1, double *angle2);

	  double getAngle1();
	  double getAngle1Rate();
	  double getAngle2();
	  double getAngle2Rate();

	  void addTorques (double torque1, double torque2);
	
	
//	  // intentionally undefined, don't use these
//	  dUniversalJoint (const dUniversalJoint &);
//	  void operator = (const dUniversalJoint &);
//
//	public:
//	  dUniversalJoint() { }
//	  dUniversalJoint (dWorld world, dJointGroup group=0)
//	    { _id = dJointCreateUniversal (world, group); }
//	  dUniversalJoint (dWorld& world, dJointGroup group=0)
//	    { _id = dJointCreateUniversal (world.id(), group); }
//
//	  void create (dWorld world, dJointGroup group=0) {
//	    if (_id) dJointDestroy (_id);
//	    _id = dJointCreateUniversal (world, group);
//	  }
//	  void create (dWorld& world, dJointGroup group=0)
//	    { create(world.id(), group); }
//
//	  void setAnchor (dReal x, dReal y, dReal z)
//	    { dJointSetUniversalAnchor (_id, x, y, z); }
//	  void setAnchor (const dVector3 a)
//	    { setAnchor(a[0], a[1], a[2]); }
//	  void setAxis1 (dReal x, dReal y, dReal z)
//	    { dJointSetUniversalAxis1 (_id, x, y, z); }
//	  void setAxis1 (const dVector3 a)
//	    { setAxis1 (a[0], a[1], a[2]); }
//	  void setAxis2 (dReal x, dReal y, dReal z)
//	    { dJointSetUniversalAxis2 (_id, x, y, z); }
//	  void setAxis2 (const dVector3 a)
//	    { setAxis2 (a[0], a[1], a[2]); }
//
//	  void getAnchor (dVector3 result) const
//	    { dJointGetUniversalAnchor (_id, result); }
//	  void getAnchor2 (dVector3 result) const
//	    { dJointGetUniversalAnchor2 (_id, result); }
//	  void getAxis1 (dVector3 result) const
//	    { dJointGetUniversalAxis1 (_id, result); }
//	  void getAxis2 (dVector3 result) const
//	    { dJointGetUniversalAxis2 (_id, result); }
//
//	  virtual void setParam (int parameter, dReal value)
//	    { dJointSetUniversalParam (_id, parameter, value); }
//	  virtual dReal getParam (int parameter) const
//	    { return dJointGetUniversalParam (_id, parameter); }
//	  // TODO: expose params through methods
//
//	  void getAngles(dReal *angle1, dReal *angle2) const
//	    { dJointGetUniversalAngles (_id, angle1, angle2); }
//
//	  dReal getAngle1() const
//	    { return dJointGetUniversalAngle1 (_id); }
//	  dReal getAngle1Rate() const
//	    { return dJointGetUniversalAngle1Rate (_id); }
//	  dReal getAngle2() const
//	    { return dJointGetUniversalAngle2 (_id); }
//	  dReal getAngle2Rate() const
//	    { return dJointGetUniversalAngle2Rate (_id); }
//
//	  void addTorques (dReal torque1, dReal torque2)
//		{ dJointAddUniversalTorques(_id, torque1, torque2); }
}
