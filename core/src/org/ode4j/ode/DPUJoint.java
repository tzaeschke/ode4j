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

public interface DPUJoint extends DJoint {

	void setAnchor (double x, double y, double z);
	void setAnchor (DVector3C a);
	void setAxis1 (double x, double y, double z);
	void setAxis1 (DVector3C a);
	void setAxis2 (double x, double y, double z);
	void setAxis3 (double x, double y, double z);
	void setAxis3 (DVector3C a);
	void setAxisP (double x, double y, double z);
	void setAxisP (DVector3C a);

	void getAnchor (DVector3 result);
	void getAxis1 (DVector3 result);
	void getAxis2 (DVector3 result);
	void getAxis3 (DVector3 result);
	void getAxisP (DVector3 result);

	double getAngle1();
	double getAngle1Rate();
	double getAngle2();
	double getAngle2Rate();

	double getPosition();
	double getPositionRate();
	void setAnchorOffset(double x, double y, double z, double dx, double dy,
			double dz);

	
//	  dPUJoint (const dPUJoint &);
//	  void operator = (const dPUJoint &);
//
//	public:
//	  dPUJoint() { }
//	  dPUJoint (dWorld world, dJointGroup group=0)
//	    { _id = dJointCreatePU (world, group); }
//	  dPUJoint (dWorld& world, dJointGroup group=0)
//	    { _id = dJointCreatePU (world.id(), group); }
//
//	  void create (dWorld world, dJointGroup group=0)
//	  {
//	    if (_id) dJointDestroy (_id);
//	    _id = dJointCreatePU (world, group);
//	  }
//	  void create (dWorld& world, dJointGroup group=0)
//	  { create(world.id(), group); }
//
//	  void setAnchor (dReal x, dReal y, dReal z)
//	    { dJointSetPUAnchor (_id, x, y, z); }
//	  void setAnchor (const dVector3 a)
//	    { setAnchor (a[0], a[1], a[2]); }
//	  void setAxis1 (dReal x, dReal y, dReal z)
//	    { dJointSetPUAxis1 (_id, x, y, z); }
//	  void setAxis1 (const dVector3 a)
//	    { setAxis1(a[0], a[1], a[2]); }
//	  void setAxis2 (dReal x, dReal y, dReal z)
//	  { dJointSetPUAxis2 (_id, x, y, z); }
//	  void setAxis3 (dReal x, dReal y, dReal z)
//	  { dJointSetPUAxis3 (_id, x, y, z); }
//	  void setAxis3 (const dVector3 a)
//	    { setAxis3(a[0], a[1], a[2]); }
//	  void setAxisP (dReal x, dReal y, dReal z)
//	  { dJointSetPUAxis3 (_id, x, y, z); }
//	  void setAxisP (const dVector3 a)
//	    { setAxisP(a[0], a[1], a[2]); }
//
//	  virtual void getAnchor (dVector3 result) const
//	    { dJointGetPUAnchor (_id, result); }
//	  void getAxis1 (dVector3 result) const
//	    { dJointGetPUAxis1 (_id, result); }
//	  void getAxis2 (dVector3 result) const
//	    { dJointGetPUAxis2 (_id, result); }
//	  void getAxis3 (dVector3 result) const
//	    { dJointGetPUAxis3 (_id, result); }
//	  void getAxisP (dVector3 result) const
//	    { dJointGetPUAxis3 (_id, result); }
//
//	  dReal getAngle1() const
//	    { return dJointGetPUAngle1 (_id); }
//	  dReal getAngle1Rate() const
//	    { return dJointGetPUAngle1Rate (_id); }
//	  dReal getAngle2() const
//	    { return dJointGetPUAngle2 (_id); }
//	  dReal getAngle2Rate() const
//	    { return dJointGetPUAngle2Rate (_id); }
//
//	  dReal getPosition() const
//	    { return dJointGetPUPosition (_id); }
//	  dReal getPositionRate() const
//	    { return dJointGetPUPositionRate (_id); }
//
//	  virtual void setParam (int parameter, dReal value)
//	  { dJointSetPUParam (_id, parameter, value); }
//	  virtual dReal getParam (int parameter) const
//	    { return dJointGetPUParam (_id, parameter); }
//	  // TODO: expose params through methods
}
