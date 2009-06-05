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

public interface DSliderJoint extends DJoint {

	void setAxis (double x, double y, double z);
	void setAxis (DVector3C a);
	void getAxis (DVector3 result);

	double getPosition();
	double getPositionRate();

	void addForce (double force);
	void setParamFMax(double d);
	void setParamLoStop(double d);
	void setParamHiStop(double d);
	void setParamVel(double d);
	void setParamBounce(double d);

	
//	  // intentionally undefined, don't use these
//	  dSliderJoint (const dSliderJoint &);
//	  void operator = (const dSliderJoint &);
//
//	public:
//	  dSliderJoint() { }
//	  dSliderJoint (dWorld world, dJointGroup group=0)
//	    { _id = dJointCreateSlider (world, group); }
//	  dSliderJoint (dWorld& world, dJointGroup group=0)
//	    { _id = dJointCreateSlider (world.id(), group); }
//
//	  void create (dWorld world, dJointGroup group=0) {
//	    if (_id) dJointDestroy (_id);
//	    _id = dJointCreateSlider (world, group);
//	  }
//	  void create (dWorld& world, dJointGroup group=0)
//	    { create(world.id(), group); }
//
//	  void setAxis (dReal x, dReal y, dReal z)
//	    { dJointSetSliderAxis (_id, x, y, z); }
//	  void setAxis (const dVector3 a)
//	    { setAxis (a[0], a[1], a[2]); }
//	  void getAxis (dVector3 result) const
//	    { dJointGetSliderAxis (_id, result); }
//
//	  dReal getPosition() const
//	    { return dJointGetSliderPosition (_id); }
//	  dReal getPositionRate() const
//	    { return dJointGetSliderPositionRate (_id); }
//
//	  virtual void setParam (int parameter, dReal value)
//	    { dJointSetSliderParam (_id, parameter, value); }
//	  virtual dReal getParam (int parameter) const
//	    { return dJointGetSliderParam (_id, parameter); }
//	  // TODO: expose params through methods
//
//	  void addForce (dReal force)
//		{ dJointAddSliderForce(_id, force); }
}
