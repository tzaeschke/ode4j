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

/**
 *  Prismatic and Rotoide.
 * 
 * The axisP must be perpendicular to axis2
 * <PRE>
 *                                        +-------------+
 *                                        |      x      |
 *                                        +------------\+
 * Prismatic articulation                   ..     ..
 *                       |                ..     ..
 *                      \/              ..      ..
 * +--------------+    --|        __..      ..  anchor2
 * |      x       | .....|.......(__)     ..
 * +--------------+    --|         ^     <
 *        |----------------------->|
 *            Offset               |--- Rotoide articulation
 * </PRE>
 */
public interface DPRJoint extends DJoint {

	void setAnchor (double x, double y, double z);
	void setAnchor (DVector3C a);
	void setAxis1 (double x, double y, double z);
	void setAxis1 (DVector3C a);
	void setAxis2 (double x, double y, double z);
	void setAxis2 (DVector3C a);

	void getAnchor (DVector3 result);
	void getAxis1 (DVector3 result);
	void getAxis2 (DVector3 result);

	double getPosition();
	double getPositionRate();
	void setParamLoStop(double d);
	void setParamHiStop(double d);
	void setParamLoStop2(double d);
	void setParamHiStop2(double d);


	//	  dPRJoint (const dPRJoint &);
	//	  void operator = (const dPRJoint &);
	//
	//	public:
	//	  dPRJoint() { }
	//	  dPRJoint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreatePR (world, group); }
	//	  dPRJoint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreatePR (world.id(), group); }
	//
	//	  void create (dWorld world, dJointGroup group=0) {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreatePR (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void setAnchor (dReal x, dReal y, dReal z)
	//	    { dJointSetPRAnchor (_id, x, y, z); }
	//	  void setAnchor (const dVector3 a)
	//	    { setAnchor (a[0], a[1], a[2]); }
	//	  void setAxis1 (dReal x, dReal y, dReal z)
	//	    { dJointSetPRAxis1 (_id, x, y, z); }
	//	  void setAxis1 (const dVector3 a)
	//	    { setAxis1(a[0], a[1], a[2]); }
	//	  void setAxis2 (dReal x, dReal y, dReal z)
	//	    { dJointSetPRAxis2 (_id, x, y, z); }
	//	  void setAxis2 (const dVector3 a)
	//	    { setAxis2(a[0], a[1], a[2]); }
	//
	//	  void getAnchor (dVector3 result) const
	//	    { dJointGetPRAnchor (_id, result); }
	//	  void getAxis1 (dVector3 result) const
	//	    { dJointGetPRAxis1 (_id, result); }
	//	  void getAxis2 (dVector3 result) const
	//	    { dJointGetPRAxis2 (_id, result); }
	//
	//	  dReal getPosition() const
	//	    { return dJointGetPRPosition (_id); }
	//	  dReal getPositionRate() const
	//	    { return dJointGetPRPositionRate (_id); }
	//
	//	  virtual void setParam (int parameter, dReal value)
	//	    { dJointSetPRParam (_id, parameter, value); }
	//	  virtual dReal getParam (int parameter) const
	//	    { return dJointGetPRParam (_id, parameter); }
}
