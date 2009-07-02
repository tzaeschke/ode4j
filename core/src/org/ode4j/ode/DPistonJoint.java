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
 * ****************************************************************************
 * Piston
 *
 * ****************************************************************************
 * Component of a Piston joint
 * <PRE>
 *                              |- Anchor point
 *      Body_1                  |                       Body_2
 *      +---------------+       V                       +------------------+
 *     /               /|                             /                  /|
 *    /               / +       |--      ______      /                  / +
 *   /      x        /./........x.......(_____()..../         x        /.......> axis
 *  +---------------+ /         |--                +------------------+ /
 *  |               |/                             |                  |/
 *  +---------------+                              +------------------+
 *          |                                                 |
 *          |                                                 |
 *          |------------------> <----------------------------|
 *              anchor1                  anchor2
 *
 *
 * </PRE>
 *
 * When the prismatic joint as been elongated (i.e. dJointGetPistonPosition)
 * return a value >  0
 * <PRE>
 *                                   |- Anchor point
 *      Body_1                       |                       Body_2
 *      +---------------+            V                       +------------------+
 *     /               /|                                  /                  /|
 *    /               / +            |--      ______      /                  / +
 *   /      x        /./........_____x.......(_____()..../         x        /.......> axis
 *  +---------------+ /              |--                +------------------+ /
 *  |               |/                                  |                  |/
 *  +---------------+                                   +------------------+
 *          |                                                      |
 *          |                                                      |
 *          |----------------.>      <----------------------------|
 *              anchor1         |----|         anchor2
 *                                ^
 *                                |-- This is what dJointGetPistonPosition will
 *                                    return
 * </PRE>
 * ****************************************************************************
 */
public interface DPistonJoint extends DJoint {

	void setAnchor (double x, double y, double z);
	void setAnchor (DVector3C a);
	void getAnchor (DVector3 result);
	void getAnchor2 (DVector3 result);
	void setAnchorOffset(DVector3C xyz, double dx, double dy, double dz);

	void setAxis (double x, double y, double z);
	void setAxis (DVector3C a);
	void getAxis (DVector3 result);

	double getPosition();
	double getPositionRate();

	void addForce (double force);
	
	double getParamLoStop2();
	double getParamHiStop2();
	void setParamLoStop2(double d);
	void setParamHiStop2(double d);
	double getAngle();
	double getAngleRate();


	//	 // intentionally undefined, don't use these
	//	  dPistonJoint (const dPistonJoint &);
	//	  void operator = (const dPistonJoint &);
	//
	//	public:
	//	  dPistonJoint() { }
	//	  dPistonJoint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreatePiston (world, group); }
	//	  dPistonJoint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreatePiston (world, group); }
	//
	//	  void create (dWorld world, dJointGroup group=0)
	//	  {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreatePiston (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void setAnchor (dReal x, dReal y, dReal z)
	//	    { dJointSetPistonAnchor (_id, x, y, z); }
	//	  void setAnchor (const dVector3 a)
	//	    { setAnchor (a[0], a[1], a[2]); }
	//	  void getAnchor (dVector3 result) const
	//	    { dJointGetPistonAnchor (_id, result); }
	//	  void getAnchor2 (dVector3 result) const
	//	    { dJointGetPistonAnchor2 (_id, result); }
	//
	//	  void setAxis (dReal x, dReal y, dReal z)
	//	    { dJointSetPistonAxis (_id, x, y, z); }
	//	  void setAxis (const dVector3 a)
	//	    { setAxis(a[0], a[1], a[2]); }
	//	  void getAxis (dVector3 result) const
	//	    { dJointGetPistonAxis (_id, result); }
	//
	//	  dReal getPosition() const
	//	    { return dJointGetPistonPosition (_id); }
	//	  dReal getPositionRate() const
	//	    { return dJointGetPistonPositionRate (_id); }
	//
	//	  virtual void setParam (int parameter, dReal value)
	//	  { dJointSetPistonParam (_id, parameter, value); }
	//	  virtual dReal getParam (int parameter) const
	//	    { return dJointGetPistonParam (_id, parameter); }
	//	  // TODO: expose params through methods
	//
	//	  void addForce (dReal force)
	//	  { dJointAddPistonForce (_id, force); }
}
