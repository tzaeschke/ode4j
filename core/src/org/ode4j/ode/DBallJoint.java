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
 * ***********************************************************************
 */
package org.ode4j.ode;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

public interface DBallJoint extends DJoint {

	void setAnchor (double x, double y, double z);
	void setAnchor2(double x, double y, double z);
	void setAnchor (DVector3C a);
	void setAnchor2 (DVector3C a);
	void getAnchor (DVector3 result);
	void getAnchor2 (DVector3 result);


	//	private:
	//		// intentionally undefined, don't use these
	//		dBallJoint (const dBallJoint &);
	//void operator= (const dBallJoint &);
	//
	//public:
	//	dBallJoint() { }
	//dBallJoint (dWorld world, dJointGroup group=0)
	//{ _id = dJointCreateBall (world, group); }
	//dBallJoint (dWorld& world, dJointGroup group=0)
	//{ _id = dJointCreateBall (world.id(), group); }
	//
	//void create (dWorld world, dJointGroup group=0) {
	//	if (_id) dJointDestroy (_id);
	//	_id = dJointCreateBall (world, group);
	//}
	//void create (dWorld& world, dJointGroup group=0)
	//{ create(world.id(), group); }
	//
	//void setAnchor (dReal x, dReal y, dReal z)
	//{ dJointSetBallAnchor (_id, x, y, z); }
	//void setAnchor (const dVector3 a)
	//{ setAnchor (a[0], a[1], a[2]); }
	//void getAnchor (dVector3 result) const
	//{ dJointGetBallAnchor (_id, result); }
	//void getAnchor2 (dVector3 result) const
	//{ dJointGetBallAnchor2 (_id, result); }
	//virtual void setParam (int parameter, dReal value)
	//{ dJointSetBallParam (_id, parameter, value); }
	//virtual dReal getParam (int parameter) const
	//{ return dJointGetBallParam (_id, parameter); }
	//// TODO: expose params through methods
	//
}
