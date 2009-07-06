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

/**
 * From the Wiki: 
 * The fixed joint maintains a fixed relative position and orientation between
 * two bodies, or between a body and the static environment. Using this joint
 * is almost never a good idea in practice, except when debugging. 
 * If you need two bodies to be glued together it is better to represent that 
 * as a single body. 
 */
public interface DFixedJoint extends DJoint {

	void set();

	void setFixed();

	//	 // intentionally undefined, don't use these
	//	  dFixedJoint (const dFixedJoint &);
	//	  void operator = (const dFixedJoint &);
	//
	//	public:
	//	  dFixedJoint() { }
	//	  dFixedJoint (dWorld world, dJointGroup group=0)
	//	    { _id = dJointCreateFixed (world, group); }
	//	  dFixedJoint (dWorld& world, dJointGroup group=0)
	//	    { _id = dJointCreateFixed (world, group); }
	//
	//	  void create (dWorld world, dJointGroup group=0) {
	//	    if (_id) dJointDestroy (_id);
	//	    _id = dJointCreateFixed (world, group);
	//	  }
	//	  void create (dWorld& world, dJointGroup group=0)
	//	    { create(world.id(), group); }
	//
	//	  void set()
	//	    { dJointSetFixed (_id); }
	//
	//	  virtual void setParam (int parameter, dReal value)
	//	    { dJointSetFixedParam (_id, parameter, value); }
	//
	//	  virtual dReal getParam (int parameter) const
	//	    { return dJointGetFixedParam (_id, parameter); }
	//	  // TODO: expose params through methods
}
