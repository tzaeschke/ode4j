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
 * From odecpp.h.
 */
public interface DJointGroup {

	void empty();
	void clear();
	void destroy();

	//	//TZ private dJointGroup _id;
//	private dxJointGroup _id;
//
//	// intentionally undefined, don't use these
//	//  dJointGroup (const dJointGroup &);
//	//  void operator= (const dJointGroup &);
//
//	//public:
//	//  dJointGroup (int dummy_arg=0)
//	//  { _id = dJointGroupCreate (0); }
//	//  dJointGroup (int dummy_arg)
//	//  { _id = dJointGroupCreate (0); }
//	public dJointGroup ()
//	{ _id = dxJointGroup.dJointGroupCreate (0); }
//
//	//~dJointGroup()
//	@Override
//	protected void DESTRUCTOR()
//	{ _id.dJointGroupDestroy (); super.DESTRUCTOR(); }
//	//  void create (int dummy_arg=0) {
//	public  void create () {
//		if (_id!=null) _id.dJointGroupDestroy ();
//		_id = dxJointGroup.dJointGroupCreate (0);
//	}
//
//	public dJointGroup id() //const
//	{ return _id; }
//	//TODO 
//	//  operator dJointGroup() //const
//	//    { return _id; }
//
//	public void empty()
//	{ dJointGroupEmpty (_id); }
//	void clear()
//	{ empty(); }
}
