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
 * collision space.
 */
public interface DSpace extends DGeom {

	public void setCleanup (boolean mode);
	public boolean getCleanup();

	public void add (DGeom x);
	public void remove (DGeom x);
	public boolean query (DGeom x);

	public int getNumGeoms();
	public DGeom getGeom (int i);

	public void collide (Object data, DNearCallback callback);
	public void setManualCleanup(int mode);
	public int getManualCleanup();

	
	// intentionally undefined, don't use these
//	dSpace (dSpace &);
//	void operator= (dSpace &);

//	protected
//		// the default constructor is protected so that you
//		// can't instance this class. you must instance one
//		// of its subclasses instead.
//		dSpace () { _id = null; }
//
//	public
//	//TODO ?
////		dSpace id() //const
////		{ return (dSpace) _id; }
//	//TODO ?
////		operator dSpace() const
////		{ return (dSpace) _id; }
//
//		void setCleanup (int mode)
//		{ dSpaceSetCleanup (id(), mode); }
//		int getCleanup()
//		{ return dSpaceGetCleanup (id()); }
//
//		void add (dGeom x)
//		{ dSpaceAdd (id(), x); }
//		void remove (dGeom x)
//		{ dSpaceRemove (id(), x); }
//		int query (dGeom x)
//		{ return dSpaceQuery (id(),x); }
//
//		int getNumGeoms()
//		{ return dSpaceGetNumGeoms (id()); }
//		dGeom getGeom (int i)
//		{ return dSpaceGetGeom (id(),i); }
//
//		void collide (Object data, dNearCallback callback)
//		{ dSpaceCollide (id(),data,callback); }
}
