/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;


/**
 * ****************************************************************************
 * dxGeomTransform class
 * @deprecated TZ: This is deprecated, see Wiki.
 */
public interface DGeomTransform extends DGeom {

	  void setGeom (DGeom geom);
	  DGeom getGeom();

	  void setCleanup (boolean mode);
	  boolean getCleanup ();

	  void setInfo (boolean mode);
	  boolean getInfo();

	
//	  // intentionally undefined, don't use these
//	  dGeomTransform (dGeomTransform &);
//	  void operator= (dGeomTransform &);
//
//	public:
//	  dGeomTransform() { }
//	  dGeomTransform (dSpaceID space)
//	    { _id = dCreateGeomTransform (space); }
//
//	  void create (dSpaceID space=0) {
//	    if (_id) dGeomDestroy (_id);
//	    _id = dCreateGeomTransform (space);
//	  }
//
//	  void setGeom (dGeomID geom)
//	    { dGeomTransformSetGeom (_id, geom); }
//	  dGeomID getGeom() const
//	    { return dGeomTransformGetGeom (_id); }
//
//	  void setCleanup (int mode)
//	    { dGeomTransformSetCleanup (_id,mode); }
//	  int getCleanup ()
//	    { return dGeomTransformGetCleanup (_id); }
//
//	  void setInfo (int mode)
//	    { dGeomTransformSetInfo (_id,mode); }
//	  int getInfo()
//	    { return dGeomTransformGetInfo (_id); }
}
