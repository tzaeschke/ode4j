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

public interface DBox extends DGeom {

	void setLengths (double lx, double ly, double lz);
	void getLengths (DVector3 result);
	void setLengths (DVector3C sides);
	DVector3C getLengths ();
	double getPointDepth(DVector3C p);

//	  // intentionally undefined, don't use these
//	  dBox (dBox &);
//	  void operator= (dBox &);
//
//	public:
//	  dBox () { }
//	  dBox (dSpace space, dReal lx, dReal ly, dReal lz)
//	    { _id = dCreateBox (space,lx,ly,lz); }
//
//	  void create (dSpace space, dReal lx, dReal ly, dReal lz) {
//	    if (_id) dGeomDestroy (_id);
//	    _id = dCreateBox (space,lx,ly,lz);
//	  }
//
//	  void setLengths (dReal lx, dReal ly, dReal lz)
//	    { dGeomBoxSetLengths (_id, lx, ly, lz); }
//	  void getLengths (dVector3 result) const
//	    { dGeomBoxGetLengths (_id,result); }
}
