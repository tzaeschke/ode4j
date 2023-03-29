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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

public interface DRay extends DGeom {

	void setLength (double length);
	double getLength();

	/** 
	 * Set origin and direction. Origin is the same as setPosition(). 
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param dx dx
	 * @param dy dy
	 * @param dz dz
	 */
	void set (double px, double py, double pz, double dx, double dy, double dz);

	/** 
	 * Set origin and direction. Origin is the same as setPosition(). 
	 * @param p p
	 * @param d d
	 */
	void set (DVector3C p, DVector3C d);
	
	/** 
	 * Get origin and direction. Origin is the same as getPosition(). 
	 * Direction is the same as getDirection(). 
	 * @param start start 
	 * @param dir direction
	 */
	void get (DVector3 start, DVector3 dir);

	/** 
	 * Get direction. Direction is the same as the 2nd parameter in get(). 
	 * @return direction
	 */ 
	DVector3C getDirection();

	/**
	 * Set/get ray flags that influence ray collision detection.
	 * These flags are currently only noticed by the trimesh collider, because
	 * they can make a major differences there.
	 * @return first contact setting
	 */
	boolean getFirstContact();
	void setFirstContact(boolean firstContact);
	/**
	 * Set/get ray flags that influence ray collision detection.
	 * These flags are currently only noticed by the trimesh collider, because
	 * they can make a major differences there.
	 * @return backface cull setting
	 */
	boolean getBackfaceCull();
	void setBackfaceCull(boolean backfaceCull);
	/**
	 * Set/get ray flags that influence ray collision detection.
	 * These flags are currently only noticed by the trimesh collider, because
	 * they can make a major differences there.
	 * @return closest hit setting
	 */
	boolean getClosestHit();
	void setClosestHit (boolean closestHit);


//	  // intentionally undefined, don't use these
//	  dRay (dRay &);
//	  void operator= (dRay &);
//
//	public:
//	  dRay() { }
//	  dRay (dSpaceID space, dReal length)
//	    { _id = dCreateRay (space,length); }
//
//	  void create (dSpaceID space, dReal length) {
//	    if (_id) dGeomDestroy (_id);
//	    _id = dCreateRay (space,length);
//	  }
//
//	  void setLength (dReal length)
//	    { dGeomRaySetLength (_id, length); }
//	  dReal getLength()
//	    { return dGeomRayGetLength (_id); }
//
//	  void set (dReal px, dReal py, dReal pz, dReal dx, dReal dy, dReal dz)
//	    { dGeomRaySet (_id, px, py, pz, dx, dy, dz); }
//	  void get (dVector3 start, dVector3 dir)
//	    { dGeomRayGet (_id, start, dir); }
//
//	  void setParams (int firstContact, int backfaceCull)
//	    { dGeomRaySetParams (_id, firstContact, backfaceCull); }
//	  void getParams (int *firstContact, int *backfaceCull)
//	    { dGeomRayGetParams (_id, firstContact, backfaceCull); }
//	  void setClosestHit (int closestHit)
//	    { dGeomRaySetClosestHit (_id, closestHit); }
//	  int getClosestHit()
//	    { return dGeomRayGetClosestHit (_id); }
}
