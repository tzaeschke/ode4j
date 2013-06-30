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
package org.ode4j.ode.internal;

import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeomTransform;
import org.ode4j.ode.internal.Objects_H.dxPosR;
import static org.ode4j.ode.OdeMath.*;


/**
 * ****************************************************************************
 * dxGeomTransform class
 * @deprecated TZ: This is deprecated, see Wiki.
 * geom transform
 */
public class DxGeomTransform extends DxGeom implements DGeomTransform {
	private DxGeom obj;		// object that is being transformed
	private boolean cleanup;		// 1 to destroy obj when destroyed
	private boolean infomode;		// 1 to put Tx geom in dContactGeom g1

	// cached final object transform (body tx + relative tx). this is set by
	// computeAABB(), and it is valid while the AABB is valid.
	private dxPosR transform_posr;

	DxGeomTransform (DxSpace space) //: dxGeom (space,1)
	{
		super(space, true);
		type = dGeomTransformClass;
		obj = null;
		cleanup = false;
		infomode = false;
		transform_posr = new dxPosR();
		//  dSetZero (transform_posr.pos,4);
		//  dRSetIdentity (transform_posr.R);
	}


	//dxGeomTransform::~dxGeomTransform()
	@Override
	public void DESTRUCTOR()
	{
		if (obj!=null && cleanup) obj.DESTRUCTOR();//delete obj;
		super.DESTRUCTOR();
	}


	@Override
	void computeAABB()
	{
		if (obj==null) {
			_aabb.setZero();//dSetZero (_aabb,6);
			return;
		}

		// backup the relative pos and R pointers of the encapsulated geom object
		dxPosR posr_bak = obj._final_posr;

		// compute temporary pos and R for the encapsulated geom object
		computeFinalTx();
		obj._final_posr = transform_posr;

		// compute the AABB
		obj.computeAABB();
		//memcpy (aabb,obj.aabb,6*sizeof(dReal));
		_aabb.set(obj._aabb);

		// restore the pos and R
		obj._final_posr = posr_bak;
	}


	// utility function for dCollideTransform() : compute final pos and R
	// for the encapsulated geom object

	private void computeFinalTx()
	{
		dMULTIPLY0_331 (transform_posr.pos,_final_posr.R,obj._final_posr.pos);
		//  transform_posr.pos[0] += final_posr.pos[0];
		//  transform_posr.pos[1] += final_posr.pos[1];
		//  transform_posr.pos[2] += final_posr.pos[2];
		transform_posr.pos.add(_final_posr.pos);
		dMULTIPLY0_333 (transform_posr.R,_final_posr.R,obj._final_posr.R);
	}

	//****************************************************************************
	// collider function:
	// this collides a transformed geom with another geom. the other geom can
	// also be a transformed geom, but this case is not handled specially.
	static class CollideTransform implements DColliderFn {
		//public int dCollideTransform (dxGeom o1, dxGeom o2, int flags,
		//	       dContactGeom *contact, int skip)
		public int dCollideTransform (DxGeom o1, DxGeom o2, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			//dIASSERT (skip >= (int)sizeof(dContactGeom));
			dIASSERT(skip==1);
			//dIASSERT (o1.type == dGeomTransformClass);
	
			DxGeomTransform tr = (DxGeomTransform) o1;
			if (tr.obj==null) return 0;
			dUASSERT (tr.obj.parent_space==null,
			"GeomTransform encapsulated object must not be in a space");
			dUASSERT (tr.obj.body==null,
					"GeomTransform encapsulated object must not be attached " +
			"to a body");
	
			// backup the relative pos and R pointers of the encapsulated geom object,
			// and the body pointer
			dxPosR posr_bak = tr.obj._final_posr;
			DxBody bodybak = tr.obj.body;
	
			// compute temporary pos and R for the encapsulated geom object.
			// note that final_pos and final_R are valid if no GEOM_AABB_BAD flag,
			// because computeFinalTx() will have already been called in
			// dxGeomTransform::computeAABB()
	
			if ((tr._gflags & GEOM_AABB_BAD)!=0) tr.computeFinalTx();
			tr.obj._final_posr = tr.transform_posr;
			tr.obj.body = o1.body;
	
			// do the collision
			int n = dCollide (tr.obj,o2,flags,contacts,skip);
	
			// if required, adjust the 'g1' values in the generated contacts so that
			// thay indicated the GeomTransform object instead of the encapsulated
			// object.
			if (tr.infomode) {
				for (int i=0; i<n; i++) {
					DContactGeom c = contacts.get(skip*i);
					c.g1 = o1;
				}
			}
	
			// restore the pos, R and body
			tr.obj._final_posr = posr_bak;
			tr.obj.body = bodybak;
			return n;
		}
		
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideTransform((DxGeom)o1, (DxGeom)o2, flags, contacts, 1);
		}
	}

	//****************************************************************************
	// public API

	//dGeom dCreateGeomTransform (dSpace space)
	public static DxGeomTransform dCreateGeomTransform (DxSpace space)
	{
		return new DxGeomTransform (space);
	}


	//void dGeomTransformSetGeom (dGeom g, dGeom obj)
	public void dGeomTransformSetGeom (DxGeom obj2)
	{
		//  dUASSERT (g && g.type == dGeomTransformClass,
		//	    "argument not a geom transform");
		//  dxGeomTransform *tr = (dxGeomTransform*) g;
		if (obj!=null && cleanup) obj.DESTRUCTOR();//delete tr.obj;
		obj = obj2;
	}


	//dGeom dGeomTransformGetGeom (dGeom g)
	public DxGeom dGeomTransformGetGeom ()
	{
		//  dUASSERT (g && g.type == dGeomTransformClass,
		//	    "argument not a geom transform");
		//  dxGeomTransform *tr = (dxGeomTransform*) g;
		return obj;
	}


	//void dGeomTransformSetCleanup (dGeom g, int mode)
	public void dGeomTransformSetCleanup (boolean mode)
	{
		//  dUASSERT (g && g.type == dGeomTransformClass,
		//	    "argument not a geom transform");
		//  dxGeomTransform *tr = (dxGeomTransform*) g;
		cleanup = mode;
	}


	//int dGeomTransformGetCleanup (dGeom g)
	public boolean dGeomTransformGetCleanup ()
	{
		//  dUASSERT (g && g.type == dGeomTransformClass,
		//	    "argument not a geom transform");
		//  dxGeomTransform *tr = (dxGeomTransform*) g;
		return cleanup;
	}


	//void dGeomTransformSetInfo (dGeom g, int mode)
	public void dGeomTransformSetInfo (boolean mode)
	{
		//  dUASSERT (g && g.type == dGeomTransformClass,
		//	    "argument not a geom transform");
		//  dxGeomTransform *tr = (dxGeomTransform*) g;
		infomode = mode;
	}


	//int dGeomTransformGetInfo (dGeom g)
	public boolean dGeomTransformGetInfo ()
	{
		//  dUASSERT (g && g.type == dGeomTransformClass,
		//	    "argument not a geom transform");
		//  dxGeomTransform *tr = (dxGeomTransform*) g;
		return infomode;
	}
	
	
	
	public void setGeom (DGeom geom)
	    { dGeomTransformSetGeom ((DxGeom) geom); }
	public DGeom getGeom() 
	    { return dGeomTransformGetGeom (); }

	public void setCleanup (boolean mode)
	    { dGeomTransformSetCleanup (mode); }
	public boolean getCleanup ()
	    { return dGeomTransformGetCleanup (); }

	public void setInfo (boolean mode)
	    { dGeomTransformSetInfo (mode); }
	public boolean getInfo()
	    { return dGeomTransformGetInfo (); }

}