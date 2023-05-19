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
package org.ode4j.ode.internal;

import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DColliderFn;

class CollideSpaceGeom implements DColliderFn {

	@Override
	public int dColliderFn (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return dCollideSpaceGeom((DxGeom)o1, (DxGeom)o2, flags, contacts, 1);
	}

	/** ********************* COLLIDER stuff from collision_kernel.cpp **********/

	// ******* TZ: Commented out, because it doesn't seem to be used.
	
//	static volatile atomicptr s_cachedPosR = 0; // dxPosR *
	//TODO make final non-volatile!?
//	static volatile AtomicReference<dxPosR> s_cachedPosR = new AtomicReference<dxPosR>(); // dxPosR *
//	
//	//TODO is this the right place for this method?
//	/** @deprecated TZ: Is this still required? */
	public static void dClearPosrCache()
	{
//		if (Common.dATOMICS_ENABLED) {
//			// No threads should be accessing ODE at this time already,
//			// hence variable may be read directly.
//			dxPosR existingPosR = (dxPosR )s_cachedPosR.get();
//
//			if (existingPosR != null)
//			{
//				//TZ dFree(existingPosR, sizeof(dxPosR));
//				existingPosR = null;
//TODO if this is reenabled, enable it also in OdeInit.dCloseODE()!!!
//				s_cachedPosR.set(null);// = 0;
//			}
//		}
	}

	private class SpaceGeomColliderData {
		int flags;			// space left in contacts array
		//TODO array or reference??
		//dContactGeom * contact;
		private DContactGeomBuffer _contacts;
		int skip;
	}


	static void space_geom_collider (Object data, DxGeom o1, DxGeom o2)
	{
		SpaceGeomColliderData d = (SpaceGeomColliderData) data;
		if ((d.flags & DxGeom.NUMC_MASK) != 0) {
//			int n = dCollide (o1,o2,d.flags,d.contact,d.skip);
//			d.contact = CONTACT (d.contact,d.skip*n);
			//
			int n = DxGeom.dCollide (o1,o2,d.flags,d._contacts, d.skip);
			//d.contact = CONTACT (d.contact,d.skip*n);
			//d.contact = ((SpaceGeomColliderData)data).contact[n];
			d._contacts = ((SpaceGeomColliderData)data)._contacts.createView(n);
			d.flags -= n;
		}
	}

//	static int dCollideSpaceGeom (dxGeom *o1, dxGeom *o2, int flags,
//			dContactGeom *contact, int skip)
//	{
//		SpaceGeomColliderData data;
//		data.flags = flags;
//		data.contact = contact;
//		data.skip = skip;
//		dSpaceCollide2 (o1,o2,&data,&space_geom_collider);
//		return (flags & NUMC_MASK) - (data.flags & NUMC_MASK);
//	}
	int dCollideSpaceGeom (DxGeom o1, DxGeom o2, int flags,
			DContactGeomBuffer contact, int skip)
	{
		SpaceGeomColliderData data = new SpaceGeomColliderData();
		data.flags = flags;
		data._contacts = contact;
		//data.setContacts(contact, 0);
		data.skip = skip;
		DxSpace.dSpaceCollide2 (o1,o2,data,new DNearCallback() {
			@Override
			public void call(Object data, DGeom g1, DGeom g2) {
				space_geom_collider(data, (DxGeom)g1, (DxGeom)g2);
			}
		});
		return (flags & DxGeom.NUMC_MASK) - (data.flags & DxGeom.NUMC_MASK);
	}

}
