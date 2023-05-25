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

import java.util.Arrays;
import java.util.Comparator;

import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.internal.gimpact.GimContact;
import org.ode4j.ode.internal.gimpact.GimDynArray;
import org.ode4j.ode.internal.trimesh.DxTriMesh;

import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.DxGeom.NUMC_MASK;
import static org.ode4j.ode.internal.gimpact.GimDynArray.GIM_DYNARRAY_POINTER;

/**
 *
 * @author Tilmann Zaeschke
 */
class CollideTrimeshTrimesh implements DColliderFn {

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideTTL((DxGimpact)o1, (DxGimpact)o2, flags, contacts, 1);
	}

	/////////////////////////////////////////////////////////////////////////

	//#if dTRIMESH_GIMPACT

	//
	// GIMPACT TRIMESH-TRIMESH COLLIDER
	//

	/*extern */
	@SuppressWarnings("deprecation")
	int dCollideTTL(DxGeom g1, DxGeom g2, int Flags, DContactGeomBuffer Contacts, int Stride)
	{
		dIASSERT (Stride >= 1);//(int)sizeof(dContactGeom));
		// dIASSERT (g1->type == dTriMeshClass);
		// dIASSERT (g2->type == dTriMeshClass);
		dIASSERT ((Flags & NUMC_MASK) >= 1);

		int result = 0;

		DxTriMesh triMesh1 = (DxTriMesh) g1;
		DxTriMesh triMesh2 = (DxTriMesh) g2;
		//Create contact list
		GimDynArray<GimContact> trimeshContacts;
		trimeshContacts = GimContact.GIM_CREATE_CONTACT_LIST();

		triMesh1.recomputeAABB();
		triMesh2.recomputeAABB();

		//Collide trimeshes
		//gim_trimesh_trimesh_collision(triMesh1.m_collision_trimesh, triMesh2.m_collision_trimesh, trimeshContacts);
		triMesh1.m_collision_trimesh().gim_trimesh_trimesh_collision(triMesh2.m_collision_trimesh(), trimeshContacts);

		// TZ: only in ode4j, see issue #76
		triMesh1.applyCallbacksToContacts(triMesh2, trimeshContacts, true);
		triMesh2.applyCallbacksToContacts(triMesh1, trimeshContacts, false);

		int contactCount = trimeshContacts.size();

		if (contactCount != 0)
		{
			GimContact[] pTriMeshContacts = GIM_DYNARRAY_POINTER(trimeshContacts);

			DxGImpactContactsExportHelper.GImpactContactAccessorI contactAccessor = new DxGIMCContactAccessor(pTriMeshContacts, g1, g2);
			int culledContactCount = DxGImpactContactsExportHelper.ExportMaxDepthGImpactContacts(contactAccessor, contactCount, Flags, Contacts, Stride);

			result = culledContactCount;
		}

		trimeshContacts.GIM_DYNARRAY_DESTROY();

		// TODO CHECK-TZ remove this, #22 has been resolved
		int contactcount = trimeshContacts.size();
		int maxcontacts = Flags & NUMC_MASK;
		if (contactcount > maxcontacts)
		{
			if (OdeConfig.ENABLE_CONTACT_SORTING) {
				Arrays.sort((Object[])trimeshContacts.GIM_DYNARRAY_POINTER(), 0, contactcount, new Comparator<Object>() {
					@Override
					public int compare(Object o1, Object o2) {
						return Float.compare(((GimContact) o2).getDepth(), ((GimContact) o1).getDepth());
					}
				});
			}
		}

		return result;
	}

	//#endif // dTRIMESH_GIMPACT

	//#endif // dTRIMESH_ENABLED
}
