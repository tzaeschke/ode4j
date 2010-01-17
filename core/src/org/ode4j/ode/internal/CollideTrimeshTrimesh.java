/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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

import org.cpp4j.java.ObjArray;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.gimpact.GimContact;
import org.ode4j.ode.internal.gimpact.GimDynArray;

public class CollideTrimeshTrimesh implements DColliderFn {

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideTTL((DxGimpact)o1, (DxGimpact)o2, flags, contacts, 1);
	}

	
	//
	// GIMPACT TRIMESH-TRIMESH COLLIDER
	//

//	int dCollideTTL(dxGeom* g1, dxGeom* g2, int Flags, dContactGeom* Contacts, int Stride)
	int dCollideTTL(DxGimpact g1, DxGimpact g2, int Flags, DContactGeomBuffer Contacts, int Stride)
	{
		Common.dIASSERT (Stride == 1);//(int)sizeof(dContactGeom));
		//dIASSERT (g1->type == dTriMeshClass);
		//dIASSERT (g2->type == dTriMeshClass);
		Common.dIASSERT ((Flags & DxGeom.NUMC_MASK) >= 1);

	    DxGimpact TriMesh1 = g1;
	    DxGimpact TriMesh2 = g2;
	    //Create contact list
	    GimDynArray<GimContact> trimeshcontacts;
	    trimeshcontacts = GimContact.GIM_CREATE_CONTACT_LIST();

		g1.recomputeAABB();
		g2.recomputeAABB();

	    //Collide trimeshes
		TriMesh1.m_collision_trimesh.gim_trimesh_trimesh_collision(TriMesh2.m_collision_trimesh,trimeshcontacts);

	    if(trimeshcontacts.size() == 0)
	    {
	    	trimeshcontacts.GIM_DYNARRAY_DESTROY();
	        return 0;
	    }

	    ObjArray<GimContact> ptrimeshcontacts = trimeshcontacts.GIM_DYNARRAY_POINTER_V();


		int contactcount = trimeshcontacts.size();
		int maxcontacts = (int)(Flags & DxGeom.NUMC_MASK);
		if (contactcount > maxcontacts)
		{
			contactcount = maxcontacts;
		}

	    DContactGeom pcontact;
		
		for (int i=0;i<contactcount;i++)
		{
	        pcontact = Contacts.getSafe(Flags, i);//SAFECONTACT(Flags, Contacts, i, Stride);

//	        pcontact->pos[0] = ptrimeshcontacts->m_point[0];
//	        pcontact->pos[1] = ptrimeshcontacts->m_point[1];
//	        pcontact->pos[2] = ptrimeshcontacts->m_point[2];
//	        pcontact->pos[3] = 1.0f;   //TODO TZ ?
	        pcontact.pos.set(ptrimeshcontacts.at0().getPoint().f);

//	        pcontact->normal[0] = ptrimeshcontacts->m_normal[0];
//	        pcontact->normal[1] = ptrimeshcontacts->m_normal[1];
//	        pcontact->normal[2] = ptrimeshcontacts->m_normal[2];
//	        pcontact->normal[3] = 0;
	        pcontact.normal.set( ptrimeshcontacts.at0().getNormal().f );

	        pcontact.depth = ptrimeshcontacts.at0().getDepth();
	        pcontact.g1 = g1;
	        pcontact.g2 = g2;
	        pcontact.side1 = ptrimeshcontacts.at0().getFeature1();
	        pcontact.side2 = ptrimeshcontacts.at0().getFeature2();

	        ptrimeshcontacts.inc();//++;
		}

		trimeshcontacts.GIM_DYNARRAY_DESTROY();

	    return contactcount;
	}

	
}
