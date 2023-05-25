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

import org.ode4j.math.DVector4;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.gimpact.GimDynArray;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec4f;

/**
 * 	TriMesh - Plane collider by David Walters, July 2006
 *  Ported to Java by Tilmann Zaeschke
 */
class CollideTrimeshPlane implements DColliderFn {


	//	#include <ode/collision.h>
	//	#include <ode/matrix.h>
	//	#include <ode/rotation.h>
	//	#include <ode/odemath.h>
	//	#include "config.h"
	//
	//	#if dTRIMESH_ENABLED
	//
	//	#include "collision_util.h"
	//	#include "collision_std.h"
	//	#include "collision_trimesh_internal.h"

	//	#if dTRIMESH_OPCODE
	//	int dCollideTrimeshPlane( dxGeom *o1, dxGeom *o2, int flags, dContactGeom* contacts, int skip )
	//	{
	//		dIASSERT( skip >= (int)sizeof( dContactGeom ) );
	//		dIASSERT( o1->type == dTriMeshClass );
	//		dIASSERT( o2->type == dPlaneClass );
	//		dIASSERT ((flags & NUMC_MASK) >= 1);
	//
	//		// Alias pointers to the plane and trimesh
	//		dxTriMesh* trimesh = (dxTriMesh*)( o1 );
	//		dxPlane* plane = (dxPlane*)( o2 );
	//
	//		int contact_count = 0;
	//
	//		// Cache the maximum contact count.
	//		const int contact_max = ( flags & NUMC_MASK );
	//
	//		// Cache trimesh position and rotation.
	//		const dVector3& trimesh_pos = *(const dVector3*)dGeomGetPosition( trimesh );
	//		const dMatrix3& trimesh_R = *(const dMatrix3*)dGeomGetRotation( trimesh );
	//
	//		//
	//		// For all triangles.
	//		//
	//
	//		// Cache the triangle count.
	//		const int tri_count = trimesh->Data->Mesh.GetNbTriangles();
	//
	//		VertexPointers VP;
	//		ConversionArea VC;
	//		dReal alpha;
	//		dVector3 vertex;
	//
	//	#if !defined(dSINGLE) || 1
	//		dVector3 int_vertex;		// Intermediate vertex for double precision mode.
	//	#endif // dSINGLE
	//
	//		// For each triangle
	//		for ( int t = 0; t < tri_count; ++t )
	//		{
	//			// Get triangle, which should also use callback.
	//			trimesh->Data->Mesh.GetTriangle( VP, t, VC);
	//
	//			// For each vertex.
	//			for ( int v = 0; v < 3; ++v )
	//			{
	//				//
	//				// Get Vertex
	//				//
	//
	//	#if defined(dSINGLE) && 0 // Always assign via intermediate array as otherwise it is an incapsulation violation
	//
	//				dMULTIPLY0_331( vertex, trimesh_R, (float*)( VP.Vertex[ v ] ) );
	//
	//	#else // dDOUBLE || 1
	//
	//				// OPCODE data is in single precision format.
	//				int_vertex[ 0 ] = VP.Vertex[ v ]->x;
	//				int_vertex[ 1 ] = VP.Vertex[ v ]->y;
	//				int_vertex[ 2 ] = VP.Vertex[ v ]->z;
	//
	//				dMULTIPLY0_331( vertex, trimesh_R, int_vertex );
	//
	//	#endif // dSINGLE/dDOUBLE
	//				
	//				vertex[ 0 ] += trimesh_pos[ 0 ];
	//				vertex[ 1 ] += trimesh_pos[ 1 ];
	//				vertex[ 2 ] += trimesh_pos[ 2 ];
	//
	//
	//				//
	//				// Collision?
	//				//
	//
	//				// If alpha < 0 then point is if front of plane. i.e. no contact
	//				// If alpha = 0 then the point is on the plane
	//				alpha = plane->p[ 3 ] - dDOT( plane->p, vertex );
	//	      
	//				// If alpha > 0 the point is behind the plane. CONTACT!
	//				if ( alpha > 0 )
	//				{
	//					// Alias the contact
	//	                dContactGeom* contact = SAFECONTACT( flags, contacts, contact_count, skip );
	//
	//					contact->pos[ 0 ] = vertex[ 0 ];
	//					contact->pos[ 1 ] = vertex[ 1 ];
	//					contact->pos[ 2 ] = vertex[ 2 ];
	//
	//					contact->normal[ 0 ] = plane->p[ 0 ];
	//					contact->normal[ 1 ] = plane->p[ 1 ];
	//					contact->normal[ 2 ] = plane->p[ 2 ];
	//
	//					contact->depth = alpha;
	//					contact->g1 = trimesh;
	//					contact->g2 = plane;
	//					contact->side1 = t;
	//					contact->side2 = -1;
	//
	//					++contact_count;
	//
	//					// All contact slots are full?
	//					if ( contact_count >= contact_max )
	//						return contact_count; // <=== STOP HERE
	//				}
	//			}
	//		}
	//
	//		// Return contact count.
	//		return contact_count;
	//	}
	//	#endif // dTRIMESH_OPCODE

	//	#if dTRIMESH_GIMPACT
	//int dCollideTrimeshPlane( dxGeom *o1, dxGeom *o2, int flags, dContactGeom* contacts, int skip )
	int dCollideTrimeshPlane( DxGimpact o1, DxPlane o2, int flags, DContactGeomBuffer contacts, int skip )
	{
		Common.dIASSERT( skip == 1);//(int)sizeof( dContactGeom ) );
		Common.dIASSERT ((flags & DxGeom.NUMC_MASK) >= 1);

		// Alias pointers to the plane and trimesh
		DxGimpact trimesh = o1;
		DVector4 plane = new DVector4();
		o2.getParams(plane);

		o1.recomputeAABB();
		o2.recomputeAABB();

		//Find collision

		GimDynArray<vec4f> collision_result = GimTrimesh.GIM_CREATE_TRIMESHPLANE_CONTACTS();

		CollisionTrimeshGimpact.gim_trimesh_plane_collisionODE(trimesh.m_collision_trimesh(), plane, collision_result);

		if(collision_result.size() == 0 )
		{
			collision_result.GIM_DYNARRAY_DESTROY();
			return 0;
		}


		vec4f[] planecontact_results = collision_result.GIM_DYNARRAY_POINTER();
		int contactcount = collision_result.size();

		DxPlaneContactAccessor contactaccessor = new DxPlaneContactAccessor(planecontact_results, plane, o1, o2);
		contactcount = DxGImpactContactsExportHelper.ExportMaxDepthGImpactContacts(contactaccessor, contactcount, flags, contacts, skip);

		collision_result.GIM_DYNARRAY_DESTROY();

		return contactcount;
	}
	//	#endif // dTRIMESH_GIMPACT
	//
	//
	//	#endif // dTRIMESH_ENABLED




	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideTrimeshPlane((DxGimpact)o1, (DxPlane)o2, flags, contacts, 1);
	}
}
