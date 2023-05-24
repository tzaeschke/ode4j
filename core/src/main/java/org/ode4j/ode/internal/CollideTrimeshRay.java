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

import org.ode4j.math.DVector3;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.gimpact.GimTriCollision;
import org.ode4j.ode.internal.trimesh.DxTriMesh;

/**
 * TriMesh code by Erwin de Vries.
 * Ported to Java by Tilmann Zaeschke
 *
 */
class CollideTrimeshRay implements DColliderFn {



	//	#include <ode/collision.h>
	//	#include <ode/matrix.h>
	//	#include <ode/rotation.h>
	//	#include <ode/odemath.h>
	//	#include "config.h"
	//
	//	#if dTRIMESH_ENABLED
	//
	//	#include "collision_util.h"
	//	#include "collision_trimesh_internal.h"
	//
	//	#if dTRIMESH_OPCODE
	//	int dCollideRTL(dxGeom* g1, dxGeom* RayGeom, int Flags, dContactGeom* Contacts, int Stride){
	//		dIASSERT (Stride >= (int)sizeof(dContactGeom));
	//		dIASSERT (g1->type == dTriMeshClass);
	//		dIASSERT (RayGeom->type == dRayClass);
	//		dIASSERT ((Flags & NUMC_MASK) >= 1);
	//
	//		dxTriMesh* TriMesh = (dxTriMesh*)g1;
	//
	//		const dVector3& TLPosition = *(const dVector3*)dGeomGetPosition(TriMesh);
	//		const dMatrix3& TLRotation = *(const dMatrix3*)dGeomGetRotation(TriMesh);
	//
	//		const unsigned uiTLSKind = TriMesh->getParentSpaceTLSKind();
	//		dIASSERT(uiTLSKind == RayGeom->getParentSpaceTLSKind()); // The colliding spaces must use matching cleanup method
	//		TrimeshCollidersCache *pccColliderCache = GetTrimeshCollidersCache(uiTLSKind);
	//		RayCollider& Collider = pccColliderCache->_RayCollider;
	//
	//		dReal Length = dGeomRayGetLength(RayGeom);
	//
	//		int FirstContact, BackfaceCull;
	//		dGeomRayGetParams(RayGeom, &FirstContact, &BackfaceCull);
	//		int ClosestHit = dGeomRayGetClosestHit(RayGeom);
	//
	//		Collider.SetFirstContact(FirstContact != 0);
	//		Collider.SetClosestHit(ClosestHit != 0);
	//		Collider.SetCulling(BackfaceCull != 0);
	//		Collider.SetMaxDist(Length);
	//
	//		dVector3 Origin, Direction;
	//		dGeomRayGet(RayGeom, Origin, Direction);
	//
	//		/* Make Ray */
	//		Ray WorldRay;
	//		WorldRay.mOrig.x = Origin[0];
	//		WorldRay.mOrig.y = Origin[1];
	//		WorldRay.mOrig.z = Origin[2];
	//		WorldRay.mDir.x = Direction[0];
	//		WorldRay.mDir.y = Direction[1];
	//		WorldRay.mDir.z = Direction[2];
	//
	//		/* Intersect */
	//		Matrix4x4 amatrix;
	//	        int TriCount = 0;
	//	        if (Collider.Collide(WorldRay, TriMesh->Data->BVTree, &MakeMatrix(TLPosition, TLRotation, amatrix))) {
	//	                TriCount = pccColliderCache->Faces.GetNbFaces();
	//	        }
	//
	//	        if (TriCount == 0) {
	//	                return 0;
	//	        }
	//		
	//		const CollisionFace* Faces = pccColliderCache->Faces.GetFaces();
	//
	//		int OutTriCount = 0;
	//		for (int i = 0; i < TriCount; i++) {
	//			if (TriMesh->RayCallback == null ||
	//	                    TriMesh->RayCallback(TriMesh, RayGeom, Faces[i].mFaceID,
	//	                                         Faces[i].mU, Faces[i].mV)) {
	//				const int& TriIndex = Faces[i].mFaceID;
	//				if (!Callback(TriMesh, RayGeom, TriIndex)) {
	//	                                continue;
	//	                        }
	//
	//				dContactGeom* Contact = SAFECONTACT(Flags, Contacts, OutTriCount, Stride);
	//
	//				dVector3 dv[3];
	//				FetchTriangle(TriMesh, TriIndex, TLPosition, TLRotation, dv);
	//
	//				dVector3 vu;
	//				vu[0] = dv[1][0] - dv[0][0];
	//				vu[1] = dv[1][1] - dv[0][1];
	//				vu[2] = dv[1][2] - dv[0][2];
	//				vu[3] = REAL(0.0);
	//					
	//				dVector3 vv;
	//				vv[0] = dv[2][0] - dv[0][0];
	//				vv[1] = dv[2][1] - dv[0][1];
	//				vv[2] = dv[2][2] - dv[0][2];
	//				vv[3] = REAL(0.0);
	//
	//				dCROSS(Contact->normal, =, vv, vu);	// Reversed
	//
	//				// Even though all triangles might be initially valid, 
	//				// a triangle may degenerate into a segment after applying 
	//				// space transformation.
	//				if (dSafeNormalize3(Contact->normal))
	//				{
	//					// No sense to save on single type conversion in algorithm of this size.
	//					// If there would be a custom typedef for distance type it could be used 
	//					// instead of dReal. However using float directly is the loss of abstraction 
	//					// and possible loss of precision in future.
	//					/*float*/ dReal T = Faces[i].mDistance;
	//					Contact->pos[0] = Origin[0] + (Direction[0] * T);
	//					Contact->pos[1] = Origin[1] + (Direction[1] * T);
	//					Contact->pos[2] = Origin[2] + (Direction[2] * T);
	//					Contact->pos[3] = REAL(0.0);
	//
	//					Contact->depth = T;
	//					Contact->g1 = TriMesh;
	//					Contact->g2 = RayGeom;
	//					Contact->side1 = TriIndex;
	//					Contact->side2 = -1;
	//						
	//					OutTriCount++;
	//
	//					// Putting "break" at the end of loop prevents unnecessary checks on first pass and "continue"
	//					if (OutTriCount >= (Flags & NUMC_MASK)) {
	//						break;
	//					}
	//				}
	//			}
	//		}
	//		return OutTriCount;
	//	}
	//	#endif // dTRIMESH_OPCODE

	//	#if dTRIMESH_GIMPACT
	//int dCollideRTL(dxGeom* g1, dxGeom* RayGeom, int Flags, dContactGeom* Contacts, int Stride)
	int dCollideRTL(DxTriMesh g1, DxRay RayGeom, int Flags, DContactGeomBuffer Contacts, int Stride)
	{
		Common.dIASSERT (Stride == 1);//(int)sizeof(dContactGeom));
		//		dIASSERT (g1->type == dTriMeshClass);
		//		dIASSERT (RayGeom->type == dRayClass);
		Common.dIASSERT ((Flags & DxGeom.NUMC_MASK) >= 1);

		DxGimpact TriMesh = (DxGimpact) g1;

		double Length = RayGeom.getLength();
		boolean FirstContact = RayGeom.getFirstContact();
		boolean BackfaceCull = RayGeom.getBackfaceCull();
		//TODO CHECK-TZ GimPAct is ignoring first contact AND BackfaceCull
		//TODO remove/report?		boolean FirstContact = RayGeom.getFirstContact();
		//TODO remove/report?		boolean BackfaceCull = RayGeom.getBackfaceCull();
		boolean ClosestHit = RayGeom.getClosestHit();//dGeomRayGetClosestHit(RayGeom);
		DVector3 Origin = new DVector3(), Direction = new DVector3();
		RayGeom.get(Origin, Direction);

		int intersect=0;
		GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA contact_data = new GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA();

		if(ClosestHit)
		{
			intersect = CollisionTrimeshGimpact.gim_trimesh_ray_closest_collisionODE(TriMesh.m_collision_trimesh(),
					Origin,Direction,Length,contact_data);
		}
		else
		{
			intersect = CollisionTrimeshGimpact.gim_trimesh_ray_collisionODE(TriMesh.m_collision_trimesh(),
					Origin,Direction,Length,contact_data);
		}

		if(intersect == 0)
		{
			return 0;
		}


		if (TriMesh.RayCallback() == null ||
				TriMesh.RayCallback().call(TriMesh, RayGeom, contact_data.getFaceID(),
				contact_data.getU(), contact_data.getV()) != 0)
		{
			// ode4j fix: see issue #76
			// TODO TZ this just returns "0" if the callback for the first "hit" returns false.
			//     It should probably check for other contacts. However, apparently even for closestHit=false,
			//     there is only ever one contact created.
			if (!TriMesh.invokeCallback(RayGeom, contact_data.getFaceID())) {
				return 0;
			}
			DContactGeom Contact = Contacts.get();//&( Contacts[ 0 ] );
			Contact.pos.set(contact_data.getPoint().f);//VEC_COPY(Contact.pos,contact_data.getPoint());
			Contact.normal.set(contact_data.getNormal().f);//VEC_COPY(Contact.normal,contact_data.getNormal());
			Contact.depth = contact_data.getTParam();
			Contact.g1 = TriMesh;
			Contact.g2 = RayGeom;
			Contact.side1 = contact_data.getFaceID();
			Contact.side2 = -1;
			return 1;
		}

		return 0;
	}
	//	#endif  // dTRIMESH_GIMPACT
	//
	//	#endif // dTRIMESH_ENABLED


	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideRTL((DxTriMesh) o1, (DxRay)o2, flags, contacts,1);
	}

}
