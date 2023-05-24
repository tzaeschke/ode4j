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

import org.ode4j.math.DVector3C;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.gimpact.GimContact;
import org.ode4j.ode.internal.gimpact.GimDynArray;

/**
 * TriMesh code by Erwin de Vries.
 * Ported to Java by Tilmann Zaeschke
 *
 */
class CollideTrimeshSphere implements DColliderFn {


//	#include <ode/collision.h>
//	#include <ode/matrix.h>
//	#include <ode/rotation.h>
//	#include <ode/odemath.h>
//	#include "collision_util.h"
//
//	#if dTRIMESH_ENABLED
//	#include "collision_trimesh_internal.h"
//
//	#if dTRIMESH_OPCODE
//	#define MERGECONTACTS
//	//#define MERGECONTACTNORMALS
//
//	// Ripped from Opcode 1.1.
//	static bool GetContactData(const dVector3& Center, dReal Radius, const dVector3 Origin, const dVector3 Edge0, const dVector3 Edge1, dReal& Dist, dReal& u, dReal& v){
//	  
//	        // now onto the bulk of the collision...
//
//		dVector3 Diff;
//		Diff[0] = Origin[0] - Center[0];
//		Diff[1] = Origin[1] - Center[1];
//		Diff[2] = Origin[2] - Center[2];
//		Diff[3] = Origin[3] - Center[3];
//
//		dReal A00 = dDOT(Edge0, Edge0);
//		dReal A01 = dDOT(Edge0, Edge1);
//		dReal A11 = dDOT(Edge1, Edge1);
//
//		dReal B0 = dDOT(Diff, Edge0);
//		dReal B1 = dDOT(Diff, Edge1);
//
//		dReal C = dDOT(Diff, Diff);
//
//		dReal Det = dFabs(A00 * A11 - A01 * A01);
//		u = A01 * B1 - A11 * B0;
//		v = A01 * B0 - A00 * B1;
//
//		dReal DistSq;
//
//		if (u + v <= Det){
//			if(u < REAL(0.0)){
//				if(v < REAL(0.0)){  // region 4
//					if(B0 < REAL(0.0)){
//						v = REAL(0.0);
//						if (-B0 >= A00){
//							u = REAL(1.0);
//							DistSq = A00 + REAL(2.0) * B0 + C;
//						}
//						else{
//							u = -B0 / A00;
//							DistSq = B0 * u + C;
//						}
//					}
//					else{
//						u = REAL(0.0);
//						if(B1 >= REAL(0.0)){
//							v = REAL(0.0);
//							DistSq = C;
//						}
//						else if(-B1 >= A11){
//							v = REAL(1.0);
//							DistSq = A11 + REAL(2.0) * B1 + C;
//						}
//						else{
//							v = -B1 / A11;
//							DistSq = B1 * v + C;
//						}
//					}
//				}
//				else{  // region 3
//					u = REAL(0.0);
//					if(B1 >= REAL(0.0)){
//						v = REAL(0.0);
//						DistSq = C;
//					}
//					else if(-B1 >= A11){
//						v = REAL(1.0);
//						DistSq = A11 + REAL(2.0) * B1 + C;
//					}
//					else{
//						v = -B1 / A11;
//						DistSq = B1 * v + C;
//					}
//				}
//			}
//			else if(v < REAL(0.0)){  // region 5
//				v = REAL(0.0);
//				if (B0 >= REAL(0.0)){
//					u = REAL(0.0);
//					DistSq = C;
//				}
//				else if (-B0 >= A00){
//					u = REAL(1.0);
//					DistSq = A00 + REAL(2.0) * B0 + C;
//				}
//				else{
//					u = -B0 / A00;
//					DistSq = B0 * u + C;
//				}
//			}
//			else{  // region 0
//				// minimum at interior point
//				if (Det == REAL(0.0)){
//					u = REAL(0.0);
//					v = REAL(0.0);
//					DistSq = FLT_MAX;
//				}
//				else{
//					dReal InvDet = REAL(1.0) / Det;
//					u *= InvDet;
//					v *= InvDet;
//					DistSq = u * (A00 * u + A01 * v + REAL(2.0) * B0) + v * (A01 * u + A11 * v + REAL(2.0) * B1) + C;
//				}
//			}
//		}
//		else{
//			dReal Tmp0, Tmp1, Numer, Denom;
//
//			if(u < REAL(0.0)){  // region 2
//				Tmp0 = A01 + B0;
//				Tmp1 = A11 + B1;
//				if (Tmp1 > Tmp0){
//					Numer = Tmp1 - Tmp0;
//					Denom = A00 - REAL(2.0) * A01 + A11;
//					if (Numer >= Denom){
//						u = REAL(1.0);
//						v = REAL(0.0);
//						DistSq = A00 + REAL(2.0) * B0 + C;
//					}
//					else{
//						u = Numer / Denom;
//						v = REAL(1.0) - u;
//						DistSq = u * (A00 * u + A01 * v + REAL(2.0) * B0) + v * (A01 * u + A11 * v + REAL(2.0) * B1) + C;
//					}
//				}
//				else{
//					u = REAL(0.0);
//					if(Tmp1 <= REAL(0.0)){
//						v = REAL(1.0);
//						DistSq = A11 + REAL(2.0) * B1 + C;
//					}
//					else if(B1 >= REAL(0.0)){
//						v = REAL(0.0);
//						DistSq = C;
//					}
//					else{
//						v = -B1 / A11;
//						DistSq = B1 * v + C;
//					}
//				}
//			}
//			else if(v < REAL(0.0)){  // region 6
//				Tmp0 = A01 + B1;
//				Tmp1 = A00 + B0;
//				if (Tmp1 > Tmp0){
//					Numer = Tmp1 - Tmp0;
//					Denom = A00 - REAL(2.0) * A01 + A11;
//					if (Numer >= Denom){
//						v = REAL(1.0);
//						u = REAL(0.0);
//						DistSq = A11 + REAL(2.0) * B1 + C;
//					}
//					else{
//						v = Numer / Denom;
//						u = REAL(1.0) - v;
//						DistSq =  u * (A00 * u + A01 * v + REAL(2.0) * B0) + v * (A01 * u + A11 * v + REAL(2.0) * B1) + C;
//					}
//				}
//				else{
//					v = REAL(0.0);
//					if (Tmp1 <= REAL(0.0)){
//						u = REAL(1.0);
//						DistSq = A00 + REAL(2.0) * B0 + C;
//					}
//					else if(B0 >= REAL(0.0)){
//						u = REAL(0.0);
//						DistSq = C;
//					}
//					else{
//						u = -B0 / A00;
//						DistSq = B0 * u + C;
//					}
//				}
//			}
//			else{  // region 1
//				Numer = A11 + B1 - A01 - B0;
//				if (Numer <= REAL(0.0)){
//					u = REAL(0.0);
//					v = REAL(1.0);
//					DistSq = A11 + REAL(2.0) * B1 + C;
//				}
//				else{
//					Denom = A00 - REAL(2.0) * A01 + A11;
//					if (Numer >= Denom){
//						u = REAL(1.0);
//						v = REAL(0.0);
//						DistSq = A00 + REAL(2.0) * B0 + C;
//					}
//					else{
//						u = Numer / Denom;
//						v = REAL(1.0) - u;
//						DistSq = u * (A00 * u + A01 * v + REAL(2.0) * B0) + v * (A01 * u + A11 * v + REAL(2.0) * B1) + C;
//					}
//				}
//			}
//		}
//
//		Dist = dSqrt(dFabs(DistSq));
//
//		if (Dist <= Radius){
//			Dist = Radius - Dist;
//			return true;
//		}
//		else return false;
//	}
//
//	int dCollideSTL(dxGeom* g1, dxGeom* SphereGeom, int Flags, dContactGeom* Contacts, int Stride){
//		dIASSERT (Stride >= (int)sizeof(dContactGeom));
//		dIASSERT (g1->type == dTriMeshClass);
//		dIASSERT (SphereGeom->type == dSphereClass);
//		dIASSERT ((Flags & NUMC_MASK) >= 1);
//
//		dxTriMesh* TriMesh = (dxTriMesh*)g1;
//
//		// Init
//		const dVector3& TLPosition = *(const dVector3*)dGeomGetPosition(TriMesh);
//		const dMatrix3& TLRotation = *(const dMatrix3*)dGeomGetRotation(TriMesh);
//
//		const unsigned uiTLSKind = TriMesh->getParentSpaceTLSKind();
//		dIASSERT(uiTLSKind == SphereGeom->getParentSpaceTLSKind()); // The colliding spaces must use matching cleanup method
//		TrimeshCollidersCache *pccColliderCache = GetTrimeshCollidersCache(uiTLSKind);
//		SphereCollider& Collider = pccColliderCache->_SphereCollider;
//
//		const dVector3& Position = *(const dVector3*)dGeomGetPosition(SphereGeom);
//		dReal Radius = dGeomSphereGetRadius(SphereGeom);
//
//		// Sphere
//		Sphere Sphere;
//		Sphere.mCenter.x = Position[0];
//		Sphere.mCenter.y = Position[1];
//		Sphere.mCenter.z = Position[2];
//		Sphere.mRadius = Radius;
//
//		Matrix4x4 amatrix;
//
//		// TC results
//		if (TriMesh->doSphereTC) {
//			dxTriMesh::SphereTC* sphereTC = 0;
//			for (int i = 0; i < TriMesh->SphereTCCache.size(); i++){
//				if (TriMesh->SphereTCCache[i].Geom == SphereGeom){
//					sphereTC = &TriMesh->SphereTCCache[i];
//					break;
//				}
//			}
//
//			if (!sphereTC){
//				TriMesh->SphereTCCache.push(dxTriMesh::SphereTC());
//
//				sphereTC = &TriMesh->SphereTCCache[TriMesh->SphereTCCache.size() - 1];
//				sphereTC->Geom = SphereGeom;
//			}
//			
//			// Intersect
//			Collider.SetTemporalCoherence(true);
//			Collider.Collide(*sphereTC, Sphere, TriMesh->Data->BVTree, null, 
//							 &MakeMatrix(TLPosition, TLRotation, amatrix));
//		}
//		else {
//			Collider.SetTemporalCoherence(false);
//			Collider.Collide(pccColliderCache->defaultSphereCache, Sphere, TriMesh->Data->BVTree, null, 
//							 &MakeMatrix(TLPosition, TLRotation, amatrix));
//	 	}
//
//		if (! Collider.GetContactStatus()) {
//			// no collision occurred
//			return 0;
//		}
//
//		// get results
//		int TriCount = Collider.GetNbTouchedPrimitives();
//		const int* Triangles = (const int*)Collider.GetTouchedPrimitives();
//
//		if (TriCount != 0){
//			if (TriMesh->ArrayCallback != null){
//				TriMesh->ArrayCallback(TriMesh, SphereGeom, Triangles, TriCount);
//			}
//
//			int OutTriCount = 0;
//			for (int i = 0; i < TriCount; i++){
//				if (OutTriCount == (Flags & NUMC_MASK)){
//					break;
//				}
//
//				const int TriIndex = Triangles[i];
//
//				dVector3 dv[3];
//				if (!Callback(TriMesh, SphereGeom, TriIndex))
//					continue;
//				
//				FetchTriangle(TriMesh, TriIndex, TLPosition, TLRotation, dv);
//
//				dVector3& v0 = dv[0];
//				dVector3& v1 = dv[1];
//				dVector3& v2 = dv[2];
//
//				dVector3 vu;
//				vu[0] = v1[0] - v0[0];
//				vu[1] = v1[1] - v0[1];
//				vu[2] = v1[2] - v0[2];
//				vu[3] = REAL(0.0);
//
//				dVector3 vv;
//				vv[0] = v2[0] - v0[0];
//				vv[1] = v2[1] - v0[1];
//				vv[2] = v2[2] - v0[2];
//				vv[3] = REAL(0.0);
//
//				// Get plane coefficients
//				dVector4 Plane;
//				dCROSS(Plane, =, vu, vv);
//
//				// Even though all triangles might be initially valid, 
//				// a triangle may degenerate into a segment after applying 
//				// space transformation.
//				if (!dSafeNormalize3(Plane)) {
//					continue;
//				}
//
//				/* If the center of the sphere is within the positive halfspace of the
//					* triangle's plane, allow a contact to be generated.
//					* If the center of the sphere made it into the positive halfspace of a
//					* back-facing triangle, then the physics update and/or velocity needs
//					* to be adjusted (penetration has occured anyway).
//					*/
//			  
//				dReal side = dDOT(Plane,Position) - dDOT(Plane, v0);
//
//				if(side < REAL(0.0)) {
//					continue;
//				}
//
//				dReal Depth;
//				dReal u, v;
//				if (!GetContactData(Position, Radius, v0, vu, vv, Depth, u, v)){
//					continue;	// Sphere doesn't hit triangle
//				}
//
//				if (Depth < REAL(0.0)){
//					continue; // Negative depth does not produce a contact
//				}
//
//				dVector3 ContactPos;
//
//				dReal w = REAL(1.0) - u - v;
//				ContactPos[0] = (v0[0] * w) + (v1[0] * u) + (v2[0] * v);
//				ContactPos[1] = (v0[1] * w) + (v1[1] * u) + (v2[1] * v);
//				ContactPos[2] = (v0[2] * w) + (v1[2] * u) + (v2[2] * v);
//
//				// Depth returned from GetContactData is depth along 
//				// contact point - sphere center direction
//				// we'll project it to contact normal
//				dVector3 dir;
//				dir[0] = Position[0]-ContactPos[0];
//				dir[1] = Position[1]-ContactPos[1];
//				dir[2] = Position[2]-ContactPos[2];
//				dReal dirProj = dDOT(dir, Plane) / dSqrt(dDOT(dir, dir));
//				
//				// Since Depth already had a requirement to be non-negative,
//				// negative direction projections should not be allowed as well,
//				// as otherwise the multiplication will result in negative contact depth.
//				if (dirProj < REAL(0.0)){
//					continue; // Zero contact depth could be ignored
//				}
//
//				dContactGeom* Contact = SAFECONTACT(Flags, Contacts, OutTriCount, Stride);
//
//				Contact->pos[0] = ContactPos[0];
//				Contact->pos[1] = ContactPos[1];
//				Contact->pos[2] = ContactPos[2];
//				Contact->pos[3] = REAL(0.0);
//
//				// Using normal as plane (reversed)
//				Contact->normal[0] = -Plane[0];
//				Contact->normal[1] = -Plane[1];
//				Contact->normal[2] = -Plane[2];
//				Contact->normal[3] = REAL(0.0);
//
//				Contact->depth = Depth * dirProj;
//				//Contact->depth = Radius - side; // (mg) penetration depth is distance along normal not shortest distance
//				
//	#if !defined MERGECONTACTS	// Merge all contacts into 1
//	            Contact->g1 = TriMesh;
//	            Contact->g2 = SphereGeom;
//
//	            Contact->side2 = -1;
//	#endif // Otherwise assigned later
//
//	            Contact->side1 = TriIndex;
//
//				OutTriCount++;
//			}
//	#if defined MERGECONTACTS	// Merge all contacts into 1
//			if (OutTriCount > 0){
//				dContactGeom* Contact = SAFECONTACT(Flags, Contacts, 0, Stride);
//	            Contact->g1 = TriMesh;
//	            Contact->g2 = SphereGeom;
//	            Contact->side2 = -1;
//
//				if (OutTriCount > 1 && !(Flags & CONTACTS_UNIMPORTANT)){
//				    dVector3 pos;
//	                pos[0] = Contact->pos[0];
//	                pos[1] = Contact->pos[1];
//	                pos[2] = Contact->pos[2];
//
//	                dVector3 normal;
//	                normal[0] = Contact->normal[0] * Contact->depth;
//	                normal[1] = Contact->normal[1] * Contact->depth;
//	                normal[2] = Contact->normal[2] * Contact->depth;
//	                
//	                int TriIndex = Contact->side1;
//
//					for (int i = 1; i < OutTriCount; i++){
//						dContactGeom* TempContact = SAFECONTACT(Flags, Contacts, i, Stride);
//						
//						pos[0] += TempContact->pos[0];
//						pos[1] += TempContact->pos[1];
//						pos[2] += TempContact->pos[2];
//						
//						normal[0] += TempContact->normal[0] * TempContact->depth;
//						normal[1] += TempContact->normal[1] * TempContact->depth;
//						normal[2] += TempContact->normal[2] * TempContact->depth;
//
//	                    TriIndex = (TriMesh->TriMergeCallback) ? TriMesh->TriMergeCallback(TriMesh, TriIndex, TempContact->side1) : -1;
//					}
//				
//	                Contact->side1 = TriIndex;
//
//	                Contact->pos[0] = pos[0] / OutTriCount;
//					Contact->pos[1] = pos[1] / OutTriCount;
//					Contact->pos[2] = pos[2] / OutTriCount;
//					
//					// Remember to divide in square space.
//					Contact->depth = dSqrt(dDOT(normal, normal) / OutTriCount);
//
//					if (Contact->depth > dEpsilon) { // otherwise the normal is too small
//	                    dVector3Copy(normal, Contact->normal);
//						dNormalize3(Contact->normal);
//					} // otherwise original Contact's normal would be used and it should be already normalized
//				}
//
//				return 1;
//			}
//			else return 0;
//	#elif defined MERGECONTACTNORMALS	// Merge all normals, and distribute between all contacts
//			if (OutTriCount != 0){
//	            if (OutTriCount != 1 && !(Flags & CONTACTS_UNIMPORTANT)){
//					dVector3 Normal;
//
//	                dContactGeom* FirstContact = SAFECONTACT(Flags, Contacts, 0, Stride);
//					Normal[0] = FirstContact->normal[0] * FirstContact->depth;
//					Normal[1] = FirstContact->normal[1] * FirstContact->depth;
//					Normal[2] = FirstContact->normal[2] * FirstContact->depth;
//					Normal[3] = FirstContact->normal[3] * FirstContact->depth;
//
//					for (int i = 1; i < OutTriCount; i++){
//						dContactGeom* Contact = SAFECONTACT(Flags, Contacts, i, Stride);
//
//						Normal[0] += Contact->normal[0] * Contact->depth;
//						Normal[1] += Contact->normal[1] * Contact->depth;
//						Normal[2] += Contact->normal[2] * Contact->depth;
//						Normal[3] += Contact->normal[3] * Contact->depth;
//					}
//
//	                dNormalize3(Normal);
//
//					for (int i = 0; i < OutTriCount; i++){
//						dContactGeom* Contact = SAFECONTACT(Flags, Contacts, i, Stride);
//
//						Contact->normal[0] = Normal[0];
//						Contact->normal[1] = Normal[1];
//						Contact->normal[2] = Normal[2];
//						Contact->normal[3] = Normal[3];
//					}
//				}
//
//				return OutTriCount;
//			}
//			else return 0;
//	#else   // none of MERGECONTACTS and MERGECONTACTNORMALS // Just return
//
//	        return OutTriCount;
//	#endif	// MERGECONTACTS
//		}
//		else return 0;
//	}
//	#endif // dTRIMESH_OPCODE

//	#if dTRIMESH_GIMPACT
	//int dCollideSTL(dxGeom* g1, dxGeom* SphereGeom, int Flags, dContactGeom* Contacts, int Stride)
	int dCollideSTL(DxGimpact g1, DxSphere SphereGeom, int Flags, DContactGeomBuffer Contacts, int Stride)
	{
		Common.dIASSERT (Stride == 1);//(int)sizeof(dContactGeom));
		//		dIASSERT (g1->type == dTriMeshClass);
		//		dIASSERT (SphereGeom->type == dSphereClass);
		Common.dIASSERT ((Flags & DxGeom.NUMC_MASK) >= 1);
		
		DxGimpact TriMesh = g1;
	    DVector3C Position = SphereGeom.getPosition();
		double Radius = SphereGeom.getRadius();
	 	//Create contact list
	    GimDynArray<GimContact> trimeshcontacts;
	    trimeshcontacts = GimContact.GIM_CREATE_CONTACT_LIST();

		g1.recomputeAABB();
		SphereGeom.recomputeAABB();

	    //Collide trimeshes
	    CollisionTrimeshGimpact.gim_trimesh_sphere_collisionODE(TriMesh.m_collision_trimesh(),Position,Radius,trimeshcontacts);
		// TZ: Only in ode4j, call callbacks, see issue #76
		g1.applyCallbacksToContacts(SphereGeom, trimeshcontacts, true);

	    if(trimeshcontacts.size() == 0)
	    {
	    	trimeshcontacts.GIM_DYNARRAY_DESTROY();
	        return 0;
	    }

	    GimContact[] ptrimeshcontacts = trimeshcontacts.GIM_DYNARRAY_POINTER();
		int contactcount = trimeshcontacts.size();

		DxGIMCContactAccessor contactaccessor = new DxGIMCContactAccessor(ptrimeshcontacts, g1, SphereGeom, -1);
		contactcount = DxGImpactContactsExportHelper.ExportMaxDepthGImpactContacts(contactaccessor, contactcount, Flags, Contacts, Stride);

		trimeshcontacts.GIM_DYNARRAY_DESTROY();

	    return contactcount;
	}
//	#endif // dTRIMESH_GIMPACT

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideSTL((DxGimpact)o1, (DxSphere)o2, flags, contacts, 1);
	}

//	#endif // dTRIMESH_ENABLED
}
