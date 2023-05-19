/**
 * ----------------------------------------------------------------------------
 * This source file is part of the ODE4J library (ported to
 * Java from the GIMPACT Library).
 * 
 * For the latest info on ODE4J, see http://www.ode4j.org/
 * For the latest info on GIMPACT, see http://gimpact.sourceforge.net/
 * 
 * Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
 * email: projectileman@yahoo.com
 * Copyright of ODE4J (c) 2009-2014 Tilmann Zäschke.
 * email: ode4j.gmx.de
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file GIMPACT-LICENSE-LGPL.TXT and LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file GIMPACT-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-BSD.TXT, LICENSE.TXT and 
 * ODE4J-LICENSE-BSD.TXT for more details.
 * 
 * ----------------------------------------------------------------------------
 */
package org.ode4j.ode.internal.gimpact;

import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.cpp4j.java.RefFloat;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec4f;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_DATA;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA;

import static org.ode4j.ode.internal.gimpact.GimGeometry.*;

/**
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
*/
class GimTrimeshRayCollision {


//	#include "GIMPACT/gim_trimesh.h"


	/**
	 *  Trimesh Ray Collisions
	 * @param trimesh trimesh
	 * @param origin origin
	 * @param dir dir
	 * @param tmax tmax
	 * @param contact contact
	 * @return 1 if the ray collides, else 0
	 */
	//int gim_trimesh_ray_collision(GimTrimesh trimesh,vec3f origin,vec3f dir, 
	//GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact)
	static int gim_trimesh_ray_collision(GimTrimesh trimesh,vec3f origin,vec3f dir, 
			float tmax, GIM_TRIANGLE_RAY_CONTACT_DATA contact)
	{
	    GimDynArrayInt collision_result = GimDynArrayInt.GIM_CREATE_BOXQUERY_LIST();

	    trimesh.m_aabbset.gim_aabbset_ray_collision(origin,dir,tmax,collision_result);

		if(collision_result.size()==0)
		{
			collision_result.GIM_DYNARRAY_DESTROY();
		    return 0;
		}

		//collide triangles

		int[] boxesresult = collision_result.GIM_DYNARRAY_POINTER();
		GIM_TRIANGLE_DATA tridata = new GIM_TRIANGLE_DATA();
		vec3f pout = new vec3f();
		RefFloat tparam = new RefFloat(0), u = new RefFloat(0), v = new RefFloat(0);
		RefBoolean does_intersect = new RefBoolean(false);

		trimesh.gim_trimesh_locks_work_data();

		for(int i=0;i<collision_result.size();i++)
		{
			trimesh.gim_trimesh_get_triangle_data(boxesresult[i],tridata);
	        
			// flip plane for correct result in ODE
			// for more info: martijn@bytehazard.com
			vec4f flippedPlane = new vec4f();
			VEC_SCALE_4(flippedPlane, -1.0f, tridata.m_planes.m_planes[0]);
	        
			GimTriCollision.RAY_TRIANGLE_INTERSECTION(origin,dir,
					tridata.m_vertices[0],tridata.m_vertices[1],tridata.m_vertices[2],
					flippedPlane,pout,u,v,tparam,tmax,does_intersect);
			if(does_intersect.b)
			{
			    contact.tparam = tparam.d;
			    contact.u = u.d;
			    contact.v = v.d;
			    contact.m_face_id = boxesresult[i];
			    VEC_COPY(contact.m_point,pout);
			    VEC_COPY(contact.m_normal,flippedPlane);

			    trimesh.gim_trimesh_unlocks_work_data();
			    collision_result.GIM_DYNARRAY_DESTROY();
			    return 1;
			}
		}

		trimesh.gim_trimesh_unlocks_work_data();
		collision_result.GIM_DYNARRAY_DESTROY();
		return 0;//no collisiion
	}


	/** 
	 * Trimesh Ray Collisions closest.
	 * Find the closest primitive collided by the ray.
	 *
	 * @param trimesh trimesh
	 * @param origin origin
	 * @param dir dir
	 * @param tmax tmax
	 * @param contact contact
	 * @return 1 if the ray collides, else 0
	 */
	//int gim_trimesh_ray_closest_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir,
	//GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact)
	static int gim_trimesh_ray_closest_collision(GimTrimesh trimesh,vec3f origin,vec3f dir, 
			float tmax, GIM_TRIANGLE_RAY_CONTACT_DATA contact)
	{
	    GimDynArrayInt collision_result = GimDynArrayInt.GIM_CREATE_BOXQUERY_LIST();

		trimesh.m_aabbset.gim_aabbset_ray_collision(origin,dir,tmax,collision_result);

		if(collision_result.size()==0)
		{
			collision_result.GIM_DYNARRAY_DESTROY();
		    return 0;
		}

		//collide triangles

		int[] boxesresult = collision_result.GIM_DYNARRAY_POINTER();
		GIM_TRIANGLE_DATA tridata = new GIM_TRIANGLE_DATA();
		vec3f pout = new vec3f();
		RefFloat tparam = new RefFloat(0), u = new RefFloat(0), v = new RefFloat(0);
		RefBoolean does_intersect = new RefBoolean(false);
		contact.tparam = tmax + 0.1f;

		trimesh.gim_trimesh_locks_work_data();

		for(int i=0;i<collision_result.size();i++)
		{
			trimesh.gim_trimesh_get_triangle_data(boxesresult[i],tridata);

			// flip plane for correct result in ODE
			// for more info: martijn@bytehazard.com
			vec4f flippedPlane = new vec4f();
			VEC_SCALE_4(flippedPlane, -1.0f, tridata.m_planes.m_planes[0]);

			GimTriCollision.RAY_TRIANGLE_INTERSECTION(origin,dir,
					tridata.m_vertices[0],tridata.m_vertices[1],tridata.m_vertices[2],
					flippedPlane,pout,u,v,tparam,tmax,does_intersect);
			if(does_intersect.b && (tparam.d < contact.tparam))
			{
	            contact.tparam = tparam.d;
			    contact.u = u.d;
			    contact.v = v.d;
			    contact.m_face_id = boxesresult[i];
			    VEC_COPY(contact.m_point,pout);
			    VEC_COPY(contact.m_normal,flippedPlane);
			}
		}

		trimesh.gim_trimesh_unlocks_work_data();
		collision_result.GIM_DYNARRAY_DESTROY();
		if(contact.tparam > tmax) return 0;
		return 1;
	}

}
