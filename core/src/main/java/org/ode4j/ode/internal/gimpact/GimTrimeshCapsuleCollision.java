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

import static org.ode4j.ode.internal.gimpact.GimGeometry.*;

import org.ode4j.ode.internal.cpp4j.java.ObjArray;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_DATA;

/**
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
*/
public class GimTrimeshCapsuleCollision {

	/** Capsule struct. */
	public static class GIM_CAPSULE_DATA
	{
	    public float m_radius;
	    public final vec3f m_point1 = new vec3f();
	    public final vec3f m_point2 = new vec3f();

		public GIM_CAPSULE_DATA() {}
	}
	//typedef struct _GIM_CAPSULE_DATA GIM_CAPSULE_DATA;

	//#define CALC_CAPSULE_AABB(capsule,aabb)\
	private static void CALC_CAPSULE_AABB(GIM_CAPSULE_DATA capsule, aabb3f aabb)
	{
	    if(capsule.m_point1.f[0]<capsule.m_point2.f[0])
	    {
	        aabb.minX = capsule.m_point1.f[0] - capsule.m_radius;
	        aabb.maxX = capsule.m_point2.f[0] + capsule.m_radius;
	    }
	    else
	    {
	        aabb.minX = capsule.m_point2.f[0] - capsule.m_radius;
	        aabb.maxX = capsule.m_point1.f[0] + capsule.m_radius;
	    }
	    if(capsule.m_point1.f[1]<capsule.m_point2.f[1])
	    {
	        aabb.minY = capsule.m_point1.f[1] - capsule.m_radius;
	        aabb.maxY = capsule.m_point2.f[1] + capsule.m_radius;
	    }
	    else
	    {
	        aabb.minY = capsule.m_point2.f[1] - capsule.m_radius;
	        aabb.maxY = capsule.m_point1.f[1] + capsule.m_radius;
	    }
	    if(capsule.m_point1.f[2]<capsule.m_point2.f[2])
	    {
	        aabb.minZ = capsule.m_point1.f[2] - capsule.m_radius;
	        aabb.maxZ = capsule.m_point2.f[2] + capsule.m_radius;
	    }
	    else
	    {
	        aabb.minZ = capsule.m_point2.f[2] - capsule.m_radius;
	        aabb.maxZ = capsule.m_point1.f[2] + capsule.m_radius;
	    }
	}







//	#endif // GIM_TRI_CAPSULE_COLLISION_H_INCLUDED

	
	/**
	 * Utility function for find the closest point between a segment and a triangle.
	 * <p> Postcondition: 
	 * The contacts array is not set to 0. It adds additional contacts.
	 * 
	 * @param triangle
	 * @param s1
	 * @param s2
	 * @param contacts Contains the closest points on the segment (1,2), and the
	 * normal points to segment, and m_depth contains the distance
	*/
	//void gim_closest_point_triangle_segment(GIM_TRIANGLE_DATA * triangle, vec3f s1,vec3f s2, GDYNAMIC_ARRAY * contacts)
	static void gim_closest_point_triangle_segment(GIM_TRIANGLE_DATA triangle, vec3f s1,vec3f s2, 
			GimDynArray<GimContact> contacts)
	{
	    vec3f[] segment_points = { new vec3f(), new vec3f(), new vec3f(), new vec3f() };//new vec3f[4];// = {{0}};  //TZ TODO ???
	    vec3f[] closest_points = { new vec3f(), new vec3f() };//new vec3f[2];// = {{0}};
	    int intersection_type, out_edge = 10;
	    float dis, dis_temp,perpend;
	    vec4f sdiff = new vec4f();

	    dis = DISTANCE_PLANE_POINT(triangle.m_planes.m_planes[0],s1);
	    dis_temp = DISTANCE_PLANE_POINT(triangle.m_planes.m_planes[0],s2);

	    if(dis<=0.0f && dis_temp<=0.0f) return;

	    VEC_DIFF(sdiff,s2,s1);
	    perpend = VEC_DOT(sdiff,triangle.m_planes.m_planes[0]);

	    if(!IS_ZERO(perpend)) // Not perpendicular
	    {
	        if(dis<dis_temp)
	        {
	            VEC_COPY(closest_points[0],s1);
	        }
	        else
	        {
	            dis = dis_temp;
	            VEC_COPY(closest_points[0],s2);
	        }

	        //Testing segment vertices over triangle
	        if(dis>=0.0f && dis_temp>=0.0f)
	        {
	            out_edge = POINT_IN_HULL_TZ(closest_points[0],triangle.m_planes.m_planes,1,3);

	            if(out_edge==0)//Point over face
	            {
	                GimContact.GIM_PUSH_CONTACT(contacts,closest_points[0] ,triangle.m_planes.m_planes[0] ,dis,null,null, 0,0);
	                return;
	            }
	        }
	        else
	        {

	            PLANE_CLIP_SEGMENT(s1,s2,triangle.m_planes.m_planes[0],closest_points[1]);

	            out_edge = POINT_IN_HULL_TZ(closest_points[1],triangle.m_planes.m_planes,1,3);

	            if(out_edge==0)//Point over face
	            {
	            	GimContact.GIM_PUSH_CONTACT(contacts,closest_points[0] ,triangle.m_planes.m_planes[0] ,dis,null,null, 0,0);
	                return;
	            }
	        }

	    }
	    else // Perpendicular Face
	    {
	        //out_edge=10
	        //Clip segment by triangle
	    //    Edge1
	    	intersection_type = PLANE_CLIP_SEGMENT_CLOSEST(s1,s2,triangle.m_planes.m_planes[1],
	    			segment_points[0],segment_points[1]);
	        if(intersection_type==0||intersection_type==1)
	        {
	            out_edge = 0;
	            VEC_COPY(closest_points[0],segment_points[0]);
	        }
	        else
	        {
	            //Edge2
	        	intersection_type = PLANE_CLIP_SEGMENT_CLOSEST(segment_points[0],segment_points[1],triangle.m_planes.m_planes[2],
	            		segment_points[2],segment_points[3]);
	            if(intersection_type==0||intersection_type==1)
	            {
	                out_edge = 1;
	                VEC_COPY(closest_points[0],segment_points[3]);
	            }
	            else
	            {
	                //Edge3
	            	intersection_type = PLANE_CLIP_SEGMENT_CLOSEST(segment_points[2],segment_points[3],triangle.m_planes.m_planes[3],
	                		closest_points[0],closest_points[1]);
	                if(intersection_type==0||intersection_type==1)
	                {
	                    out_edge = 2;
	                }
	            }
	        }
	        //POST closest_points[0] and closest_points[1] are inside the triangle, if out_edge>2
	        if(out_edge>2) // Over triangle
	        {
	            dis = DISTANCE_PLANE_POINT(triangle.m_planes.m_planes[0],closest_points[0]);
	            GimContact.GIM_PUSH_CONTACT(contacts,closest_points[0] ,triangle.m_planes.m_planes[0] ,dis,null,null, 0,0);
	            GimContact.GIM_PUSH_CONTACT(contacts,closest_points[1] ,triangle.m_planes.m_planes[0] ,dis,null,null, 0,0);
	            return;
	        }
	    }

	    //Find closest edges
	    out_edge = 10;
	    dis = G_REAL_INFINITY;
	    int i;
	    for(i=0;i<3;i++)
	    {
	        SEGMENT_COLLISION(s1,s2,triangle.m_vertices[i],triangle.m_vertices[(i+1)%3],segment_points[0],segment_points[1]);
	        VEC_DIFF(sdiff,segment_points[0],segment_points[1]);
	        dis_temp = VEC_DOT(sdiff,sdiff);
	        if(dis_temp< dis)
	        {
	            dis = dis_temp;
	            out_edge = i;
	            VEC_COPY(closest_points[0],segment_points[0]);
	            VEC_COPY(closest_points[1],sdiff);//normal
	        }
	    }
	    if(out_edge>2) return ;// ???? ASSERT this please

	    if(IS_ZERO(dis))
	    {
	        //Set face plane
	        GimContact.GIM_PUSH_CONTACT(contacts,closest_points[0] ,triangle.m_planes.m_planes[0] ,0.0f,null,null, 0,0);

	    }
	    else
	    {
	        dis = GIM_SQRT(dis);//GIM_SQRT(dis,dis);
	        VEC_SCALE(closest_points[1],(1.0f/dis),closest_points[1]);//normal
	        GimContact.GIM_PUSH_CONTACT(contacts,closest_points[0] ,closest_points[1],dis,null,null, 0,0);
	    }
	}


	/**
	 * Utility function for find the closest point between a capsule and a triangle
	 * <p>Postcondition: The contacts array is not set to 0. It adds aditional contacts
	 * @param triangle
	 * @param capsule
	 * @param contacts Contains the closest points on the capsule, and the normal points to triangle
	 * @return 1 if the triangle collides the capsule
	 */
	//int gim_triangle_capsule_collision(GIM_TRIANGLE_DATA * triangle, GIM_CAPSULE_DATA * capsule, 
	//GDYNAMIC_ARRAY * contacts)
	static int gim_triangle_capsule_collision(GIM_TRIANGLE_DATA triangle, GIM_CAPSULE_DATA capsule, 
			GimDynArray<GimContact> contacts)
	{
	    int old_contact_size = contacts.size();
	    gim_closest_point_triangle_segment(triangle,capsule.m_point1,capsule.m_point2,contacts);
	    
	    if (contacts.size() == old_contact_size)
	    {
	        return 0;
	    }

		ObjArray<GimContact> pcontacts = contacts.GIM_DYNARRAY_POINTER_V();//GIM_CONTACT ,);
	    pcontacts.inc(old_contact_size);//+= old_contact_size;

	    if(pcontacts.at0().m_depth > capsule.m_radius)
	    {
	        contacts.m_size = old_contact_size;
	        return 0;
	    }

	    vec3f vec = new vec3f();
	    while(old_contact_size<contacts.size())
	    {
	    	GimContact pcontact = pcontacts.at0();
	        //Scale the normal for pointing to triangle
	        VEC_SCALE(pcontact.m_normal,-1.0f,pcontact.m_normal);
	        //Fix the contact point
	        VEC_SCALE(vec,capsule.m_radius,pcontact.m_normal);
	        VEC_SUM(pcontact.m_point,vec,pcontact.m_point);
	        //Fix the depth
	        pcontact.m_depth = capsule.m_radius - pcontact.m_depth;

	        pcontacts.inc();//++;
	        old_contact_size++;
	    }

	    return 1;
	}


	/**
	 * Trimesh Capsule collision.
	 * Find the closest primitive collided by the ray.
	 * @param trimesh
	 * @param capsule
	 * @param contact
	 * @param contacts A GIM_CONTACT array. Must be initialized
	 */
	static void gim_trimesh_capsule_collision(GimTrimesh trimesh, GIM_CAPSULE_DATA capsule, GimDynArray<GimContact> contacts)
	{
	    contacts.m_size = 0;

	    aabb3f test_aabb = new aabb3f();
	    CALC_CAPSULE_AABB(capsule,test_aabb);

		GimDynArrayInt collision_result = GimDynArrayInt.GIM_CREATE_BOXQUERY_LIST();

		trimesh.m_aabbset.gim_aabbset_box_collision(test_aabb, collision_result);

		if(collision_result.size()==0)
		{
			collision_result.GIM_DYNARRAY_DESTROY();
		}

		//collide triangles
		//Locks trimesh
		trimesh.gim_trimesh_locks_work_data();
		 //dummy contacts
	    GimDynArray<GimContact> dummycontacts = GimContact.GIM_CREATE_CONTACT_LIST();

		int cresult;
		//unsigned 
		int i;
		int[] boxesresult = collision_result.GIM_DYNARRAY_POINTER();
		GIM_TRIANGLE_DATA tri_data = new GIM_TRIANGLE_DATA();
		int old_contact_size;
		ObjArray<GimContact> pcontacts;

		for(i=0;i<collision_result.size();i++)
		{
		    old_contact_size = dummycontacts.size();
		    trimesh.gim_trimesh_get_triangle_data(boxesresult[i],tri_data);
			cresult = gim_triangle_capsule_collision(tri_data, capsule, dummycontacts);
			if(cresult!=0)
			{
			    pcontacts = dummycontacts.GIM_DYNARRAY_POINTER_V();
	            pcontacts.inc(old_contact_size);//+= old_contact_size;
			    while(old_contact_size<dummycontacts.size())
	            {
					GimContact pcontact = pcontacts.at0();
	                pcontact.m_handle1 = trimesh;
	                pcontact.m_handle2 = capsule;
	                pcontact.m_feature1 = boxesresult[i];
	                pcontact.m_feature2 = 0;
	                pcontacts.inc();//++;
	                old_contact_size++;
	            }
			}
		}
		///unlocks
		trimesh.gim_trimesh_unlocks_work_data();
		///Destroy box result
		collision_result.GIM_DYNARRAY_DESTROY();

		 //merge contacts
	    GimContact.gim_merge_contacts(dummycontacts,contacts);

	    //Destroy dummy
	    dummycontacts.GIM_DYNARRAY_DESTROY();
	}

	private GimTrimeshCapsuleCollision() {}
}
