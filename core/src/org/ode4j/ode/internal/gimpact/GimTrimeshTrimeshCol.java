package org.ode4j.ode.internal.gimpact;

import org.cpp4j.java.ObjArray;
import org.cpp4j.java.RefFloat;
import org.cpp4j.java.RefInt;
import org.ode4j.ode.internal.gimpact.GimAABBSet.GIM_PAIR;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec4f;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_CONTACT_DATA;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_DATA;

import static org.ode4j.ode.internal.gimpact.GimGeometry.*;
import static org.ode4j.ode.internal.gimpact.GimTriCollision.*;

public class GimTrimeshTrimeshCol {
	
	// ******************************************************
	// TZ: gim_trimesh_trimesh.collision.cpp
	// ******************************************************
	
	//#define CLASSIFY_TRI_BY_FACE(v1,v2,v3,faceplane,out_of_face)\
	private static void CLASSIFY_TRI_BY_FACE(vec3f v1, vec3f v2, vec3f v3, vec4f faceplane, RefInt out_of_face,
			final vec3f _distances)
	{   
	    _distances.f[0] = DISTANCE_PLANE_POINT(faceplane,v1);
	    _distances.f[1] =  _distances.f[0] * DISTANCE_PLANE_POINT(faceplane,v2);
	    _distances.f[2] =  _distances.f[0] * DISTANCE_PLANE_POINT(faceplane,v3); 
		if(_distances.f[1]>0.0f && _distances.f[2]>0.0f)
		{
		    out_of_face.i = 1;
		}
		else
		{
		    out_of_face.i = 0;
		}
	}


	//! Receives the 3 edge planes
	//#define MOST_DEEP_POINTS(plane,points,point_count,deep_points,deep_points_count,maxdeep)\
	private static int MOST_DEEP_POINTS(vec4f plane, vec3f[] points, final int point_count,
			vec3f[] deep_points, RefFloat maxdeep, int[] _max_candidates)
	{
	    maxdeep.d=-1000.0f;
	    int _k;
		float _dist;
		int deep_points_count = 0;
		for(_k=0;_k<point_count;_k++)
		{
		    _dist = -DISTANCE_PLANE_POINT(plane,points[_k]);
			if(_dist>maxdeep.d)
			{
				maxdeep.d = _dist;
				_max_candidates[0] = _k;
				deep_points_count=1;
			}
			else if((_dist+G_EPSILON)>=maxdeep.d)
			{
			    _max_candidates[deep_points_count] = _k;
				deep_points_count++;
			}
		}
		if(maxdeep.d<0.0f)
	    {
	        deep_points_count = 0;
	    }
	    else
	    {
	        for(_k=0;_k<deep_points_count;_k++)
	        {
	            VEC_COPY(deep_points[_k],points[_max_candidates[_k]]);
	        }
	    }
		return deep_points_count;
	}

	//! Receives the 3 edge planes
	//#define CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(tri_points,tri_edge_planes, clipped_points, clipped_point_count)\
	private static int CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(vec3f[] tri_points, vec4f[] tri_edge_planes, int pos, 
			vec3f[] clipped_points, vec3f[] _temp_clip, vec3f[] _temp_clip2)//RefInt clipped_point_count)
	{
//	    clipped_point_count.i = 0;    
//	    int _temp_clip_count = 0;
//	    PLANE_CLIP_POLYGON(tri_edge_planes[0],tri_points,3,_temp_clip,_temp_clip_count,MAX_TRI_CLIPPING);
//	    if(_temp_clip_count>0)
//	    {
//	        int _temp_clip_count2 = 0;
//	        PLANE_CLIP_POLYGON(tri_edge_planes[1],_temp_clip,_temp_clip_count,_temp_clip2,_temp_clip_count2,MAX_TRI_CLIPPING);
//	        if(_temp_clip_count2>0)
//	        {
//	            PLANE_CLIP_POLYGON(tri_edge_planes[2],_temp_clip2,_temp_clip_count2,clipped_points,clipped_point_count,MAX_TRI_CLIPPING);
//	        }
//	    }
	    int clipped_point_count = 0;    
	    int _temp_clip_count = 0;
	    _temp_clip_count = GimTriCollision.PLANE_CLIP_POLYGON(tri_edge_planes[0+pos],tri_points,3,_temp_clip,MAX_TRI_CLIPPING);
	    if(_temp_clip_count>0)
	    {
	        int _temp_clip_count2 = 0;
	        _temp_clip_count2 = GimTriCollision.PLANE_CLIP_POLYGON(
	        		tri_edge_planes[1+pos],_temp_clip,_temp_clip_count,_temp_clip2,MAX_TRI_CLIPPING);
	        if(_temp_clip_count2>0)
	        {
	        	clipped_point_count = GimTriCollision.PLANE_CLIP_POLYGON(
	        			tri_edge_planes[2+pos],_temp_clip2,_temp_clip_count2,clipped_points,MAX_TRI_CLIPPING);
	        }
	    }
	    return clipped_point_count;
	}



//	int _gim_triangle_triangle_collision(
//			GIM_TRIANGLE_DATA *tri1,
//			GIM_TRIANGLE_DATA *tri2,
//			GIM_TRIANGLE_CONTACT_DATA * contact_data)
	static int _gim_triangle_triangle_collision(
			GIM_TRIANGLE_DATA tri1,
			GIM_TRIANGLE_DATA tri2,
			GIM_TRIANGLE_CONTACT_DATA contact_data)
	{
	    //Cache variables for triangle intersection
	    int[] _max_candidates = new int[MAX_TRI_CLIPPING];
	    vec3f[] _temp_clip = new vec3f[MAX_TRI_CLIPPING];
//	    int _temp_clip_count = 0;
	    vec3f[] _temp_clip2 = new vec3f[MAX_TRI_CLIPPING];
//	    int _temp_clip_count2 = 0;
	    vec3f[] clipped_points2 = new vec3f[MAX_TRI_CLIPPING];
	    vec3f[] deep_points2 = new vec3f[MAX_TRI_CLIPPING];
	    vec3f[] clipped_points1 = new vec3f[MAX_TRI_CLIPPING];
	    vec3f[] deep_points1 = new vec3f[MAX_TRI_CLIPPING];
	    for (int i = 0; i < MAX_TRI_CLIPPING; i++) {
	    	_temp_clip[i] = new vec3f(); //TODO is all this really necessary ??? TZ
	    	_temp_clip2[i] = new vec3f();
	    	clipped_points1[i] = new vec3f();
	    	clipped_points2[i] = new vec3f();
	    	deep_points1[i] = new vec3f();
	    	deep_points2[i] = new vec3f();
	    }



	    //State variabnles
		int mostdir=0;
		int clipped2_count=0;

		//Clip tri2 by tri1 edges

		clipped2_count = CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(tri2.m_vertices,tri1.m_planes.m_planes,1, clipped_points2,
				_temp_clip, _temp_clip2);

		if(clipped2_count == 0 )
		{
		     return 0;//Reject
		}

		//find most deep interval face1
		int deep2_count=0;

		RefFloat maxdeep = new RefFloat();

		deep2_count = MOST_DEEP_POINTS((tri1.m_planes.m_planes[0]), clipped_points2, clipped2_count, 
				deep_points2, maxdeep, _max_candidates);
		if(deep2_count==0)
		{
//		    *perror = 0.0f;
		     return 0;//Reject
		}

		//Normal pointing to triangle1
		VEC_SCALE(contact_data.m_separating_normal,-1.0f,(tri1.m_planes.m_planes[0]));


		//Clip tri1 by tri2 edges

		int clipped1_count=0;

		clipped1_count = CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(tri1.m_vertices,tri2.m_planes.m_planes,1, clipped_points1,
				_temp_clip, _temp_clip2);

		if(clipped2_count == 0 )
		{
//		    *perror = 0.0f;
		     return 0;//Reject
		}


		//find interval face2
		int deep1_count=0;

		RefFloat dist = new RefFloat();

		deep1_count = MOST_DEEP_POINTS((tri2.m_planes.m_planes[0]), clipped_points1, clipped1_count, 
				deep_points1, dist, _max_candidates);

		if(deep1_count==0)
		{
//		    *perror = 0.0f;
		    return 0;
		}

		if(dist.d<maxdeep.d)
		{
			maxdeep = dist;
			mostdir = 1;
			VEC_COPY(contact_data.m_separating_normal,(tri2.m_planes.m_planes[0]));
		}
		//set deep
		contact_data.m_penetration_depth = maxdeep.d;

		////check most dir for contacts
		if(mostdir==0)
		{
		    contact_data.m_point_count = deep2_count;
		    for(mostdir=0;mostdir<deep2_count;mostdir++)
		    {
		        VEC_COPY(contact_data.m_points[mostdir] ,deep_points2[mostdir]);
		    }
		}
		else
		{
			contact_data.m_point_count = deep1_count;
		    for(mostdir=0;mostdir<deep1_count;mostdir++)
		    {
		        VEC_COPY(contact_data.m_points[mostdir] ,deep_points1[mostdir]);
		    }
		}
		return 1;
	}



	//! Finds the contact points from a collision of two triangles
	/*!
	Returns the contact points, the penetration depth and the separating normal of the collision
	between two triangles. The normal is pointing toward triangle 1 from triangle 2
	*/
//	int gim_triangle_triangle_collision(
//			GIM_TRIANGLE_DATA *tri1,
//			GIM_TRIANGLE_DATA *tri2,
//			GIM_TRIANGLE_CONTACT_DATA * contact_data)
	static int gim_triangle_triangle_collision(
			GIM_TRIANGLE_DATA tri1,
			GIM_TRIANGLE_DATA tri2,
			GIM_TRIANGLE_CONTACT_DATA contact_data)
	{
	    vec3f _distances = new vec3f();
	    RefInt out_of_face=new RefInt(0);//TODO remove TZ

	    CLASSIFY_TRI_BY_FACE(tri1.m_vertices[0],tri1.m_vertices[1],tri1.m_vertices[2],tri2.m_planes.m_planes[0],
	    		out_of_face,_distances);
	    if(out_of_face.i==1) return 0;

	    CLASSIFY_TRI_BY_FACE(tri2.m_vertices[0],tri2.m_vertices[1],tri2.m_vertices[2],tri1.m_planes.m_planes[0],
	    		out_of_face, _distances);
	    if(out_of_face.i==1) return 0;

	    return _gim_triangle_triangle_collision(tri1,tri2,contact_data);
	}

	//! Trimesh Trimesh Collisions
	/*!

	In each contact
	<ul>
	<li> m_handle1 points to trimesh1.
	<li> m_handle2 points to trimesh2.
	<li> m_feature1 Is a triangle index of trimesh1.
	<li> m_feature2 Is a triangle index of trimesh2.
	</ul>

	\param trimesh1 Collider
	\param trimesh2 Collidee
	\param contacts A GIM_CONTACT array. Must be initialized
	*/
	//void gim_trimesh_trimesh_collision(GIM_TRIMESH * trimesh1, GIM_TRIMESH * trimesh2, GDYNAMIC_ARRAY * contacts)
	static void gim_trimesh_trimesh_collision(GimTrimesh trimesh1, GimTrimesh trimesh2, GimDynArray<GimContact> contacts)
	{
	    contacts.m_size = 0;
	    GimDynArray<GIM_PAIR> collision_pairs = GimAABBSet.GIM_CREATE_PAIR_SET();

	    GimAABBSet.gim_aabbset_bipartite_intersections(trimesh1.m_aabbset,trimesh2.m_aabbset,collision_pairs);

	    if(collision_pairs.size()==0)
	    {
	    	collision_pairs.GIM_DYNARRAY_DESTROY();
	    	return; //no collisioin
	    }

	    //Locks meshes
	    trimesh1.gim_trimesh_locks_work_data();
	    trimesh2.gim_trimesh_locks_work_data();


	    //pair pointer
	    ObjArray<GIM_PAIR> pairs = collision_pairs.GIM_DYNARRAY_POINTER_V();
	    //dummy contacts
	    GimDynArray<GimContact> dummycontacts;
	    dummycontacts = GimContact.GIM_CREATE_CONTACT_LIST();

	    //Auxiliary triangle data
	    GIM_TRIANGLE_CONTACT_DATA tri_contact_data = new GIM_TRIANGLE_CONTACT_DATA();
	    GIM_TRIANGLE_DATA tri1data = new GIM_TRIANGLE_DATA(),tri2data = new GIM_TRIANGLE_DATA();


	    int ti1,ti2,ci;
	    int colresult;
	    for (int i=0;i<collision_pairs.size(); i++)
	    {
	        ti1 = pairs.at(i).m_index1;
	        ti2 = pairs.at(i).m_index2;
	        //Get triangles data
	        trimesh1.gim_trimesh_get_triangle_data(ti1,tri1data);
	        trimesh2.gim_trimesh_get_triangle_data(ti2,tri2data);

	        //collide triangles
	        colresult = gim_triangle_triangle_collision(tri1data,tri2data,tri_contact_data);
	        if(colresult == 1)
	        {
	            //Add contacts
	            for (ci=0;ci<tri_contact_data.m_point_count ;ci++ )
	            {
	            	GimContact.GIM_PUSH_CONTACT(dummycontacts, tri_contact_data.m_points[ci],
	            			tri_contact_data.m_separating_normal,
	            			tri_contact_data.m_penetration_depth, trimesh1, trimesh2, ti1, ti2);
	            }
	        }
	    }

	    if(dummycontacts.size() == 0) //reject
	    {
	    	dummycontacts.GIM_DYNARRAY_DESTROY();
	    	collision_pairs.GIM_DYNARRAY_DESTROY();
	        return;
	    }
	    //merge contacts
	    GimContact.gim_merge_contacts(dummycontacts,contacts);

	    //Terminate
	    dummycontacts.GIM_DYNARRAY_DESTROY();
	    collision_pairs.GIM_DYNARRAY_DESTROY();

	    //Unlocks meshes
	    trimesh1.gim_trimesh_unlocks_work_data();
	    trimesh2.gim_trimesh_unlocks_work_data();
	}


	//! Trimesh Plane Collisions
	/*!

	\param trimesh
	\param plane vec4f plane
	\param contacts A vec4f array. Must be initialized (~100). Each element have the coordinate point in the first 3 elements, and vec4f[3] has the penetration depth.
	*/
	//void gim_trimesh_plane_collision(GIM_TRIMESH * trimesh,vec4f plane, GDYNAMIC_ARRAY * contacts)
	static void gim_trimesh_plane_collision(GimTrimesh trimesh,vec4f plane, GimDynArray<vec4f> contacts)
	{
	    contacts.m_size = 0;
	    int classify = PLANE_CLASSIFY_BOX(plane,trimesh.m_aabbset.m_global_bound);
	    if(classify>1) return; // in front of plane

	    //Locks mesh
	    trimesh.gim_trimesh_locks_work_data();
	    //Get vertices
	    int i, vertcount = trimesh.m_transformed_vertex_buffer.size();//m_element_count;
	    ObjArray<vec3f> vertices = trimesh.m_transformed_vertex_buffer.GIM_BUFFER_ARRAY_POINTER(0);//vec3f,,0);

	    float dist;

	    for (i=0;i<vertcount;i++)
	    {
	        dist = DISTANCE_PLANE_POINT(plane,vertices.at(i));
	        if(dist<=0.0f)
	        {
	    	    vec4f result_contact = new vec4f();
	    	    contacts.GIM_DYNARRAY_PUSH_ITEM_TZ(result_contact);
//	             GIM_DYNARRAY_PUSH_EMPTY(vec4f,(*contacts));
//	             result_contact = GIM_DYNARRAY_POINTER_LAST(vec4f,(*contacts));
	             VEC_COPY(result_contact,vertices.at(i));
	             result_contact.f[3] = -dist;
	        }
	    }
	    trimesh.gim_trimesh_unlocks_work_data();
	}

}
