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

import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.cpp4j.java.IntArray;
import org.ode4j.ode.internal.cpp4j.java.ObjArray;
import org.ode4j.ode.internal.gimpact.GimBufferArrayFloat.GIM_PROCESS_BUFFER_ARRAY_FN;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_DATA;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIPLANES_CACHE;
import org.ode4j.ode.internal.gimpact.GimTrimeshCapsuleCollision.GIM_CAPSULE_DATA;

/**
	A Trimesh is the basic geometric structure for representing solid objects.
	<p><strong>CREATING TRIMESHES</strong></p>
	<ul>
	<li> For creating trimeshes, you must initialize Buffer managers by calling \ref gimpact_init
	<li> Then you must define the vertex and index sources by creating them with \ref BUFFER_ARRAYS routines, and then call  \ref gim_trimesh_create_from_arrays.
	<li> An alternative way for creaing trimesh objects is calling  \ref gim_trimesh_create_from_data.
	<li> For access to the trimesh data (vertices, triangle indices), you must call  \ref gim_trimesh_locks_work_data , and  \ref gim_trimesh_unlocks_work_data for finish the access.
	<li> Each time when the trimesh data is modified, you must call  \ref gim_trimesh_update after.
	<li> When a trimesh is no longer needed, you must call \ref gim_trimesh_destroy.
	</ul>

	<p>This is an example of how to create a deformable trimesh that shares vertices with the user application:</p>
	<code>
	//Declaration of vertices
	vec3f trimeshvertices[200];
	//Declaration of indices
	GUINT trimeshindices[100];

	... Initializing vertices and triangle indices at beginning

	//Then create trimesh
	GIM_TRIMESH mytrimesh;

	//Calling trimesh create function

	gim_trimesh_create_from_data(
	mytrimesh,
	trimeshvertices,200,
	0 ,//copy_vertices is 0
	trimeshindices,
	100,
	0, //copy_indices is 0
	0 //transformed_reply is 0
	);
	</code>
	<p>Note that parameter transformed_reply is 0, that means that m_transformed_vertex_buffer is a reference to m_source_vertex on the trimesh, and transformations are not avaliable. Use that configuration if you have to simulate a deformable trimesh like cloth or elastic bodies.</p>
	<p>When the trimesh is no longer needed, destroy it safely with gim_trimesh_destroy()</p>
	<p><strong>UPDATING TRIMESHES</strong></p>
	<p>On simulation loops, is needed to update trimeshes every time for update vertices althought updating triangle boxes and planes cache. There is two ways for update trimeshes: </p>
	<ul>
	<li> Updating vertices directly. You need to access to the \ref GIM_TRIMESH.m_source_vertex_buffer member; a vertex buffer which has access to the source vertices.
	<pre>
	// Access to the source vertices
	gim_buffer_array_lock(mytrimesh.m_source_vertex_buffer, G_MA_READ_WRITE);

	//Get a pointer to the vertex buffer
	vec3f * vertexpointer = GIM_BUFFER_ARRAY_POINTER(vec3f,mytrimesh.m_source_vertex_buffer,0);

	//Get the amount of vertices
	int veccount = mytrimesh.m_source_vertex_buffer.m_element_count;

	//Modify vertices
	for (int i=0;i &lt; veccount ;i++ )
	{
	    .....
	    .....
	    processing vertices
	    .....
		.....
	}

	// Don't forget to unlock the source vertex array
	gim_buffer_array_unlock(mytrimesh.m_source_vertex_buffer);

	// Notify that the state of the trimesh is changed
	gim_trimesh_post_update(mytrimesh.m_source_vertex_buffer);

	</pre>
	For making trimeshes that allow to update their vertices, use \ref gim_trimesh_create_from_data with parameter <strong>transformed_reply</strong> = 0.
	</ul>
	<ul>
	<li> Aplying a transformation. Simply use \ref gim_trimesh_set_tranform . Remember that with this method trimeshes must be created with \ref gim_trimesh_create_from_data with parameter <strong>transformed_reply</strong> = 1.
	</ul>
	<p> After updating vertices, you must call \ref gim_trimesh_update()</p>
	<p><strong>TRIMESHES COLLISION</strong></p>
	<p>Before collide trimeshes, you need to update them first.</p>
	<p>Then you must use \ref gim_trimesh_trimesh_collision().</p>
 * 
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimTrimesh implements GimConstants {


	///Mask defines
	private static final int GIM_TRIMESH_TRANSFORMED_REPLY = 1;
	private static final int GIM_TRIMESH_NEED_UPDATE = 2;

	/** Prototype for updating vertices. */
	//typedef void * gim_update_trimesh_function(struct _GIM_TRIMESH *);
	interface gim_update_trimesh_function {
		void run(GimTrimesh gt);
	}

	///Original
	GimBufferArrayFloat m_source_vertex_buffer;//!< Buffer of vec3f coordinates

	/** 
	 * (GUINT) Indices of triangles,groups of three elements.
	 * Array of GUINT. Triangle indices. Each triple contains indices of the vertices for each triangle.
	 * @invariant must be aligned
	 */
	GimBufferArrayInt m_tri_index_buffer;

	// Allocated
	int m_mask;//!< Don't use directly

	/**
	 * Allocated transformed vertices vec3f.
	 * Array of vec3f.If gim_trimesh_has_tranformed_reply(this) == 1 then it refers to the m_source_vertex_buffer.
	 * @invariant must be aligned
	 */
	GimBufferArrayFloat m_transformed_vertex_buffer = new GimBufferArrayFloat();

	// Auxiliary data

	GimAABBSet m_aabbset;
	GimDynArray<GIM_TRIPLANES_CACHE> m_planes_cache_buffer;//! Allocated GIM_TRIPLANES_CACHE
	GimBitSet m_planes_cache_bitset;
	gim_update_trimesh_function m_update_callback;//! If null, then m_transform is applied.
	mat4f m_transform = new mat4f();


	private GimTrimesh() {}

	/**
	 * Trimesh Trimesh Collisions
	 * Before use this function you must update each trimesh:
	 * <code>
	 * gim_trimesh_update(TriMesh1);
	 * gim_trimesh_update(TriMesh2);
	 * </code>
	 * Then you must use the trimesh collision in this way:
	 * <code>
	 * int collide_trimeshes(GIM_TRIMESH * TriMesh1, GIM_TRIMESH * TriMesh2)
	 * {
	 *     //Create contact list
	 *     GDYNAMIC_ARRAY trimeshcontacts;
	 *     GIM_CREATE_CONTACT_LIST(trimeshcontacts);
	 *     
	 *     //Collide trimeshes
	 *     gim_trimesh_trimesh_collision(TriMesh1,TriMesh2,&amp; trimeshcontacts);
	 *     
	 *     if(trimeshcontacts.m_size == 0) //do  nothing
	 *     {
	 *         GIM_DYNARRAY_DESTROY(trimeshcontacts);//clean contact array
	 *         return 0;
	 *     }
	 *     
	 *     //Getting a pointer to the contact array
	 *     GIM_CONTACT * ptrimeshcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,trimeshcontacts);
	 *     
	 *     int contactcount = trimeshcontacts.m_size;
	 *     int i;
	 *     //Process contacts
	 *     for (i=0;i &lt; contactcount ;i++)
	 *     {
	 *         //Do something with the contact (ptrimeshcontacts)
	 *         ......
	 *         ......
	 *         // Like creating joints or anything else
	 *         ......
	 *         ......
	 *         ptrimeshcontacts++;
	 *     }
	 *     GIM_DYNARRAY_DESTROY(trimeshcontacts);
	 *     return contactcount;
	 * }
	 * </code>
	 * In each contact
	 * <ul>
	 * <li> m_handle1 points to trimesh1.
	 * <li> m_handle2 points to trimesh2.
	 * <li> m_feature1 Is a triangle index of trimesh1.
	 * <li> m_feature2 Is a triangle index of trimesh2.
	 * </ul>
	 * 
	 * @param trimesh2 Collidee
	 * @param contacts A GIM_CONTACT array. Must be initialized
	 */
	//* @param trimesh1 Collider
	//void gim_trimesh_trimesh_collision(GimTrimesh * trimesh1, GimTrimesh * trimesh2, GDYNAMIC_ARRAY * contacts);
	public void gim_trimesh_trimesh_collision(GimTrimesh trimesh2, GimDynArray<GimContact> contacts) {
		GimTrimeshTrimeshCol.gim_trimesh_trimesh_collision(this, trimesh2, contacts);
	}


	//! Trimesh Sphere Collisions
	/*!
	Before use this function you must update the trimesh:
	\code
	gim_trimesh_update(trimesh);
	\endcode
	Then you must use this function in this way:
	\code
	int collide_trimesh_sphere(GIM_TRIMESH * trimesh, vec3f center,GREAL radius)
	{
	    //Create contact list
	    GDYNAMIC_ARRAY trimeshcontacts;
	    GIM_CREATE_CONTACT_LIST(trimeshcontacts);

	    //Collide trimeshes
	    gim_trimesh_sphere_collision(trimesh,center,radius,&trimeshcontacts);

	    if(trimeshcontacts.m_size == 0) //do  nothing
	    {
	        GIM_DYNARRAY_DESTROY(trimeshcontacts);//clean contact array
	        return 0;
	    }

	    //Getting a pointer to the contact array
	    GIM_CONTACT * ptrimeshcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,trimeshcontacts);

	    int contactcount = trimeshcontacts.m_size;
	    int i;
	    //Process contacts
	    for (i=0;i<contactcount ;i++)
	    {
	        //Do something with the contact (ptrimeshcontacts)
	        ......
	        ......
	        // Like creating joints or anything else
	        ......
	        ......
	        ptrimeshcontacts++;
	    }
	    GIM_DYNARRAY_DESTROY(trimeshcontacts);
	    return contactcount;
	}
	\endcode

	In each contact
	<ul>
	<li> m_handle1 points to trimesh.
	<li> m_handle2 points to NULL.
	<li> m_feature1 Is a triangle index of trimesh.
	</ul>

	\param trimesh
	\param center
	\param radius
	\param contacts A GIM_CONTACT array. Must be initialized
	*/
	//void gim_trimesh_sphere_collision(GimTrimesh * trimesh,vec3f center,GREAL radius, GDYNAMIC_ARRAY * contacts);
	public void gim_trimesh_sphere_collision(vec3f center,float radius, GimDynArray<GimContact> contacts) {
		GimTrimeshSphereCollision.gim_trimesh_sphere_collision(this, center, radius, contacts);
	}


	//! Trimesh Capsule collision
	/*!
	Find the closest primitive collided by the ray.

	Before use this function you must update the trimesh:
	\code
	gim_trimesh_update(trimesh);
	\endcode
	Then you must use this function in this way:
	\code
	int collide_trimesh_capsule(GIM_TRIMESH * trimesh, GIM_CAPSULE_DATA * capsule)
	{
	    //Create contact list
	    GDYNAMIC_ARRAY trimeshcontacts;
	    GIM_CREATE_CONTACT_LIST(trimeshcontacts);

	    //Collide trimeshes
	    gim_trimesh_capsule_collision(trimesh,capsule,&trimeshcontacts);

	    if(trimeshcontacts.m_size == 0) //do  nothing
	    {
	        GIM_DYNARRAY_DESTROY(trimeshcontacts);//clean contact array
	        return 0;
	    }

	    //Getting a pointer to the contact array
	    GIM_CONTACT * ptrimeshcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,trimeshcontacts);

	    int contactcount = trimeshcontacts.m_size;
	    int i;
	    //Process contacts
	    for (i=0;i<contactcount ;i++)
	    {
	        //Do something with the contact (ptrimeshcontacts)
	        ......
	        ......
	        // Like creating joints or anything else
	        ......
	        ......
	        ptrimeshcontacts++;
	    }
	    GIM_DYNARRAY_DESTROY(trimeshcontacts);
	    return contactcount;
	}
	\endcode

	In each contact
	<ul>
	<li> m_handle1 points to trimesh.
	<li> m_handle2 points to NULL.
	<li> m_feature1 Is a triangle index of trimesh.
	</ul>

	\param trimesh
	\param capsule
	\param contacts A GIM_CONTACT array. Must be initialized
	*/
	//void gim_trimesh_capsule_collision(GimTrimesh * trimesh, GIM_CAPSULE_DATA * capsule, GDYNAMIC_ARRAY * contacts);
	public void gim_trimesh_capsule_collision(GIM_CAPSULE_DATA capsule, GimDynArray<GimContact> contacts) {
		GimTrimeshCapsuleCollision.gim_trimesh_capsule_collision(this, capsule, contacts);
	}


	///Function for create Trimesh Plane  collision result
	//#define GIM_CREATE_TRIMESHPLANE_CONTACTS(dynarray) GIM_DYNARRAY_CREATE(vec4f,dynarray,G_ARRAY_GROW_SIZE)
	public static GimDynArray<vec4f> GIM_CREATE_TRIMESHPLANE_CONTACTS() { 
		return GimDynArray.GIM_DYNARRAY_CREATE(vec4f.class, GimDynArray.G_ARRAY_GROW_SIZE);//vec4f,dynarray,G_ARRAY_GROW_SIZE);
	}

	//! Trimesh Plane Collisions
	/*!

	Before use this function you must update the trimesh:
	\code
	gim_trimesh_update(trimesh);
	\endcode
	Then you must use this function in this way:
	\code
	int collide_trimesh_plane(GIM_TRIMESH * trimesh, vec4f plane)
	{
	    //Create contact list
	    GDYNAMIC_ARRAY tri_plane_contacts;
	    GIM_CREATE_TRIMESHPLANE_CONTACTS(tri_plane_contacts);

	    //Collide trimeshes
	    gim_trimesh_plane_collision(trimesh,plane,&tri_plane_contacts);

	    if(tri_plane_contacts.m_size == 0) //do  nothing
	    {
	        GIM_DYNARRAY_DESTROY(tri_plane_contacts);//clean contact array
	        return 0;
	    }

	    //Getting a pointer to the contact array
	    vec4f * planecontacts = GIM_DYNARRAY_POINTER(vec4f,tri_plane_contacts);

	    int contactcount = tri_plane_contacts.m_size;
	    int i;
	    //Process contacts
	    for (i=0;i<contactcount ;i++)
	    {
	        vec3f contactpoint;
	        GREAL contactdis;

	        VEC_COPY(contactpoint,planecontacts[i]); //Get contact point
	        contactdis = planecontacts[i][3]; // Get distance depth

	        //Do something with the contact
	        ......
	        ......
	        // Like creating joints or anything else
	        ......
	        ......
	    }
	    GIM_DYNARRAY_DESTROY(tri_plane_contacts);
	    return contactcount;
	}
	\endcode

	In each contact the 3 first coordinates refers to the contact point, the fourth refers to the distance depth and the normal is the normal of the plane.

	\param trimesh
	\param plane vec4f plane
	\param contacts A vec4f array. Must be initialized (~100). Each element have the coordinate point in the first 3 elements, and vec4f[3] has the penetration depth.
	*/
	//void gim_trimesh_plane_collision(GimTrimesh * trimesh,vec4f plane, GDYNAMIC_ARRAY * contacts);
	public void gim_trimesh_plane_collision(vec4f plane, GimDynArray<vec4f> contacts) {
		GimTrimeshTrimeshCol.gim_trimesh_plane_collision(this, plane, contacts);
	}


	//! Trimesh Ray Collisions
	/*!
	\param trimesh
	\param origin
	\param dir
	\param tmax
	\param contact
	\return 1 if the ray collides, else 0
	*/
	//int gim_trimesh_ray_collision(GimTrimesh * trimesh,vec3f origin,vec3f dir, 
	//GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact);
	public int gim_trimesh_ray_collision(vec3f origin, vec3f dir, 
			final float tmax, GIM_TRIANGLE_RAY_CONTACT_DATA contact) {
		return GimTrimeshRayCollision.gim_trimesh_ray_collision(this, origin, dir, tmax, contact);
	}


	//! Trimesh Ray Collisions closest
	/*!
	Find the closest primitive collided by the ray
	\param trimesh
	\param origin
	\param dir
	\param tmax
	\param contact
	\return 1 if the ray collides, else 0
	*/
	//int gim_trimesh_ray_closest_collision(GimTrimesh * trimesh,
	//vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact);
	public int gim_trimesh_ray_closest_collision(
			vec3f origin, vec3f dir, float tmax, GIM_TRIANGLE_RAY_CONTACT_DATA contact) {
		return GimTrimeshRayCollision.gim_trimesh_ray_closest_collision(this, origin, dir, tmax, contact);
	}

	//! @}


	/*
	-----------------------------------------------------------------------------
	This source file is part of GIMPACT Library.

	For the latest info, see http://gimpact.sourceforge.net/

	Copyright (c) 2006 Francisco Leon. C.C. 80087371.
	email: projectileman@yahoo.com

	 This library is free software; you can redistribute it and/or
	 modify it under the terms of EITHER:
	   (1) The GNU Lesser General Public License as published by the Free
	       Software Foundation; either version 2.1 of the License, or (at
	       your option) any later version. The text of the GNU Lesser
	       General Public License is included with this library in the
	       file GIMPACT-LICENSE-LGPL.TXT.
	   (2) The BSD-style license that is included with this library in
	       the file GIMPACT-LICENSE-BSD.TXT.

	 This library is distributed in the hope that it will be useful,
	 but WITHOUT ANY WARRANTY; without even the implied warranty of
	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
	 GIMPACT-LICENSE-LGPL.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

	-----------------------------------------------------------------------------
	*/



	/// Info about mesh
	//! Return the trimesh triangle count
	//GUINT32 gim_trimesh_get_triangle_count(GimTrimesh * trimesh);
	public int gim_trimesh_get_triangle_count()
	{
	    return m_tri_index_buffer.getElementCount()/3;
	}

	//! Creates the aabb set and the triangles cache
	/*!

	\param trimesh
	\param vertex_array
	\param triindex_array
	\param transformed_reply If 1, then the m_transformed_vertices is a reply of the source vertices. Else it just be a reference to the original array.
	\post it copies the arrays by reference, and creates the auxiliary data (m_aabbset,m_planes_cache_buffer)
	*/
//	void gim_trimesh_create_from_arrays(GBUFFER_MANAGER_DATA buffer_managers[],
//			GIM_TRIMESH * trimesh, GBUFFER_ARRAY * vertex_array, GBUFFER_ARRAY * triindex_array,char transformed_reply)
	static GimTrimesh gim_trimesh_create_from_arrays(
			GimBufferArrayFloat vertex_array, 
			GimBufferArrayInt triindex_array,final boolean transformed_reply)
	{
		GimTrimesh trimesh = new GimTrimesh();
	    assert(trimesh!=null);
	    assert(vertex_array!=null);
	    assert(triindex_array!=null);
	    //GimBufferArrayFloat.gim_buffer_array_copy_ref(vertex_array,trimesh.m_source_vertex_buffer);
	    trimesh.m_source_vertex_buffer = vertex_array;
	    //GimBufferArrayInt.gim_buffer_array_copy_ref(triindex_array,trimesh.m_tri_index_buffer);
	    trimesh.m_tri_index_buffer = triindex_array;

	    trimesh.m_mask = GIM_TRIMESH_NEED_UPDATE;//needs update
	    //Create the transformed vertices
	    if(transformed_reply)
	    {
	        trimesh.m_mask |= GIM_TRIMESH_TRANSFORMED_REPLY;
//	        GimBufferArrayFloat.gim_buffer_array_copy_value(vertex_array,
//				buffer_managers,trimesh.m_transformed_vertex_buffer,G_BUFFER_MANAGER.SYSTEM,G_MU_DYNAMIC_READ_WRITE);
	        trimesh.m_transformed_vertex_buffer = vertex_array.cloneValues();
	    }
	    else
	    {
	    	//GimBufferArrayFloat.gim_buffer_array_copy_ref(vertex_array,trimesh.m_transformed_vertex_buffer);
	    	trimesh.m_transformed_vertex_buffer = vertex_array.cloneRefs();
	    }
	    //create the box set
	    int facecount = trimesh.gim_trimesh_get_triangle_count();

	    trimesh.m_aabbset = GimAABBSet.gim_aabbset_alloc(facecount);
	    //create the planes cache
	    trimesh.m_planes_cache_buffer = GimDynArray.GIM_DYNARRAY_CREATE_SIZED(facecount);
	    //Create the bitset
	    trimesh.m_planes_cache_bitset = GimBitSet.GIM_BITSET_CREATE_SIZED(facecount);
	    //Callback is 0
	    trimesh.m_update_callback = null;
	    //set to identity
	    IDENTIFY_MATRIX_4X4(trimesh.m_transform);
	    
	    return trimesh;
	}


	//! Create a trimesh from vertex array and an index array
	/*!
	\param trimesh An uninitialized GimTrimesh  structure
	\param vertex_array A buffer to a vec3f array
	\param vertex_count
	\param triindex_array
	\param index_count
	\param copy_vertices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
	\param copy_indices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
	\param transformed_reply If 1, then the m_transformed_vertices is a reply of the source vertices. Else it just be a reference to the original array. Use 1 if you will apply transformations to the trimesh. See \ref gim_trimesh_set_tranform().
	*/
//	void gim_trimesh_create_from_data(GBUFFER_MANAGER_DATA buffer_managers[],
//			GIM_TRIMESH * trimesh, vec3f * vertex_array, GUINT32 vertex_count,char copy_vertices, 
//			GUINT32 * triindex_array, GUINT32 index_count,char copy_indices,char transformed_reply)
	public static GimTrimesh gim_trimesh_create_from_data(
			float[] vertex_array, boolean copy_vertices, 
			int[] triindex_array, boolean copy_indices, boolean transformed_reply)
	{
		GimTrimesh THIS;
		
		GimBufferArrayFloat buffer_vertex_array;// = new GimBufferArrayFloat();
		GimBufferArrayInt buffer_triindex_array;// = new GimBufferArrayInt();

    	//Create vertices
	    if(copy_vertices)
	    {
//	        GimBufferArrayFloat.gim_create_common_buffer_from_data(buffer_managers, 
//				new GPTR<vec3f[]>(vertex_array2), vertex_count,//*sizeof(vec3f), 
//				buffer_vertex_array.m_buffer_id);
	        buffer_vertex_array = GimBufferArrayFloat.createCopy(vertex_array);
	    }
	    else//Create a shared buffer
	    {
//	    	GimBufferArrayFloat.gim_create_shared_buffer_from_data(buffer_managers, 
//				new GPTR<vec3f[]>(vertex_array2), vertex_count,//*sizeof(vec3f), 
//				buffer_vertex_array.m_buffer_id);
	        buffer_vertex_array = GimBufferArrayFloat.createRef(vertex_array);
	    }
	    //buffer_vertex_array.GIM_BUFFER_ARRAY_INIT_TYPE(buffer_vertex_array.m_buffer_id,vertex_count); //TZ: /3 TODO ?


	    //Create vertices
	    if(copy_indices)
	    {
//	        GimBufferArray.gim_create_common_buffer_from_data(buffer_managers, 
//				new GPTR(triindex_array), index_count,//*sizeof(GUINT32), 
//				buffer_triindex_array.m_buffer_id,int[].class);
	    	buffer_triindex_array = GimBufferArrayInt.createCopy(triindex_array);
	    }
	    else//Create a shared buffer
	    {
//	    	GimBufferArray.gim_create_shared_buffer_from_data(buffer_managers, 
//				new GPTR(triindex_array), index_count,//*sizeof(GUINT32), 
//				buffer_triindex_array.m_buffer_id,int[].class);
	    	buffer_triindex_array = GimBufferArrayInt.createRef(triindex_array);
	    }
	    //buffer_triindex_array.GIM_BUFFER_ARRAY_INIT_TYPE(buffer_triindex_array.m_buffer_id,index_count);

	    THIS = gim_trimesh_create_from_arrays(//buffer_managers, 
			buffer_vertex_array, buffer_triindex_array,transformed_reply);

	    ///always call this after create a buffer_array
	    buffer_vertex_array.GIM_BUFFER_ARRAY_DESTROY();
	    buffer_triindex_array.GIM_BUFFER_ARRAY_DESTROY();
	    
	    return THIS;
	}

	//! Clears auxiliary data and releases buffer arrays
	//void gim_trimesh_destroy(GIM_TRIMESH * trimesh)
	public void gim_trimesh_destroy()
	{
		m_aabbset.gim_aabbset_destroy();

	    m_planes_cache_buffer.GIM_DYNARRAY_DESTROY();
	    m_planes_cache_bitset.GIM_DYNARRAY_DESTROY();

	    m_transformed_vertex_buffer.GIM_BUFFER_ARRAY_DESTROY();
	    m_source_vertex_buffer.GIM_BUFFER_ARRAY_DESTROY();
	    m_tri_index_buffer.GIM_BUFFER_ARRAY_DESTROY();
	}

	//! Copies two meshes
	/*!
	\pre dest_trimesh shouldn't be created
	\post dest_trimesh will be created
	\param source_trimesh
	\param dest_trimesh
	\param copy_by_reference If 1, it attach a reference to the source vertices, else it copies the vertices
	\param transformed_reply IF 1, then it forces the m_trasnformed_vertices to be  a reply of the source vertices
	\param transformed_reply If 1, transformed vertices are reply of source vertives. 1 Is recommended
	*/
//	void gim_trimesh_copy(GIM_TRIMESH * source_trimesh,
//			GBUFFER_MANAGER_DATA dest_buffer_managers[], GIM_TRIMESH * dest_trimesh, 
//			char copy_by_reference, char transformed_reply)
	static GimTrimesh gim_trimesh_copy(GimTrimesh source_trimesh,
			//GimTrimesh dest_trimesh, 
			boolean copy_by_reference, boolean transformed_reply)
	{
	/* -- trimesh can not be copied by reference until GBUFFER_MANAGER_DATA is rewritten
		to be thread safe and until it is moved back to global variables.
	    if(copy_by_reference==1)
	    {
	        gim_trimesh_create_from_arrays(dest_trimesh, &source_trimesh->m_source_vertex_buffer, &source_trimesh->m_tri_index_buffer,transformed_reply);
	    }
	    else
	*/
		if (copy_by_reference) { //TZ
			System.out.println("Copying TRIMESH by ref is not supported.");
		}
	    {
	        GimBufferArrayFloat buffer_vertex_array;// = new GimBufferArrayFloat();
	        GimBufferArrayInt buffer_triindex_array;// = new GimBufferArrayInt();

//	        GimBufferArrayFloat.gim_buffer_array_copy_value(source_trimesh.m_source_vertex_buffer,
//				dest_buffer_managers,buffer_vertex_array,G_BUFFER_MANAGER.SYSTEM,G_MU_DYNAMIC_READ_WRITE);
	        buffer_vertex_array = source_trimesh.m_source_vertex_buffer.cloneValues(); 

//	        GimBufferArrayInt.gim_buffer_array_copy_value(source_trimesh.m_tri_index_buffer,
//				dest_buffer_managers,buffer_triindex_array,G_BUFFER_MANAGER.SYSTEM,G_MU_DYNAMIC_READ_WRITE);
	        buffer_triindex_array = source_trimesh.m_tri_index_buffer.cloneValues(); 

	        GimTrimesh dest_trimesh = gim_trimesh_create_from_arrays(//dest_buffer_managers,
				buffer_vertex_array, buffer_triindex_array, transformed_reply);

	        ///always call this after create a buffer_array
	        buffer_vertex_array.GIM_BUFFER_ARRAY_DESTROY();
	        buffer_triindex_array.GIM_BUFFER_ARRAY_DESTROY();
	        return dest_trimesh;
	    }
	}

	//! Locks the trimesh for working with it
	/*!
	\post locks m_tri_index_buffer and m_transformed_vertex_buffer.
	\param trimesh
	*/
	//void gim_trimesh_locks_work_data(GIM_TRIMESH * trimesh)
	public void gim_trimesh_locks_work_data()
	{
	    //GINT32 res;
		//TODO remove TZ	    int res=m_tri_index_buffer.gim_buffer_array_lock(G_MA_READ_ONLY);
		//TODO remove TZ	    assert(res==G_BUFFER_OP_SUCCESS);
//	    res=m_transformed_vertex_buffer.gim_buffer_array_lock(G_MA_READ_ONLY); TODO remove TZ
		//TODO remove TZ	    assert(res==G_BUFFER_OP_SUCCESS);
	}

	//! unlocks the trimesh
	/*!
	\post unlocks m_tri_index_buffer and m_transformed_vertex_buffer.
	\param trimesh
	*/
	//void gim_trimesh_unlocks_work_data(GIM_TRIMESH * trimesh)
	public void gim_trimesh_unlocks_work_data()
	{
		//TODO remove TZ		m_tri_index_buffer.gim_buffer_array_unlock();
//TODO remove TZ		m_transformed_vertex_buffer.gim_buffer_array_unlock();
	}


	//! Returns 1 if the m_transformed_vertex_buffer is a reply of m_source_vertex_buffer
	//char gim_trimesh_has_tranformed_reply(GIM_TRIMESH * trimesh)
	boolean gim_trimesh_has_tranformed_reply()
	{
//	    if((m_mask&GIM_TRIMESH_TRANSFORMED_REPLY)!=0) return true;
//	    return false;
		return (m_mask&GIM_TRIMESH_TRANSFORMED_REPLY) != 0;
	}

	//! Returns 1 if the trimesh needs to update their aabbset and the planes cache.
	//char gim_trimesh_needs_update(GIM_TRIMESH * trimesh)
	boolean gim_trimesh_needs_update()
	{
//	    if((m_mask&GIM_TRIMESH_NEED_UPDATE)!=0) return true;
//	    return false;
	    return (m_mask&GIM_TRIMESH_NEED_UPDATE) != 0;
	}

	//! Change the state of the trimesh for force it to update
	/*!
	Call it after made changes to the trimesh.
	\post gim_trimesh_need_update(trimesh) will return 1
	\sa gim_trimesh_needs_update,gim_trimesh_has_tranformed_reply
	*/
	//void gim_trimesh_post_update(GIM_TRIMESH * trimesh)
	void gim_trimesh_post_update()
	{
	    m_mask |= GIM_TRIMESH_NEED_UPDATE;
	}

	//kernel
	//#define MULT_MAT_VEC4_KERNEL(_mat,_src,_dst) MAT_DOT_VEC_3X4((_dst),(_mat),(_src))
	private final GIM_PROCESS_BUFFER_ARRAY_FN MULT_MAT_VEC4_KERNEL = 
		new GIM_PROCESS_BUFFER_ARRAY_FN() {
		@Override
		public void run(mat4f _mat, vec3f _src, vec3f _dst) {MAT_DOT_VEC_3X4( _dst, _mat, _src); }	
	};

	//! Updates m_transformed_vertex_buffer
	/*!
	\pre m_transformed_vertex_buffer must be unlocked
	*/
	//void gim_trimesh_update_vertices(GIM_TRIMESH * trimesh)
	void gim_trimesh_update_vertices()
	{
	    if(gim_trimesh_has_tranformed_reply() == false) return; //Don't perform transformation

	    //Vertices
	    GimBufferArrayFloat psource_vertex_buffer = m_source_vertex_buffer;
	    GimBufferArrayFloat ptransformed_vertex_buffer = m_transformed_vertex_buffer;
	    //Temp transform
	    mat4f transform = new mat4f();
	    COPY_MATRIX_4X4(transform,m_transform);  //TODO TZ why copy?

	    GimBufferArrayFloat.GIM_PROCESS_BUFFER_ARRAY(transform, psource_vertex_buffer, ptransformed_vertex_buffer,
	    		MULT_MAT_VEC4_KERNEL);//,vec3f,vec3f);
	}

	//! Updates m_aabbset and m_planes_cache_bitset
	/*!
	\pre gim_trimesh_locks_work_data must be called before
	*/
	//void gim_trimesh_update_aabbset(GIM_TRIMESH * trimesh)
	void gim_trimesh_update_aabbset()
	{
	    ObjArray<vec3f> transformed_vertices = m_transformed_vertex_buffer.GIM_BUFFER_ARRAY_POINTER(0);
	    assert(transformed_vertices!= null);

	    IntArray triangle_indices = m_tri_index_buffer.GIM_BUFFER_ARRAY_POINTER(0);
	    assert(triangle_indices!=null);
	    // box set
	    //aabb3f[] paabb = m_aabbset.m_boxes;
	    int triangle_count = gim_trimesh_get_triangle_count();
	    //float[] v1,v2,v3;
	    //vec3f v1,v2,v3;
	    int i;
	    for (i=0; i<triangle_count;i++)
	    {
//	        v1 = transformed_vertices.at(triangle_indices.at(0));//[0];
//	        v2 = transformed_vertices.at(triangle_indices.at(1));//[0];
//	        v3 = transformed_vertices.at(triangle_indices.at(2));//[0];
//	        COMPUTEAABB_FOR_TRIANGLE((paabb[i]),v1,v2,v3);
	        COMPUTEAABB_FOR_TRIANGLE(m_aabbset.at(i),//paabb[i],	        
	        		transformed_vertices.at( triangle_indices.at(0) ),
	        		transformed_vertices.at( triangle_indices.at(1) ),
	        		transformed_vertices.at( triangle_indices.at(2) ));
	        triangle_indices.inc(3);//+=3;
	        //paabb++;
	    }
	    //Clear planes cache
	    m_planes_cache_bitset.GIM_BITSET_CLEAR_ALL();
	    //Sorts set
	    m_aabbset.gim_aabbset_update();
	}

	//! Calls before perfom collisions. Updates the trimesh if needed
	/*!
	\post If gim_trimesh_needs_update returns 1, then it calls  gim_trimesh_update_vertices and gim_trimesh_update_aabbset
	*/
	//void gim_trimesh_update(GIM_TRIMESH * trimesh)
	public void gim_trimesh_update()
	{
		if(gim_trimesh_needs_update()==false) return;
		gim_trimesh_update_vertices();
		gim_trimesh_locks_work_data();
		gim_trimesh_update_aabbset();
		gim_trimesh_unlocks_work_data();

		//Clear update flag
		m_mask &= ~GIM_TRIMESH_NEED_UPDATE;
	}

	//! Set the transform of a trimesh
	/*!
	\post This function calls to gim_trimesh_post_update
	*/
	//void gim_trimesh_set_tranform(GIM_TRIMESH * trimesh, mat4f transform)
	public void gim_trimesh_set_tranform(mat4f transform)
	{
	    float diff = 0.0f;
	    float[] originaltrans = m_transform.f;//[0][0];
	    float[] newtrans = transform.f;//[0][0];
	    int i;
	    for (i=0;i<16;i++)
	    {
	    	diff += Math.abs(originaltrans[i]-newtrans[i]);
	    }

//	    if(IS_ZERO(diff)) return ;///don't need to update
	    if(diff< 0.00001f) return ;///don't need to update

	    COPY_MATRIX_4X4(m_transform,transform);

	    gim_trimesh_post_update();
	}

	//! Fetch triangle data
	/*!
	\pre gim_trimesh_locks_work_data must be called before
	*/
	//void gim_trimesh_get_triangle_data(GIM_TRIMESH * trimesh, GUINT32 triangle_index, 
	//GIM_TRIANGLE_DATA * tri_data)
	void gim_trimesh_get_triangle_data(int triangle_index, 
			final GIM_TRIANGLE_DATA tri_data)
	{
	    ObjArray<vec3f> transformed_vertices = m_transformed_vertex_buffer.GIM_BUFFER_ARRAY_POINTER(0);

	    IntArray triangle_indices = m_tri_index_buffer.GIM_BUFFER_ARRAY_POINTER(triangle_index*3);


	    //Copy the vertices
	    VEC_COPY(tri_data.m_vertices[0],transformed_vertices.at( triangle_indices.at(0) ));
	    VEC_COPY(tri_data.m_vertices[1],transformed_vertices.at( triangle_indices.at(1) ));
	    VEC_COPY(tri_data.m_vertices[2],transformed_vertices.at( triangle_indices.at(2) ));

	    //Get the planes
	    ObjArray<GIM_TRIPLANES_CACHE> planes = m_planes_cache_buffer.GIM_DYNARRAY_POINTER_V();
	    planes.inc(triangle_index);
	    if (planes.at0()==null) planes.setAt0(new GIM_TRIPLANES_CACHE());
    	GIM_TRIPLANES_CACHE plane = planes.at0();

	    //verify planes cache
	    boolean bit_eval;
	    bit_eval = m_planes_cache_bitset.GIM_BITSET_GET(triangle_index);
	    if(bit_eval == false)// Needs to calc the planes
	    {
	        //Calc the face plane
	        TRIANGLE_PLANE(tri_data.m_vertices[0], tri_data.m_vertices[1], tri_data.m_vertices[2], plane.m_planes[0]);
	        //Calc the edge 1
	        EDGE_PLANE(tri_data.m_vertices[0], tri_data.m_vertices[1], plane.m_planes[0], plane.m_planes[1] );

	        //Calc the edge 2
	        EDGE_PLANE(tri_data.m_vertices[1], tri_data.m_vertices[2], plane.m_planes[0], plane.m_planes[2] );

	        //Calc the edge 3
	        EDGE_PLANE(tri_data.m_vertices[2], tri_data.m_vertices[0], plane.m_planes[0], plane.m_planes[3] );

	        //mark
	        m_planes_cache_bitset.GIM_BITSET_SET(triangle_index);
	    }


	    VEC_COPY_4( tri_data.m_planes.m_planes[0], plane.m_planes[0] );//face plane
	    VEC_COPY_4( tri_data.m_planes.m_planes[1], plane.m_planes[1] );//edge1
	    VEC_COPY_4( tri_data.m_planes.m_planes[2], plane.m_planes[2] );//edge2
	    VEC_COPY_4( tri_data.m_planes.m_planes[3], plane.m_planes[3] );//edge3
	}

	//! Fetch triangle vertices
	/*!
	\pre gim_trimesh_locks_work_data must be called before
	*/
	//void gim_trimesh_get_triangle_vertices(GIM_TRIMESH * trimesh, 
	//GUINT32 triangle_index, vec3f v1,vec3f v2,vec3f v3)
//	public void gim_trimesh_get_triangle_vertices(
//			int triangle_index, vec3f v1, vec3f v2, vec3f v3)
//	{
//		//vec3f[] transformed_vertices = GIM_BUFFER_ARRAY_POINTER(vec3f.class,trimesh.m_transformed_vertex_buffer,0);
//		ObjArray<vec3f> transformed_vertices = m_transformed_vertex_buffer.GIM_BUFFER_ARRAY_POINTER(0);
//
//		//int[] triangle_indices = GIM_BUFFER_ARRAY_POINTER(GUINT32,trimesh.m_tri_index_buffer,triangle_index*3);
//		IntArray triangle_indices = m_tri_index_buffer.GIM_BUFFER_ARRAY_POINTER(triangle_index*3);
//
//		//Copy the vertices
//		if (v1 != null) {
//			VEC_COPY(v1, transformed_vertices.at(triangle_indices.at(0)));
//		}
//		if (v2 != null) {
//			VEC_COPY(v2, transformed_vertices.at(triangle_indices.at(1)));
//		}
//		if (v3 != null) {
//			VEC_COPY(v3, transformed_vertices.at(triangle_indices.at(2)));
//		}
//	}
	public void gim_trimesh_get_triangle_vertices(
			int triangle_index, DVector3 v1, DVector3 v2, DVector3 v3)
	{
		//vec3f[] transformed_vertices = GIM_BUFFER_ARRAY_POINTER(vec3f.class,trimesh.m_transformed_vertex_buffer,0);
		ObjArray<vec3f> transformed_vertices = m_transformed_vertex_buffer.GIM_BUFFER_ARRAY_POINTER(0);

		//int[] triangle_indices = GIM_BUFFER_ARRAY_POINTER(GUINT32,trimesh.m_tri_index_buffer,triangle_index*3);
		IntArray triangle_indices = m_tri_index_buffer.GIM_BUFFER_ARRAY_POINTER(triangle_index*3);

		//Copy the vertices
		if (v1 != null) {
			VEC_COPY(v1, transformed_vertices.at(triangle_indices.at(0)));
		}
		if (v2 != null) {
			VEC_COPY(v2, transformed_vertices.at(triangle_indices.at(1)));
		}
		if (v3 != null) {
			VEC_COPY(v3, transformed_vertices.at(triangle_indices.at(2)));
		}
	}

	
	public GimAABBSet getAabbSet() {
		return m_aabbset;
	}
}
