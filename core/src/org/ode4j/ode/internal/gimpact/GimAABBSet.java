/**
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
package org.ode4j.ode.internal.gimpact;

import org.cpp4j.java.RefBoolean;
import org.cpp4j.java.RefFloat;

import static org.ode4j.ode.internal.gimpact.GimGeometry.*;

import org.ode4j.ode.internal.gimpact.GimRadixSort.GIM_RSORT_TOKEN;

/** 
 * Tools for find overlapping objects on a scenary. 
 * These functions sort boxes for faster collisioin queries, using radix 
 * sort or quick sort as convenience. See \ref SORTING .
 * <ul>
 * <li> For using these collision routines, you must create a \ref GIM_AABB_SET
 *      by using this function : \ref gim_aabbset_alloc.
 * <li> The  GIM_AABB_SET objects must be updated on their boxes on each query, 
 *      and they must be update by calling \ref gim_aabbset_update
 * <li> Before calling collision functions, you must create a pair set with \ref GIM_CREATE_PAIR_SET
 * <li> For finding collision pairs on a scene (common space for objects), 
 *      call \ref gim_aabbset_self_intersections
 * <li> For finding collision pairs between two box sets , call \ref gim_aabbset_box_collision
 * <li> After using collision routines, you must destroy the pairset with \ref GIM_DESTROY_PAIR_SET
 * <li> When the box set is no longer used, you must destroy it by calling \ref gim_aabbset_destroy
 * </ul>
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Le�n
 * 
 */
public class GimAABBSet { //Formerly GimBoxPruning

	//! @{
	/** Overlapping pair. */
	static class GIM_PAIR
	{
		int m_index1;
		int m_index2;
	};
	//typedef struct _GIM_PAIR GIM_PAIR;

	/** Box container */
	//	static class GIM_AABB_SET
	//	{
	//	    GUINT32 m_count;
	//	    aabb3f m_global_bound;//!< Global calculated bound of all boxes
	//	    aabb3f * m_boxes;
	//	    GUINT32 * m_maxcoords;//!<Upper corners of the boxes, in integer representation
	//	    GIM_RSORT_TOKEN * m_sorted_mincoords;//!< sorted min coords (lower corners), with their coord value as the m_key and m_value as the box index
	//	    char m_shared;//!< if m_shared == 0 then the memory is allocated and the set must be destroyed, else the pointers are shared and the set should't be destroyed
	int m_count;
	aabb3f m_global_bound = new aabb3f();//!< Global calculated bound of all boxes
	aabb3f[] m_boxes = new aabb3f[0];  //TZ Why init
	int[] m_maxcoords;//!<Upper corners of the boxes, in integer representation
	GIM_RSORT_TOKEN[] m_sorted_mincoords;//!< sorted min coords (lower corners), with their coord value as the m_key and m_value as the box index
	char m_shared;//!< if m_shared == 0 then the memory is allocated and the set must be destroyed, else the pointers are shared and the set should't be destroyed
	//	};
	//typedef  struct _GIM_AABB_SET GIM_AABB_SET;

	public aabb3f getGlobalBound() {
		return m_global_bound;
	}


	/** Function for creating  an overlapping pair set. */
	//#define GIM_CREATE_PAIR_SET(dynarray) GIM_DYNARRAY_CREATE(GIM_PAIR,dynarray,G_ARRAY_GROW_SIZE)
	static GimDynArray<GIM_PAIR> GIM_CREATE_PAIR_SET() { 
		return GimDynArray.GIM_DYNARRAY_CREATE(GimDynArray.G_ARRAY_GROW_SIZE);  //GIM_PAIR !! TZ TODO ? 
	}
	/**
	 * Function for destroying an overlapping pair set.
	 * @param dynarray
	 */
	//#define GIM_DESTROY_PAIR_SET(dynarray) GIM_DYNARRAY_DESTROY(dynarray)
	void GIM_DESTROY_PAIR_SET(GimDynArray<GIM_PAIR> dynarray) { dynarray.GIM_DYNARRAY_DESTROY(); }

	//! Allocate memory for all aabb set.
	//	void gim_aabbset_alloc(GIM_AABB_SET aabbset, GUINT32 count);









	/*
	                Brute-Force Vs Sorted pruning
	Different approaches must be applied when colliding sets with different number of
	elements. When sets have less of 100 boxes, is often better to apply force brute
	approach instead of sorted methods, because at lowlevel bruteforce routines gives
	better perormance and consumes less resources, due of their simplicity.
	But when sets are larger, the complexiity of bruteforce increases exponencially.
	In the case of large sets, sorted approach is applied. So GIMPACT has the following
	strategies:

	On Sorting sets:
	!) When sets have more of 140 boxes, the boxes are sorted by its coded min coord
	and the global box is calculated. But when sets are smaller (less of 140 boxes),
	Is convenient to apply brute force approach.

	 *******************************************************************************/

	//! Constant for apply approaches between brute force and sorted pruning on bipartite queries
	private static final int GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES = 600;
	//! Constant for apply approaches between brute force and sorted pruning for box collision
	private static final int GIM_MIN_SORTED_PRUNING_BOXES = 140;





	///Function for create Box collision result set

	//#define GIM_CREATE_BOXQUERY_LIST(dynarray) GIM_DYNARRAY_CREATE(GUINT32,dynarray,G_ARRAY_GROW_SIZE)
	static GimDynArrayInt GIM_CREATE_BOXQUERY_LIST() {
		return GimDynArrayInt.GIM_DYNARRAY_CREATE(GimDynArray.G_ARRAY_GROW_SIZE);
	}


	/*
	For sorting, each box corner must be discretized to a 32 bit integer.
	For this, we take the x and z coordinates from the box corner (a vector vec3f)
	Then convert the (x,z) pair to an integer. For convenience, we choose an error
	constant for converting the coordinates (0.05).
	 *******************************************************************************/

	/**
	 * For fitting the coordinate to an integer, we need to constraint the 
	 * range of its values. So each coord component (x, z) must lie between 0 and 65536.
	 * 20 give us a 0.05 floating point error
	 */
	private static final float ERROR_AABB = 20.0f;

	/**
	 * An error of 0.05 allows to make coordinates up to 1638.0f and no less of -1638.0f.
	 * So the maximum size of a room should be about 3276x3276 . Its dimensions must lie between  [-1638,1638.0f]
	 */
	private static final float MAX_AABB_SIZE = 1638.0f; //TZ ??? TODO ??

	/**
	 * Converts a vector coordinate to an integer for box sorting.
	 * @param vx X component
	 * @param vz Z component
	 * @param uint_key a GUINT
	 */
	//#define GIM_CONVERT_VEC3F_GUINT_XZ(vx,vz,uint_key)\
	int GIM_CONVERT_VEC3F_GUINT_XZ(float vx, float vz)
	{
		//	    int _z = ((GUINT32)(vz*ERROR_AABB))+32768;
		//	    uint_key = ((GUINT32)(vx*ERROR_AABB))+32768;
		//	    uint_key = (uint_key<<16) + _z;
		int _z = ((int)(vz*ERROR_AABB))+32768;
		int uint_key = ((int)(vx*ERROR_AABB))+32768;
		return (uint_key<<16) + _z;
	}

	/**
	 * Converts a vector coordinate to an integer for box sorting, rounding to the upper int.
	 * @param vx X component
	 * @param vz Z component
	 * @param uint_key a GUINT
	 */
	//#define GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(vx,vz,uint_key)\
	int GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(float vx, float vz)
	{
		//	    GUINT32 _z = ((GUINT32)ceilf(vz*ERROR_AABB))+32768;
		//	    uint_key = ((GUINT32)ceilf(vx*ERROR_AABB))+32768;
		//	    uint_key = (uint_key<<16) + _z;
		int _z = ((int)Math.ceil(vz*ERROR_AABB))+32768;
		int uint_key = ((int)Math.ceil(vx*ERROR_AABB))+32768;
		return (uint_key<<16) + _z;
	}


	/**
	 * Converts a vector coordinate to an integer for box sorting. Secure clamped.
	 * @param vx X component
	 * @param vz Z component
	 * @param uint_key a GUINT
	 */
	//#define GIM_CONVERT_VEC3F_GUINT_XZ_CLAMPED(vx,vz,uint_key)\
	int GIM_CONVERT_VEC3F_GUINT_XZ_CLAMPED(float vx, float vz)
	{
		//	    GREAL _cx = CLAMP(vx,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		//	    GREAL _cz = CLAMP(vz,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		//	    GUINT32 _z = ((GUINT32)(_cz*ERROR_AABB))+32768;
		//	    uint_key = ((GUINT32)(_cx*ERROR_AABB))+32768;
		//	    uint_key = (uint_key<<16) + _z;
		float _cx = CLAMP(vx,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		float _cz = CLAMP(vz,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		int _z = ((int)(_cz*ERROR_AABB))+32768;  //TZ should work, because is always positive!
		int uint_key = ((int)(_cx*ERROR_AABB))+32768;
		return (uint_key<<16) + _z;
	}

	/**
	 * Converts a vector coordinate to an integer for box sorting. Secure clamped, rounded.
	 * @param vx X component
	 * @param vz Z component
	 * @param uint_key a GUINT
	 */
	//#define GIM_CONVERT_VEC3F_GUINT_XZ_UPPER_CLAMPED(vx,vz,uint_key)\
	int GIM_CONVERT_VEC3F_GUINT_XZ_UPPER_CLAMPED(float vx, float vz)
	{
		//	    GREAL _cx = CLAMP(vx,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		//	    GREAL _cz = CLAMP(vz,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		//	    GUINT32 _z = ((GUINT32)ceilf(_cz*ERROR_AABB))+32768;
		//	    uint_key = ((GUINT32)ceilf(_cx*ERROR_AABB))+32768;
		//	    uint_key = (uint_key<<16) + _z;
		float _cx = CLAMP(vx,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		float _cz = CLAMP(vz,-MAX_AABB_SIZE,MAX_AABB_SIZE);
		int _z = ((int)Math.ceil(_cz*ERROR_AABB))+32768;
		int uint_key = ((int)Math.ceil(_cx*ERROR_AABB))+32768;
		return (uint_key<<16) + _z;
	}

	private GimAABBSet() {
		//private!!
	}

	/** Allocate memory for all aabb set. */
	//	void gim_aabbset_alloc(GIM_AABB_SET aabbset, GUINT32 count)
	static GimAABBSet gim_aabbset_alloc(int count)
	{
		GimAABBSet x = new GimAABBSet();

		//GimAABBSet aabbset = new GimAABBSet(); //TZ 
		x.m_count = count;
		x.m_boxes = new aabb3f[count];//(aabb3f *)gim_alloc(sizeof(aabb3f)*count);
		for (int i = 0; i < count; i++) x.m_boxes[i] = new aabb3f(); //TZ

		if(count<GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES)
		{
			x.m_maxcoords = null;
			x.m_sorted_mincoords = null;
		}
		else
		{
			x.m_maxcoords = new int[x.m_count];//(GUINT32 *)gim_alloc(sizeof(GUINT32)*aabbset.m_count );
			//m_sorted_mincoords = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN)*aabbset.m_count);
			x.m_sorted_mincoords = new GIM_RSORT_TOKEN[x.m_count];
			for (int i = 0; i < x.m_count; i++) x.m_sorted_mincoords[i] = new GIM_RSORT_TOKEN(); 
		}
		x.m_shared = 0;
		GimGeometry.INVALIDATE_AABB(x.m_global_bound);
		return x;
	}

	/** Destroys the aabb set. */
	void gim_aabbset_destroy()
	{
		m_count = 0;
		//TZ just set to null, will be gc'd if not shared
		//	    if(m_shared==0)
		//	    {
		//	        gim_free(m_boxes,0);
		//	        gim_free(m_maxcoords,0);
		//	        gim_free(m_sorted_mincoords,0);
		//	    }
		m_boxes = null;
		m_sorted_mincoords = null;
		m_maxcoords = null;
	}

	/**
	 * Calcs the global bound only.
	 * @pre aabbset must be allocated. And the boxes must be already set.
	 */
	//void gim_aabbset_calc_global_bound(GIM_AABB_SET * aabbset);
	void gim_aabbset_calc_global_bound()
	{
		aabb3f[] paabb = m_boxes;
		int paabbPos = 0;//TZ
		aabb3f globalbox = m_global_bound;
		AABB_COPY(globalbox,paabb[paabbPos]);

		int count = m_count-1;
		paabbPos++;
		while(count!=0)
		{
			MERGEBOXES(globalbox,paabb[paabbPos]);
			paabbPos++;
			count--;
		}
	}


	/**
	 * Sorts the boxes for box prunning.
	 * 1) find the integer representation of the aabb coords
	 * 2) Sorts the min coords
	 * 3) Calcs the global bound
	 * @pre aabbset must be allocated. And the boxes must be already set.
	 * @param aabbset
	 * @param calc_global_bound If 1 , calcs the global bound
	 * @post If aabbset.m_sorted_mincoords == 0, then it allocs the sorted coordinates
	 */
	void gim_aabbset_sort(final boolean calc_global_bound)
	{
		if(m_sorted_mincoords == null)
		{//allocate
			m_maxcoords = new int[m_count];//(GUINT32 *)gim_alloc(sizeof(GUINT32)*aabbset.m_count );
			m_sorted_mincoords = new GIM_RSORT_TOKEN[m_count];//(GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN)*aabbset.m_count);
		}

		int i, count = m_count;
		aabb3f[] paabb = m_boxes;
		int[] maxcoords = m_maxcoords;
		GIM_RSORT_TOKEN[] sorted_tokens = m_sorted_mincoords;

		if(count<860)//Calibrated on a Pentium IV
		{
			//Sort by quick sort
			//Calculate keys
			for(i=0;i<count;i++)
			{
				maxcoords[i] = GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(paabb[i].maxX,paabb[i].maxZ);
				sorted_tokens[i].m_key = GIM_CONVERT_VEC3F_GUINT_XZ(paabb[i].minX,paabb[i].minZ);
				sorted_tokens[i].m_value = i;
			}
			//GIM_QUICK_SORT_ARRAY(GIM_RSORT_TOKEN , sorted_tokens, count, RSORT_TOKEN_COMPARATOR,GIM_DEF_EXCHANGE_MACRO);
			GimRadixSort.GIM_QUICK_SORT_ARRAY(sorted_tokens, count, 
					GimRadixSort.RSORT_TOKEN_COMPARATOR, GimRadixSort.GIM_DEF_EXCHANGE_MACRO);
		}
		else
		{
			//Sort by radix sort
			//GIM_RSORT_TOKEN[] unsorted = gim_alloc(count, GIM_RSORT_TOKEN.class);//(GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN )*count);
			GIM_RSORT_TOKEN[] unsorted = new GIM_RSORT_TOKEN[count]; //TZ TODO clean up
			//Calculate keys
			for(i=0;i<count;i++)
			{
				unsorted[i] = new GIM_RSORT_TOKEN();
				maxcoords[i] = GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(paabb[i].maxX,paabb[i].maxZ);
				unsorted[i].m_key = GIM_CONVERT_VEC3F_GUINT_XZ(paabb[i].minX,paabb[i].minZ);
				unsorted[i].m_value = i;
			}
			GimRadixSort.GIM_RADIX_SORT_RTOKENS(unsorted,sorted_tokens,count);
			//gim_free(unsorted,0); //TODO remove TZ
		}

		if(calc_global_bound) gim_aabbset_calc_global_bound();
	}

	//utility macros

	/*#define PUSH_PAIR(i,j,pairset)\
	{\
	    GIM_PAIR _pair={i,j};\
	    GIM_DYNARRAY_PUSH_ITEM(GIM_PAIR,pairset,_pair);\
	}*/

	private interface PushPairMacro {
		void run(final int i, final int j, GimDynArray<GIM_PAIR> pairset);
	}
	private static PushPairMacro PUSH_PAIR = new PushPairMacro() {
		@Override public void run(int i, int j, GimDynArray<GIM_PAIR> pairset) {
			PUSH_PAIR(i, j, pairset);
		}
	};
	private static PushPairMacro PUSH_PAIR_INV = new PushPairMacro() {
		@Override public void run(int i, int j, GimDynArray<GIM_PAIR> pairset) {
			PUSH_PAIR_INV(i, j, pairset);
		}
	};

	//#define PUSH_PAIR(i,j,pairset)\
	static void PUSH_PAIR(final int i, final int j, GimDynArray<GIM_PAIR> pairset)
	{
		//		pairset.GIM_DYNARRAY_PUSH_EMPTY();
		//	    GIM_PAIR[] _pair = pairset.GIM_DYNARRAY_POINTER() + (pairset).m_size - 1;
		GIM_PAIR _pair = new GIM_PAIR();
		_pair.m_index1 = i;
		_pair.m_index2 = j;
		pairset.GIM_DYNARRAY_PUSH_ITEM_TZ(_pair);
	}

	//#define PUSH_PAIR_INV(i,j,pairset)\
	static void PUSH_PAIR_INV(final int i, final int j, GimDynArray<GIM_PAIR> pairset)
	{
		//		pairset.GIM_DYNARRAY_PUSH_EMPTY();
		//	    GIM_PAIR[] _pair = GIM_DYNARRAY_POINTER(GIM_PAIR,pairset) + (pairset).m_size - 1;
		GIM_PAIR _pair = new GIM_PAIR();
		_pair.m_index1 = j;
		_pair.m_index2 = i;
		pairset.GIM_DYNARRAY_PUSH_ITEM_TZ(_pair);
	}

	//	#define FIND_OVERLAPPING_FOWARD(\
	//	 curr_index,\
	//	 test_count,\
	//	 test_aabb,\
	//	 max_coord_uint,\
	//	 sorted_tokens,\
	//	 aabbarray,\
	//	 pairset,\
	//	 push_pair_macro)\
	static void FIND_OVERLAPPING_FOWARD(
			final int curr_index,
			final int test_count,
			aabb3f test_aabb,
			final int max_coord_uint,
			GIM_RSORT_TOKEN[] sorted_tokensA,
			int sorted_tokensP,
			aabb3f[] aabbarray,
			GimDynArray<GIM_PAIR> pairset,
			PushPairMacro push_pair_macro)
	{
		int _i = test_count;
		boolean _intersected;
		GIM_RSORT_TOKEN[] _psorted_tokensA = sorted_tokensA;
		int _psorted_tokensP = sorted_tokensP;
		while(_i>0 && max_coord_uint >= _psorted_tokensA[_psorted_tokensP].m_key)
		{
			_intersected = AABBCOLLISION(test_aabb,aabbarray[_psorted_tokensA[_psorted_tokensP].m_value]);
			if(_intersected)
			{
				push_pair_macro.run(curr_index, _psorted_tokensA[_psorted_tokensP].m_value,pairset);
			}
			_psorted_tokensP++;
			_i--;
		}
	}

	/**
	 * log(N) Complete box pruning. Returns a list of overlapping pairs of 
	 * boxes, each box of the pair belongs to the same set.
	 * @pre aabbset must be allocated and sorted, the boxes must be already set.
	 * @param aabbset Must be sorted. Global bound isn't required
	 * @param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_self_intersections_sorted(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
	void gim_aabbset_self_intersections_sorted(GimDynArray<GIM_PAIR> collision_pairs)
	{
		collision_pairs.m_size = 0;
		int count = m_count;
		aabb3f[] paabb = m_boxes;
		int[] maxcoords = m_maxcoords;
		GIM_RSORT_TOKEN[] sorted_tokensA = m_sorted_mincoords;
		int sorted_tokensP = 0;
		aabb3f test_aabb = new aabb3f();
		while(count>1)
		{
			///current cache variables
			int curr_index = sorted_tokensA[sorted_tokensP].m_value;
			int max_coord_uint = maxcoords[curr_index];
			AABB_COPY(test_aabb,paabb[curr_index]);

			///next pairs
			sorted_tokensP++;
			count--;
			FIND_OVERLAPPING_FOWARD( curr_index, count, test_aabb, max_coord_uint, 
					sorted_tokensA, sorted_tokensP, paabb, collision_pairs,PUSH_PAIR);
		}
	}

	/**
	 * NxN Complete box pruning. Returns a list of overlapping pairs of 
	 * boxes, each box of the pair belongs to the same set.
	 * @pre aabbset must be allocated, the boxes must be already set.
	 * @param aabbset Global bound isn't required. Doen't need to be sorted.
	 * @param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_self_intersections_brute_force(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
	void gim_aabbset_self_intersections_brute_force(GimDynArray<GIM_PAIR> collision_pairs)
	{
		collision_pairs.m_size = 0;
		int i,j;
		int count = m_count;
		aabb3f[] paabb = m_boxes;
		boolean intersected;
		for (i=0;i< count-1 ;i++ )
		{
			for (j=i+1;j<count ;j++ )
			{
				intersected = AABBCOLLISION(paabb[i],paabb[j]);
				if(intersected)
				{
					PUSH_PAIR(i,j,collision_pairs);
				}
			}
		}
	}

	/**
	 * log(N) Bipartite box pruning. Returns a list of overlapping pairs of 
	 * boxes, each box of the pair belongs to a different set.
	 * @pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
	 * @param aabbset1 Must be sorted, Global bound is required.
	 * @param aabbset2 Must be sorted, Global bound is required.
	 * @param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_bipartite_intersections_sorted(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
	static void gim_aabbset_bipartite_intersections_sorted(
			GimAABBSet aabbset1, GimAABBSet aabbset2, GimDynArray<GIM_PAIR> collision_pairs)
	{
		boolean intersected;
		collision_pairs.m_size = 0;

		intersected = AABBCOLLISION(aabbset1.m_global_bound,aabbset2.m_global_bound);
		if(intersected == false) return;

		int count1 = aabbset1.m_count;
		aabb3f[] paabb1 = aabbset1.m_boxes;
		int[] maxcoords1 = aabbset1.m_maxcoords;
		GIM_RSORT_TOKEN[] sorted_tokens1 = aabbset1.m_sorted_mincoords;

		int count2 = aabbset2.m_count;
		aabb3f[] paabb2 = aabbset2.m_boxes;
		int[] maxcoords2 = aabbset2.m_maxcoords;
		GIM_RSORT_TOKEN[] sorted_tokens2 = aabbset2.m_sorted_mincoords;

		int  curr_index;

		int max_coord_uint;
		aabb3f test_aabb = new aabb3f();

		//Classify boxes
		//Find  Set intersection
		aabb3f int_abbb = new aabb3f();
		BOXINTERSECTION(aabbset1.m_global_bound,aabbset2.m_global_bound, int_abbb);

		//Clasify set 1
		GIM_RSORT_TOKEN[] classified_tokens1 = new GIM_RSORT_TOKEN[count1];//(GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN)*count1);
		int i,classified_count1 = 0,classified_count2 = 0;


		for (i=0;i<count1;i++ )
		{
			curr_index = sorted_tokens1[i].m_value;
			intersected = AABBCOLLISION(paabb1[curr_index],int_abbb);
			if(intersected)
			{
				classified_tokens1[classified_count1] = sorted_tokens1[i];
				classified_count1++;
			}
		}

		if(classified_count1==0)
		{
			//TODO remove TZ gim_free(classified_tokens1 ,0);
			return; // no pairs
		}

		//Clasify set 2
		//GIM_RSORT_TOKEN * classified_tokens2 = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN)*count2);
		GIM_RSORT_TOKEN[] classified_tokens2 = new GIM_RSORT_TOKEN[count2];

		for (i=0;i<count2;i++ )
		{
			curr_index = sorted_tokens2[i].m_value;
			intersected = AABBCOLLISION(paabb2[curr_index],int_abbb);
			if(intersected)
			{
				classified_tokens2[classified_count2] = sorted_tokens2[i];
				classified_count2++;
			}
		}

		if(classified_count2==0)
		{
			//TODO remove TZ
			//	        gim_free(classified_tokens1 ,0);
			//	        gim_free(classified_tokens2 ,0);
			return; // no pairs
		}

		sorted_tokens1 = classified_tokens1;
		sorted_tokens2 = classified_tokens2;
		int pos1 = 0, pos2 = 0;

		while(classified_count1>0&&classified_count2>0)
		{
			if(sorted_tokens1[pos1].m_key <= sorted_tokens2[pos2].m_key)
			{
				///current cache variables
				curr_index = sorted_tokens1[pos1].m_value;
				max_coord_uint = maxcoords1[curr_index];
				AABB_COPY(test_aabb,paabb1[curr_index]);
				///next pairs
				pos1++;//sorted_tokens1++;
				classified_count1--;
				FIND_OVERLAPPING_FOWARD( curr_index, classified_count2, test_aabb, 
						max_coord_uint, sorted_tokens2, 0 , paabb2, collision_pairs, PUSH_PAIR);
			}
			else ///Switch test
			{
				///current cache variables
				curr_index = sorted_tokens2[pos2].m_value;
				max_coord_uint = maxcoords2[curr_index];
				AABB_COPY(test_aabb,paabb2[curr_index]);
				///next pairs
				pos2++;//sorted_tokens2++;
				classified_count2--;
				FIND_OVERLAPPING_FOWARD( curr_index, classified_count1, test_aabb, 
						max_coord_uint, sorted_tokens1, 0 , paabb1, collision_pairs, PUSH_PAIR_INV );
			}
		}
		//TODO TZ remove
		//	    gim_free(classified_tokens1 ,0);
		//	    gim_free(classified_tokens2 ,0);
	}

	/**
	 * NxM Bipartite box pruning. Returns a list of overlapping pairs of boxes,
	 * each box of the pair belongs to a different set.
	 * @pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
	 * @param aabbset1 Must be sorted, Global bound is required.
	 * @param aabbset2 Must be sorted, Global bound is required.
	 * @param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_bipartite_intersections_brute_force(GIM_AABB_SET * aabbset1,GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
	static void gim_aabbset_bipartite_intersections_brute_force(GimAABBSet aabbset1, GimAABBSet aabbset2, 
			GimDynArray<GIM_PAIR> collision_pairs)
	{
		boolean intersected;
		collision_pairs.m_size = 0;
		intersected = AABBCOLLISION(aabbset1.m_global_bound,aabbset2.m_global_bound);
		if(intersected == false) return;

		aabb3f int_abbb = new aabb3f();
		//Find  Set intersection
		BOXINTERSECTION(aabbset1.m_global_bound,aabbset2.m_global_bound, int_abbb);
		//Clasify set 1
		int i,j;
		int classified_count = 0;

		int count = aabbset1.m_count;
		aabb3f[] paabb1 = aabbset1.m_boxes;
		aabb3f[] paabb2 = aabbset2.m_boxes;

		//GUINT32 * classified = (GUINT32 *) gim_alloc(sizeof(GUINT32)*count);
		int[] classified = new int[count];//(GUINT32 *) gim_alloc(sizeof(GUINT32)*count);

		for (i=0;i<count;i++ )
		{
			intersected = AABBCOLLISION(paabb1[i],int_abbb);
			if(intersected)
			{
				classified[classified_count] = i;
				classified_count++;
			}
		}

		if(classified_count==0)
		{
			//gim_free(classified,0); TODO TZ remove
			return; // no pairs
		}

		//intesect set2
		count = aabbset2.m_count;
		for (i=0;i<count;i++)
		{
			intersected = AABBCOLLISION(paabb2[i],int_abbb);
			if(intersected)
			{
				for (j=0;j<classified_count;j++)
				{
					intersected = AABBCOLLISION(paabb2[i],paabb1[classified[j]]);
					if(intersected)
					{
						PUSH_PAIR(classified[j],i,collision_pairs);
					}
				}
			}
		}
		//gim_free(classified,0); TODO TZ remove
	}


	//Use these functions for general initialization

	/**
	 * Initalizes the set. Sort Boxes if needed.
	 * @pre aabbset must be allocated. And the boxes must be already set.
	 * @post If the set has less of GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES boxes, only calcs the global box,
	 * else it Sorts the entire set( Only applicable for large sets)
	 */
	//void gim_aabbset_update(GIM_AABB_SET * aabbset)
	void gim_aabbset_update()
	{
		if(m_count < GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES)
		{//Brute force approach
			gim_aabbset_calc_global_bound();
		}
		else
		{//Sorted force approach
			gim_aabbset_sort(true);
		}
	}

	///Use these functions for general collision

	/**
	 * Complete box pruning. Returns a list of overlapping pairs of boxes, 
	 * each box of the pair belongs to the same set.
	 * This function sorts the set and then it calls to 
	 * gim_aabbset_self_intersections_brute_force or 
	 * gim_aabbset_self_intersections_sorted. This is an example of how to use this function:
	 * <code>
	 * //Create contact list
	 * GDYNAMIC_ARRAY collision_pairs;
	 * GIM_CREATE_PAIR_SET(collision_pairs);
	 * //Do collision
	 * gim_aabbset_self_intersections(&aabbset,&collision_pairs);
	 * if(collision_pairs.m_size==0)
	 * {
	 *     GIM_DYNARRAY_DESTROY(collision_pairs);//
	 *     return; //no collisioin
	 * }
	 * //pair pointer
	 * GIM_PAIR *pairs = GIM_DYNARRAY_POINTER(GIM_PAIR,collision_pairs);
	 * GUINT i, ti1,ti2;
	 * for (i=0;i<collision_pairs.m_size; i++)
	 * {
	 *     ti1 = pairs[i].m_index1;
	 *     ti2 = pairs[i].m_index2;
	 *     //Do something with the pairs
	 *     ....
	 *     ....
	 *     ...
	 * }
	 * //Terminate
	 * GIM_DYNARRAY_DESTROY(dummycontacts);
	 * GIM_DYNARRAY_DESTROY(collision_pairs);
	 * </code>
	 * @param aabbset Set of boxes. Sorting isn't required.
	 * @param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
	 * @pre aabbset must be allocated and initialized.
	 * @post If aabbset->m_count >= GIM_MIN_SORTED_PRUNING_BOXES, then it calls to gim_aabbset_sort and then to gim_aabbset_self_intersections_sorted.
	 */
	//void gim_aabbset_self_intersections(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
	void gim_aabbset_self_intersections(GimDynArray<GIM_PAIR> collision_pairs)
	{
		if(m_count < GIM_MIN_SORTED_PRUNING_BOXES)
		{//Brute force approach
			gim_aabbset_self_intersections_brute_force(collision_pairs);
		}
		else
		{//Sorted force approach
			gim_aabbset_sort(false);//aabbset,0);
			gim_aabbset_self_intersections_sorted(collision_pairs);
		}
	}

	/**
	 * Collides two sets. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
	 * @pre aabbset1 and aabbset2 must be allocated and updated. See gim_aabbset_update.
	 * @param aabbset1 Must be sorted, Global bound is required.
	 * @param aabbset2 Must be sorted, Global bound is required.
	 * @param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_bipartite_intersections(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
	static void gim_aabbset_bipartite_intersections(GimAABBSet aabbset1, 
			GimAABBSet aabbset2, GimDynArray<GIM_PAIR> collision_pairs)
	{
		if(aabbset1.m_sorted_mincoords == null||aabbset2.m_sorted_mincoords == null)
		{//Brute force approach
			gim_aabbset_bipartite_intersections_brute_force(aabbset1,aabbset2,collision_pairs);
		}
		else
		{//Sorted force approach
			gim_aabbset_bipartite_intersections_sorted(aabbset1,aabbset2,collision_pairs);
		}
	}

	/**
	 * Finds intersections between a box and a set. Return the colliding boxes of the set
	 * @pre aabbset must be allocated and initialized.
	 * @param test_aabb Box for collision query
	 * @param aabbset Set of boxes .Global bound is required.
	 * @param collided Array of GUINT elements, indices of boxes. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_box_collision(aabb3f *test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
	public void gim_aabbset_box_collision(aabb3f test_aabb, GimDynArrayInt collided)
	{
		collided.m_size = 0;
		boolean intersected = GimGeometry.AABBCOLLISION(m_global_bound,test_aabb);
		if(intersected == false) return;

		int i;
		int count = m_count;
		aabb3f[] paabb = m_boxes;
		aabb3f _testaabb = new aabb3f();
		GimGeometry.AABB_COPY(_testaabb,test_aabb);

		for (i=0;i< count;i++ )
		{
			intersected = GimGeometry.AABBCOLLISION(paabb[i],_testaabb);
			if(intersected)
			{
				collided.GIM_DYNARRAY_PUSH_ITEM(i);
			}
		}
	}

	/**
	 * Finds intersections between a box and a set. Return the colliding boxes of the set
	 * @pre aabbset must be allocated and initialized.
	 * @param vorigin Origin point of ray.
	 * @param vdir Direction vector of ray.
	 * @param tmax Max distance param for ray.
	 * @param aabbset Set of boxes .Global bound is required.
	 * @param collided Array of GUINT elements, indices of boxes. Must be initialized before (Reserve size ~ 100)
	 */
	//void gim_aabbset_ray_collision(vec3f vorigin,vec3f vdir, GREAL tmax, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
	void gim_aabbset_ray_collision(vec3f vorigin,vec3f vdir, float tmax, 
			GimDynArrayInt collided)
	{
		collided.m_size = 0;
		RefBoolean intersected = new RefBoolean(false);
		RefFloat tparam = new RefFloat();
		GimGeometry.BOX_INTERSECTS_RAY(m_global_bound, vorigin, vdir, tparam, tmax,intersected);
		if(intersected.b==false) return;

		int i;
		int count = m_count;
		aabb3f[] paabb = m_boxes;

		for (i=0;i< count;i++ )
		{
			GimGeometry.BOX_INTERSECTS_RAY(paabb[i], vorigin, vdir, tparam, tmax,intersected);
			if(intersected.b)
			{
				collided.GIM_DYNARRAY_PUSH_ITEM(i);//GUINT32,(collided),i);
			}
		}
	}
}
