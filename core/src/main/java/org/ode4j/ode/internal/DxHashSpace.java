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

import java.util.ArrayList;

import org.ode4j.ode.DAABB;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.cpp4j.C_All.*;

/**
 * From collision_space.cpp
 */
public class DxHashSpace extends DxSpace implements DHashSpace {
	//****************************************************************************
	// utility stuff for hash table space

	// kind of silly, but oh well...
	//	#ifndef MAXINT
	//	#define MAXINT ((int)((((unsigned int)(-1)) << 1) >> 1))
	//	#endif
	private static final int MAXINT = Integer.MAX_VALUE;


	// prime[i] is the largest prime smaller than 2^i
	//	#define NUM_PRIMES 31
	//	static const long int prime[NUM_PRIMES] = {1L,2L,3L,7L,13L,31L,61L,127L,251L,509L,
	//	  1021L,2039L,4093L,8191L,16381L,32749L,65521L,131071L,262139L,
	//	  524287L,1048573L,2097143L,4194301L,8388593L,16777213L,33554393L,
	//	  67108859L,134217689L,268435399L,536870909L,1073741789L};
	private static final int NUM_PRIMES = 31;
	private static final int[] prime =  {1,2,3,7,13,31,61,127,251,509,
		1021,2039,4093,8191,16381,32749,65521,131071,262139,
		524287,1048573,2097143,4194301,8388593,16777213,33554393,
		67108859,134217689,268435399,536870909,1073741789};


	// an axis aligned bounding box in the hash table
	private static class dxAABB {
		int level;		// the level this is stored in (cell size = 2^level)
		int[] dbounds = new int[6];	// AABB bounds, discretized to cell size
		DxGeom geom;		// corresponding geometry object (AABB stored here)
		int index;		// index of this AABB, starting from 0
	}


	// a hash table node that represents an AABB that intersects a particular cell
	// at a particular level
	private static class Node {
		Node next;		// next node in hash table collision list, 0 if none
		int x,y,z;		// cell position in space, discretized to cell size
		dxAABB aabb;		// axis aligned bounding box that intersects this cell
	}


	// return the `level' of an AABB. the AABB will be put into cells at this
	// level - the cell size will be 2^level. the level is chosen to be the
	// smallest value such that the AABB occupies no more than 8 cells, regardless
	// of its placement. this means that:
	//		size/2 < q <= size
	// where q is the maximum AABB dimension.

	private static int findLevel (DAABB boundsV) //[6])
	{
		//double[] bounds = boundsV.v;
		//	  if (bounds[0] <= -dInfinity || bounds[1] >= dInfinity ||
		//	      bounds[2] <= -dInfinity || bounds[3] >= dInfinity ||
		//	      bounds[4] <= -dInfinity || bounds[5] >= dInfinity) {
		//	    return MAXINT;
		//	  }
		if (!boundsV.isValid()) {
			return MAXINT;
		}

		// compute q
		double q,q2;
		q = boundsV.len0();//bounds[1] - bounds[0];	// x bounds
		q2 = boundsV.len1();//bounds[3] - bounds[2];	// y bounds
		if (q2 > q) q = q2;
		q2 = boundsV.len2();//bounds[5] - bounds[4];	// z bounds
		if (q2 > q) q = q2;

		// find level such that 0.5 * 2^level < q <= 2^level
		RefInt level = new RefInt();
		frexp (q,level);	// q = (0.5 .. 1.0) * 2^level (definition of frexp)
		return level.i;
	}
	// find a virtual memory address for a cell at the given level and x,y,z
	// position.
	// @@@ currently this is not very sophisticated, e.g. the scaling
	// factors could be better designed to avoid collisions, and they should
	// probably depend on the hash table physical size.

	private static long getVirtualAddressBase(int level, int x, int y) {
		long r = level * 1000L + x * 100L + y * 10L;
		//assert (r >= 0);
		return r;
	}

	//****************************************************************************
	// hash space

	//	struct dxHashSpace : public dxSpace {
	private int global_minlevel;	// smallest hash table level to put AABBs in
	private int global_maxlevel;	// objects that need a level larger than this will be
	// put in a "big objects" list instead of a hash table

	//	  dxHashSpace (dSpace _space);
	//	  void setLevels (int minlevel, int maxlevel);
	//	  void getLevels (int *minlevel, int *maxlevel);
	//	  void cleanGeoms();
	//	  void collide (void *data, dNearCallback *callback);
	//	  void collide2 (void *data, dxGeom *geom, dNearCallback *callback);
	//	};


	DxHashSpace (DxSpace _space)// : dxSpace (_space)
	{
		super(_space);
		type = dHashSpaceClass;
		global_minlevel = -3;
		global_maxlevel = 10;
	}


	@Override
	public void setLevels (int minlevel, int maxlevel)
	{
		dAASSERT (minlevel <= maxlevel);
		global_minlevel = minlevel;
		global_maxlevel = maxlevel;
	}


	//	void getLevels (int *minlevel, int *maxlevel)
	@Override
	public int getLevelMin ()
	{
		return global_minlevel;
	}
	@Override
	public int getLevelMax ()
	{
		return global_maxlevel;
	}


	@Override
	public void cleanGeoms()
	{
		// compute the AABBs of all dirty geoms, and clear the dirty flags
		lock_count++;
		for (DxGeom g : getGeomsDx()) {
			//if ((g._gflags & GEOM_DIRTY)==0) break;
			if (!g.hasFlagDirty()) break;
			if (g instanceof DxSpace) {
				((DxSpace)g).cleanGeoms();
			}

			g.recomputeAABB();
			// dIASSERT((g->gflags & GEOM_AABB_BAD) == 0);
			// g->gflags &= ~GEOM_DIRTY;
			dIASSERT(!g.hasFlagAabbBad());
			g.unsetFlagDirty();
		}
		lock_count--;
	}

	@Override
	public void collide (Object data, DNearCallback callback)
	{
		dAASSERT(callback);
		//dxGeom geom; //*
		int i,maxlevel;

		// 0 or 1 geoms can't collide with anything
		if (getNumGeoms() < 2) return;

		lock_count++;
		cleanGeoms();

		// create a list of auxiliary information for all geom axis aligned bounding
		// boxes. set the level for all AABBs. put AABBs larger than the space's
		// global_maxlevel in the big_boxes list, check everything else against
		// that list at the end. for AABBs that are not too big, record the maximum
		// level that we need.

		ArrayList<dxAABB> hash_boxes = new ArrayList<>();	// list of AABBs in hash table
		ArrayList<dxAABB> big_boxes = new ArrayList<>();	// list of AABBs too big for hash table
		maxlevel = global_minlevel - 1;
		for (DxGeom geom : getGeomsDx()) {
			if (!GEOM_ENABLED(geom)){
				continue;
			}
			dxAABB aabb = new dxAABB(); //(dxAABB) ALLOCA (sizeof(dxAABB));
			aabb.geom = geom;
			// compute level, but prevent cells from getting too small
			int level = findLevel (geom._aabb);
			if (level < global_minlevel) level = global_minlevel;
			if (level <= global_maxlevel) {
				aabb.level = level;
				if (level > maxlevel) maxlevel = level;
				// cellsize = 2^level
				double cellSizeRecip = dRecip(ldexp(1.0, level)); // No computational errors here!
				// discretize AABB position to cell size
				for (i=0; i < 3; i++) {
					double aabbBoundMin = Math.floor (geom._aabb.getMin(i) * cellSizeRecip); // No computational errors so far!
					double aabbBoundMax = Math.floor (geom._aabb.getMax(i) * cellSizeRecip); // No computational errors so far!
					dICHECK(aabbBoundMin >= Integer.MIN_VALUE && aabbBoundMin </*=*/ Integer.MAX_VALUE); // Otherwise the scene is too large for integer types used
					dICHECK(aabbBoundMax >= Integer.MIN_VALUE && aabbBoundMax </*=*/ Integer.MAX_VALUE); // Otherwise the scene is too large for integer types used
					aabb.dbounds[2*i] = (int)Math.floor (aabbBoundMin);
					aabb.dbounds[2*i+1] = (int)Math.floor (aabbBoundMax);
				}
				// set AABB index
				aabb.index = hash_boxes.size();
	            // aabb goes in main list
	            hash_boxes.add(aabb);
			}
			else {
				// aabb is too big, put it in the big_boxes list. we don't care about
				// setting level, dbounds, index, or the maxlevel
				big_boxes.add(aabb);
			}
		}

	    int n = hash_boxes.size(); // number of AABBs in main list

		// for `n' objects, an n*n array of bits is used to record if those objects
		// have been intersection-tested against each other yet. this array can
		// grow large with high n, but oh well...
		int tested_rowsize = (n+7) >> 3;	// number of bytes needed for n bits
		//std::vector<unsigned char> tested(n * tested_rowsize);
		int[] tested = new int[n * tested_rowsize];

		// create a hash table to store all AABBs. each AABB may take up to 8 cells.
		// we use chaining to resolve collisions, but we use a relatively large table
		// to reduce the chance of collisions.

		// compute hash table size sz to be a prime > 8*n
		for (i=0; i<NUM_PRIMES; i++) {
			if (prime[i] >= (8*n)) break;
		}
		if (i >= NUM_PRIMES) 
			i = NUM_PRIMES-1;	// probably pointless
		int sz = prime[i];

		// allocate and initialize hash table node pointers
		//std::vector<Node*> table(sz);
		Node[] table = new Node[sz];

		// add each AABB to the hash table (may need to add it to up to 8 cells)
		//TODO use proper List here!
		//for (aabb=first_aabb; aabb != null; aabb=aabb.next) {
		for (dxAABB aabb: hash_boxes) {
			int[] dbounds = aabb.dbounds;
			int xend = dbounds[1];
			for (int xi = dbounds[0]; xi <= xend; xi++) {
				int yend = dbounds[3];
				for (int yi = dbounds[2]; yi <= yend; yi++) {
					int zbegin = dbounds[4];
					int hi = (int) Math.floorMod((getVirtualAddressBase (aabb.level,xi,yi) + zbegin) , (long) sz);
					int zend = dbounds[5];
					for (int zi = zbegin; zi <= zend; hi = hi + 1 != sz ? hi + 1 : 0, zi++) {
						// add a new node to the hash table
						Node node = new Node();//(Node) ALLOCA (sizeof (Node));
						node.x = xi;
						node.y = yi;
						node.z = zi;
						node.aabb = aabb;
						node.next = table[hi];
						table[hi] = node;
					}
				}
			}
		}

		// now that all AABBs are loaded into the hash table, we do the actual
		// collision detection. for all AABBs, check for other AABBs in the
		// same cells for collisions, and then check for other AABBs in all
		// intersecting higher level cells.

		int[] db = new int[6];			// discrete bounds at current level
		for (dxAABB aabb: hash_boxes) {
			// we are searching for collisions with aabb
			for (i=0; i<6; i++) db[i] = aabb.dbounds[i];
			for (int level = aabb.level; ; ) {
				dIASSERT(level <= maxlevel);
	            final int xend = db[1];
				for (int xi = db[0]; xi <= xend; xi++) {
					final int yend = db[3];
					for (int yi = db[2]; yi <= yend; yi++) {
						int zbegin = db[4];
						// get the hash index
						int hi = (int)Math.floorMod((getVirtualAddressBase(level, xi, yi) + zbegin), (long)sz);
						final int zend = db[5];
						for (int zi = zbegin; zi <= zend; hi = hi + 1 != sz ? hi + 1 : 0, zi++) {
							// search all nodes at this index
							for (Node node = table[hi]; node != null; node=node.next) {
								// node points to an AABB that may intersect aabb
								if (node.aabb == aabb) 
									continue;
								if (node.aabb.level == level &&
										node.x == xi && node.y == yi && node.z == zi) {
									// see if aabb and node->aabb have already been tested
									// against each other
									//TZ unsigned char mask;
									int mask;
									if (aabb.index <= node.aabb.index) {
										i = (aabb.index * tested_rowsize)+(node.aabb.index >> 3);
										mask = 1 << (node.aabb.index & 7);
									}
									else {
										i = (node.aabb.index * tested_rowsize)+(aabb.index >> 3);
										mask = 1 << (aabb.index & 7);
									}
									dIASSERT (i >= 0 && i < (tested_rowsize*n));
									if ((tested[i] & mask)==0) {
										tested[i] |= mask;
										collideAABBs (aabb.geom,node.aabb.geom,data,callback);
									}
								}
							}
						}
					}
				}

				if (level == maxlevel) {
					break;
				}
				++level;
				// get the discrete bounds for the next level up
				for (i=0; i<6; i++) 
					db[i] >>= 1;
			}
		}

		// every AABB in the normal list must now be intersected against every
		// AABB in the big_boxes list. so let's hope there are not too many objects
		// in the big_boxes list.
		for (dxAABB aabb: hash_boxes) {
			for (dxAABB aabb2: big_boxes) {
				collideAABBs (aabb.geom,aabb2.geom,data,callback);
			}
		}

		// intersected all AABBs in the big_boxes list together
		for (dxAABB aabb: big_boxes) {
			for (dxAABB aabb2: big_boxes) {
				collideAABBs (aabb.geom,aabb2.geom,data,callback);
			}
		}

		//(TZ): Not necessary in Java
//	    // deallocate table
//	    for (int ii=0; ii<table.length; ++i)
//	        for (Node node = table[ii]; node != null;) {
//	            Node next = node.next;
//	            delete node;
//	            node = next;
//	        }

		lock_count--;
	}

	@Override
	void collide2 (Object data, DxGeom geom,
			DNearCallback callback)
	{
		dAASSERT (geom, callback);

		// this could take advantage of the hash structure to avoid
		// O(n2) complexity, but it does not yet.

		lock_count++;
		cleanGeoms();
		geom.recomputeAABB();

		// intersect bounding boxes
		for (DxGeom g : getGeomsDx()) {
			if (GEOM_ENABLED(g)) collideAABBs (g,geom,data,callback);
		}

		lock_count--;
	}

	//****************************************************************************
	// space functions

	//	dxSpace *dSimpleSpaceCreate (dxSpace *space)
	//	{
	//	  return new dxSimpleSpace (space);
	//	}
	//TZ move to dxSpace

	public static DxHashSpace dHashSpaceCreate (DxSpace space)
	{
		return new DxHashSpace (space);
	}

	//void dHashSpaceSetLevels (dxSpace space, int minlevel, int maxlevel)
	void dHashSpaceSetLevels (int minlevel, int maxlevel)
	{
		//dAASSERT (space);
		dUASSERT (minlevel <= maxlevel,"must have minlevel <= maxlevel");
		//dUASSERT (space.type == dHashSpaceClass,"argument must be a hash space");
		//dxHashSpace hspace = (dxHashSpace) space;
		setLevels (minlevel,maxlevel);
	}


	//	void dSpaceDestroy (dxSpace *space)
	//	{
	//	  dAASSERT (space);
	//	  dUASSERT (dGeomIsSpace(space),"argument not a space");
	//	  dGeomDestroy (space);
	//	}
	//-> moved to dxSpace
}
