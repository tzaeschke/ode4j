/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2013 Tilmann Zaeschke     *
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
		dxAABB next;		// next in the list of all AABBs
		int level;		// the level this is stored in (cell size = 2^level)
		//TODO do not initialise?
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
	};


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

	//	static unsigned long getVirtualAddress (int level, int x, int y, int z)
	private static int getVirtualAddress (int level, int x, int y, int z)
	{
		int r = level*1000 + x*100 + y*10 + z;
		//TODO remove check ?!?
		if (r>=Integer.MAX_VALUE) {
		    throw new IllegalArgumentException("level: " + level + " x=" + x + 
		            " y=" + y + " z="+ z);
		}
		return Math.abs(r);
		//return level*1000 + x*100 + y*10 + z;
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
		//for (dxGeom g=_first; (g!= null) && (g.gflags & GEOM_DIRTY) != 0;
		//g=g.getNext()) {
		for (DxGeom g: _geoms) {
			//if ((g._gflags & GEOM_DIRTY)==0) break;
			if (!g.hasFlagDirty()) break;
			if (g instanceof DxSpace) {
				((DxSpace)g).cleanGeoms();
			}
			g.recomputeAABB();
			//g._gflags &= (~(GEOM_DIRTY|GEOM_AABB_BAD));
			g.unsetFlagDirtyAndBad();
		}
		lock_count--;
	}

	@Override
	public void collide (Object data, DNearCallback callback)
	{
		dAASSERT(this, callback);
		//dxGeom geom; //*
		dxAABB aabb; //*
		int i,maxlevel;

		// 0 or 1 geoms can't collide with anything
		if (count < 2) return;

		lock_count++;
		cleanGeoms();

		// create a list of auxiliary information for all geom axis aligned bounding
		// boxes. set the level for all AABBs. put AABBs larger than the space's
		// global_maxlevel in the big_boxes list, check everything else against
		// that list at the end. for AABBs that are not too big, record the maximum
		// level that we need.

		int n = 0;			// number of AABBs in main list
		dxAABB first_aabb = null;	// list of AABBs in hash table
		dxAABB big_boxes = null;	// list of AABBs too big for hash table
		maxlevel = global_minlevel - 1;
		//for (geom = _first; geom != null; geom=geom.getNext()) {
		for (DxGeom geom: _geoms) {
			if (!GEOM_ENABLED(geom)){
				continue;
			}
			dxAABB aabbP = new dxAABB(); //(dxAABB) ALLOCA (sizeof(dxAABB));
			aabbP.geom = geom;
			// compute level, but prevent cells from getting too small
			int level = findLevel (geom._aabb);
			if (level < global_minlevel) level = global_minlevel;
			if (level <= global_maxlevel) {
				// aabb goes in main list
				aabbP.next = first_aabb;
				first_aabb = aabbP;
				aabbP.level = level;
				if (level > maxlevel) maxlevel = level;
				// cellsize = 2^level
				double cellsize = ldexp (1.0,level);
				// discretize AABB position to cell size
				for (i=0; i < 3; i++) {
					aabbP.dbounds[2*i] = (int)Math.floor (geom._aabb.getMin(i)/cellsize);
					aabbP.dbounds[2*i+1] = (int)Math.floor (geom._aabb.getMax(i)/cellsize);
				}
				// set AABB index
				aabbP.index = n;
				n++;
			}
			else {
				// aabb is too big, put it in the big_boxes list. we don't care about
				// setting level, dbounds, index, or the maxlevel
				aabbP.next = big_boxes;
				big_boxes = aabbP;
			}
		}

		// for `n' objects, an n*n array of bits is used to record if those objects
		// have been intersection-tested against each other yet. this array can
		// grow large with high n, but oh well...
		int tested_rowsize = (n+7) >> 3;	// number of bytes needed for n bits
		// unsigned char *tested = (unsigned char *) ALLOCA (n * tested_rowsize);
		//TODO correct?
		int[] tested = new int[n*tested_rowsize];
		//memset (tested,0,n * tested_rowsize);

		// create a hash table to store all AABBs. each AABB may take up to 8 cells.
		// we use chaining to resolve collisions, but we use a relatively large table
		// to reduce the chance of collisions.

		// compute hash table size sz to be a prime > 8*n
		for (i=0; i<NUM_PRIMES; i++) {
			if (prime[i] >= (8*n)) break;
		}
		if (i >= NUM_PRIMES) i = NUM_PRIMES-1;	// probably pointless
		int sz = prime[i];

		// allocate and initialize hash table node pointers
		//Node **table = (Node **) ALLOCA (sizeof(Node*) * sz);
		//TODO correct ??
		Node[] table = new Node[sz];
		//for (i=0; i<sz; i++) table[i] = null;

		// add each AABB to the hash table (may need to add it to up to 8 cells)
		//TODO use proper List here!
		for (aabb=first_aabb; aabb != null; aabb=aabb.next) {
			int[] dbounds = aabb.dbounds;
			for (int xi = dbounds[0]; xi <= dbounds[1]; xi++) {
				for (int yi = dbounds[2]; yi <= dbounds[3]; yi++) {
					for (int zi = dbounds[4]; zi <= dbounds[5]; zi++) {
						// get the hash index  TODO TZ: cast to int
						int hi = getVirtualAddress (aabb.level,xi,yi,zi) % sz;
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
		for (aabb=first_aabb; aabb != null; aabb=aabb.next) {
			// we are searching for collisions with aabb
			for (i=0; i<6; i++) db[i] = aabb.dbounds[i];
			for (int level = aabb.level; level <= maxlevel; level++) {
				for (int xi = db[0]; xi <= db[1]; xi++) {
					for (int yi = db[2]; yi <= db[3]; yi++) {
						for (int zi = db[4]; zi <= db[5]; zi++) {
							// get the hash index
							//int TZ long 
							int hi = getVirtualAddress (level,xi,yi,zi) % sz;
							// search all nodes at this index
							Node node;
							for (node = table[hi]; node != null; node=node.next) {
								// node points to an AABB that may intersect aabb
								if (node.aabb == aabb) continue;
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
										collideAABBs (aabb.geom,node.aabb.geom,data,callback);
									}
									tested[i] |= mask;
								}
							}
						}
					}
				}
				// get the discrete bounds for the next level up
				for (i=0; i<6; i++) db[i] >>= 1;
			}
		}

		// every AABB in the normal list must now be intersected against every
		// AABB in the big_boxes list. so let's hope there are not too many objects
		// in the big_boxes list.
		for (aabb=first_aabb; aabb != null; aabb=aabb.next) {
			for (dxAABB aabb2=big_boxes; aabb2 != null; aabb2=aabb2.next) {
				collideAABBs (aabb.geom,aabb2.geom,data,callback);
			}
		}

		// intersected all AABBs in the big_boxes list together
		for (aabb=big_boxes; aabb != null; aabb=aabb.next) {
			for (dxAABB aabb2=aabb.next; aabb2 != null; aabb2=aabb2.next) {
				collideAABBs (aabb.geom,aabb2.geom,data,callback);
			}
		}

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
//		for (dxGeom g=_first; g != null; g=g.getNext()) {
		for (DxGeom g: _geoms) {
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

	//TODO change to (dxHashSpace) space
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
