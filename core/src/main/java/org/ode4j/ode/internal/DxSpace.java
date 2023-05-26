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

import java.util.Iterator;

import org.ode4j.ode.DAABB;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;

import static org.ode4j.ode.internal.Common.*;

/**
 * ****************************************************************************
 * the base space class
 * 
 * the contained geoms are divided into two kinds: clean and dirty.
 * the clean geoms have not moved since they were put in the list,
 * and their AABBs are valid. the dirty geoms have changed position, and
 * their AABBs are may not be valid. the two types are distinguished by the
 * GEOM_DIRTY flag. all dirty geoms come *before* all clean geoms in the list.
 */ 
public abstract class DxSpace extends DxGeom implements DSpace {

	static final int dSPACE_TLS_KIND_INIT_VALUE;
	private static final int dSPACE_TLS_KIND_MANUAL_VALUE;
	static {
//		if (dTLS_ENABLED) {
//			throw new UnsupportedOperationException();
//			dSPACE_TLS_KIND_INIT_VALUE = OTK__DEFAULT;
//			dSPACE_TLS_KIND_MANUAL_VALUE = OTK_MANUALCLEANUP;
//		} else {
			dSPACE_TLS_KIND_INIT_VALUE = 0;
			dSPACE_TLS_KIND_MANUAL_VALUE = 0;
//		}
	}
//	#define dSPACE_TLS_KIND_MANUAL_VALUE 
//	#if dTLS_ENABLED
//	#define dSPACE_TLS_KIND_INIT_VALUE OTK__DEFAULT
//	#define dSPACE_TLS_KIND_MANUAL_VALUE OTK_MANUALCLEANUP
//	#else
//	#define dSPACE_TLS_KIND_INIT_VALUE 0
//	#define dSPACE_TLS_KIND_MANUAL_VALUE 0
//	#endif

	protected int count;			// number of geoms in this space
	//	  dxGeom *first;		// first geom in list
	//	dxGeom first;		// first geom in list
	//	protected final Ref<dxGeom> first = new Ref<dxGeom>();		// first geom in list
	protected DxGeom _first = null;		// first geom in list
	boolean cleanup;			// cleanup mode, 1=destroy geoms on exit
	int sublevel;         // space sublevel (used in dSpaceCollide2). NOT TRACKED AUTOMATICALLY!!!
	//unsigned
	int tls_kind;	// space TLS kind to be used for global caches retrieval

	// cached state for getGeom()
	int current_index;		// only valid if current_geom != 0
	DxGeom current_geom;		// if 0 then there is no information

	// locking stuff. the space is locked when it is currently traversing its
	// internal data structures, e.g. in collide() and collide2(). operations
	// that modify the contents of the space are not permitted when the space
	// is locked.
	int lock_count;

	/**
	 * Turn all dirty geoms into clean geoms by computing their AABBs and any
	 * other space data structures that are required. this should clear the
	 * GEOM_DIRTY and GEOM_AABB_BAD flags of all geoms.
	 */
	@Override
	public abstract void cleanGeoms();

	/** This is equivalent to OdeHelper.spaceCollide(...) */
	@Override
	public abstract void collide (Object data, DNearCallback callback);
	abstract void collide2 (Object data, DxGeom geom, DNearCallback callback);


	public void dSpaceDestroy ()
	{
		dGeomDestroy ();
	}

	public void dSpaceSetCleanup (boolean mode)
	{
		setCleanup (mode);
	}


	boolean dSpaceGetCleanup ()
	{
		return getCleanup();
	}


//	private void dSpaceSetSublevel (int sublevel)
//	{
//		setSublevel (sublevel);
//	}
//
//
//	private int dSpaceGetSublevel ()
//	{
//		return getSublevel();
//	}
//
//
//	private void dSpaceSetManualCleanup (int mode)
//	{
//		setManualCleanup(mode);
//	}
//
//	private int dSpaceGetManualCleanup ()
//	{
//		return getManualCleanup();
//	}


	public void dSpaceAdd (DxGeom g)
	{
		CHECK_NOT_LOCKED ();
		add (g);
	}


	void dSpaceRemove (DxGeom g)
	{
		CHECK_NOT_LOCKED ();
		remove (g);
	}


	boolean dSpaceQuery (DxGeom g)
	{
		return query (g);
	}

	void dSpaceClean (){
		cleanGeoms();
	}

	public int dSpaceGetNumGeoms ()
	{
		return getNumGeoms();
	}

	/** 
	 * @param i id
	 * @return geometry object
	 * @deprecated 2016-01-17 / TODO remove in 0.6.0
	 */
	@Deprecated
	public DxGeom dSpaceGetGeom (int i)
	{
		return (DxGeom) getGeom (i);
	}

	int dSpaceGetClass ()
	{
		return type;
	}


//	public void dSpaceCollide (dxSpace space, Object[] data, dNearCallback callback)
	public void dSpaceCollide (Object data, DNearCallback callback)
	{
		//dAASSERT (space, callback);
		dAASSERT (callback);
		//dUASSERT (dGeomIsSpace(),"argument not a space");
		collide (data,callback);
	}


	private static class DataCallback {
		Object data;
		DNearCallback callback;
		public DataCallback(Object data, DNearCallback callback) {
			this.data = data;
			this.callback = callback;
		}
	}

	// Invokes the callback with arguments swapped
	static void swap_callback(Object data, DGeom g1, DGeom g2)
	{
		DataCallback dc = (DataCallback)data;
		dc.callback.call(dc.data, g2, g1);
	}


	public static void dSpaceCollide2 (DxGeom g1, DxGeom g2, Object data,
			DNearCallback callback)
	{
		dAASSERT (callback);
		DxSpace s1,s2;

		// see if either geom is a space
		if (g1 instanceof DxSpace) s1 = (DxSpace) g1; else s1 = null;
		if (g2 instanceof DxSpace) s2 = (DxSpace) g2; else s2 = null;

		if (s1 != null && s2 != null) {
			int l1 = s1.getSublevel();
			int l2 = s2.getSublevel();
			if (l1 != l2) {
				if (l1 > l2) {
					s2 = null;
				} else {
					s1 = null;
				}
			}
		}

		// handle the four space/geom cases
		if (s1 != null) {
			if (s2 != null) {
				// g1 and g2 are spaces.
				if (s1==s2) {
					// collide a space with itself -. interior collision
					s1.collide (data,callback);
				}
				else {
					// iterate through the space that has the fewest geoms, calling
					// collide2 in the other space for each one.
					if (s1.count < s2.count) {
						DataCallback dc = new DataCallback(data, callback);
						//for (dxGeom g = s1._first; g != null; g=g.getNext()) {
						for (DxGeom g: s1.getGeomsDx()) {
							s2.collide2 (dc,g, DxSpace::swap_callback);
						}
					}
					else {
						//for (dxGeom g = s2._first; g != null; g=g.getNext()) {
						for (DxGeom g: s2.getGeomsDx()) {
							s1.collide2 (data,g,callback);
						}
					}
				}
			}
			else {
				// g1 is a space, g2 is a geom
				s1.collide2 (data,g2,callback);
			}
		}
		else {
			if (s2 != null) {
				// g1 is a geom, g2 is a space
				DataCallback dc = new DataCallback(data, callback);
				s2.collide2 (dc,g1, DxSpace::swap_callback
				);
			}
			else {
				// g1 and g2 are geoms
				// make sure they have valid AABBs
				g1.recomputeAABB();
				g2.recomputeAABB();
				collideAABBs(g1,g2, data, callback);
			}
		}
	}

	//****************************************************************************
	// dxSpace

	DxSpace (DxSpace _space)// : dxGeom (_space,0)
	{
		super(_space, false);
		count = 0;
		//first.set) = null;
		cleanup = true;
		sublevel = 0;
		tls_kind = dSPACE_TLS_KIND_INIT_VALUE;
		current_index = 0;
		current_geom = null;
		lock_count = 0;
	}


		//				dxSpace::~dxSpace()
	@Override
	public void DESTRUCTOR() {
		CHECK_NOT_LOCKED ();
		if (cleanup) {
			// note that destroying each geom will call remove()
			DxGeom g,n;
			for (g = _first; g != null; g=n) {
				n = g.getNext();
				g.dGeomDestroy ();
			}
		}
		else {
			DxGeom g,n;
			for (g = _first; g != null; g=n) {
				n = g.getNext();
				remove (g);
			}
		}
		super.DESTRUCTOR();
	}


	@Override
    protected void computeAABB()
	{
		if (_first != null) {
//			int i;
//			double[] a = new double[6];
			DAABB aabb = new DAABB();
			aabb.set( dInfinity, -dInfinity,
					dInfinity, -dInfinity,
					dInfinity, -dInfinity);
//			a[0] = dInfinity;
//			a[1] = -dInfinity;
//			a[2] = dInfinity;
//			a[3] = -dInfinity;
//			a[4] = dInfinity;
//			a[5] = -dInfinity;
			for (DxGeom g=_first; g != null; g=g.getNext()) {
				g.recomputeAABB();
//				for (i=0; i<6; i += 2) if (g._aabb.get(i) < a[i]) a[i] = g._aabb.get(i);
//				for (i=1; i<6; i += 2) if (g._aabb.get(i) > a[i]) a[i] = g._aabb.get(i);
				aabb.expand(g.getAABB());
			}
			//memcpy(aabb,a,6*sizeof(double));
			_aabb.set(aabb);
		}
		else {
			_aabb.setZero();
		}
	}


	@Override
	public void setCleanup (boolean mode)
	{
		cleanup = mode;//(mode != 0);
	}


	@Override
	public boolean getCleanup()
	{
		return cleanup;
	}


	@Override
	public void setSublevel(int value)
	{
		sublevel = value;
	}


	@Override
	public int getSublevel() 
	{
		return sublevel;
	}

	
	@Override
	public void setManualCleanup(int value) { 
		tls_kind = (value != 0 ? dSPACE_TLS_KIND_MANUAL_VALUE : dSPACE_TLS_KIND_INIT_VALUE); 
	}
	
	
	@Override
	public int getManualCleanup() { 
		return (tls_kind == dSPACE_TLS_KIND_MANUAL_VALUE) ? 1 : 0; 
	}

	  
	boolean query (DxGeom geom)
	{
//		dAASSERT (geom);
		return (geom.parent_space == this);
	}


	@Override
	public int getNumGeoms()
	{
		return count;
	}


	// the dirty geoms are numbered 0..k, the clean geoms are numbered k+1..count-1

	/** @deprecated 2016-01-17  - to be removed in 0.6.0 */
	@Deprecated
	@Override
	public DGeom getGeom (int i)
	{
		dUASSERT (i >= 0 && i < count,"index out of range");
		if (current_geom!=null && current_index == i-1) {
			current_geom = current_geom.getNext();
			current_index = i;
			return current_geom;
		}
		else {
			DxGeom g=_first;
			for (int j=0; j<i; j++) {
				if (g != null) g = g.getNext(); else return null;
			}
			current_geom = g;
			current_index = i;
			return g;
		}
	}


	void add (DxGeom geom)
	{
		CHECK_NOT_LOCKED ();
		//dAASSERT (geom);
		dUASSERT(this != geom, "Cannot add space to itself" );
		dUASSERT (geom.parent_space == null && geom.getNext() == null,
				"geom is already in a space");

		// add
		geom.parent_space = this;
		geom.spaceAdd (_first, this);
		count++;

		// enumerator has been invalidated
		current_geom = null;

		dGeomMoved ();
	}

	void remove (DxGeom geom)
	{
		CHECK_NOT_LOCKED ();
		//dAASSERT (geom);
		dUASSERT(this != geom, "Cannot remove space from itself" );
		dUASSERT (geom.parent_space == this,"object is not in this space");

		// remove
		geom.spaceRemove(this);
		count--;

		// safeguard
		//geom._next = null;
		//geom._tome = null;
		geom.setNextPrevNull();
		geom.parent_space = null;

		// enumerator has been invalidated
		current_geom = null;

		// the bounding box of this space (and that of all the parents) may have
		// changed as a consequence of the removal.
		dGeomMoved ();
	}


	void dirty (DxGeom geom)
	{
		geom.spaceRemove(this);
		geom.spaceAdd (_first, this);
	}

    public void CHECK_NOT_LOCKED() {
        dUASSERT (lock_count==0, "invalid operation for locked space");
    }

    public static void CHECK_NOT_LOCKED(DxSpace s) {
        dUASSERT (s == null || s.lock_count==0, "invalid operation for locked space");
    }

	public void setFirst(DxGeom dxGeom) {
		_first = dxGeom;
	}
	
	// *********************************************
	// dSpace API
	// *********************************************
 
//	public void setCleanup (boolean mode)
//	{ dSpaceSetCleanup (mode); }
//	public int getCleanup()
//	{ return dSpaceGetCleanup (id()); }

	@Override
	public void add (DGeom x)
	{ dSpaceAdd ((DxGeom) x); }
	@Override
	public void remove (DGeom x)
	{ dSpaceRemove ((DxGeom) x); }
	@Override
	public boolean query (DGeom x)
	{ return dSpaceQuery ((DxGeom) x); }

//	public int getNumGeoms()
//	{ return dSpaceGetNumGeoms (id()); }
//	public dGeom getGeom (int i)
//	{ return dSpaceGetGeom (id(),i); }

	public Iterable<DxGeom> getGeomsDx() {
		return () -> new Iterator<DxGeom>() {

			private DxGeom current = _first;

			@Override
			public boolean hasNext() {
				return current != null;
			}

			@Override
			public DxGeom next() {
				DxGeom g = current;
				current = current.getNext();
				return g;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("remove");
			}
		};
	}

	@Override
	public Iterable<DGeom> getGeoms() {
		return () -> new Iterator<DGeom>() {

			private DxGeom current = _first;

			@Override
			public boolean hasNext() {
				return current != null;
			}

			@Override
			public DGeom next() {
				DxGeom g = current;
				current = current.getNext();
				return g;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("remove");
			}
		};
	}
}
