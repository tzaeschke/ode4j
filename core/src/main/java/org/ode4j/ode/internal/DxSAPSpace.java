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

import static org.ode4j.ode.internal.Common.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.ode4j.ode.DAABB;
import org.ode4j.ode.DSapSpace;


/**
 *  Sweep and Prune adaptation/tweaks for ODE by Aras Pranckevicius.
 *  Additional work by David Walters
 *  Original code:
 *		OPCODE - Optimized Collision Detection
 *		Copyright (C) 2001 Pierre Terdiman
 *		Homepage: http://www.codercorner.com/Opcode.htm
 *
 *	This version does complete radix sort, not "classical" SAP. So, we
 *	have no temporal coherence, but are able to handle any movement
 *	velocities equally well.
 *
 *  2009-Apr-29
 *  Implementation has been modified for Java port. It now used merge-sort
 *  provided by Java. See doc of <tt>BoxPruning</tt>.
 *  Tilmann Zaeschke 
 */
public class DxSAPSpace extends DxSpace implements DSapSpace {

	// --------------------------------------------------------------------------
	//  SAP space code
	// --------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// Helpers
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// Implementation Data
	//--------------------------------------------------------------------------

	// We have two lists (arrays of pointers) to dirty and clean
	// geoms. Each geom knows it's index into the corresponding list
	// (see macros above).
	private List<DxGeom> DirtyList = new ArrayList<DxGeom>(); // dirty geoms
	private List<DxGeom> GeomList = new ArrayList<DxGeom>();	// clean geoms

	// For SAP, we ultimately separate "normal" geoms and the ones that have
	// infinite AABBs. No point doing SAP on infinite ones (and it doesn't handle
	// infinite geoms anyway).
	//private dArray<dxGeom> TmpGeomList;	// temporary for normal geoms
	private ArrayList<DxGeom> TmpGeomList = new ArrayList<DxGeom>();	// temporary for normal geoms
	private List<DxGeom> TmpInfGeomList = new ArrayList<DxGeom>();	// temporary for geoms with infinite AABBs

	// Our sorting axes. (X,Z,Y is often best). Stored *2 for minor speedup
	// Axis indices into geom's aabb are: min=idx, max=idx+1
	//	uint32 ax0idx;
	//	uint32 ax1idx;
	//	uint32 ax2idx;
	private int ax0id;
	private int ax1id;
	private int ax2id;

	// pruning position array scratch pad
	// NOTE: this is float not dReal because of the OPCODE radix sorter
	//private dArray< Float > poslist;
	//TZ: TODO remove, not required in this implementation, see BoxPruning(..).
//	private float[] poslist;
//	private RaixSortContext sortContext = new RaixSortContext();
	//};

	/**
	 * Creation.
	 * @param space space
	 * @param axisorder axis order
	 * @return SAPSpace
	 */
	public static DxSAPSpace dSweepAndPruneSpaceCreate( DxSpace space, int axisorder ) {
		return new DxSAPSpace( space, axisorder );
	}


	//==============================================================================

	//#define GEOM_ENABLED(g) (((g)->gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE)
//	private boolean GEOM_ENABLED(dxGeom g) { 
//		return (((g)->gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE); }
	//
	//// HACK: We abuse 'next' and 'tome' members of dxGeom to store indices into dirty/geom lists.
	//#define GEOM_SET_DIRTY_IDX(g,idx) { (g)->next = (dxGeom*)(size_t)(idx); }
	//#define GEOM_SET_GEOM_IDX(g,idx) { (g)->tome = (dxGeom**)(size_t)(idx); }
	//#define GEOM_GET_DIRTY_IDX(g) ((int)(size_t)(g)->next)
	//#define GEOM_GET_GEOM_IDX(g) ((int)(size_t)(g)->tome)
	//#define GEOM_INVALID_IDX (-1)
	//TODO implement
	/**
	 * As an alternative to the above, be implement separate fields in dxGeom.
	 * This should be faster than having a generic class for SAP and QT-Space info.
	 */
	private void GEOM_SET_DIRTY_IDX(DxGeom g,int idx) { g._sapIdxDirtyEx = idx; }
	private void GEOM_SET_GEOM_IDX(DxGeom g,int idx) { g._sapIdxGeomEx = idx; }
	private int GEOM_GET_DIRTY_IDX(DxGeom g) { return g._sapIdxDirtyEx; }
	private int GEOM_GET_GEOM_IDX(DxGeom g) { return g._sapIdxGeomEx; }
	private static final int GEOM_INVALID_IDX = -1;
	
	/**
	 *  A bit of repetitive work - similar to collideAABBs, but doesn't check
	 *  if AABBs intersect (because SAP returns pairs with overlapping AABBs).
	 */
	//static void collideGeomsNoAABBs( dxGeom *g1, dxGeom *g2, void *data, dNearCallback *callback )
	static void collideGeomsNoAABBs( DxGeom g1, DxGeom g2, Object data, DNearCallback callback )
	{
		dIASSERT( !g1.hasFlagAabbBad() );//(g1._gflags & GEOM_AABB_BAD)==0 );
		dIASSERT( !g2.hasFlagAabbBad() );//(g2._gflags & GEOM_AABB_BAD)==0 );

		// no contacts if both geoms on the same body, and the body is not 0
		if (g1.body == g2.body && g1.body!=null) return;

		// test if the category and collide bitfields match
		if ( ((g1.category_bits & g2.collide_bits)!=0 ||
				(g2.category_bits & g1.collide_bits)!=0) == false) {
			return;
		}

		DAABB bounds1 = g1._aabb;
		DAABB bounds2 = g2._aabb;

		// check if either object is able to prove that it doesn't intersect the
		// AABB of the other
		if (g1.AABBTest (g2,bounds2) == false) return;
		if (g2.AABBTest (g1,bounds1) == false) return;

		// the objects might actually intersect - call the space callback function
		callback.call (data,g1,g2);
	}


	//dxSAPSpace::dxSAPSpace( dSpaceID _space, int axisorder ) : dxSpace( _space )
	private DxSAPSpace( DxSpace space, int axisorder )// : dxSpace( _space )
	{
		super(space);
		type = dSweepAndPruneSpaceClass;

		// Init AABB to infinity
		//	_aabb[0] = -dInfinity;
		//	_aabb[1] = dInfinity;
		//	_aabb[2] = -dInfinity;
		//	_aabb[3] = dInfinity;
		//	_aabb[4] = -dInfinity;
		//	_aabb[5] = dInfinity;
		_aabb.setZero();

//		ax0idx = ( ( axisorder ) & 3 ) << 1;
//		ax1idx = ( ( axisorder >> 2 ) & 3 ) << 1;
//		ax2idx = ( ( axisorder >> 4 ) & 3 ) << 1;
		//TZ the new AABB class does not need '<< 1'.
		ax0id = ( ( axisorder ) & 3 );
		ax1id = ( ( axisorder >> 2 ) & 3 );
		ax2id = ( ( axisorder >> 4 ) & 3 );
	}

	//TODO check super class?!?
	//dxSAPSpace::~dxSAPSpace()
	@Override
	public void DESTRUCTOR()
	{
		CHECK_NOT_LOCKED(this);
		if ( cleanup ) {
			// note that destroying each geom will call remove()
			//		for ( ; DirtyList.size()!=0; dGeomDestroy( DirtyList.get( 0 ) ) ) {}
			//		for ( ; GeomList.size()!=0; dGeomDestroy( GeomList.get( 0 ) ) ) {}
			while ( DirtyList.size()!=0 ) DirtyList.get( 0 ).dGeomDestroy( ); 
			while ( GeomList.size()!=0) GeomList.get( 0 ).dGeomDestroy( );

		}
		else {
			// just unhook them
			for ( ; DirtyList.size()!=0; remove( DirtyList.get( 0 ) ) ) {}
			for ( ; GeomList.size()!=0; remove( GeomList.get( 0 ) ) ) {}
		}
		super.DESTRUCTOR();
	}

	//void dxSAPSpace::add( dxGeom* g )
	@Override
	void add( DxGeom g )
	{
		CHECK_NOT_LOCKED (this);
		//dAASSERT(g);
		dUASSERT(g.parent_space == null, "geom is already in a space");

		// add to dirty list
		GEOM_SET_DIRTY_IDX( g, DirtyList.size() );
		GEOM_SET_GEOM_IDX( g, GEOM_INVALID_IDX );
		DirtyList.add( g );

		super.add(g);
	}

	//void dxSAPSpace::remove( dxGeom* g )
	@Override
	void remove( DxGeom g )
	{
		CHECK_NOT_LOCKED(this);
		//dAASSERT(g);
		dUASSERT(g.parent_space == this,"object is not in this space");

		// remove
		int dirtyIdx = GEOM_GET_DIRTY_IDX(g);
		int geomIdx = GEOM_GET_GEOM_IDX(g);
		// must be in one list, not in both
		dUASSERT(
				(dirtyIdx==GEOM_INVALID_IDX && geomIdx>=0 && geomIdx<GeomList.size()) ||
				(geomIdx==GEOM_INVALID_IDX && dirtyIdx>=0 && dirtyIdx<DirtyList.size()),
		"geom indices messed up" );
		if( dirtyIdx != GEOM_INVALID_IDX ) {
			// we're in dirty list, remove
			int dirtySize = DirtyList.size();
			if (dirtyIdx != dirtySize-1) {
				DxGeom lastG = DirtyList.get(dirtySize - 1);
				DirtyList.set(dirtyIdx, lastG);
				GEOM_SET_DIRTY_IDX(lastG, dirtyIdx);
			}
			GEOM_SET_DIRTY_IDX(g,GEOM_INVALID_IDX);
			DirtyList.remove( dirtySize-1 );
		} else {
			// we're in geom list, remove
			int geomSize = GeomList.size();
			if (geomIdx != geomSize-1) {
				DxGeom lastG = GeomList.get(geomSize - 1);
				GeomList.set(geomIdx, lastG);
				GEOM_SET_GEOM_IDX(lastG, geomIdx);
			}
			GEOM_SET_GEOM_IDX(g,GEOM_INVALID_IDX);
			GeomList.remove( geomSize-1 );
		}

		//g->tome_ex = 0;
		g._qtIdxEx = null;
		// dUASSERT((g->next_ex = 0, true), "Needed for an assertion check only");
		g.setNextEx(null);
		dUASSERT(true, "Needed for an assertion check only");

		super.remove(g);
	}

	//void dxSAPSpace::dirty( dxGeom* g )
	@Override
	void dirty( DxGeom g )
	{
		//dAASSERT(g);
		dUASSERT(g.parent_space == this, "object is not in this space");

		// check if already dirtied
		int dirtyIdx = GEOM_GET_DIRTY_IDX(g);
		if( dirtyIdx != GEOM_INVALID_IDX )
			return;

		int geomIdx = GEOM_GET_GEOM_IDX(g);
		dUASSERT( geomIdx>=0 && geomIdx<GeomList.size(), "geom indices messed up" );

		// remove from geom list, place last in place of this
		int geomSize = GeomList.size();
		if (geomIdx != geomSize-1) {
			DxGeom lastG = GeomList.get(geomSize - 1);
			GeomList.set(geomIdx, lastG);
			GEOM_SET_GEOM_IDX(lastG, geomIdx);
		}
		//GeomList.setSize( geomSize-1 );
		GeomList.remove(geomSize-1);

		// add to dirty list
		GEOM_SET_GEOM_IDX( g, GEOM_INVALID_IDX );
		GEOM_SET_DIRTY_IDX( g, DirtyList.size() );
		DirtyList.add( g );
	}

	//void dxSAPSpace::cleanGeoms()
	@Override
	public void cleanGeoms()
	{
		int dirtySize = DirtyList.size();
		if( dirtySize==0 )
			return;

		// compute the AABBs of all dirty geoms, clear the dirty flags,
		// remove from dirty list, place into geom list
		lock_count++;

		int geomSize = GeomList.size();

		for( int i = 0; i < dirtySize; ++i ) {
			DxGeom g = DirtyList.get(i);
			if (g instanceof DxSpace) {//IS_SPACE(g) ) {
				((DxSpace) g).cleanGeoms();
			}

			g.recomputeAABB();
//			dIASSERT((g->gflags & GEOM_AABB_BAD) == 0);
//			g->gflags &= ~GEOM_DIRTY;
			dIASSERT(!g.hasFlagAabbBad());
			g.unsetFlagDirty();

			// remove from dirty list, add to geom list
			GEOM_SET_DIRTY_IDX( g, GEOM_INVALID_IDX );
			GEOM_SET_GEOM_IDX( g, geomSize + i );
			GeomList.add(g);
		}
		// clear dirty list
		DirtyList.clear();

		lock_count--;
	}

	//void dxSAPSpace::collide( void *data, dNearCallback *callback )
	@Override
	public void collide( Object data, DNearCallback callback )
	{
		dAASSERT (callback);

		lock_count++;

		cleanGeoms();

		// by now all geoms are in GeomList, and DirtyList must be empty
		int geom_count = GeomList.size();
		dUASSERT( geom_count == getNumGeoms(), "geom counts messed up" );

		// separate all ENABLED geoms into infinite AABBs and normal AABBs
		TmpGeomList.clear();//setSize(0);
		TmpInfGeomList.clear();
		int axis0max = ax0id;// + 1;
		for( int i = 0; i < geom_count; ++i ) {
			DxGeom g = GeomList.get(i);
			if( !GEOM_ENABLED(g) ) // skip disabled ones
				continue;
			final double amax = g._aabb.getMax(axis0max);
			if( amax == dInfinity ) // HACK? probably not...
				TmpInfGeomList.add( g );
			else
				TmpGeomList.add( g );//push( g );
		}

		// do SAP on normal AABBs
		int tmp_geom_count = TmpGeomList.size();
		if ( tmp_geom_count > 0 )
		{
			// Generate a list of overlapping boxes
			//BoxPruning( tmp_geom_count, (final dxGeom**)TmpGeomList.data(), overlapBoxes );
			BoxPruning( TmpGeomList, data, callback );
		}

		int infSize = TmpInfGeomList.size();
		int m, n;

		for ( m = 0; m < infSize; ++m )
		{
			DxGeom g1 = TmpInfGeomList.get( m );

			// collide infinite ones
			for( n = m+1; n < infSize; ++n ) {
				DxGeom g2 = TmpInfGeomList.get(n);
				collideGeomsNoAABBs( g1, g2, data, callback );
			}

			// collide infinite ones with normal ones
			for( n = 0; n < tmp_geom_count; ++n ) {
				DxGeom g2 = TmpGeomList.get(n);
				collideGeomsNoAABBs( g1, g2, data, callback );
			}
		}

		lock_count--;
	}

	//void dxSAPSpace::collide2( void *data, dxGeom *geom, dNearCallback *callback )
	@Override
	void collide2( Object data, DxGeom geom, DNearCallback callback )
	{
		dAASSERT (geom!=null && callback!=null);

		// TODO: This is just a simple N^2 implementation

		lock_count++;

		cleanGeoms();
		geom.recomputeAABB();

		// intersect bounding boxes
		int geom_count = GeomList.size();
		for ( int i = 0; i < geom_count; ++i ) {
			DxGeom g = GeomList.get(i);
			if ( GEOM_ENABLED(g) )
				collideAABBs (g,geom,data,callback);
		}

		lock_count--;
	}

	
	private class GeomComparator implements Comparator<DxGeom> {
		@Override
		public int compare(DxGeom arg0, DxGeom arg1) {
			double a0 = arg0._aabb.getMin(ax0id);
			double a1 = arg1._aabb.getMin(ax0id);
			return a1 > a0 ? -1 : (a1 < a0 ? 1 : 0);
		}
	}
	
	/**
	 *	Complete box pruning.
	 *  Returns a list of overlapping pairs of boxes, each box of the pair
	 *  belongs to the same set.
	 *  
	 *  TZ: This is a new implementation for Java. It uses the Java internal 
	 *  merge-sort instead of a custom radix-sort. The first is between 
	 *  O(N) and O(N*log(N)), the second is always O(N*log(N)), so not much lost.
	 *  This greatly simplifies the code.
	 *
	 *	@param	geoms	[in] geoms of boxes.
	 */
	//void dxSAPSpace::BoxPruning( int count, const dxGeom** geoms, dArray< Pair >& pairs )
	void BoxPruning(final List<DxGeom> geoms, Object data, DNearCallback callback)
	{
		// 1) Build main list using the primary axis
		//  NOTE: uses floats instead of dReals because that's what radix sort wants
		//TZ: not required in this implementation

		// 2) Sort the list
		List<DxGeom> buffer = geoms;
		Collections.sort(buffer, new GeomComparator());
		int size = buffer.size();
		// 3) Prune the list
		for (int i = 0; i < size; i++) {
			DxGeom g0 = buffer.get(i);
			DAABB aabb0 = g0._aabb;
			final double idx0ax0max = aabb0.getMax(ax0id);//(ax0idx+1);
			for (int j = i+1; j < size; j++) {
				DxGeom g1 = buffer.get(j);
				if (g1._aabb.getMin(ax0id) > idx0ax0max) {
					//This and following elements can not intersect with g1.
					break;
				}
//				if ( aabb0.get(ax1idx+1) >= g1._aabb.get(ax1idx)) 
//					if (g1._aabb.get(ax1idx+1) >= aabb0.get(ax1idx) )
//						if ( aabb0.get(ax2idx+1) >= g1._aabb.get(ax2idx))
//							if (g1._aabb.get(ax2idx+1) >= aabb0.get(ax2idx) )
				if ( aabb0.getMax(ax1id) >= g1._aabb.getMin(ax1id)) 
					if (g1._aabb.getMax(ax1id) >= aabb0.getMin(ax1id) )
						if ( aabb0.getMax(ax2id) >= g1._aabb.getMin(ax2id))
							if (g1._aabb.getMax(ax2id) >= aabb0.getMin(ax2id) )
								collideGeomsNoAABBs(g0, g1, data, callback);
			}
		}
	}

	//Commented out, see above (TZ)
//	/**
//	 *	Complete box pruning.
//	 *  Returns a list of overlapping pairs of boxes, each box of the pair
//	 *  belongs to the same set.
//	 *
//	 *	@param	count	[in] number of boxes.
//	 *	@param	geoms	[in] geoms of boxes.
//	 *	@param	pairs	[out] array of overlapping pairs.
//	 */
//	//void dxSAPSpace::BoxPruning( int count, const dxGeom** geoms, dArray< Pair >& pairs )
//	void BoxPruning( int count, final ArrayList<dxGeom> geoms, ArrayList< Pair > pairs )
//	{
//      TZ: I removed this because it is also outdated
//	}


	//==============================================================================

	//------------------------------------------------------------------------------
	// Radix Sort
	//------------------------------------------------------------------------------

	// --------------------------------------------------------------------------
	//  Radix Sort Context
	// --------------------------------------------------------------------------

	//TZ removed. May be added in future again.
}