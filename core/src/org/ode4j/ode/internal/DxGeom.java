/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import java.util.LinkedList;
import java.util.List;


import static org.ode4j.ode.OdeMath.*;

import org.ode4j.ode.DColliderFn;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector6;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.internal.DBase;
import org.ode4j.ode.internal.DxSpace;
import org.ode4j.ode.internal.Objects_H.dxPosR;
import org.ode4j.ode.internal.DxQuadTreeSpace.Block;

/**
 * geometry (collision object).
 *
 * From collision_kernel.cpp.
 */
public abstract class DxGeom extends DBase implements DGeom {


	//****************************************************************************
	// constants and macros

	// mask for the number-of-contacts field in the dCollide() flags parameter
	//#define NUMC_MASK (0xffff)
	public static final int NUMC_MASK = 0xffff;  //TODO long?

	//	#define IS_SPACE(geom) \
	//	  ((geom)->type >= dFirstSpaceClass && (geom)->type <= dLastSpaceClass)
//	protected static boolean IS_SPACE(dxGeom geom) {
//		//return ((geom).type >= dFirstSpaceClass && (geom).type <= dLastSpaceClass);
//		return (geom instanceof dSpace);
//	}

	// should not be necessary, is also expensive to create a new view.
//	protected dContactGeomBuffer CONTACT(dContactGeomBuffer b, int skip) {
//		return b.createView(skip);
//	}
	
	//****************************************************************************
	// geometry object base class


	// geom flags.
	//
	// GEOM_DIRTY means that the space data structures for this geom are
	// potentially not up to date. NOTE THAT all space parents of a dirty geom
	// are themselves dirty. this is an invariant that must be enforced.
	//
	// GEOM_AABB_BAD means that the cached AABB for this geom is not up to date.
	// note that GEOM_DIRTY does not imply GEOM_AABB_BAD, as the geom might
	// recalculate its own AABB but does not know how to update the space data
	// structures for the space it is in. but GEOM_AABB_BAD implies GEOM_DIRTY.
	// the valid combinations are: 
	//			0
	//			GEOM_DIRTY
	//			GEOM_DIRTY|GEOM_AABB_BAD
	//			GEOM_DIRTY|GEOM_AABB_BAD|GEOM_POSR_BAD

	//	enum E {
	//	  GEOM_DIRTY	(1),    // geom is 'dirty', i.e. position unknown
	//	  GEOM_POSR_BAD  (2),    // geom's final posr is not valid
	//	  GEOM_AABB_BAD	 (4),    // geom's AABB is not valid
	//	  GEOM_PLACEABLE  (8),   // geom is placeable
	//	  GEOM_ENABLED  (16),    // geom is enabled
	//	  GEOM_ZERO_SIZED (32), // geom is zero sized
	//
	//	  GEOM_ENABLE_TEST_MASK ( GEOM_ENABLED | GEOM_ZERO_SIZED),
	//	  GEOM_ENABLE_TEST_VALUE ( GEOM_ENABLED),
	//
	//	  // Ray specific
	//	  RAY_FIRSTCONTACT ( 0x10000),
	//	  RAY_BACKFACECULL ( 0x20000),
	//	  RAY_CLOSEST_HIT  ( 0x40000);
	//	  
	//	  private final int _i;
	//	  E(int i) {
	//		  _i = i;
	//	  }
	//	};
	protected final static int GEOM_DIRTY	= 1;    // geom is 'dirty', i.e. position unknown
	protected final static int GEOM_POSR_BAD = 2;    // geom's final posr is not valid
	protected final static int GEOM_AABB_BAD	= 4;    // geom's AABB is not valid
	protected final static int GEOM_PLACEABLE = 8;   // geom is placeable
	protected final static int GEOM_ENABLED = 16;    // geom is enabled
	protected final static int GEOM_ZERO_SIZED = 32; // geom is zero sized

	protected final static int GEOM_ENABLE_TEST_MASK = GEOM_ENABLED | GEOM_ZERO_SIZED;
	protected final static int GEOM_ENABLE_TEST_VALUE = GEOM_ENABLED;


	// geometry object base class. pos and R will either point to a separately
	// allocated buffer (if body is 0 - pos points to the dxPosR object) or to
	// the pos and R of the body (if body nonzero).
	// a dGeom is a pointer to this object.

	//	struct dxGeom : public dBase {
	public int type;		// geom type number, set by subclass constructor
	int _gflags;		// flags used by geom and space
	Object _data;		// user-defined data pointer
	//	  dBody body;		// dynamics body associated with this object (if any)
	DxBody body;		// dynamics body associated with this object (if any)
	DxGeom body_next;	// next geom in body's linked list of associated geoms
	dxPosR _final_posr;	// final position of the geom in world coordinates
	dxPosR offset_posr;	// offset from body in local coordinates

	// information used by spaces
	//TODO use linked list, seems simpler.
	private List<DxGeom> _list = new LinkedList<DxGeom>();
	//dxGeom next;		// next geom in linked list of geoms
//	private final Ref<dxGeom> next = new Ref<dxGeom>();
	/** 'tome' is pointer to a pointer to (this).
	 * Usually it is a pointer to the (prev.next) field.
	 * For the first element, it is a pointer to (container.first).
	 */
//	private final Ref<dxGeom> tome = new Ref<dxGeom>();
	DxGeom _next = null;
	private DxGeom _prev = null;
	//dxGeom tome;	// linked list backpointer
	DxSpace parent_space;// the space this geom is contained in, 0 if none
	int _sapIdxDirty; // TZ: Used by SAP-Space.
	int _sapIdxGeom; // TZ: Used by SAP-Space.
	Block _qtIdx; // TZ: Used by QuadTree-Space.
	
	//double[] aabb = new double[6];	// cached AABB for this space
	DAABB _aabb = new DAABB();	// cached AABB for this space
	//TODO unsigned
	long category_bits,collide_bits;

	//	  dxGeom (dSpace _space, int is_placeable);
	//	  virtual ~dxGeom();

	// Set or clear GEOM_ZERO_SIZED flag
	void updateZeroSizedFlag(boolean is_zero_sized) { 
		_gflags = is_zero_sized ? (_gflags | GEOM_ZERO_SIZED) : (_gflags & ~GEOM_ZERO_SIZED); 
	}

	// calculate our new final position from our offset and body
	//	  void computePosr();

	// recalculate our new final position if needed
	void recomputePosr()
	{
		if ((_gflags & GEOM_POSR_BAD) != 0) {
			computePosr();
			_gflags &= ~GEOM_POSR_BAD;
		}
	}

	/**
	 * compute the AABB for this object and put it in aabb. this function
	 * always performs a fresh computation, it does not inspect the
	 * GEOM_AABB_BAD flag.
	 */
	abstract void computeAABB();


	// utility functions

	// compute the AABB only if it is not current. this function manipulates
	// the GEOM_AABB_BAD flag.

	void recomputeAABB() {
		if ((_gflags & GEOM_AABB_BAD) != 0) {
			// our aabb functions assume final_posr is up to date
			recomputePosr(); 
			computeAABB();
			_gflags &= ~GEOM_AABB_BAD;
		}
	}

	// add and remove this geom from a linked list maintained by a space.

//	void spaceAdd (dxGeom **first_ptr) {
//	void spaceAdd (Ref<dxGeom> first_ptr) {
////		next = *first_ptr;
////		tome = first_ptr;
////		if (*first_ptr) (*first_ptr).tome = &next;
////		*first_ptr = this;
//
//		next.set(first_ptr.get());
//		tome = first_ptr;
//		if (first_ptr.get() != null) first_ptr.get().tome.set(next);
//		first_ptr.set(this);
//		//System.out.println("Added:" + first_p)
//	}
//	void spaceRemove() {
////		   if (next) next->tome = tome;
////		    *tome = next;
//		if (next != null) next.tome = tome;
//		tome.set(next);
//	}

	void spaceAdd (DxGeom next, DxSpace parent, List<DxGeom> geoms) {
//		next = *first_ptr;
//		tome = first_ptr;
//		if (*first_ptr) (*first_ptr).tome = &next;
//		*first_ptr = this;

		//TZ:
//		if (parent_space != null) {
//			throw new IllegalStateException();
//		}
		
		_next = next;
		if (_next != null) {
			_next._prev = this;
		}
		
		_prev = null;
		parent.setFirst(this);
//		parent_space = parent;
		
		geoms.add(0, this);
	}

	void spaceRemove(DxSpace parent, List<DxGeom> geoms) {
//		   if (next) next->tome = tome;
//		    *tome = next;
//		if (next != null) next.tome = tome;
//		tome.set(next);
		if (_next != null) {
			_next._prev = _prev;
		}
		parent.setFirst(_next);
//		parent_space.
		
		//TODO use HashSet or IdentitySet or ArrayList? Check call hierarchy for type of usage!
		geoms.remove(this);
	}

	
	/**
	 * @return next dxGeom.
	 * @author Tilmann Zaeschke
	 */
	final DxGeom getNext() {
		return _next;
	}

	// add and remove this geom from a linked list maintained by a body.

	private void bodyAdd (DxBody b) {
		body = b;
		body_next = b.geom;
		b.geom = this;
	}


	//	dxGeom::dxGeom (dSpace _space, int is_placeable)
	protected DxGeom (DxSpace space, boolean isPlaceable)
	{
		// setup body vars. invalid type of -1 must be changed by the constructor.
		type = -1;
		_gflags = GEOM_DIRTY | GEOM_AABB_BAD | GEOM_ENABLED;
		if (isPlaceable) _gflags |= GEOM_PLACEABLE;
		_data = null;
		body = null;
		body_next = null;
		if (isPlaceable) {
			_final_posr = dAllocPosr();
			_final_posr.pos.setValues(0);//dSetZero (_final_posr.pos.v,4);
			dRSetIdentity (_final_posr.R);
		}
		else {
			_final_posr = null;
		}
		offset_posr = null;

		// setup space vars
		//TODO remove.
		_next = null;
		_prev = null;//tome = null;
		parent_space = null;
		_aabb.setValues(0);//dSetZero (_aabb.v,6);
		category_bits = ~0;
		collide_bits = ~0;

		// put this geom in a space if required
		if (space != null) space.dSpaceAdd (this);
	}


	//	dxGeom::~dxGeom()
	public void DESTRUCTOR() {
		if (parent_space != null) parent_space.dSpaceRemove (this);
		//	if ((gflags & GEOM_PLACEABLE) && (!body || (body && offset_posr)))
		if (((_gflags & GEOM_PLACEABLE)!=0) && (body==null || (body!= null && offset_posr!=null)))
			dFreePosr(_final_posr);
		if (offset_posr != null) dFreePosr(offset_posr);
		bodyRemove();
	}


	/**
	 * test whether the given AABB object intersects with this object, return
	 * 1=yes, 0=no. this is used as an early-exit test in the space collision
	 * functions. the default implementation returns 1, which is the correct
	 * behavior if no more detailed implementation can be provided.
	 * 
	 * @param o
	 * @return
	 */
	//	  abstract int AABBTest (dxGeom o, dReal aabb[6]);
	boolean AABBTest (DxGeom o, DVector6 aabb)
	{
		return true;
	}

	/**
	 * 
	 */
	private void bodyRemove()
	{
		if (body != null) {
			// delete this geom from body list
		    //dxGeom **last = &body->geom, *g = body->geom;
			DxGeom g = body.geom;
			while (g != null) {
				if (g == this) {
					body.geom = g.body_next;//last = g.body_next;
					break;
				}
				body.geom = g.body_next; //last = g.body_next;
				g = g.body_next;
			}
			body = null;
			body_next = null;
		}
	}

	//	private void myswap(dReal& a, dReal& b) { 
//	private void myswap(double a, double b) { 
//		double t=b; b=a; a=t;
//		throw new UnsupportedOperationException("TODO make it work!");
//	}


	private void matrixInvert(final DMatrix3C inMat, DMatrix3 outMat)
	{
//		memcpy(outMat, inMat, sizeof(dMatrix3));
		outMat.set(inMat);
		//TZ unused: dMatrix3 dummy = new dMatrix3();
		double x;
		// swap _12 and _21
		//myswap(outMat[0 + 4*1], outMat[1 + 4*0]);
		x = outMat.get10();//v[0 + 4*1]; 
		outMat.set10( outMat.get01() );//v[0 + 4*1] = outMat.v[1 + 4*0]; 
		outMat.set01( x );//v[1 + 4*0] = x;
		// swap _31 and _13
		//myswap(outMat[2 + 4*0], outMat[0 + 4*2]);
		//x = outMat.v[2 + 4*0]; outMat.v[2 + 4*0] = outMat.v[0 + 4*2]; outMat.v[0 + 4*2] = x;
		x = outMat.get02(); outMat.set02( outMat.get20() ); outMat.set20( x );
		// swap _23 and _32
		//myswap(outMat[1 + 4*2], outMat[2 + 4*1]);
		//x = outMat.v[1 + 4*2]; outMat.v[1 + 4*2] = outMat.v[2 + 4*1]; outMat.v[2 + 4*1] = x;
		x = outMat.get21(); outMat.set21( outMat.get12() ); outMat.set12( x );
	}

	void getBodyPosr(final dxPosR offset_posr, final dxPosR final_posr, dxPosR body_posr)
	{
		DMatrix3 inv_offset = new DMatrix3();
		matrixInvert(offset_posr.R, inv_offset);

		dMULTIPLY0_333(body_posr.R, final_posr.R, inv_offset);
		DVector3 world_offset = new DVector3();  //TZ
		dMULTIPLY0_331(world_offset, body_posr.R, offset_posr.pos);
//		body_posr.pos.v[0] = final_posr.pos.v[0] - world_offset.v[0];
//		body_posr.pos.v[1] = final_posr.pos.v[1] - world_offset.v[1];
//		body_posr.pos.v[2] = final_posr.pos.v[2] - world_offset.v[2];
		body_posr.pos.eqDiff( final_posr.pos, world_offset );
	}

	void getWorldOffsetPosr(final dxPosR body_posr, final dxPosR world_posr, 
			dxPosR offset_posr)
	{
		DMatrix3 inv_body = new DMatrix3();
		matrixInvert(body_posr.R, inv_body);

		dMULTIPLY0_333(offset_posr.R, inv_body, world_posr.R);
		DVector3 world_offset = new DVector3();
//		world_offset.v[0] = world_posr.pos.v[0] - body_posr.pos.v[0];
//		world_offset.v[1] = world_posr.pos.v[1] - body_posr.pos.v[1];
//		world_offset.v[2] = world_posr.pos.v[2] - body_posr.pos.v[2];
		world_offset.eqDiff( world_posr.pos, body_posr.pos );
		dMULTIPLY0_331(offset_posr.pos, inv_body, world_offset);
	}

	/**
	 * calculate our new final position from our offset and body.
	 */
	void computePosr()
	{
		// should only be recalced if we need to - ie offset from a body
		dIASSERT(offset_posr != null);
		dIASSERT(body != null);

		dMULTIPLY0_331 (_final_posr.pos,body._posr.R,offset_posr.pos);
//		_final_posr.pos.v[0] += body._posr.pos.v[0];
//		_final_posr.pos.v[1] += body._posr.pos.v[1];
//		_final_posr.pos.v[2] += body._posr.pos.v[2];
		_final_posr.pos.add( body._posr.pos );
		dMULTIPLY0_333 (_final_posr.R,body._posr.R,offset_posr.R);
	}

	// ************ TZ from collision_kernel.cpp ********************

	//****************************************************************************
	// misc

	/**
	 * private functions that must be implemented by the collision library:
	 * (1) indicate that a geom has moved (TZ: -> dGeomMoved),
	 * (2) get the next geom in a body list.
	 * these functions are called whenever the position of geoms connected to a
	 * body have changed, e.g. with dBodySetPosition(), dBodySetRotation(), or
	 * when the ODE step function updates the body state.
	 */
//	dxGeom dGeomGetBodyNext (dxGeom geom)
	DxGeom dGeomGetBodyNext ()
	{
		return body_next;
	}

	//****************************************************************************
	// public API for geometry objects

	//	#define CHECK_NOT_LOCKED(space) \
	//	  dUASSERT (!(space && space->lock_count), \
	//		    "invalid operation for geom in locked space");
	//TODO remove, this is already in dxSpace.
	protected void CHECK_NOT_LOCKED(DxSpace space) {
		dUASSERT (!(space != null && space.lock_count != 0), 
		"invalid operation for geom in locked space");
	}


//	public void dGeomDestroy (dxGeom g)
	public void dGeomDestroy ()
	{
		//dAASSERT (g);
		//TZ delete g;
		DESTRUCTOR();
	}


//	void dGeomSetData (dxGeom g, Object[] data)
	public void dGeomSetData (Object data)
	{
//		dAASSERT (g);
		_data = data;
	}


//	Object[] dGeomGetData (dxGeom g)
	public Object dGeomGetData ()
	{
		//dAASSERT (g);
		return _data;
	}


//	public void dGeomSetBody (dxGeom g, dxBody b)
	public void dGeomSetBody (DxBody b)
	{
		//dAASSERT (g);
		dUASSERT (b == null || (_gflags & GEOM_PLACEABLE) == 0,
		"geom must be placeable");
		CHECK_NOT_LOCKED (parent_space);

		if (b != null) {
			if (body == null) dFreePosr(_final_posr);
			if (body != b) {
				if (offset_posr != null) {
					dFreePosr(offset_posr);
					offset_posr = null;
				}
				_final_posr = b._posr;
				bodyRemove();
				bodyAdd (b);
			}
			dGeomMoved ();
		}
		else {
			if (body != null) {
				if (offset_posr != null)
				{
					// if we're offset, we already have our own final position, make sure its updated
					recomputePosr();
					dFreePosr(offset_posr);
					offset_posr = null;
				}
				else
				{
					_final_posr = new dxPosR(); //dAllocPosr();
					//	        memcpy (g.final_posr.pos,g.body.posr.pos,sizeof(dVector3));
					_final_posr.pos.set(body._posr.pos); //,sizeof(dVector3));
					//	        memcpy (g.final_posr.R,g.body.posr.R,sizeof(dMatrix3));
					_final_posr.R.set(body._posr.R);//,sizeof(dMatrix3));
				}
				bodyRemove();
			}
			// dGeomMoved() should not be called if the body is being set to 0, as the
			// new position of the geom is set to the old position of the body, so the
			// effective position of the geom remains unchanged.
		}
	}


//	public dxBody dGeomGetBody (dxGeom g)
	public DxBody dGeomGetBody ()
	{
		//dAASSERT (g);
		return body;
	}


//	public void dGeomSetPosition (dxGeom g, double x, double y, double z)
	public void dGeomSetPosition (DVector3C xyz)
	{
		//dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		CHECK_NOT_LOCKED (parent_space);
		if (offset_posr != null) {
			// move body such that body+offset = position
			DVector3 world_offset = new DVector3();
			dMULTIPLY0_331(world_offset, body._posr.R, offset_posr.pos);
			//TZ body could be null...?
			world_offset.eqDiff(xyz, world_offset);
//			body.dBodySetPosition(//g.body,
//					x - world_offset.v[0],
//					y - world_offset.v[1],
//					z - world_offset.v[2]);
			body.dBodySetPosition(world_offset);
		}
		else if (body != null) {
			// this will call dGeomMoved (g), so we don't have to
			body.dBodySetPosition (xyz);
		}
		else {
//			_final_posr.pos.v[0] = x;
//			_final_posr.pos.v[1] = y;
//			_final_posr.pos.v[2] = z;
			_final_posr.pos.set(xyz);
			dGeomMoved ();
		}
	}


//	void dGeomSetRotation (dxGeom g, final dMatrix3 R)
	public void dGeomSetRotation (final DMatrix3C R)
	{
		//dAASSERT (g, R);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		CHECK_NOT_LOCKED (parent_space);
		if (offset_posr != null) {
			recomputePosr();
			// move body such that body+offset = rotation
			dxPosR new_final_posr = new dxPosR();
			dxPosR new_body_posr = new dxPosR();
			//	    memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
			//	    memcpy(new_final_posr.R, R, sizeof(dMatrix3));
			new_final_posr.pos.set(_final_posr.pos);//, sizeof(dVector3));
			new_final_posr.R.set(R);//, sizeof(dMatrix3));
			getBodyPosr(offset_posr, new_final_posr, new_body_posr);
			body.dBodySetRotation(new_body_posr.R);
			body.dBodySetPosition(new_body_posr.pos);
		}
		else if (body != null) {
			// this will call dGeomMoved (g), so we don't have to
			body.dBodySetRotation (R);
		}
		else {
			//memcpy (g.final_posr.R,R,sizeof(dMatrix3));
			_final_posr.R.set(R);//,sizeof(dMatrix3));
			dGeomMoved ();
		}
	}


	void dGeomSetQuaternion (DxGeom g, final DQuaternionC quat)
	{
		dAASSERT (g, quat);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		CHECK_NOT_LOCKED (g.parent_space);
		if (g.offset_posr != null) {
			g.recomputePosr();
			// move body such that body+offset = rotation
			dxPosR new_final_posr = new dxPosR();
			dxPosR new_body_posr = new dxPosR();
			dQtoR (quat, new_final_posr.R);
			//memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
			new_final_posr.pos.set(g._final_posr.pos);
			
			getBodyPosr(g.offset_posr, new_final_posr, new_body_posr);
			g.body.dBodySetRotation( new_body_posr.R);
			g.body.dBodySetPosition(new_body_posr.pos);
		}
		if (g.body != null) {
			// this will call dGeomMoved (g), so we don't have to
			g.body.dBodySetQuaternion (quat);
		}
		else {
			dQtoR (quat, g._final_posr.R);
			g.dGeomMoved ();
		}
	}


	public final DVector3C dGeomGetPosition ()
	//	public final double[] dGeomGetPosition (dxGeom g)
	{
//		dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		recomputePosr();
		return _final_posr.pos;
	}


	void dGeomCopyPosition(DxGeom g, DVector3 pos)
	{
		dAASSERT (g);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		g.recomputePosr();
		//final double[] src = g.final_posr.pos;
		pos.set(g._final_posr.pos);
		//	  pos[0] = src[0];
		//	  pos[1] = src[1];
		//	  pos[2] = src[2];
	}


//	public final dMatrix3 dGeomGetRotation (dxGeom g)
	public final DMatrix3C dGeomGetRotation ()
	{
//		dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		recomputePosr();
		return _final_posr.R;
	}


	void dGeomCopyRotation(DxGeom g, DMatrix3 R)
	{
		dAASSERT (g);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		g.recomputePosr();
//		final double[] src = g._final_posr.R.v;
//		R.v[0]  = src[0];
//		R.v[1]  = src[1];
//		R.v[2]  = src[2];
//		R.v[4]  = src[4];
//		R.v[5]  = src[5];
//		R.v[6]  = src[6];
//		R.v[8]  = src[8];
//		R.v[9]  = src[9];
//		R.v[10] = src[10];
		R.set(g._final_posr.R);
	}


//	void dGeomGetQuaternion (dxGeom g, dQuaternion quat)
	public void dGeomGetQuaternion (DQuaternion quat)
	{
		//dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		//  if (g.body && !g.offset_posr) {
		if (body != null && offset_posr == null) {
			//final DQuaternionC body_quat = body.dBodyGetQuaternion ();
			quat.set( body.dBodyGetQuaternion() );
//			quat.v[0] = body_quat[0];
//			quat.v[1] = body_quat[1];
//			quat.v[2] = body_quat[2];
//			quat.v[3] = body_quat[3];
		}
		else {
			recomputePosr();
			dQfromR(quat, _final_posr.R);
		}
	}
	@Override
	public DQuaternionC getQuaternion ()
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		if (body != null && offset_posr == null) {
			return body.dBodyGetQuaternion();
		}
		else {
			recomputePosr();
			DQuaternion quat = new DQuaternion();
			dQfromR(quat, _final_posr.R);
			return quat;
		}
	}

	//void dGeomGetAABB (dxGeom *g, double aabb[6])
	public void dGeomGetAABB (DVector6 aabb)
	{
		//dAASSERT (g);
		//dAASSERT (aabb);
		recomputeAABB();
		//memcpy (aabb,g.aabb,6 * sizeof(double));
		aabb.set(_aabb);
	}
	@Override
	public DAABBC getAABB () {
		recomputeAABB();
		return _aabb;
	}

	//TODO remove this method???
//	boolean dGeomIsSpace (dxGeom g)
	public boolean dGeomIsSpace ()
	{
		//dAASSERT (g);
		return this instanceof DSpace;
	}


	DxSpace dGeomGetSpace (DxGeom g)
	{
		dAASSERT (g);
		return g.parent_space;
	}

	//TODO
	//public int dGeomGetClass (dxGeom g)
	public int dGeomGetClass ()
	{
		//dAASSERT (g);
		return type;
	}
//	public Class<?> dGeomGetClass ()
//	{
//		return getClass();  //Return public interface instead???
//	}


	//	void dGeomSetCategoryBits (dxGeom g, unsigned long bits)
	public void dGeomSetCategoryBits (long bits)
	{
		//dAASSERT (g);
		CHECK_NOT_LOCKED (parent_space);
		category_bits = bits;
	}


	//	void dGeomSetCollideBits (dxGeom g, unsigned long bits)
	public void dGeomSetCollideBits (long bits)
	{
		//dAASSERT (g);
		CHECK_NOT_LOCKED (parent_space);
		collide_bits = bits;
	}


	long dGeomGetCategoryBits (DxGeom g)
	//	unsigned long dGeomGetCategoryBits (dxGeom g)
	{
		dAASSERT (g);
		return g.category_bits;
	}


	long dGeomGetCollideBits (DxGeom g)
	//	unsigned long dGeomGetCollideBits (dxGeom g)
	{
		dAASSERT (g);
		return g.collide_bits;
	}


	void dGeomEnable (DxGeom g)
	{
		dAASSERT (g);
		g._gflags |= GEOM_ENABLED;
	}

	void dGeomDisable (DxGeom g)
	{
		dAASSERT (g);
		g._gflags &= ~GEOM_ENABLED;
	}

//	int dGeomIsEnabled (dxGeom g)
	boolean dGeomIsEnabled ()
	{
//		dAASSERT (g);
		return (_gflags & GEOM_ENABLED) != 0;// ? 1 : 0;
	}


//	/** **************************************************************************
//	 * C interface that lets the user make new classes. this interface is a lot
//	 * more cumbersome than C++ subclassing, which is what is used internally
//	 * in ODE. this API is mainly to support legacy code.
//	 * @deprecated see above (TZ) TODO remove?
//	 */
//	private static dGeomClass[] user_classes = new dGeomClass[dMaxUserClasses];
	private static int num_user_classes = 0;
//
//
//	public class dxUserGeom extends dxGeom {
//		//void *user_data;
//		Object[] user_data;
//
//		//	  dxUserGeom (int class_num);
//		//	  ~dxUserGeom();
//		//	  void computeAABB();
//		//	  int AABBTest (dxGeom *o, double aabb[6]);
//		//	};
//
//
//		//		dxUserGeom::dxUserGeom (int class_num) : dxGeom (0,1)
//		dxUserGeom (int class_num) {
//			//		super (0,1);
//			super (null,true);
//			type = class_num;
//			int size = user_classes[type-dFirstUserClass].bytes;
//			user_data = null;// TZ: dAlloc (size);  TODO ?!?!?
//			//TZ TODO ??!?! memset (user_data,0,size);
//		}
//
//
//		//	dxUserGeom::~dxUserGeom()
//		public void DESTRUCTOR() {
//			dGeomClass c = user_classes[type-dFirstUserClass];
//			if (c.dtor!=null) c.dtor(this);
//			//TZ dFree (user_data,c.bytes);
//			user_data = null;
//			super.DESTRUCTOR();
//		}
//
//
//		//	void dxUserGeom::computeAABB()
//		void computeAABB()
//		{
//			user_classes[type-dFirstUserClass].aabb(this,_aabb);
//		}
//
//
//		//	int dxUserGeom::AABBTest (dxGeom *o, double aabb[6])
//		boolean AABBTest (dxGeom o, dVector6 aabb)
//		{
//			dGeomClass c = user_classes[type-dFirstUserClass];
//			if (c.aabb_test != null) return c.aabb_test(this,o,aabb);
//			else return true;
//		}
//	}
//
////	static int dCollideUserGeomWithGeom (dxGeom o1, dxGeom o2, int flags,
////	dContactGeom contact, int skip)
//	static int dCollideUserGeomWithGeom (dxGeom o1, dxGeom o2, int flags,
//	dContactGeomBuffer contacts)
//	{
//		// this generic collider function is called the first time that a user class
//		// tries to collide against something. it will find out the correct collider
//		// function and then set the colliders array so that the correct function is
//		// called directly the next time around.
//
//		int t1 = o1.type;	// note that o1 is a user geom
//		int t2 = o2.type;	// o2 *may* be a user geom
//
//		// find the collider function to use. if o1 does not know how to collide with
//		// o2, then o2 might know how to collide with o1 (provided that it is a user
//		// geom).
//		dColliderFn fn = user_classes[t1-dFirstUserClass].collider(t2);
//		boolean reverse = false;
//		if (fn==null && t2 >= dFirstUserClass && t2 <= dLastUserClass) {
//			fn = user_classes[t2-dFirstUserClass].collider (t1);
//			reverse = true;
//		}
//
//		// set the colliders array so that the correct function is called directly
//		// the next time around. note that fn can be 0 here if no collider was found,
//		// which means that dCollide() will always return 0 for this case.
//		colliders[t1][t2].fn = fn;
//		colliders[t1][t2].reverse = reverse;
//		colliders[t2][t1].fn = fn;
//		colliders[t2][t1].reverse = !reverse;
//
//		// now call the collider function indirectly through dCollide(), so that
//		// contact reversing is properly handled.
//		return dCollide (o1,o2,flags,contacts, 1);
//	}
//
//	int dCreateGeomClass (final dGeomClass c)
//	{
//		dUASSERT(c != null && c.bytes >= 0 && 
//				c.collider != null && c.aabb != null,"bad geom class");
//
//		if (num_user_classes >= dMaxUserClasses) {
//			dDebug (0,"too many user classes, you must increase the limit and " +
//			"recompile ODE");
//		}
//		user_classes[num_user_classes] = c;
//		int class_number = num_user_classes + dFirstUserClass;
//		//		setAllColliders (class_number,&dCollideUserGeomWithGeom);
//		setAllColliders (class_number,new dColliderFn(){
//			public int dColliderFn(dGeom o2, dGeom o1, int flags,
//					dContactGeomBuffer contacts) {
//				return dCollideUserGeomWithGeom((dxGeom)o1, (dxGeom)o2, flags, contacts);
//			}});
//
//		num_user_classes++;
//		return class_number;
//	}
//
	/*extern */static void dFinitUserClasses()
	{
		num_user_classes = 0;
	}
//
//	Object[] dGeomGetClassData (dxGeom g)
//	{
//		dUASSERT (g != null && g.type >= dFirstUserClass &&
//				g.type <= dLastUserClass,"not a custom class");
//		dxUserGeom user = (dxUserGeom) g;
//		return user.user_data;
//	}
//
//
//	dxGeom dCreateGeom (int classnum)
//	{
//		dUASSERT (classnum >= dFirstUserClass &&
//				classnum <= dLastUserClass,"not a custom class");
//		return new dxUserGeom (classnum);
//	}



	/* ************************************************************************ */
	/* geom offset from body */

//	void dGeomCreateOffset (dxGeom g)
	private void dGeomCreateOffset ()
	{
		//dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
		if (offset_posr != null)
		{
			return; // already created
		}
		dIASSERT (_final_posr == body._posr);

		_final_posr = dAllocPosr();
		offset_posr = dAllocPosr();
		offset_posr.pos.setValues(0);//dSetZero (offset_posr.pos.v,4);
		dRSetIdentity (offset_posr.R);

		_gflags |= GEOM_POSR_BAD;
	}

//	void dGeomSetOffsetPosition (dxGeom g, double x, double y, double z)
	public void dGeomSetOffsetPosition (double x, double y, double z)
	{
		//dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
		CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset();
		}
//		g.offset_posr.pos.v[0] = x;
//		g.offset_posr.pos.v[1] = y;
//		g.offset_posr.pos.v[2] = z;
		offset_posr.pos.set(x, y, z);
		dGeomMoved();
	}

//	void dGeomSetOffsetRotation (dxGeom g, final dMatrix3 R)
	public void dGeomSetOffsetRotation (final DMatrix3C R)
	{
		//dAASSERT (g, R);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
		CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset ();
		}
//		memcpy (g.offset_posr.R,R,sizeof(dMatrix3));
		offset_posr.R.set(R);
		dGeomMoved();
	}

	void dGeomSetOffsetQuaternion (DxGeom g, final DQuaternionC quat)
	{
		dAASSERT (g, quat);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (g.body, "geom must be on a body");
		CHECK_NOT_LOCKED (g.parent_space);
		if (g.offset_posr == null)
		{
			g.dGeomCreateOffset ();
		}
		dQtoR (quat, g.offset_posr.R);
		g.dGeomMoved();
	}

	void dGeomSetOffsetWorldPosition (DxGeom g, double x, double y, double z)
	{
		dAASSERT (g);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (g.body, "geom must be on a body");
		CHECK_NOT_LOCKED (g.parent_space);
		if (g.offset_posr == null)
		{
			g.dGeomCreateOffset();
		}
		g.body.dBodyGetPosRelPoint(new DVector3(x, y, z), g.offset_posr.pos);
		g.dGeomMoved();
	}

	void dGeomSetOffsetWorldRotation (DxGeom g, final DMatrix3C R)
	{
		dAASSERT (g, R);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (g.body, "geom must be on a body");
		CHECK_NOT_LOCKED (g.parent_space);
		if (g.offset_posr == null)
		{
			g.dGeomCreateOffset ();
		}
		g.recomputePosr();

		dxPosR new_final_posr = new dxPosR();
//		memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
		new_final_posr.pos.set(g._final_posr.pos);
//		memcpy(new_final_posr.R, R, sizeof(dMatrix3));
		new_final_posr.R.set(R);

		getWorldOffsetPosr(g.body._posr, new_final_posr, g.offset_posr);
		g.dGeomMoved();
	}

	void dGeomSetOffsetWorldQuaternion (DxGeom g, final DQuaternionC quat)
	{
		dAASSERT (g, quat);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (g.body, "geom must be on a body");
		CHECK_NOT_LOCKED (g.parent_space);
		if (g.offset_posr == null)
		{
			g.dGeomCreateOffset ();
		}

		g.recomputePosr();

		dxPosR new_final_posr = new dxPosR();
//		memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
		new_final_posr.pos.set(g._final_posr.pos);
		dQtoR (quat, new_final_posr.R);

		getWorldOffsetPosr(g.body._posr, new_final_posr, g.offset_posr);
		g.dGeomMoved();
	}

	void dGeomClearOffset(DxGeom g)
	{
		dAASSERT (g);
		dUASSERT (g._gflags & GEOM_PLACEABLE,"geom must be placeable");
		if (g.offset_posr != null)
		{
			dIASSERT(g.body != null);
			// no longer need an offset posr
			dFreePosr(g.offset_posr);
			g.offset_posr = null;
			// the geom will now share the position of the body
			dFreePosr(g._final_posr);
			g._final_posr = g.body._posr;
			// geom has moved
			g._gflags &= ~GEOM_POSR_BAD;
			g.dGeomMoved();
		}
	}

	int dGeomIsOffset(DxGeom g)
	{
		dAASSERT (g);
		return ((null != g.offset_posr) ? 1 : 0);
	}

	private static final DVector3C OFFSET_POSITION_ZERO = new DVector3( 0.0f, 0.0f, 0.0f );

	//	final double * dGeomGetOffsetPosition (dxGeom g)
	final DVector3C dGeomGetOffsetPosition (DxGeom g)
	{
		dAASSERT (g);
		if (g.offset_posr != null)
		{
			return g.offset_posr.pos;
		}
		return OFFSET_POSITION_ZERO;
	}

	void dGeomCopyOffsetPosition (DxGeom g, DVector3 pos)
	{
		dAASSERT (g);
		if (g.offset_posr != null)
		{
			//final double[] src = g.offset_posr.pos.v;
			pos.set(g.offset_posr.pos);
			//	    pos[0] = src[0];
			//		 pos[1] = src[1];
			//		 pos[2] = src[2];
		}
		else
		{
			pos.set(DVector3.ZERO);
			//	    pos[0] = 0;
			//		 pos[1] = 0;
			//		 pos[2] = 0;
		}
	}

	private static final DMatrix3C OFFSET_ROTATION_ZERO = new DMatrix3(
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0);

	//double *
	final DMatrix3C dGeomGetOffsetRotation (DxGeom g)
	{
		dAASSERT (g);
		if (g.offset_posr != null)
		{
			return g.offset_posr.R;
		}
		return OFFSET_ROTATION_ZERO;
	}

	void dGeomCopyOffsetRotation (DxGeom g, DMatrix3 R)
	{
		dAASSERT (g);
		if (g.offset_posr != null)
		{
//			final double[] src = g._final_posr.R.v;
//			R.v[0]  = src[0];
//			R.v[1]  = src[1];
//			R.v[2]  = src[2];
//			R.v[4]  = src[4];
//			R.v[5]  = src[5];
//			R.v[6]  = src[6];
//			R.v[8]  = src[8];
//			R.v[9]  = src[9];
//			R.v[10] = src[10];
			R.set(g._final_posr.R);
		}
		else
		{
//			R.v[0]  = OFFSET_ROTATION_ZERO.v[0];
//			R.v[1]  = OFFSET_ROTATION_ZERO.v[1];
//			R.v[2]  = OFFSET_ROTATION_ZERO.v[2];
//			R.v[4]  = OFFSET_ROTATION_ZERO.v[4];
//			R.v[5]  = OFFSET_ROTATION_ZERO.v[5];
//			R.v[6]  = OFFSET_ROTATION_ZERO.v[6];
//			R.v[8]  = OFFSET_ROTATION_ZERO.v[8];
//			R.v[9]  = OFFSET_ROTATION_ZERO.v[9];
//			R.v[10] = OFFSET_ROTATION_ZERO.v[10];
			R.set(OFFSET_ROTATION_ZERO);
		}
	}

	void dGeomGetOffsetQuaternion (DxGeom g, DQuaternion result)
	{
		dAASSERT (g);
		if (g.offset_posr != null)
		{
			dQfromR(result, g.offset_posr.R);
		}
		else
		{
//			dSetZero (result.v,4);
//			result.v[0] = 1;
			result.set(1, 0, 0, 0);
		}
	}


	private dxPosR dAllocPosr() {
		return new dxPosR();
	}
	//	static inline dxPosR* dAllocPosr()
	//	{
	//		dxPosR *retPosR;
	//	
	////	#if dATOMICS_ENABLED
	////		retPosR = (dxPosR *)AtomicExchangePointer(&s_cachedPosR, NULL);
	////	
	////		if (!retPosR)
	////	#endif
	//		{
	//			retPosR = (dxPosR*) dAlloc (sizeof(dxPosR));
	//		}
	//	
	//		return retPosR;
	//	}


	/**
	 * Frees memory ?
	 * @deprecated
	 */
	private void dFreePosr(dxPosR oldPosR) {
		//TODO see collision_kernel.h:73
		//#if dATOMICS_ENABLED
		//		if (!AtomicCompareExchangePointer(&s_cachedPosR, NULL, (atomicptr)oldPosR))
		//#endif
		//		{
		//			dFree(oldPosR, sizeof(dxPosR));
		//		}
	}

	/**
	 * private functions that must be implemented by the collision library:
	 * (1) indicate that a geom has moved, (2) get the next geom in a body list 
	 * (TZ: -> dGeomGetBodyNext() ).
	 * these functions are called whenever the position of geoms connected to a
	 * body have changed, e.g. with dBodySetPosition(), dBodySetRotation(), or
	 * when the ODE step function updates the body state.
	 * 
	 * make the geom dirty by setting the GEOM_DIRTY and GEOM_BAD_AABB flags
	 * and moving it to the front of the space's list. all the parents of a
	 * dirty geom also become dirty.
	 */ 
//	void dGeomMoved (dxGeom geom)
	void dGeomMoved ()
	{
		//dAASSERT (geom);

		// if geom is offset, mark it as needing a calculate
		if (offset_posr != null) {
			_gflags |= GEOM_POSR_BAD;
		}

		// from the bottom of the space heirarchy up, process all clean geoms
		// turning them into dirty geoms.
		DxSpace parent = parent_space;

		DxGeom geom = this;
		while (parent != null && (geom._gflags & GEOM_DIRTY)==0) {
			CHECK_NOT_LOCKED (parent);
			geom._gflags |= GEOM_DIRTY | GEOM_AABB_BAD;
			parent.dirty (geom);
			geom = parent;
			parent = parent.parent_space;
		}

		// all the remaining dirty geoms must have their AABB_BAD flags set, to
		// ensure that their AABBs get recomputed
		while (geom != null) {
			geom._gflags |= GEOM_DIRTY | GEOM_AABB_BAD;
			CHECK_NOT_LOCKED (geom.parent_space);
			geom = geom.parent_space;
		}
	}



	//#define GEOM_ENABLED(g) (((g).gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE)
	protected boolean GEOM_ENABLED(DxGeom g) {
		return ((g._gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE);
	}


	//****************************************************************************
	// dispatcher for the N^2 collider functions

	// function pointers and modes for n^2 class collider functions

	private static class dColliderEntry {
		DColliderFn fn;	// collider function, 0 = no function available
		boolean reverse;		// 1 = reverse o1 and o2
	}
	private static dColliderEntry[][] colliders = new dColliderEntry[dGeomNumClasses][dGeomNumClasses];
	private static boolean colliders_initialized = false;


	// setCollider() will refuse to write over a collider entry once it has
	// been written.

	private static void setCollider (int i, int j, DColliderFn fn)
	{
		//TZ
		if (colliders[i][j] == null) colliders[i][j] = new dColliderEntry();
		if (colliders[j][i] == null) colliders[j][i] = new dColliderEntry();
		if (colliders[i][j].fn == null) {
			colliders[i][j].fn = fn;
			colliders[i][j].reverse = false;
		}
		if (colliders[j][i].fn == null) {
			colliders[j][i].fn = fn;
			colliders[j][i].reverse = true;
		}
	}


	private static void setAllColliders (int i, DColliderFn fn)
	{
		for (int j=0; j<dGeomNumClasses; j++) setCollider (i,j,fn);
	}

	//TODO put into a different class?
	/*extern */
	public static void dInitColliders()
	{
		dIASSERT(!colliders_initialized);
		colliders_initialized = true;

		//memset (colliders,0,sizeof(colliders));
		for (int i = 0; i < colliders.length; i++) {
			//TZ
			colliders[i] = new dColliderEntry[dGeomNumClasses];
			for (int j = 0; j < colliders[i].length; j++) {
				//colliders[i][j] = null;
				colliders[i][j] = new dColliderEntry();
			}
		}

		// setup space colliders
		for (int i=dFirstSpaceClass; i <= dLastSpaceClass; i++) {
			for (int j=0; j < dGeomNumClasses; j++) {
				//setCollider (i,j, dCollideSpaceGeom);
				//setCollider(i, j, createFn(dxGeom.class, "dCollideSpaceGeom"));
				setCollider(i, j, new CollideSpaceGeom());
			}
		}

//TODO		
		setCollider(dSphereClass, dSphereClass, new DxSphere.CollideSphereSphere());
		setCollider(dSphereClass, dBoxClass, new DxSphere.CollideSphereBox());
		setCollider(dSphereClass, dPlaneClass, new DxSphere.CollideSpherePlane());
		setCollider (dBoxClass,dBoxClass, new CollideBoxBox());//dCollideBoxBox);
		setCollider (dBoxClass,dPlaneClass, new CollideBoxPlane());//dCollideBoxPlane);
		setCollider (dCapsuleClass,dSphereClass, new DxCapsule.CollideCapsuleSphere());//dCollideCapsuleSphere);
		setCollider (dCapsuleClass,dBoxClass, new DxCapsule.CollideCapsuleBox());//dCollideCapsuleBox);
		setCollider (dCapsuleClass,dCapsuleClass, new DxCapsule.CollideCapsuleCapsule());//dCollideCapsuleCapsule);
		setCollider (dCapsuleClass,dPlaneClass, new DxCapsule.CollideCapsulePlane());//dCollideCapsulePlane);
		setCollider (dRayClass,dSphereClass, new DxRay.CollideRaySphere());//dCollideRaySphere);
		setCollider (dRayClass,dBoxClass, new DxRay.CollideRayBox());//dCollideRayBox);
		setCollider (dRayClass,dCapsuleClass, new DxRay.CollideRayCapsule());//dCollideRayCapsule);
		setCollider (dRayClass,dPlaneClass, new DxRay.CollideRayPlane());//dCollideRayPlane);
		setCollider (dRayClass,dCylinderClass, new DxRay.CollideRayCylinder());//dCollideRayCylinder);
//		if (dTRIMESH_ENABLED) {
//			setCollider (dTriMeshClass,dSphereClass, dCollideSTL);
//			setCollider (dTriMeshClass,dBoxClass, dCollideBTL);
//			setCollider (dTriMeshClass,dRayClass, dCollideRTL);
//			setCollider (dTriMeshClass,dTriMeshClass, dCollideTTL);
//			setCollider (dTriMeshClass,dCapsuleClass, dCollideCCTL);
//			setCollider (dTriMeshClass,dPlaneClass, dCollideTrimeshPlane);
//			setCollider (dCylinderClass,dTriMeshClass, dCollideCylinderTrimesh);
//		}
		setCollider (dCylinderClass,dBoxClass, new CollideCylinderBox());//dCollideCylinderBox);
		setCollider (dCylinderClass,dSphereClass, new CollideCylinderSphere());//dCollideCylinderSphere);
		setCollider (dCylinderClass,dPlaneClass, new CollideCylinderPlane());//dCollideCylinderPlane);
//		//setCollider (dCylinderClass,dCylinderClass, dCollideCylinderCylinder);
//
//		//--> Convex Collision
		setCollider (dConvexClass,dPlaneClass, new DxConvex.CollideConvexPlane());//dCollideConvexPlane);
		setCollider (dSphereClass,dConvexClass, new DxConvex.CollideSphereConvex());//dCollideSphereConvex);
		setCollider (dConvexClass,dBoxClass, new DxConvex.CollideConvexBox());//dCollideConvexBox);
		setCollider (dConvexClass,dCapsuleClass, new DxConvex.CollideConvexCapsule());//dCollideConvexCapsule);
		setCollider (dConvexClass,dConvexClass, new DxConvex.CollideConvexConvex());//dCollideConvexConvex);
		setCollider (dRayClass,dConvexClass, new DxConvex.CollideRayConvex());//dCollideRayConvex);
//		//<-- Convex Collision
//
//		//--> dHeightfield Collision
		setCollider (dHeightfieldClass,dRayClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
		setCollider (dHeightfieldClass,dSphereClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
		setCollider (dHeightfieldClass,dBoxClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
		setCollider (dHeightfieldClass,dCapsuleClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
		setCollider (dHeightfieldClass,dCylinderClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
		setCollider (dHeightfieldClass,dConvexClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
//		if (dTRIMESH_ENABLED) {
//			setCollider (dHeightfieldClass,dTriMeshClass, dCollideHeightfield);
//		}
//		//<-- dHeightfield Collision
//
		setAllColliders (dGeomTransformClass, new DxGeomTransform.CollideTransform());//dCollideTransform);
	}

//	/** @deprecated (TZ) References should be removed, as this is not a valid collider. */
//	private static class ColliderDummy implements dColliderFn {
//
//		@Override
//		public int dColliderFn(dGeom o1, dGeom o2, int flags,
//				dContactGeomBuffer contact) {
//			// TODO
//			System.err.println("Collider not implemented: " + 
//					o1.getClass().getName() + " / " + o2.getClass().getName());
//			throw new RuntimeException();
//			//return 0;
//		}
//		
//	}
	

	/*extern */
	static void dFinitColliders()
	{
		colliders_initialized = false;
	}

	void dSetColliderOverride (int i, int j, DColliderFn fn)
	{
		dIASSERT( colliders_initialized );
		dAASSERT( i < dGeomNumClasses );
		dAASSERT( j < dGeomNumClasses );

		colliders[i][j].fn = fn;
		colliders[i][j].reverse = false;
		colliders[j][i].fn = fn;
		colliders[j][i].reverse = true;
	}

	/**
	 *	NOTE!
	 *	If it is necessary to add special processing mode without contact generation
	 *	use NULL contact parameter value as indicator, not zero in flags.
	 */
	public static int dCollide (DxGeom o1, DxGeom o2, int flags, 
			DContactGeomBuffer contacts, int skip)
	{
		dAASSERT(o1, o2, contacts);
		dUASSERT(colliders_initialized,"colliders array not initialized");
		dUASSERT(o1.type >= 0 && o1.type < dGeomNumClasses,"bad o1 class number");
		dUASSERT(o2.type >= 0 && o2.type < dGeomNumClasses,"bad o2 class number");
		// Even though comparison for greater or equal to one is used in all the
		// other places, here it is more logical to check for greater than zero
		// because function does not require any specific number of contact slots -
		// it must be just a positive.
		dUASSERT((flags & NUMC_MASK) > 0, "no contacts requested");

		// Extra precaution for zero contact count in parameters
		if ((flags & NUMC_MASK) == 0) return 0;
		// no contacts if both geoms are the same
		if (o1 == o2) return 0;

		// no contacts if both geoms on the same body, and the body is not 0
		if (o1.body == o2.body && o1.body != null) return 0;

		o1.recomputePosr();
		o2.recomputePosr();

		dColliderEntry ce = colliders[o1.type][o2.type];
		System.out.println("COLLIDE:" + o1.getClass() + " / " + o2.getClass());
		System.out.println("COLLIDE2:" + o1.type + " / " + o2.type);
		int count = 0;
		if (ce.fn != null) {
			if (ce.reverse) {
				count = (ce.fn).dColliderFn (o2,o1,flags,contacts);
				for (int i=0; i<count; i++) {
					//dContactGeom c = CONTACT(contact,skip*i);
					DContactGeom c = contacts.get(i);
//					c.normal.v[0] = -c.normal.v[0];
//					c.normal.v[1] = -c.normal.v[1];
//					c.normal.v[2] = -c.normal.v[2];
					c.normal.scale(-1);
					DxGeom tmp = c.g1;
					c.g1 = c.g2;
					c.g2 = tmp;
					int tmpint = c.side1;
					c.side1 = c.side2;
					c.side2 = tmpint;
				}
			}
			else {
				count = (ce.fn).dColliderFn (o1,o2,flags,contacts);
			}
		}
		System.out.println("COLLIDE3:" + count);
		return count;
	}


	// **************** from collision_space_internal.h TZ


	//	#define CHECK_NOT_LOCKED(space) \
	//	  dUASSERT ((space)==0 || (space)->lock_count==0, \
	//		    "invalid operation for locked space");


	// collide two geoms together. for the hash table space, this is
	// called if the two AABBs inhabit the same hash table cells.
	// this only calls the callback function if the AABBs actually
	// intersect. if a geom has an AABB test function, that is called to
	// provide a further refinement of the intersection.
	//
	// NOTE: this assumes that the geom AABBs are valid on entry
	// and that both geoms are enabled.

	static void collideAABBs (DxGeom g1, DxGeom g2,
			Object data, DNearCallback callback)
	{
		dIASSERT((g1._gflags & GEOM_AABB_BAD)==0);
		dIASSERT((g2._gflags & GEOM_AABB_BAD)==0);

		// no contacts if both geoms on the same body, and the body is not 0
		if (g1.body == g2.body && g1.body!= null) return;

		// test if the category and collide bitfields match
		if ( ((g1.category_bits & g2.collide_bits)!=0 ||
				(g2.category_bits & g1.collide_bits)!=0) == false) {
			return;
		}

		// if the bounding boxes are disjoint then don't do anything
		DAABB bounds1 = g1._aabb;
		DAABB bounds2 = g2._aabb;
//		if (bounds1.v[0] > bounds2.v[1] ||
//				bounds1.v[1] < bounds2.v[0] ||
//				bounds1.v[2] > bounds2.v[3] ||
//				bounds1.v[3] < bounds2.v[2] ||
//				bounds1.v[4] > bounds2.v[5] ||
//				bounds1.v[5] < bounds2.v[4]) {
//			return;
//		}
		if (bounds1.isDisjoint( bounds2 )) {
			return;
		}

		// check if either object is able to prove that it doesn't intersect the
		// AABB of the other
		if (!g1.AABBTest (g2,bounds2)) return;
		if (!g2.AABBTest (g1,bounds1)) return;

		// the objects might actually intersect - call the space callback function
		callback.call (data,g1,g2);
	}

	public String toString() {
		return super.toString() + " body=" + body;  
	}
	
//	final void setGFlags(int flag) {
//		xyz
//	}
//	
//	final int getGFlags() {
//		return _gflags;
//	}

	void setNextPrevNull() {
		_next = null;
		_prev = null;
	}
	
	// *********************************************
	// dGeom API
	// *********************************************
	
	//~dGeom()
//	public void DESTRUCTOR()
//	{ if (_id!=null) dGeomDestroy (_id); }

//	public dGeom id() //const
//	{ return _id; }
	//TODO TZ ?
	//operator dGeom() //const
	//{ return _id; }

	public void destroy() {
		//if (_id!=null) dGeomDestroy (_id);
		//_id = null;
		dGeomDestroy();
	}

	public int getClassID()// const
	{ return dGeomGetClass (); }

	public DSpace getSpace() //const
	{ return dGeomGetSpace (this); }

	public void setData (Object data)
	{ dGeomSetData (data); }
	public Object getData() //const
	{ return dGeomGetData (); }

	public void setBody (DBody b)
	{ dGeomSetBody ((DxBody)b); }
	public DBody getBody() //const
	{ return dGeomGetBody (); }

	public void setPosition (double x, double y, double z)
	{ dGeomSetPosition (new DVector3(x,y,z)); }
	public void setPosition (DVector3C xyz)
	{ dGeomSetPosition (xyz); }
	//const dReal * getPosition() const
	public DVector3C getPosition()
	{ return dGeomGetPosition (); }

	public void setRotation (final DMatrix3 R)
	{ dGeomSetRotation (R); }
	//const dReal * getRotation() const
	public DMatrix3C getRotation()
	{ return dGeomGetRotation (); }

	public void setQuaternion (final DQuaternion quat)
	{ dGeomSetQuaternion (this,quat); }

//	public int isSpace()
//	{ return dGeomIsSpace (_id); }

	public void setCategoryBits (long bits)//unsigned long bits)
	{ dGeomSetCategoryBits (bits); }
	public void setCollideBits (long bits)//unsigned long bits)
	{ dGeomSetCollideBits (bits); }
	//unsigned 
	public long getCategoryBits()
	{ return dGeomGetCategoryBits (this); }
	//unsigned 
	public long getCollideBits()
	{ return dGeomGetCollideBits (this); }

	public void enable()
	{ dGeomEnable (this); }
	public void disable()
	{ dGeomDisable (this); }
	public boolean isEnabled()
	{ return dGeomIsEnabled (); }

	public void collide2 (DGeom g, Object data, DNearCallback callback)
	{ DxSpace.dSpaceCollide2 (this,(DxGeom)g,data,callback); }

	public void setOffsetPosition(double x, double y, double z) {
		dGeomSetOffsetPosition(x, y, z);
	}

	public void setOffsetRotation(DMatrix3C R) {
		dGeomSetOffsetRotation(R);
	}

}
