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

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Rotation.dQfromR;

import org.ode4j.ode.*;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.Objects_H.DxPosRC;
import org.ode4j.ode.internal.Objects_H.DxPosR;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefInt;
import org.ode4j.ode.internal.DxQuadTreeSpace.Block;

import static org.ode4j.ode.internal.CollisionLibccd.*;

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
	//	  dIN_RANGE((geom)->type, dFirstSpaceClass, dLastSpaceClass + 1)
	//	protected static boolean IS_SPACE(dxGeom geom) {
	//		//return ((geom).type >= dFirstSpaceClass && (geom).type <= dLastSpaceClass xxxx);
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

	enum dxContactMergeOptions {
	    DONT_MERGE_CONTACTS,
	    MERGE_CONTACT_NORMALS,
	    MERGE_CONTACTS_FULLY,
	}


	// geometry object base class. pos and R will either point to a separately
	// allocated buffer (if body is 0 - pos points to the dxPosR object) or to
	// the pos and R of the body (if body nonzero).
	// a dGeom is a pointer to this object.

	//	struct dxGeom : public dBase {
	public int type;		// geom type number, set by subclass constructor
	private int _gflags;		// flags used by geom and space
	Object _data;		// user-defined data pointer
	//	  dBody body;		// dynamics body associated with this object (if any)
	DxBody body;		// dynamics body associated with this object (if any)
	DxGeom body_next;	// next geom in body's linked list of associated geoms
	DxPosR _final_posr;	// final position of the geom in world coordinates
	private DxPosR offset_posr;	// offset from body in local coordinates

	// information used by spaces
	//TODO use linked list, seems simpler.
	//private List<DxGeom> _list = new LinkedList<DxGeom>();
	//dxGeom next;		// next geom in linked list of geoms
//	private final Ref<dxGeom> next = new Ref<dxGeom>();
	/** 'tome' is pointer to a pointer to (this).
	 * Usually it is a pointer to the (prev.next) field.
	 * For the first element, it is a pointer to (container.first).
	 */
//	private final Ref<dxGeom> tome = new Ref<dxGeom>();
	private DxGeom _next = null;// next geom in linked list of geoms
	private DxGeom _prev = null;
	//dxGeom tome;	// linked list backpointer
    private DxGeom _next_ex;	// next geom in extra linked list of geoms (for higher level structures)
    //private DxGeom _tome_ex;	// extra linked list backpointer (for higher level structures)
	DxSpace parent_space;// the space this geom is contained in, 0 if none
	int _sapIdxDirtyEx; // TZ: Used by SAP-Space.
	int _sapIdxGeomEx; // TZ: Used by SAP-Space.
	Block _qtIdxEx; // TZ: Used by QuadTree-Space.
	
	//double[] aabb = new double[6];	// cached AABB for this space
	protected DAABB _aabb = new DAABB();	// cached AABB for this space
	//TODO unsigned
	long category_bits,collide_bits;

	//	  dxGeom (dSpace _space, int is_placeable);
	//	  virtual ~dxGeom();

	// Set or clear GEOM_ZERO_SIZED flag
	void updateZeroSizedFlag(boolean is_zero_sized) { 
		_gflags = is_zero_sized ? (_gflags | GEOM_ZERO_SIZED) : (_gflags & ~GEOM_ZERO_SIZED); 
	}

    DVector3C buildUpdatedPosition() {
		dIASSERT((_gflags & GEOM_PLACEABLE) != 0);
		recomputePosr();
		return _final_posr.pos();
	}

    DMatrix3C buildUpdatedRotation() {
		dIASSERT((_gflags & GEOM_PLACEABLE) != 0);
		recomputePosr();
		return _final_posr.R();
	}

	// recalculate our new final position if needed
	void recomputePosr()
	{
		if ((_gflags & GEOM_POSR_BAD) != 0) {
			computePosr();
			_gflags &= ~GEOM_POSR_BAD;
		}
	}

	// calculate our new final position from our offset and body
	//	  void computePosr();

	/**
	 * compute the AABB for this object and put it in aabb. this function
	 * always performs a fresh computation, it does not inspect the
	 * GEOM_AABB_BAD flag.
	 */
	protected abstract void computeAABB();

	/** 
	 * Calculate oriented bounding box.
	 * @param R Orthogonalized rotation matrix.
	 */
	void computeOBB(DMatrix3C R) {
		throw new UnsupportedOperationException();
//		//memcpy(b->posr.R, R, sizeof(dMatrix3));
////		_posr.R.set(R);
////		 dOrthogonalizeR(_posr.R); //TODO
////		 DQuaternion q = new DQuaternion();
////		 dQfromR (q, R);
////		 dNormalize4 (q);
//		
//		 DVector3C pos = _aabb
//		 DVector3C side = _aabb.getLengths();
//		 
//			double xrange = 0.5 * ( dFabs (R.get00() * side.get0()) +
//					dFabs (R.get01() * side.get1()) + dFabs (R.get02() * side.get2()) );
//			double yrange = 0.5 * ( dFabs (R.get10() * side.get0()) +
//					dFabs (R.get11() * side.get1()) + dFabs (R.get12() * side.get2()) );
//			double zrange = 0.5 * ( dFabs (R.get20() * side.get0()) +
//					dFabs (R.get21() * side.get1()) + dFabs (R.get22() * side.get2()) );
////			_aabb.v[0] = pos.v[0] - xrange;
////			_aabb.v[1] = pos.v[0] + xrange;
////			_aabb.v[2] = pos.v[1] - yrange;
////			_aabb.v[3] = pos.v[1] + yrange;
////			_aabb.v[4] = pos.v[2] - zrange;
////			_aabb.v[5] = pos.v[2] + zrange;
//			_aabb.set( pos.get0() - xrange,
//					pos.get0() + xrange,
//					pos.get1() - yrange,
//					pos.get1() + yrange,
//					pos.get2() - zrange,
//					pos.get2() + zrange);
//
//			
//			
//		 
//		// notify all attached geoms that this body has moved
//		for (DxGeom geom2 = geom; geom2 != null; geom2 = geom2.dGeomGetBodyNext ()) {
//			geom2.dGeomMoved ();
//		}
//		if (offset_posr != null) {
//			recomputePosr();
//			// move body such that body+offset = rotation
//			DxPosR new_final_posr = new DxPosR();
//			DxPosR new_body_posr = new DxPosR();
//			//	    memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
//			//	    memcpy(new_final_posr.R, R, sizeof(dMatrix3));
//			new_final_posr.pos.set(_final_posr.pos);//, sizeof(dVector3));
//			new_final_posr.R.set(R);//, sizeof(dMatrix3));
//			getBodyPosr(offset_posr, new_final_posr, new_body_posr);
//			body.dBodySetRotation(new_body_posr.R);
//			body.dBodySetPosition(new_body_posr.pos);
//		}
//		else if (body != null) {
//			// this will call dGeomMoved (g), so we don't have to
//			body.dBodySetRotation (R);
//		}
//		else {
//			//memcpy (g.final_posr.R,R,sizeof(dMatrix3));
//			_final_posr.R.set(R);//,sizeof(dMatrix3));
//			dGeomMoved ();
//		}

	}

	// utility functions

	// compute the AABB only if it is not current. this function manipulates
	// the GEOM_AABB_BAD flag.
	protected void recomputeAABB() {
		if ((_gflags & GEOM_AABB_BAD) != 0) {
			// our aabb functions assume final_posr is up to date
			recomputePosr(); 
			computeAABB();
			_gflags &= ~GEOM_AABB_BAD;
		}
	}

	//void markAABBBad();
	protected void markAABBBad() {
		_gflags |= (GEOM_DIRTY | GEOM_AABB_BAD);
		DxSpace.CHECK_NOT_LOCKED(parent_space);
	}

	/** _gflags |= (GEOM_DIRTY|GEOM_AABB_BAD); */
	protected final void setFlagDirtyAndBad() {
		_gflags |= (GEOM_DIRTY|GEOM_AABB_BAD);
	}
	
	/** _gflags &= (~(GEOM_DIRTY|GEOM_AABB_BAD)); */
	final void unsetFlagDirtyAndBad() {
		_gflags &= (~(GEOM_DIRTY|GEOM_AABB_BAD));
	}

	/** _gflags &= (~(GEOM_DIRTY|GEOM_AABB_BAD)); */
	final void unsetFlagDirty() {
		_gflags &= ~GEOM_DIRTY;
	}

	/** (_gflags & GEOM_DIRTY)!=0; */
	final boolean hasFlagDirty() {
		return (_gflags & GEOM_DIRTY)!=0;
	}

	/** (_gflags & GEOM_AABB_BAD)!=0; */
	final boolean hasFlagAabbBad() {
		return (_gflags & GEOM_AABB_BAD)!=0;
	}

	/** (_gflags & GEOM_PLACEABLE)!=0; */
	final boolean hasFlagPlaceable() {
		return 	(_gflags & GEOM_PLACEABLE)!=0;
	}
	
	final int getFlags() {
		return _gflags;
	}
	
	final void setFlags(int flags) {
		_gflags = flags;
	}
	
	/** _gflags |= customFlag; */
	final void setFlagCustom(int customFlag) {
		_gflags |= customFlag;
	}
	
	/** _gflags &= ~customFlag; */
	final void unsetFlagCustom(int customFlag) {
		_gflags &= ~customFlag;
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

	void spaceAdd (DxGeom next, DxSpace parent) {
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
		
	}

	void spaceRemove(DxSpace parent) {
//		   if (next) next->tome = tome;
//		    *tome = next;
//		if (next != null) next.tome = tome;
//		tome.set(next);
		if (_next != null) {
			_next._prev = _prev;
		}
        if (_prev != null) {
            _prev._next = _next;
        } else {
            parent.setFirst(_next);
        }
		
	}

	
	/**
	 * @return next dxGeom.
	 * @author Tilmann Zaeschke
	 */
	final DxGeom getNext() {
		return _next;
	}

	/**
	 * @return next dxGeom.
	 * @author Tilmann Zaeschke
	 */
	final DxGeom getNextEx() {
		return _next_ex;
	}
	final void setNextEx(DxGeom g) {
		_next_ex = g;
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
			_final_posr.pos.setZero();//dSetZero (_final_posr.pos.v,4);
			_final_posr.Rw().setIdentity();
		}
		else {
			_final_posr = null;
		}
		offset_posr = null;

		// setup space vars
		//TODO remove.
		_next = null;
		_prev = null;//tome = null;
		_next_ex = null;
		//_tome_ex = null;
		parent_space = null;
		_aabb.setZero();//dSetZero (_aabb.v,6);
		category_bits = ~0;
		collide_bits = ~0;

		// put this geom in a space if required
		if (space != null) space.dSpaceAdd (this);
	}


	//	dxGeom::~dxGeom()
	@Override
	public void DESTRUCTOR() {
		if (parent_space != null) parent_space.dSpaceRemove (this);
		//	if ((gflags & GEOM_PLACEABLE) && (!body || (body && offset_posr)))
		if (((_gflags & GEOM_PLACEABLE)!=0) && (body==null || (body!= null && offset_posr!=null)))
			dFreePosr(_final_posr);
		if (offset_posr != null) dFreePosr(offset_posr);
		bodyRemove();
	}

	
	/** Get parent space TLS kind. */
	//unsigned
	int getParentSpaceTLSKind()
	{
	  return parent_space!=null ? parent_space.tls_kind : DxSpace.dSPACE_TLS_KIND_INIT_VALUE;
	}


	/**
	 * test whether the given AABB object intersects with this object, return
	 * 1=yes, 0=no. this is used as an early-exit test in the space collision
	 * functions. the default implementation returns 1, which is the correct
	 * behavior if no more detailed implementation can be provided.
	 */
	//	  abstract int AABBTest (dxGeom o, dReal aabb[6]);
	boolean AABBTest (DxGeom o, DAABBC aabb)
	{
		return true;
	}

	/**
	 * 
	 */
    private void bodyRemove()
    {
        if (body != null) {
            DxGeom last = null;
            DxGeom g = body.geom;
            while (g != null) {
                if (g == this) {
                    if (last == null) {
                        body.geom = g.body_next;
                    } else {
                        last.body_next = g.body_next;
                    }
                    break;
                }
                last = g;
                g = g.body_next;
            }
            body = null;
            body_next = null;
        }
    }

    //TODO remove, replaced with above fix for issue #19.
//	private void bodyRemoveOld()
//	{
//		if (body != null) {
//			// delete this geom from body list
//		    //dxGeom **last = &body->geom, *g = body->geom;
//			DxGeom g = body.geom;
//			while (g != null) {
//				if (g == this) {
//					body.geom = g.body_next;//last = g.body_next;
//					break;
//				}
//				body.geom = g.body_next; //last = g.body_next;
//				g = g.body_next;
//			}
//			body = null;
//			body_next = null;
//		}
//	}

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

	void getBodyPosr(final DxPosR offset_posr, final DxPosR final_posr, DxPosR body_posr)
	{
		DMatrix3 inv_offset = new DMatrix3();
		matrixInvert(offset_posr.Rw(), inv_offset);

		dMultiply0_333(body_posr.Rw(), final_posr.R(), inv_offset);
		DVector3 world_offset = new DVector3();  //TZ
		dMultiply0_331(world_offset, body_posr.R(), offset_posr.pos());
//		body_posr.pos.v[0] = final_posr.pos.v[0] - world_offset.v[0];
//		body_posr.pos.v[1] = final_posr.pos.v[1] - world_offset.v[1];
//		body_posr.pos.v[2] = final_posr.pos.v[2] - world_offset.v[2];
		body_posr.pos.eqDiff( final_posr.pos(), world_offset );
	}

	void getWorldOffsetPosr(final DxPosRC body_posr, final DxPosRC world_posr, 
			DxPosR offset_posr)
	{
		DMatrix3 inv_body = new DMatrix3();
		matrixInvert(body_posr.R(), inv_body);

		dMultiply0_333(offset_posr.Rw(), inv_body, world_posr.R());
		DVector3 world_offset = new DVector3();
//		world_offset.v[0] = world_posr.pos.v[0] - body_posr.pos.v[0];
//		world_offset.v[1] = world_posr.pos.v[1] - body_posr.pos.v[1];
//		world_offset.v[2] = world_posr.pos.v[2] - body_posr.pos.v[2];
		world_offset.eqDiff( world_posr.pos(), body_posr.pos() );
		dMultiply0_331(offset_posr.pos, inv_body, world_offset);
	}

	/**
	 * calculate our new final position from our offset and body.
	 */
	void computePosr()
	{
		// should only be recalced if we need to - ie offset from a body
//		dIASSERT(offset_posr != null);
//		dIASSERT(body != null);

		dMultiply0_331 (_final_posr.pos,body.posr().R(),offset_posr.pos());
//		_final_posr.pos.v[0] += body._posr.pos.v[0];
//		_final_posr.pos.v[1] += body._posr.pos.v[1];
//		_final_posr.pos.v[2] += body._posr.pos.v[2];
		_final_posr.pos.add( body.posr().pos() );
		dMultiply0_333 (_final_posr.Rw(),body.posr().R(),offset_posr.R());
	}

//	boolean checkControlValueSizeValidity(void *dataValue, int *dataSize, 
//	        int iRequiresSize) {
    boolean checkControlValueSizeValidity(Ref<?> dataValue, RefInt dataSize, 
            int iRequiresSize) {
	    // Here it is the intent to return true for 0 required size in any case
        //return (*dataSize == iRequiresSize && dataValue != 0) ? true : !(*dataSize = iRequiresSize);
        if (dataSize.get() == iRequiresSize && dataValue.get() != null) {
            return true;
        }
        dataSize.set(iRequiresSize);
        return iRequiresSize == 0;
	} 

//	@SuppressWarnings("deprecation")
//	boolean controlGeometry(CONTROL_CLASS controlClass, CONTROL_CODE controlCode, 
//			DataValue dataValue, RefInt dataSize) {
//	    throw new IllegalArgumentException("Control class/code is not supported for current geom");
////	   dAASSERT(false && "Control class/code is not supported for current geom");
////
////	   //*dataSize = 0;
////	   dataSize.set(0);
////	   return false;
//	 }
	
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
		dUASSERT (b == null || (_gflags & GEOM_PLACEABLE) != 0,
		"geom must be placeable");
		DxSpace.CHECK_NOT_LOCKED (parent_space);

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
					_final_posr = new DxPosR(); //dAllocPosr();
					//	        memcpy (g.final_posr.pos,g.body.posr.pos,sizeof(dVector3));
					_final_posr.pos.set(body.posr().pos()); //,sizeof(dVector3));
					//	        memcpy (g.final_posr.R,g.body.posr.R,sizeof(dMatrix3));
					_final_posr.Rw().set(body.posr().R());//,sizeof(dMatrix3));
				}
				bodyRemove();
			}
			// dGeomMoved() should not be called if the body is being set to 0, as the
			// new position of the geom is set to the old position of the body, so the
			// effective position of the geom remains unchanged.
		}
	}


//	public dxBody dGeomGetBody (dxGeom g)
	private DxBody dGeomGetBody ()
	{
		//dAASSERT (g);
		return body;
	}


//	public void dGeomSetPosition (dxGeom g, double x, double y, double z)
	private void dGeomSetPosition (DVector3C xyz)
	{
		//dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr != null) {
			// move body such that body+offset = position
			DVector3 world_offset = new DVector3();
			dMultiply0_331(world_offset, body.posr().R(), offset_posr.pos());
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
	private void dGeomSetRotation (DMatrix3C R)
	{
		//dAASSERT (g, R);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr != null) {
			recomputePosr();
			// move body such that body+offset = rotation
			DxPosR new_final_posr = new DxPosR();
			DxPosR new_body_posr = new DxPosR();
			//	    memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
			//	    memcpy(new_final_posr.R, R, sizeof(dMatrix3));
			new_final_posr.pos.set(_final_posr.pos());//, sizeof(dVector3));
			new_final_posr.Rw().set(R);//, sizeof(dMatrix3));
			getBodyPosr(offset_posr, new_final_posr, new_body_posr);
			body.dBodySetRotation(new_body_posr.R());
			body.dBodySetPosition(new_body_posr.pos());
		}
		else if (body != null) {
			// this will call dGeomMoved (g), so we don't have to
			body.dBodySetRotation (R);
		}
		else {
			//memcpy (g.final_posr.R,R,sizeof(dMatrix3));
			_final_posr.Rw().set(R);//,sizeof(dMatrix3));
			dGeomMoved ();
		}
	}


	private void dGeomSetQuaternion (DQuaternionC quat)
	{
		//dAASSERT (quat);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr != null) {
			recomputePosr();
			// move body such that body+offset = rotation
			DxPosR new_final_posr = new DxPosR();
			DxPosR new_body_posr = new DxPosR();
			dRfromQ (new_final_posr.Rw(), quat);
			//memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
			new_final_posr.pos.set(_final_posr.pos());
			
			getBodyPosr(offset_posr, new_final_posr, new_body_posr);
			body.dBodySetRotation(new_body_posr.R());
			body.dBodySetPosition(new_body_posr.pos());
		}
		if (body != null) {
			// this will call dGeomMoved (g), so we don't have to
			body.dBodySetQuaternion (quat);
		}
		else {
			dRfromQ (_final_posr.Rw(), quat);
			dGeomMoved ();
		}
	}


	DVector3C dGeomGetPosition ()
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		return buildUpdatedPosition();
	}


	private void dGeomCopyPosition(DVector3 pos)
	{
		pos.set(buildUpdatedPosition());
	}


	DMatrix3C dGeomGetRotation ()
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		return buildUpdatedRotation();
	}


	private void dGeomCopyRotation(DMatrix3 R)
	{
		R.set(buildUpdatedRotation());
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
			dQfromR(quat, _final_posr.R());
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
			dQfromR(quat, _final_posr.R());
			return quat;
		}
	}

	//void dGeomGetAABB (dxGeom *g, double aabb[6])
	public void dGeomGetAABB (DAABB aabb)
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

//	boolean dGeomIsSpace (dxGeom g)
//	public boolean dGeomIsSpace ()
//	{
//		//dAASSERT (g);
//		return this instanceof DSpace;
//	}


	private DxSpace dGeomGetSpace ()
	{
		return parent_space;
	}

	//public int dGeomGetClass (dxGeom g)
	private int dGeomGetClass ()
	{
		return type;
	}
//	public Class<?> dGeomGetClass ()
//	{
//		return getClass();  //Return public interface instead???
//	}


	//	void dGeomSetCategoryBits (dxGeom g, unsigned long bits)
	private void dGeomSetCategoryBits (long bits)
	{
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		category_bits = bits;
	}


	//	void dGeomSetCollideBits (dxGeom g, unsigned long bits)
	private void dGeomSetCollideBits (long bits)
	{
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		collide_bits = bits;
	}


	//	unsigned long dGeomGetCategoryBits (dxGeom g)
	private long dGeomGetCategoryBits ()
	{
		return category_bits;
	}


	//	unsigned long dGeomGetCollideBits (dxGeom g)
	private long dGeomGetCollideBits ()
	{
		return collide_bits;
	}


	private void dGeomEnable ()
	{
		_gflags |= GEOM_ENABLED;
	}

	private void dGeomDisable ()
	{
		_gflags &= ~GEOM_ENABLED;
	}

	private boolean dGeomIsEnabled ()
	{
		return (_gflags & GEOM_ENABLED) != 0;// ? 1 : 0;
	}

	void dGeomGetRelPointPos (double px, double py, double pz,
	        DVector3 result) {
	    if ((_gflags & GEOM_PLACEABLE) == 0) {
	        result.set(px, py, pz);
	        return;
	    }

	    recomputePosr();

	    DVector3 prel,p = new DVector3();
	    prel = new DVector3(px, py, pz);
	    dMultiply0_331 (p,_final_posr.R(),prel);
	    result.eqSum(p, _final_posr.pos());
	}


	void dGeomGetPosRelPoint(double px, double py, double pz, DVector3 result) {
	    if ((_gflags & GEOM_PLACEABLE) == 0) {
	        result.set(px, py, pz);
	        return;
	    }

	    recomputePosr();

	    DVector3 prel = new DVector3(px, py, pz);
	    prel.sub(_final_posr.pos());
//	    prel[0] = px - g->final_posr->pos[0];
//	    prel[1] = py - g->final_posr->pos[1];
//	    prel[2] = pz - g->final_posr->pos[2];
//	    prel[3] = 0;
	    dMultiply1_331 (result,_final_posr.R(),prel);
	}


	void dGeomVectorToWorld(double px, double py, double pz, DVector3 result) {
	    if ((_gflags & GEOM_PLACEABLE) == 0) {
	        result.set(px, py, pz);
	        return;
	    }

	    recomputePosr();

	    DVector3 p = new DVector3(px, py, pz);
	    dMultiply0_331 (result,_final_posr.R(),p);
	}


	void dGeomVectorFromWorld(double px, double py, double pz, DVector3 result) {
	    if ((_gflags & GEOM_PLACEABLE) == 0) {
	        result.set(px, py, pz);
	        return;
	    }

	    recomputePosr();

	    DVector3 p = new DVector3(px, py, pz);
	    dMultiply1_331 (result,_final_posr.R(),p);
	}



//	@SuppressWarnings("deprecation")
//	boolean dGeomLowLevelControl(CONTROL_CLASS controlClass, CONTROL_CODE controlCode, 
//			DataValue dataValue, RefInt dataSize) {
//	    dAASSERT (dataSize);
//
//	    //if (!dataSize) {
//	    if (dataSize == null) {
//	        return false;
//	    }
//
//	    boolean result = controlGeometry(controlClass, controlCode, dataValue, dataSize);
//	    return result;
//	}


//	/** **************************************************************************
//	 * C interface that lets the user make new classes. this interface is a lot
//	 * more cumbersome than C++ subclassing, which is what is used internally
//	 * in ODE. this API is mainly to support legacy code.
//	 * @deprecated see above (TZ) TODO remove?
//	 */
//	private static dGeomClass[] user_classes = new dGeomClass[dMaxUserClasses];
//	private static int num_user_classes = 0;
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
		//TODO num_user_classes = 0;
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
		offset_posr.pos.setZero();//dSetZero (offset_posr.pos.v,4);
		offset_posr.Rw().setIdentity();

		_gflags |= GEOM_POSR_BAD;
	}

//	void dGeomSetOffsetPosition (dxGeom g, double x, double y, double z)
	public void dGeomSetOffsetPosition (double x, double y, double z)
	{
		//dAASSERT (g);
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
        DxSpace.CHECK_NOT_LOCKED (parent_space);
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
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset ();
		}
//		memcpy (g.offset_posr.R,R,sizeof(dMatrix3));
		offset_posr.Rw().set(R);
		dGeomMoved();
	}

	void dGeomSetOffsetQuaternion (DQuaternionC quat)
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset ();
		}
		dRfromQ (offset_posr.Rw(), quat);
		dGeomMoved();
	}

	void dGeomSetOffsetWorldPosition (double x, double y, double z)
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset();
		}
		body.dBodyGetPosRelPoint(new DVector3(x, y, z), offset_posr.pos);
		dGeomMoved();
	}

	void dGeomSetOffsetWorldRotation (DMatrix3C R)
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset ();
		}
		recomputePosr();

		DxPosR new_final_posr = new DxPosR();
//		memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
		new_final_posr.pos.set(_final_posr.pos());
//		memcpy(new_final_posr.R, R, sizeof(dMatrix3));
		new_final_posr.Rw().set(R);

		getWorldOffsetPosr(body.posr(), new_final_posr, offset_posr);
		dGeomMoved();
	}

	void dGeomSetOffsetWorldQuaternion (DQuaternionC quat)
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		dUASSERT (body, "geom must be on a body");
        DxSpace.CHECK_NOT_LOCKED (parent_space);
		if (offset_posr == null)
		{
			dGeomCreateOffset ();
		}

		recomputePosr();

		DxPosR new_final_posr = new DxPosR();
//		memcpy(new_final_posr.pos, g.final_posr.pos, sizeof(dVector3));
		new_final_posr.pos.set(_final_posr.pos());
		dRfromQ (new_final_posr.Rw(), quat);

		getWorldOffsetPosr(body.posr(), new_final_posr, offset_posr);
		dGeomMoved();
	}

	void dGeomClearOffset()
	{
		dUASSERT (_gflags & GEOM_PLACEABLE,"geom must be placeable");
		if (offset_posr != null)
		{
			//dIASSERT(body != null);
			// no longer need an offset posr
			dFreePosr(offset_posr);
			offset_posr = null;
			// the geom will now share the position of the body
			dFreePosr(_final_posr);
			_final_posr = body._posr;
			// geom has moved
			_gflags &= ~GEOM_POSR_BAD;
			dGeomMoved();
		}
	}

	boolean dGeomIsOffset()
	{
		return null != offset_posr;
	}

	private static final DVector3C OFFSET_POSITION_ZERO = new DVector3( 0.0f, 0.0f, 0.0f );

	//	final double * dGeomGetOffsetPosition (dxGeom g)
	private DVector3C dGeomGetOffsetPosition ()
	{
		if (offset_posr != null)
		{
			return offset_posr.pos();
		}
		return OFFSET_POSITION_ZERO;
	}

	private void dGeomCopyOffsetPosition (DVector3 pos)
	{
		pos.set(dGeomGetOffsetPosition());
	}

	private static final DMatrix3C OFFSET_ROTATION_ZERO = new DMatrix3(
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0);

	//double *
	private DMatrix3C dGeomGetOffsetRotation ()
	{
		if (offset_posr != null)
		{
			return offset_posr.R();
		}
		return OFFSET_ROTATION_ZERO;
	}

	private void dGeomCopyOffsetRotation (DMatrix3 R)
	{
		R.set(dGeomGetOffsetRotation());
	}

	void dGeomGetOffsetQuaternion (DQuaternion result)
	{
		if (offset_posr != null)
		{
			dQfromR(result, offset_posr.R());
		}
		else
		{
//			dSetZero (result.v,4);
//			result.v[0] = 1;
			result.set(1, 0, 0, 0);
		}
	}


	private DxPosR dAllocPosr() {
		return new DxPosR();
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
	@Deprecated
    private void dFreePosr(DxPosR oldPosR) {
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
		// if geom is offset, mark it as needing a calculate
		if (offset_posr != null) {
			_gflags |= GEOM_POSR_BAD;
		}

		// from the bottom of the space heirarchy up, process all clean geoms
		// turning them into dirty geoms.
		DxSpace parent = parent_space;

		DxGeom geom = this;
		while (parent != null && (geom._gflags & GEOM_DIRTY)==0) {
			geom.markAABBBad();
			parent.dirty (geom);
			geom = parent;
			parent = parent.parent_space;
		}

		// all the remaining dirty geoms must have their AABB_BAD flags set, to
		// ensure that their AABBs get recomputed
		while (geom != null) {
			geom.markAABBBad();
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
	private static final dColliderEntry[][] colliders = new dColliderEntry[dGeomNumClasses][dGeomNumClasses];
	private static boolean colliders_initialized = false;

	private static final boolean LIBCCD = OdeConfig.isLibCCDEndabled(); 
    private static final boolean dLIBCCD_BOX_CYL = LIBCCD;

    private static final boolean dLIBCCD_CYL_CYL = LIBCCD;

    private static final boolean dLIBCCD_CAP_CYL = LIBCCD;

    private static final boolean dLIBCCD_CONVEX_BOX = LIBCCD;

    private static final boolean dLIBCCD_CONVEX_CAP = LIBCCD;

    private static final boolean dLIBCCD_CONVEX_CYL = LIBCCD;

    private static final boolean dLIBCCD_CONVEX_SPHERE = LIBCCD;

    private static final boolean dLIBCCD_CONVEX_CONVEX = LIBCCD;


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


//	private static void setAllColliders (int i, DColliderFn fn)
//	{
//		for (int j=0; j<dGeomNumClasses; j++) setCollider (i,j,fn);
//	}

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
			setCollider (dTriMeshClass,dSphereClass, new CollideTrimeshSphere());//.  dCollideSTL);
			setCollider (dTriMeshClass,dBoxClass, new CollideTrimeshBox());// dCollideBTL);
			setCollider (dTriMeshClass,dRayClass, new CollideTrimeshRay());// dCollideRTL);
			setCollider (dTriMeshClass,dTriMeshClass, new CollideTrimeshTrimesh());// dCollideTTL);
			setCollider (dTriMeshClass,dCapsuleClass, new CollideTrimeshCCylinder());// dCollideCCTL);
			setCollider (dTriMeshClass,dPlaneClass, new CollideTrimeshPlane());// dCollideTrimeshPlane);
			setCollider (dCylinderClass,dTriMeshClass, new CollideCylinderTrimesh());// dCollideCylinderTrimesh);
			setCollider (dConvexClass,dTriMeshClass, new CollideConvexTrimesh());
//		}

		if (dLIBCCD_BOX_CYL) {//#ifdef dLIBCCD_BOX_CYL
		    setCollider (dBoxClass,dCylinderClass, new CollideBoxCylinderCCD());
		} else {//#else
            setCollider (dCylinderClass,dBoxClass, new CollideCylinderBox());//dCollideCylinderBox);
		} //#endif
		setCollider (dCylinderClass,dSphereClass, new CollideCylinderSphere());//dCollideCylinderSphere);
		setCollider (dCylinderClass,dPlaneClass, new CollideCylinderPlane());//dCollideCylinderPlane);

		if (dLIBCCD_CYL_CYL) {
		    setCollider (dCylinderClass, dCylinderClass, new CollideCylinderCylinder());
		}
		if (dLIBCCD_CAP_CYL) {
		    setCollider (dCapsuleClass, dCylinderClass, new CollideCapsuleCylinder());
		}

		//--> Convex Collision
		if (dLIBCCD_CONVEX_BOX) {
		    setCollider (dConvexClass, dBoxClass, new CollideConvexBoxCCD());
		} else {
		    setCollider (dConvexClass,dBoxClass, new DxConvex.CollideConvexBox());
		}

		if (dLIBCCD_CONVEX_CAP) {
		    setCollider (dConvexClass,dCapsuleClass, new CollideConvexCapsuleCCD());
		} else {
		    setCollider (dConvexClass,dCapsuleClass, new DxConvex.CollideConvexCapsule());
		}

		if (dLIBCCD_CONVEX_CYL) {
		    setCollider (dConvexClass,dCylinderClass, new CollideConvexCylinderCCD());
		}

		if (dLIBCCD_CONVEX_SPHERE) {
		    setCollider (dConvexClass,dSphereClass, new CollideConvexSphereCCD());
		} else {
		    setCollider (dSphereClass,dConvexClass,new DxConvex.CollideSphereConvex());
		}

		if (dLIBCCD_CONVEX_CONVEX) {
		    setCollider (dConvexClass,dConvexClass, new CollideConvexConvexCCD());
		} else {
		    setCollider (dConvexClass,dConvexClass, new DxConvex.CollideConvexConvex());
		}

		setCollider (dConvexClass,dPlaneClass, new DxConvex.CollideConvexPlane());
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
			setCollider (dHeightfieldClass,dTriMeshClass, new DxHeightfield.CollideHeightfield());//dCollideHeightfield);
//		}
//		//<-- dHeightfield Collision
	}

	/*extern */
	static void dFinitColliders()
	{
		colliders_initialized = false;
	}

	public static void dSetColliderOverride (int i, int j, DColliderFn fn)
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
	 * @param o1 o1
	 * @param o2 o2
	 * @param flags flags
	 * @param contacts contacts
	 * @param skip skip
	 * @return count
	 */
	public static int dCollide (DxGeom o1, DxGeom o2, int flags, 
			DContactGeomBuffer contacts, int skip)
	{
		dAASSERT(contacts);
		dUASSERT(colliders_initialized,
				"Please call ODE initialization (dInitODE() or similar) before using the library");
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
		int count = 0;
		if (ce.fn != null) {
			if (ce.reverse) {
				count = ce.fn.dColliderFn (o2,o1,flags,contacts);
				for (int i=0; i<count; i++) {
					//dContactGeom c = CONTACT(contact,skip*i);
					DContactGeom c = contacts.get(i);
//					c.normal.v[0] = -c.normal.v[0];
//					c.normal.v[1] = -c.normal.v[1];
//					c.normal.v[2] = -c.normal.v[2];
					c.normal.scale(-1);
					DGeom tmp = c.g1;
					c.g1 = c.g2;
					c.g2 = tmp;
					int tmpint = c.side1;
					c.side1 = c.side2;
					c.side2 = tmpint;
				}
			} else {
				count = ce.fn.dColliderFn (o1,o2,flags,contacts);
			}
		}
		//else System.out.println("Collider not found: " + o1.getClass() + " / " + o2.getClass());//TODO
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

	@Override
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
	
	DxPosRC final_posr() {
		return _final_posr;
	}
	
	DxPosRC offset_posr() {
		return offset_posr;
	}
	
	// *********************************************
	// dGeom API
	// *********************************************
	
	@Override
	public void destroy() {
		//if (_id!=null) dGeomDestroy (_id);
		//_id = null;
		dGeomDestroy();
	}

	@Override
	public int getClassID()// const
	{ return dGeomGetClass (); }

	@Override
	public DSpace getSpace() //const
	{ return dGeomGetSpace (); }

	@Override
	public void setData (Object data)
	{ dGeomSetData (data); }
	@Override
	public Object getData() //const
	{ return dGeomGetData (); }

	@Override
	public void setBody (DBody b)
	{ dGeomSetBody ((DxBody)b); }
	@Override
	public DBody getBody() //const
	{ return dGeomGetBody (); }

	@Override
	public void setPosition (double x, double y, double z)
	{ dGeomSetPosition (new DVector3(x,y,z)); }
	@Override
	public void setPosition (DVector3C xyz)
	{ dGeomSetPosition (xyz); }
	//const dReal * getPosition() const
	@Override
	public DVector3C getPosition()
	{ return dGeomGetPosition (); }

	@Override
	public void setRotation (DMatrix3C R)
	{ dGeomSetRotation (R); }
	//const dReal * getRotation() const
	@Override
	public DMatrix3C getRotation()
	{ return dGeomGetRotation (); }

	@Override
	public void setQuaternion (DQuaternionC quat)
	{ dGeomSetQuaternion (quat); }

	public boolean isSpace()
	{ return this instanceof DSpace; }

	@Override
	public void setCategoryBits (long bits)//unsigned long bits)
	{ dGeomSetCategoryBits (bits); }
	@Override
	public void setCollideBits (long bits)//unsigned long bits)
	{ dGeomSetCollideBits (bits); }
	//unsigned 
	@Override
	public long getCategoryBits()
	{ return dGeomGetCategoryBits (); }
	//unsigned 
	@Override
	public long getCollideBits()
	{ return dGeomGetCollideBits (); }

	@Override
	public void enable()
	{ dGeomEnable (); }
	@Override
	public void disable()
	{ dGeomDisable (); }
	@Override
	public boolean isEnabled()
	{ return dGeomIsEnabled (); }

	@Override
	public void collide2 (DGeom g, Object data, DNearCallback callback)
	{ DxSpace.dSpaceCollide2 (this,(DxGeom)g,data,callback); }

	@Override
	public void setOffsetPosition(double x, double y, double z) {
		dGeomSetOffsetPosition(x, y, z);
	}

	@Override
	public void setOffsetPosition(DVector3C xyz) {
		dGeomSetOffsetPosition(xyz.get0(), xyz.get1(), xyz.get2());
	}

	@Override
	public void setOffsetRotation(DMatrix3C R) {
		dGeomSetOffsetRotation(R);
	}



	@Override
	public void clearOffset() {
		dGeomClearOffset();
	}


	@Override
	public DVector3C getOffsetPosition() {
		return dGeomGetOffsetPosition();
	}


	@Override
	public void getOffsetQuaternion(DQuaternion result) {
		dGeomGetOffsetQuaternion(result);
	}


	@Override
	public DMatrix3C getOffsetRotation() {
		return dGeomGetOffsetRotation();
	}


	@Override
	public boolean isOffset() {
		return dGeomIsOffset();
	}


	@Override
	public void setOffsetQuaternion(DQuaternionC q) {
		dGeomSetOffsetQuaternion(q);
	}


	@Override
	public void setOffsetWorldPosition(double x, double y, double z) {
		dGeomSetOffsetWorldPosition(x, y, z);
	}


	@Override
	public void setOffsetWorldQuaternion(DQuaternionC q) {
		dGeomSetOffsetWorldQuaternion(q);
	}


	@Override
	public void setOffsetWorldRotation(DMatrix3C R) {
		dGeomSetOffsetRotation(R);
	}
	
	@Override
	public void copyOffsetPosition(DVector3 pos) {
		dGeomCopyOffsetPosition(pos);
	}

	@Override
	public void copyOffsetRotation(DMatrix3 R) {
		dGeomCopyOffsetRotation(R);
	}

	@Override
	public void copyPosition(DVector3 pos) {
		dGeomCopyPosition(pos);
	}

	@Override
	public void copyRotation(DMatrix3 R) {
		dGeomCopyRotation(R);
	}

//    @SuppressWarnings("deprecation")
//	@Override
//    public boolean lowLevelControl(CONTROL_CLASS controlClass,
//            CONTROL_CODE controlCode, DataValue dataValue, RefInt dataSize) {
//        return dGeomLowLevelControl(controlClass, controlCode, dataValue, dataSize);
//    }

    @Override
    public void getRelPointPos(double px, double py, double pz, DVector3 result) {
        dGeomGetRelPointPos(px, py, pz, result);
    }

    @Override
    public void getPosRelPoint(double px, double py, double pz, DVector3 result) {
        dGeomGetPosRelPoint(px, py, pz, result);
    }

    @Override
    public void vectorToWorld(double px, double py, double pz, DVector3 result) {
        dGeomVectorToWorld(px, py, pz, result);
    }

    @Override
    public void vectorFromWorld(double px, double py, double pz, DVector3 result) {
        dGeomVectorFromWorld(px, py, pz, result);
    }
	
	
}
