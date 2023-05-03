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

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.*;

import java.util.ArrayList;
import java.util.List;

import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DQuadTreeSpace;
import org.ode4j.ode.internal.cpp4j.java.ObjArray;
import org.ode4j.ode.internal.cpp4j.java.RefInt;



/**
 * QuadTreeSpace by Erwin de Vries.
 * 
 * From collision_quadtreespace.cpp
 */
public class DxQuadTreeSpace extends DxSpace implements DQuadTreeSpace {


	//#define AXIS0 0
	//#define AXIS1 1
	//#define UP 2
	private static final int AXIS0 = 0;
	private static final int AXIS1 = 1;
//	private static final int UP = 2;

	//#define DRAWBLOCKS
	private static final boolean DRAWBLOCKS = false; 

	//	const int SPLITAXIS = 2;
	//	const int SPLITS = SPLITAXIS * SPLITAXIS;
	private static final int SPLITAXIS = 2;
	private static final int SPLITS = SPLITAXIS * SPLITAXIS;

	//TZ this is also in dxGeom defined.
	//#define GEOM_ENABLED(g) (((g)->gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE)
	//	private boolean GEOM_ENABLED(dxGeom g) {
	//		return (g.gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE;
	//	}

	class Block {
		//public:
		double mMinX, mMaxX;
		double mMinZ, mMaxZ;

		DxGeom mFirst;
		int mGeomCount;

		//	Block* Parent;
		//	Block* Children;
		Block mParent;
		ObjArray<Block> mChildren;

		//#ifdef DRAWBLOCKS
		//#include "..\..\Include\drawstuff\\drawstuff.h"

		//static void DrawBlock(Block* Block){
		void DrawBlock(Block Block){
			throw new UnsupportedOperationException();
			//TODO?
			//	if (!DRAWBLOCKS) return;
			//	dVector3 v[8];
			//	v[0][AXIS0] = Block->mMinX;
			//	v[0][UP] = REAL(-1.0);
			//	v[0][AXIS1] = Block->mMinZ;
			//	
			//	v[1][AXIS0] = Block->mMinX;
			//	v[1][UP] = REAL(-1.0);
			//	v[1][AXIS1] = Block->mMaxZ;
			//	
			//	v[2][AXIS0] = Block->mMaxX;
			//	v[2][UP] = REAL(-1.0);
			//	v[2][AXIS1] = Block->mMinZ;
			//	
			//	v[3][AXIS0] = Block->mMaxX;
			//	v[3][UP] = REAL(-1.0);
			//	v[3][AXIS1] = Block->mMaxZ;
			//	
			//	v[4][AXIS0] = Block->mMinX;
			//	v[4][UP] = REAL(1.0);
			//	v[4][AXIS1] = Block->mMinZ;
			//	
			//	v[5][AXIS0] = Block->mMinX;
			//	v[5][UP] = REAL(1.0);
			//	v[5][AXIS1] = Block->mMaxZ;
			//	
			//	v[6][AXIS0] = Block->mMaxX;
			//	v[6][UP] = REAL(1.0);
			//	v[6][AXIS1] = Block->mMinZ;
			//	
			//	v[7][AXIS0] = Block->mMaxX;
			//	v[7][UP] = REAL(1.0);
			//	v[7][AXIS1] = Block->mMaxZ;
			//	
			//	// Bottom
			//	dsDrawLine(v[0], v[1]);
			//	dsDrawLine(v[1], v[3]);
			//	dsDrawLine(v[3], v[2]);
			//	dsDrawLine(v[2], v[0]);
			//	
			//	// Top
			//	dsDrawLine(v[4], v[5]);
			//	dsDrawLine(v[5], v[7]);
			//	dsDrawLine(v[7], v[6]);
			//	dsDrawLine(v[6], v[4]);
			//	
			//	// Sides
			//	dsDrawLine(v[0], v[4]);
			//	dsDrawLine(v[1], v[5]);
			//	dsDrawLine(v[2], v[6]);
			//	dsDrawLine(v[3], v[7]);
		}
		//#endif	//DRAWBLOCKS


		//void Block::Create(const dVector3 Center, const dVector3 Extents, 
		//Block* Parent, int Depth, Block*& Blocks){
		void Create(final double MinX, final double MaxX,
		        final double MinZ, final double MaxZ,
				Block Parent, int Depth, Block[] BlocksA, RefInt BlocksP){
		    dIASSERT(MinX <= MaxX);
		    dIASSERT(MinZ <= MaxZ);
			mGeomCount = 0;
			mFirst = null;

			mMinX = MinX;
			mMaxX = MaxX;
			
			mMinZ = MinZ;
			mMaxZ = MaxZ;
			
			this.mParent = Parent;
			if (Depth > 0){
				mChildren = new ObjArray<Block>(Blocks, BlocksP.get());
				BlocksP.add( SPLITS );

		        final double ChildExtentX = (MaxX - MinX) / SPLITAXIS;
		        final double ChildExtentZ = (MaxZ - MinZ) / SPLITAXIS;

		        final int ChildDepth = Depth - 1;
		        int Index = 0;

		        double ChildRightX = MinX;
				for (int i = 0; i < SPLITAXIS; i++){
		            final double ChildLeftX = ChildRightX;
		            ChildRightX = (i != SPLITAXIS - 1) ? ChildLeftX + ChildExtentX : MaxX;

		            double ChildRightZ = MinZ;
		            for (int j = 0; j < SPLITAXIS; j++){
		                final double ChildLeftZ = ChildRightZ;
		                ChildRightZ = (j != SPLITAXIS - 1) ? ChildLeftZ + ChildExtentZ : MaxZ;

		                mChildren.at(Index).Create(ChildLeftX, ChildRightX, ChildLeftZ, ChildRightZ, this, ChildDepth, BlocksA, BlocksP);
		                ++Index;
		            }
				}
			}
			else mChildren = null;
		}

		//void Block::Collide(void* UserData, dNearCallback* Callback){
		void Collide(Object UserData, DNearCallback Callback){
			if (DRAWBLOCKS) {//#ifdef DRAWBLOCKS
				DrawBlock(this);
			}//#endif
			// Collide the local list
			DxGeom g = mFirst;
			while (g != null){
				if (GEOM_ENABLED(g)){
					Collide(g, g.getNextEx(), UserData, Callback);
				}
				g = g.getNextEx();
			}

			// Recurse for children
			if (mChildren!=null){
				for (int i = 0; i < SPLITS; i++){
	                Block CurrentChild = mChildren.at(i);
	                if (CurrentChild.mGeomCount <= 1){  // Early out
	                    continue;
	                }
	                CurrentChild.Collide(UserData, Callback);
				}
			}
		}

		// Note: g2 is assumed to be in this Block
		//void Block::Collide(dxGeom* g1, dxGeom* g2, void* UserData, 
		//dNearCallback* Callback){
		void Collide(DxGeom g1, DxGeom g2, Object UserData, 
				DNearCallback Callback){
			if (DRAWBLOCKS) {//#ifdef DRAWBLOCKS
				DrawBlock(this);
			}//#endif
			// Collide against local list
			while (g2!=null){
				if (GEOM_ENABLED(g2)){
					collideAABBs (g1, g2, UserData, Callback);
				}
				g2 = g2.getNextEx();
			}

			// Collide against children
			if (mChildren!=null){
				for (int i = 0; i < SPLITS; i++){
		            Block CurrentChild = mChildren.at(i);
					// Early out for empty blocks
					if (CurrentChild.mGeomCount == 0){
						continue;
					}

					// Does the geom's AABB collide with the block?
					// Don't do AABB tests for single geom blocks.
					if (CurrentChild.mGeomCount == 1){
						//
					}
					else if (true){
//						if (g1._aabb.get(AXIS0 * 2 + 0) > Children.at(i).MaxX ||
//								g1._aabb.get(AXIS0 * 2 + 1) < Children.at(i).MinX ||
//								g1._aabb.get(AXIS1 * 2 + 0) > Children.at(i).MaxZ ||
//								g1._aabb.get(AXIS1 * 2 + 1) < Children.at(i).MinZ) continue;
						if (g1._aabb.getMin(AXIS0) >= CurrentChild.mMaxX ||
								g1._aabb.getMax(AXIS0) < CurrentChild.mMinX ||
								g1._aabb.getMin(AXIS1) >= CurrentChild.mMaxZ ||
								g1._aabb.getMax(AXIS1) < CurrentChild.mMinZ) continue;
					}
					CurrentChild.Collide(g1, CurrentChild.mFirst, UserData, Callback);
				}
			}
		}

		//void Block::CollideLocal(dxGeom* g2, void* UserData, 
		//dNearCallback* Callback){
		void CollideLocal(DxGeom g2, Object userData, 
				DNearCallback callback){
			// Collide against local list
			DxGeom g1 = mFirst;
			while (g1!=null){
				if (GEOM_ENABLED(g1)){
					collideAABBs (g1, g2, userData, callback);
				}
				g1 = g1.getNextEx();
			}
		}

		//void Block::AddObject(dGeom Object){
		void AddObject(DxGeom aObject){
			// Add the geom
			aObject.setNextEx( mFirst );
			mFirst = aObject;
			//XXX TZ aObject.tome = (dxGeom**)this;
			aObject._qtIdxEx = this;

			// Now traverse upwards to tell that we have a geom
			Block Block = this;
			do{
				Block.mGeomCount++;
				Block = Block.mParent;
			}
			while (Block!=null);
		}

		//void Block::DelObject(dGeom Object){
		void DelObject(DxGeom aObject){
			// Del the geom
			DxGeom Last = null, g = mFirst;
			boolean Found = false;

			if (g == aObject){
				mFirst = g.getNextEx();
				Found = true;
			}
			else {
				Last = g;
				g = g.getNextEx();
			}

			for (; !Found && g != null; Found = false){
				if (g == aObject){
					Last.setNextEx( g.getNextEx() );
					break;
				}
				Last = g;
				g = g.getNextEx();
			}

			//XXX TZ aObject.tome = null;
			aObject._qtIdxEx = null;
			// dUASSERT((aObject.getNext() = 0, true), "Needed for an assertion check only");
			aObject.setNextEx(null);
			dUASSERT(true, "Needed for an assertion check only");

			// Now traverse upwards to tell that we have lost a geom
			Block Block = this;
			do{
				Block.mGeomCount--;
				Block = Block.mParent;
			}
			while (Block!=null);
		}

		//void Block::Traverse(dGeom Object){
		void Traverse(DxGeom aObject){
			Block NewBlock = GetBlock(aObject._aabb);

			if (NewBlock != this){
				// Remove the geom from the old block and add it to the new block.
				// This could be more optimal, but the loss should be very small.
				DelObject(aObject);
				NewBlock.AddObject(aObject);
			}
		}

		//bool Block::Inside(const dReal* AABB){
		boolean Inside(DAABBC AABB){
//			return AABB.get(AXIS0 * 2 + 0) >= MinX 
//			&& AABB.get(AXIS0 * 2 + 1) <= MaxX 
//			&& AABB.get(AXIS1 * 2 + 0) >= MinZ 
//			&& AABB.get(AXIS1 * 2 + 1) <= MaxZ;
			return AABB.getMin(AXIS0) >= mMinX 
			&& AABB.getMax(AXIS0) < mMaxX 
			&& AABB.getMin(AXIS1) >= mMinZ 
			&& AABB.getMax(AXIS1) < mMaxZ;
		}

		//Block* Block::GetBlock(const dReal* AABB){
		Block GetBlock(DAABBC AABB){
			if (Inside(AABB)){
				return GetBlockChild(AABB);	// Child or this will have a good block
			}
			else if (mParent!=null){
				return mParent.GetBlock(AABB);	// Parent has a good block
			}
			else return this;	// We are at the root, so we have little choice
		}

		//Block* Block::GetBlockChild(final double[] AABB){
		Block GetBlockChild(DAABBC AABB){
			if (mChildren!=null){
				for (int i = 0; i < SPLITS; i++){
		            Block CurrentChild = mChildren.at(i);
					if (CurrentChild.Inside(AABB)){
						return CurrentChild.GetBlockChild(AABB);	// Child will have good block
					}
				}
			}
			return this;	// This is the best block
		}
	}

	//****************************************************************************
	// quadtree space

	//struct dxQuadTreeSpace : public dxSpace{
	private Block[] Blocks;//Block* Blocks;	// Blocks[0] is the root

	private List<DxGeom> DirtyList = new ArrayList<DxGeom>();//dArray<dxGeom*> DirtyList;

	//	dxQuadTreeSpace(dSpace _space, dVector3 Center, dVector3 Extents, int Depth);
	//	~dxQuadTreeSpace();
	//
	//	dxGeom* getGeom(int i);
	//	
	//	void add(dxGeom* g);
	//	void remove(dxGeom* g);
	//	void dirty(dxGeom* g);
	//
	//	void computeAABB();
	//	
	//	void cleanGeoms();
	//	void collide(void* UserData, dNearCallback* Callback);
	//	void collide2(void* UserData, dxGeom* g1, dNearCallback* Callback);

	// Temp data
	//	Block* CurrentBlock;	// Only used while enumerating
	//	int* CurrentChild;	// Only used while enumerating
	//	int CurrentLevel;	// Only used while enumerating
	//	dxGeom* CurrentObject;	// Only used while enumerating
	//	int CurrentIndex;
	Block CurrentBlock;	// Only used while enumerating
	int[] CurrentChild;	// Only used while enumerating
	int CurrentLevel;	// Only used while enumerating
	DxGeom CurrentObject;	// Only used while enumerating
	int CurrentIndex;
	//};

	private int numNodes(int depth) 
    {
        // A 4-ary tree has (4^(depth+1) - 1)/3 nodes
        // Note: split up into multiple constant expressions for readability
        final int k = depth+1;
        final int fourToNthPlusOne = 1 << (2*k); // 4^k = 2^(2k)
        return (fourToNthPlusOne - 1) / 3;
    }

	
	//dxQuadTreeSpace::dxQuadTreeSpace(dSpace _space, 
	//dVector3 Center, dVector3 Extents, int Depth) : dxSpace(_space){
	DxQuadTreeSpace(DxSpace _space, 
			DVector3C Center, DVector3C Extents, int Depth) {
		super(_space);
		type = dQuadTreeSpaceClass;

		int BlockCount = numNodes(Depth);

		this.Blocks = new Block[BlockCount];//(Block*)dAlloc(BlockCount * sizeof(Block));
		for (int i = 0; i < this.Blocks.length; i++) Blocks[i] = new Block(); 
		//Block* Blocks = this.Blocks + 1;	// This pointer gets modified!
		Block[] BlocksA = this.Blocks;	// This pointer gets modified!
		RefInt BlocksP = new RefInt(1);

	    double MinX = Center.get(AXIS0) - Extents.get(AXIS0);
	    double MaxX = dNextAfter((Center.get(AXIS0) + Extents.get(AXIS0)), dInfinity);
	    double MinZ = Center.get(AXIS1) - Extents.get(AXIS1);
	    double MaxZ = dNextAfter((Center.get(AXIS1) + Extents.get(AXIS1)), dInfinity);
	    this.Blocks[0].Create(MinX, MaxX, MinZ, MaxZ, null, Depth, BlocksA, BlocksP);
	    
		CurrentBlock = null;
		CurrentChild = new int[Depth+1];//(int*)dAlloc((Depth + 1) * sizeof(int));
		CurrentLevel = 0;
		CurrentObject = null;
		CurrentIndex = -1;

		// Init AABB. We initialize to infinity because it is not illegal for an object to be outside of the tree. Its simply inserted in the root block
//		_aabb.v[0] = -dInfinity;
//		_aabb.v[1] = dInfinity;
//		_aabb.v[2] = -dInfinity;
//		_aabb.v[3] = dInfinity;
//		_aabb.v[4] = -dInfinity;
//		_aabb.v[5] = dInfinity;
		_aabb.set( -dInfinity, dInfinity, -dInfinity, dInfinity, -dInfinity, dInfinity);
	}

	//dxQuadTreeSpace::~dxQuadTreeSpace(){
	@Override
	public void DESTRUCTOR(){
		//(TZ) This really does nothing useful in Java...
//		int Depth = 0;
//		//Block* Current = &Blocks[0];
//		Block Current = Blocks[0];
//		while (Current!=null && Current.mChildren != null){
//			Depth++;
//			Current = Current.mChildren.at(0);//Current.Children;
//		}

		///TODO call DESTRUCTORS (?, if any)
		// int BlockCount = numNode(Depth);
		//
		//	dFree(Blocks, BlockCount);// * sizeof(Block));
		//	dFree(CurrentChild, (Depth + 1));// * sizeof(int));

		super.DESTRUCTOR();
	}

	//void dxQuadTreeSpace::add(dxGeom* g){
	@Override
	void add(DxGeom g){
		CHECK_NOT_LOCKED (this);
		//Common.dAASSERT(g);
		dUASSERT(g._qtIdxEx == null && g.getNextEx() == null,
		"geom is already in a space");

		DirtyList.add(g);

		// add
		Blocks[0].GetBlock(g._aabb).AddObject(g);	// Add to best block

		super.add(g);
	}

	//void dxQuadTreeSpace::remove(dxGeom* g){
	@Override
	void remove(DxGeom g){
		CHECK_NOT_LOCKED(this);
		//Common.dAASSERT(g);
		dUASSERT(g.parent_space == this,"object is not in this space");

		// remove
		//TZ XXX ((Block*)g.tome).DelObject(g);
		g._qtIdxEx.DelObject(g);

		for (int i = 0; i < DirtyList.size(); i++){
			if (DirtyList.get(i) == g){
				DirtyList.remove(i);
				// (mg) there can be multiple instances of a dirty object on stack  be sure to remove ALL and not just first, for this we decrement i
				--i;
			}
		}
		
		super.remove(g);
	}

	//void dxQuadTreeSpace::dirty(dxGeom* g){
	@Override
	void dirty(DxGeom g){
		DirtyList.add(g);
	}

	//void dxQuadTreeSpace::computeAABB(){
	@Override
    protected void computeAABB(){
		//
	}

	//void dxQuadTreeSpace::cleanGeoms(){
	@Override
	public void cleanGeoms(){
		// compute the AABBs of all dirty geoms, and clear the dirty flags
		lock_count++;

		for (int i = 0; i < DirtyList.size(); i++){
			DxGeom g = DirtyList.get(i);
			if (g instanceof DxSpace){
				((DxSpace)g).cleanGeoms();
			}

			g.recomputeAABB();

			dIASSERT(!g.hasFlagAabbBad());

			//g->gflags &= ~GEOM_DIRTY;
			g.unsetFlagDirty();

			//TZ XXX ((Block)g.tome).Traverse(g);
			g._qtIdxEx.Traverse(g);
		}
		DirtyList.clear();

		lock_count--;
	}

	//void dxQuadTreeSpace::collide(void* UserData, dNearCallback* Callback){
	@Override
	public void collide(Object UserData, DNearCallback Callback){
		dAASSERT(Callback);

		lock_count++;
		cleanGeoms();

		Blocks[0].Collide(UserData, Callback);

		lock_count--;
	}


	//struct DataCallback {
	//    void *data;
	//    dNearCallback *callback;
	//};
	private static class DataCallback {
		Object data;
		DNearCallback callback;
		public DataCallback(Object userData, DNearCallback callback2) {
			data = userData;
			callback = callback2;
		}
	}

	private DNearCallback swap_callback = new DNearCallback() {
		// Invokes the callback with arguments swapped
		//static void swap_callback(void *data, dxGeom *g1, dxGeom *g2)
		void swap_callback(DataCallback data, DGeom g1, DGeom g2)
		{
			//DataCallback *dc = (DataCallback*)data;
			//data.callback(data.data, g2, g1);
			data.callback.call(data.data, g2, g1);
		}

		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			//TODO why is this cast not necessary????
			//TODO why is this cast not necessary????
			//TODO why is this cast not necessary????
			//TODO why is this cast not necessary????
			//TODO why is this cast not necessary????
			//TODO why is this cast not necessary????
			swap_callback((DataCallback)data, o1, o2);
		}
	};


	//void dxQuadTreeSpace::collide2(void* UserData, dxGeom* g2, 
	//dNearCallback* Callback){
	@Override
	void collide2(Object UserData, DxGeom g2, DNearCallback Callback){
		dAASSERT(Callback);

		lock_count++;
		cleanGeoms();
		g2.recomputeAABB();

		if (g2.parent_space == this){
			// The block the geom is in
			//XXX TZ Block* CurrentBlock = (Block*)g2.tome;
			Block CurrentBlock = g2._qtIdxEx;

			// Collide against block and its children
			DataCallback dc = new DataCallback(UserData, Callback);
			CurrentBlock.Collide(g2, CurrentBlock.mFirst, dc, swap_callback);

			// Collide against parents
			while ((CurrentBlock = CurrentBlock.mParent) != null)
				CurrentBlock.CollideLocal(g2, UserData, Callback);

		}
		else {
			DataCallback dc = new DataCallback(UserData, Callback);
			Blocks[0].Collide(g2, Blocks[0].mFirst, dc, swap_callback);
		}

		lock_count--;
	}

	//dSpace dQuadTreeSpaceCreate(dxSpace* space, dVector3 Center, 
	//dVector3 Extents, int Depth){
	public static DxQuadTreeSpace dQuadTreeSpaceCreate(DxSpace space, DVector3C Center, 
			DVector3C Extents, int Depth){
		return new DxQuadTreeSpace(space, Center, Extents, Depth);
	}
}
