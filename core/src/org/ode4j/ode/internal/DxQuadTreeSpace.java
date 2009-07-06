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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DQuadTreeSpace;
import org.cpp4j.java.ObjArray;
import org.cpp4j.java.RefInt;

import static org.cpp4j.C_All.*;
import static org.ode4j.ode.internal.Common.*;


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
	private static final int UP = 2;

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
		double MinX, MaxX;
		double MinZ, MaxZ;

		DxGeom First;
		int GeomCount;

		//	Block* Parent;
		//	Block* Children;
		Block Parent;
		ObjArray<Block> Children;

		//#ifdef DRAWBLOCKS
		//#include "..\..\Include\drawstuff\\drawstuff.h"

		//static void DrawBlock(Block* Block){
		void DrawBlock(Block Block){
			throw new UnsupportedOperationException();
			//TODO?
			//	if (!DRAWBLOCKS) return;
			//	dVector3 v[8];
			//	v[0][AXIS0] = Block->MinX;
			//	v[0][UP] = REAL(-1.0);
			//	v[0][AXIS1] = Block->MinZ;
			//	
			//	v[1][AXIS0] = Block->MinX;
			//	v[1][UP] = REAL(-1.0);
			//	v[1][AXIS1] = Block->MaxZ;
			//	
			//	v[2][AXIS0] = Block->MaxX;
			//	v[2][UP] = REAL(-1.0);
			//	v[2][AXIS1] = Block->MinZ;
			//	
			//	v[3][AXIS0] = Block->MaxX;
			//	v[3][UP] = REAL(-1.0);
			//	v[3][AXIS1] = Block->MaxZ;
			//	
			//	v[4][AXIS0] = Block->MinX;
			//	v[4][UP] = REAL(1.0);
			//	v[4][AXIS1] = Block->MinZ;
			//	
			//	v[5][AXIS0] = Block->MinX;
			//	v[5][UP] = REAL(1.0);
			//	v[5][AXIS1] = Block->MaxZ;
			//	
			//	v[6][AXIS0] = Block->MaxX;
			//	v[6][UP] = REAL(1.0);
			//	v[6][AXIS1] = Block->MinZ;
			//	
			//	v[7][AXIS0] = Block->MaxX;
			//	v[7][UP] = REAL(1.0);
			//	v[7][AXIS1] = Block->MaxZ;
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
		void Create(DVector3C Center, DVector3C Extents, 
				Block Parent, int Depth, Block[] BlocksA, RefInt BlocksP){
			GeomCount = 0;
			First = null;

			MinX = Center.get(AXIS0) - Extents.get(AXIS0);
			MaxX = Center.get(AXIS0) + Extents.get(AXIS0);

			MinZ = Center.get(AXIS1) - Extents.get(AXIS1);
			MaxZ = Center.get(AXIS1) + Extents.get(AXIS1);

			this.Parent = Parent;
			if (Depth > 0){
				Children = new ObjArray<Block>(Blocks, BlocksP.get());
				BlocksP.add( SPLITS );

				DVector3 ChildExtents = new DVector3();
				ChildExtents.set(AXIS0, Extents.get(AXIS0) / SPLITAXIS );
				ChildExtents.set(AXIS1, Extents.get(AXIS1) / SPLITAXIS );
				ChildExtents.set(UP, Extents.get(UP) );

				for (int i = 0; i < SPLITAXIS; i++){
					for (int j = 0; j < SPLITAXIS; j++){
						int Index = i * SPLITAXIS + j;

						DVector3 ChildCenter = new DVector3();
						ChildCenter.set(AXIS0, Center.get(AXIS0) - Extents.get(AXIS0) + ChildExtents.get(AXIS0) + i * (ChildExtents.get(AXIS0) * 2) );
						ChildCenter.set(AXIS1, Center.get(AXIS1) - Extents.get(AXIS1) + ChildExtents.get(AXIS1) + j * (ChildExtents.get(AXIS1) * 2) );
						ChildCenter.set(UP, Center.get(UP) );

						Children.at(Index).Create(ChildCenter, ChildExtents, this, Depth - 1, BlocksA, BlocksP);
					}
				}
			}
			else Children = null;
		}

		//void Block::Collide(void* UserData, dNearCallback* Callback){
		void Collide(Object UserData, DNearCallback Callback){
			if (DRAWBLOCKS) {//#ifdef DRAWBLOCKS
				DrawBlock(this);
			}//#endif
			// Collide the local list
			DxGeom g = First;
			while (g != null){
				if (GEOM_ENABLED(g)){
					Collide(g, g.getNext(), UserData, Callback);
				}
				g = g.getNext();
			}

			// Recurse for children
			if (Children!=null){
				for (int i = 0; i < SPLITS; i++){
					if (Children.at(i).GeomCount <= 1){	// Early out
						continue;
					}
					Children.at(i).Collide(UserData, Callback);
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
				g2 = g2.getNext();
			}

			// Collide against children
			if (Children!=null){
				for (int i = 0; i < SPLITS; i++){
					// Early out for empty blocks
					if (Children.at(i).GeomCount == 0){
						continue;
					}

					// Does the geom's AABB collide with the block?
					// Dont do AABB tests for single geom blocks.
					if (Children.at(i).GeomCount == 1 && Children.at(i).First!=null){
						//
					}
					else if (true){
//						if (g1._aabb.get(AXIS0 * 2 + 0) > Children.at(i).MaxX ||
//								g1._aabb.get(AXIS0 * 2 + 1) < Children.at(i).MinX ||
//								g1._aabb.get(AXIS1 * 2 + 0) > Children.at(i).MaxZ ||
//								g1._aabb.get(AXIS1 * 2 + 1) < Children.at(i).MinZ) continue;
						if (g1._aabb.getMin(AXIS0) > Children.at(i).MaxX ||
								g1._aabb.getMax(AXIS0) < Children.at(i).MinX ||
								g1._aabb.getMin(AXIS1) > Children.at(i).MaxZ ||
								g1._aabb.getMax(AXIS1) < Children.at(i).MinZ) continue;
					}
					Children.at(i).Collide(g1, Children.at(i).First, UserData, Callback);
				}
			}
		}

		//void Block::CollideLocal(dxGeom* g2, void* UserData, 
		//dNearCallback* Callback){
		void CollideLocal(DxGeom g2, Object userData, 
				DNearCallback callback){
			// Collide against local list
			DxGeom g1 = First;
			while (g1!=null){
				if (GEOM_ENABLED(g1)){
					collideAABBs (g1, g2, userData, callback);
				}
				g1 = g1.getNext();
			}
		}

		//void Block::AddObject(dGeom Object){
		void AddObject(DxGeom aObject){
			// Add the geom
			aObject._next = First;
			First = aObject;
			//XXX TZ aObject.tome = (dxGeom**)this;
			aObject._qtIdx = this;

			// Now traverse upwards to tell that we have a geom
			Block Block = this;
			do{
				Block.GeomCount++;
				Block = Block.Parent;
			}
			while (Block!=null);
		}

		//void Block::DelObject(dGeom Object){
		void DelObject(DxGeom aObject){
			// Del the geom
			DxGeom g = First;
			DxGeom Last = null;
			while (g!=null){
				if (g == aObject){
					if (Last!=null){
						Last._next = g._next;
					}
					else First = g.getNext();

					break;
				}
				Last = g;
				g = g.getNext();
			}

			//XXX TZ aObject.tome = null;
			aObject._qtIdx = null;
			

			// Now traverse upwards to tell that we have lost a geom
			Block Block = this;
			do{
				Block.GeomCount--;
				Block = Block.Parent;
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
			return AABB.getMin(AXIS0) >= MinX 
			&& AABB.getMax(AXIS0) <= MaxX 
			&& AABB.getMin(AXIS1) >= MinZ 
			&& AABB.getMax(AXIS1) <= MaxZ;
		}

		//Block* Block::GetBlock(const dReal* AABB){
		Block GetBlock(DAABBC AABB){
			if (Inside(AABB)){
				return GetBlockChild(AABB);	// Child or this will have a good block
			}
			else if (Parent!=null){
				return Parent.GetBlock(AABB);	// Parent has a good block
			}
			else return this;	// We are at the root, so we have little choice
		}

		//Block* Block::GetBlockChild(final double[] AABB){
		Block GetBlockChild(DAABBC AABB){
			if (Children!=null){
				for (int i = 0; i < SPLITS; i++){
					if (Children.at(i).Inside(AABB)){
						return Children.at(i).GetBlockChild(AABB);	// Child will have good block
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

	private DArray<DxGeom> DirtyList = new DArray<DxGeom>();//dArray<dxGeom*> DirtyList;

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

	//dxQuadTreeSpace::dxQuadTreeSpace(dSpace _space, 
	//dVector3 Center, dVector3 Extents, int Depth) : dxSpace(_space){
	DxQuadTreeSpace(DxSpace _space, 
			DVector3C Center, DVector3C Extents, int Depth) {
		super(_space);
		type = dQuadTreeSpaceClass;

		int BlockCount = 0;
		// TODO: should be just BlockCount = (4^(n+1) - 1)/3
		for (int i = 0; i <= Depth; i++){
			BlockCount += (int)pow((double)SPLITS, i);
		}

		this.Blocks = new Block[BlockCount];//(Block*)dAlloc(BlockCount * sizeof(Block));
		for (int i = 0; i < this.Blocks.length; i++) Blocks[i] = new Block(); 
		//Block* Blocks = this.Blocks + 1;	// This pointer gets modified!
		Block[] BlocksA = this.Blocks;	// This pointer gets modified!
		RefInt BlocksP = new RefInt(1);

		this.Blocks[0].Create(Center, Extents, null, Depth, BlocksA, BlocksP);

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
		int Depth = 0;
		//Block* Current = &Blocks[0];
		Block Current = Blocks[0];
		while (Current!=null && Current.Children != null){
			Depth++;
			Current = Current.Children.at(0);//Current.Children;
		}

		int BlockCount = 0;
		for (int i = 0; i < Depth; i++){
			BlockCount += (int)pow((double)SPLITS, i);
		}

		///TODO call DESTRUCTORS (?, if any)
		//	dFree(Blocks, BlockCount);// * sizeof(Block));
		//	dFree(CurrentChild, (Depth + 1));// * sizeof(int));

		super.DESTRUCTOR();
	}

	//dxGeom* dxQuadTreeSpace::getGeom(int Index){
	public DxGeom getGeom(int Index) {
		Common.dUASSERT(Index >= 0 && Index < count, "index out of range");

		//@@@
		Common.dDebug (0,"dxQuadTreeSpace::getGeom() not yet implemented");

		return null;

		// This doesnt work

		/*if (CurrentIndex == Index){
		// Loop through all objects in the local list
CHILDRECURSE:
		if (CurrentObject){
			dGeom g = CurrentObject;
			CurrentObject = CurrentObject->next;
			CurrentIndex++;

#ifdef DRAWBLOCKS
			DrawBlock(CurrentBlock);
#endif	//DRAWBLOCKS
			return g;
		}
		else{
			// Now lets loop through our children. Starting at index 0.
			if (CurrentBlock->Children){
				CurrentChild[CurrentLevel] = 0;
PARENTRECURSE:
				for (int& i = CurrentChild[CurrentLevel]; i < SPLITS; i++){
					if (CurrentBlock->Children[i].GeomCount == 0){
						continue;
					}
					CurrentBlock = &CurrentBlock->Children[i];
					CurrentObject = CurrentBlock->First;

					i++;

					CurrentLevel++;
					goto CHILDRECURSE;
				}
			}
		}

		// Now lets go back to the parent so it can continue processing its other children.
		if (CurrentBlock->Parent){
			CurrentBlock = CurrentBlock->Parent;
			CurrentLevel--;
			goto PARENTRECURSE;
		}
	}
	else{
		CurrentBlock = &Blocks[0];
		CurrentLevel = 0;
		CurrentObject = CurrentObject;
		CurrentIndex = 0;

		// Other states are already set
		CurrentObject = CurrentBlock->First;
	}


	if (current_geom && current_index == Index - 1){
		//current_geom = current_geom->next; // next
		current_index = Index;
		return current_geom;
	}
	else for (int i = 0; i < Index; i++){	// this will be verrrrrrry slow
		getGeom(i);
	}*/

		//TODO return null;
	}

	//void dxQuadTreeSpace::add(dxGeom* g){
	void add(DxGeom g){
		CHECK_NOT_LOCKED (this);
		Common.dAASSERT(g);
		Common.dUASSERT(g.parent_space == null && g.getNext() == null, 
		"geom is already in a space");

		g._gflags |= GEOM_DIRTY | GEOM_AABB_BAD;
		DirtyList.push(g);

		// add
		g.parent_space = this;
		Blocks[0].GetBlock(g._aabb).AddObject(g);	// Add to best block
		count++;

		// enumerator has been invalidated
		current_geom = null;

		this.dGeomMoved();
	}

	//void dxQuadTreeSpace::remove(dxGeom* g){
	void remove(DxGeom g){
		CHECK_NOT_LOCKED(this);
		Common.dAASSERT(g);
		Common.dUASSERT(g.parent_space == this,"object is not in this space");

		// remove
		//TZ XXX ((Block*)g.tome).DelObject(g);
		g._qtIdx.DelObject(g);
		count--;

		for (int i = 0; i < DirtyList.size(); i++){
			if (DirtyList.get(i) == g){
				DirtyList.remove(i);
				// (mg) there can be multiple instances of a dirty object on stack  be sure to remove ALL and not just first, for this we decrement i
				--i;
			}
		}

		// safeguard
// FIXME		g.next = null;
		// FIXME		g.tome = null;
		g.parent_space = null;

		// enumerator has been invalidated
		current_geom = null;

		// the bounding box of this space (and that of all the parents) may have
		// changed as a consequence of the removal.
		this.dGeomMoved();
	}

	//void dxQuadTreeSpace::dirty(dxGeom* g){
	void dirty(DxGeom g){
		DirtyList.push(g);
	}

	//void dxQuadTreeSpace::computeAABB(){
	void computeAABB(){
		//
	}

	//void dxQuadTreeSpace::cleanGeoms(){
	void cleanGeoms(){
		// compute the AABBs of all dirty geoms, and clear the dirty flags
		lock_count++;

		for (int i = 0; i < DirtyList.size(); i++){
			DxGeom g = DirtyList.get(i);
			if (g instanceof DxSpace){
				((DxSpace)g).cleanGeoms();
			}
			g.recomputeAABB();
			g._gflags &= (~(GEOM_DIRTY|GEOM_AABB_BAD));

			//TZ XXX ((Block)g.tome).Traverse(g);
			g._qtIdx.Traverse(g);
		}
		DirtyList.setSize(0);

		lock_count--;
	}

	//void dxQuadTreeSpace::collide(void* UserData, dNearCallback* Callback){
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
			((DataCallback)data).callback.call(data.data, g2, g1);
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
	void collide2(Object UserData, DxGeom g2, DNearCallback Callback){
		dAASSERT(g2, Callback);

		lock_count++;
		cleanGeoms();
		g2.recomputeAABB();

		if (g2.parent_space == this){
			// The block the geom is in
			//XXX TZ Block* CurrentBlock = (Block*)g2.tome;
			Block CurrentBlock = g2._qtIdx;

			// Collide against block and its children
			DataCallback dc = new DataCallback(UserData, Callback);
			CurrentBlock.Collide(g2, CurrentBlock.First, dc, swap_callback);

			// Collide against parents
			while ((CurrentBlock = CurrentBlock.Parent) != null)
				CurrentBlock.CollideLocal(g2, UserData, Callback);

		}
		else {
			DataCallback dc = new DataCallback(UserData, Callback);
			Blocks[0].Collide(g2, Blocks[0].First, dc, swap_callback);
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
