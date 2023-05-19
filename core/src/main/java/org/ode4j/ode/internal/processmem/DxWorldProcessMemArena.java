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
package org.ode4j.ode.internal.processmem;

import org.ode4j.ode.internal.Common;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;

import java.util.concurrent.atomic.AtomicIntegerArray;

public final class DxWorldProcessMemArena {
	
    //   public:
    // #define BUFFER_TO_ARENA_EXTRA (EFFICIENT_ALIGNMENT + dEFFICIENT_SIZE(sizeof(dxWorldProcessMemArena)))
    private final static int BUFFER_TO_ARENA_EXTRA () {
        return 0; //(DxUtil.EFFICIENT_ALIGNMENT + DxUtil.dEFFICIENT_SIZE(
                //DxUtil.sizeof(DxWorldProcessMemArena.class)));
    }

    static boolean IsArenaPossible(int nBufferSize)
    {
        return Common.SIZE_MAX - BUFFER_TO_ARENA_EXTRA() >= nBufferSize; // This ensures there will be no overflow
    }

    static int MakeArenaSize(int nBufferSize)
    {
        return BUFFER_TO_ARENA_EXTRA() + nBufferSize;
    }
    //     #undef BUFFER_TO_ARENA_EXTRA

    public DxWorldProcessMemArena() {}

    boolean IsStructureValid() //const
    {
        return m_pAllocBegin!=null 
        && m_pAllocEnd!=null 
        && m_pAllocBegin.toInt() <= m_pAllocEnd.toInt() 
        && (m_pAllocCurrentOrNextArena == null || m_pAllocCurrentOrNextArena == m_pAllocBegin) 
        && m_pArenaBegin!=null  
        && m_pArenaBegin.toInt() <= m_pAllocBegin.toInt(); 
    }

    int GetMemorySize() //const
    {
        return m_pAllocEnd.toInt() - m_pAllocBegin.toInt();
    }

    public BlockPointer SaveState() //const
    {
        return m_pAllocCurrentOrNextArena;
    }

    public void RestoreState(BlockPointer state)
    {
    	m_pAllocCurrentOrNextArena = state;
    }

    public void ResetState()
    {
    	m_pAllocCurrentOrNextArena = m_pAllocBegin;
    }

    public BlockPointer PeekBufferRemainder() //const
    {
        return m_pAllocCurrentOrNextArena;
    }

    //dxWorldProcessMemArena *GetNextMemArena() const { return (dxWorldProcessMemArena *)m_pAllocCurrentOrNextArena; }
    DxWorldProcessMemArena GetNextMemArena() { return m_pAllocCurrentOrNextArena.asDxWorldProcessMemArena(); }
    //void SetNextMemArena(dxWorldProcessMemArena *pArenaInstance) { m_pAllocCurrentOrNextArena = pArenaInstance; }
    void SetNextMemArena(DxWorldProcessMemArena pArenaInstance) { m_pAllocCurrentOrNextArena.setTo( pArenaInstance ); }

    
    private BlockPointer m_pAllocCurrentOrNextArena = new BlockPointer(this, 0);
    private BlockPointer m_pAllocBegin = new BlockPointer(this, 0);
    private BlockPointer m_pAllocEnd = new BlockPointer(this, 0);
    private BlockPointer m_pArenaBegin;

    DxWorldProcessMemoryManager m_pArenaMemMgr;


    //****************************************************************************
    // World processing context management

    static DxWorldProcessMemArena ReallocateMemArena (
            DxWorldProcessMemArena oldarena, int memreq, 
            final DxWorldProcessMemoryManager memmgr, double rsrvfactor, 
            int rsrvminimum)
    {
        DxWorldProcessMemArena arena = oldarena;
        boolean allocsuccess = false;

        int nOldArenaSize = 0; 
        BlockPointer pOldArenaBuffer = null;

        do {
            int oldmemsize = oldarena!=null ? oldarena.GetMemorySize() : 0;
            if (oldarena == null || oldmemsize < memreq) {
                nOldArenaSize = oldarena!=null ? MakeArenaSize(oldmemsize) : 0;
                pOldArenaBuffer = oldarena!=null ? oldarena.m_pArenaBegin : null;

                if (!IsArenaPossible(memreq)) {
                    break;
                }

                int arenareq = DxWorldProcessMemArena.MakeArenaSize(memreq);
                int arenareq_with_reserve = AdjustArenaSizeForReserveRequirements(arenareq, rsrvfactor, rsrvminimum);
                int memreq_with_reserve = memreq + (arenareq_with_reserve - arenareq);

                if (oldarena != null) {
                    oldarena.m_pArenaMemMgr.m_fnFree.run(pOldArenaBuffer, nOldArenaSize);
                    oldarena = null;

                    // Zero variables to avoid another freeing on exit
                    pOldArenaBuffer = null;
                    nOldArenaSize = 0;
                }

                // Allocate new arena
                BlockPointer pNewArenaBuffer = memmgr.m_fnAlloc.run(arenareq_with_reserve);
                if (pNewArenaBuffer == null) {
                    break;
                }

                arena = DxUtil.dEFFICIENT_PTR(pNewArenaBuffer).asDxWorldProcessMemArena();

                BlockPointer blockbegin = DxUtil.dEFFICIENT_PTR(arena, 1);
                BlockPointer blockend = DxUtil.dOFFSET_EFFICIENTLY(blockbegin, memreq_with_reserve);

                arena.m_pAllocBegin = blockbegin;
                arena.m_pAllocEnd = blockend;
                arena.m_pArenaBegin = pNewArenaBuffer;
                arena.m_pAllocCurrentOrNextArena.setTo(null);
                arena.m_pArenaMemMgr = memmgr;
            }

            allocsuccess = true;
        } 
        while (false);

        if (!allocsuccess) {
            if (pOldArenaBuffer != null) {
                Common.dIASSERT(oldarena != null);
                oldarena.m_pArenaMemMgr.m_fnFree.run(pOldArenaBuffer, nOldArenaSize);
            }
            arena = null;
        }

        return arena;
    }

    @SuppressWarnings("unused")
	static void FreeMemArena (DxWorldProcessMemArena arena)
    {
        int memsize = arena.GetMemorySize();
        int arenasize = DxWorldProcessMemArena.MakeArenaSize(memsize);

        BlockPointer pArenaBegin = arena.m_pArenaBegin;
        //TODO TZ couldn get this to work easily...
        //TODO remove?
        if (false) {
        	arena.m_pArenaMemMgr.m_fnFree.run(pArenaBegin, arenasize);
        }
    }


    static int AdjustArenaSizeForReserveRequirements(int arenareq, double rsrvfactor, 
            int rsrvminimum)
    {
        double scaledarena = arenareq * rsrvfactor;
        int adjustedarena = (scaledarena < Common.SIZE_MAX) ? (int)scaledarena : Common.SIZE_MAX;
        int boundedarena = (adjustedarena > rsrvminimum) ? adjustedarena : rsrvminimum;
        return DxUtil.dEFFICIENT_SIZE(boundedarena);
    }

    // ***********************************************
    // Java methods to simulate the C++ manager (TZ)
    // ***********************************************

    public final double[] AllocateArrayDReal(int size) {
        return new double[size];
    }

    public final int[] AllocateArrayInt(int size) {
        return new int[size];
    }

    public double[] AllocateOveralignedArrayDReal(int count, int alignment)
    {
        //return (ElementType *)AllocateOveralignedBlock(count * sizeof(ElementType), alignment);
        // TZ: we assume that alignment is for the whole block so it can be safely ignored.
        //     Do we have to make sure to have size be a multiple of alignment?
        return new double[count];
    }

    public final AtomicIntegerArray AllocateArrayAtomicord32(int size) {
        return new AtomicIntegerArray(size);
    }

    /**
     * Reminder function. At the place where it is called, something needs to 
     * be implemented with respect to ProcessMemManagement.
     */
    public final void dummy() {
        // TODO Auto-generated method stub

    }

    public static final void dummyStatic() {
        // TODO Auto-generated method stub

    }

    public final void ShrinkArrayDJointWithInfo1(
            Object[] jointiinfos,
            int _nj, int njXXX) {
        // TODO Auto-generated method stub

    }

    public final double[][] AllocateArrayDRealDReal(int n) {
        return new double[n][];
    }

    public final boolean[] AllocateArrayBool(int n) {
        return new boolean[n];
    }

    public BlockPointer BEGIN_STATE_SAVE() {
        return SaveState();
    }

    public void END_STATE_SAVE(BlockPointer saveInner) {
        RestoreState(saveInner);
    }

    public static DxWorldProcessMemArena allocateTemporary(int memreq,
            Object object, Object object2) {
        return new DxWorldProcessMemArena();
    }

    public static void freeTemporary(DxWorldProcessMemArena arena) {
        // TODO Auto-generated method stub

    }

    public DxBody[] AllocateArrayDxBody(int nb) {
        return new DxBody[nb];
    }

    public DxJoint[] AllocateArrayDxJoint(int nj) {
        return new DxJoint[nj];
    }

}
