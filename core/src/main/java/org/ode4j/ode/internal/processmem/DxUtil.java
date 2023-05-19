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

import org.ode4j.ode.internal.DxWorld;


/**
 * From util.cpp and util.h.
 *
 * @author Tilmann Zaeschke
 */
public class DxUtil {

	//#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
	//#define dMAX(A,B)  ((B)>(A) ? (B) : (A))
	public static int dMIN(int A, int B) { return Math.min(A, B); }
	public static int dMAX(int B, int A) { return Math.min(A, B); }
	
	
    //****************************************************************************
    // Malloc based world stepping memory manager

	public static final BlockPointer NULL_BP = new BlockPointer(new DxWorldProcessMemArena(), -1);
	
    /*extern */
    @SuppressWarnings("deprecation")
	static final DxWorldProcessMemoryManager g_WorldProcessMallocMemoryManager = 
        new DxWorldProcessMemoryManager(
        		DxWorld.DWorldStepMemoryFunctionsInfo.alloc_block,
        		DxWorld.DWorldStepMemoryFunctionsInfo.shrink_block,
        		DxWorld.DWorldStepMemoryFunctionsInfo.free_block);
    /*extern */
    static final DxWorldProcessMemoryReserveInfo g_WorldProcessDefaultReserveInfo = 
        new DxWorldProcessMemoryReserveInfo(
                1.2f, //dWORLDSTEP_RESERVEFACTOR_DEFAULT, 
                65536);// dWORLDSTEP_RESERVESIZE_DEFAULT);


    /* the efficient alignment. most platforms align data structures to some
     * number of bytes, but this is not always the most efficient alignment.
     * for example, many x86 compilers align to 4 bytes, but on a pentium it
     * is important to align doubles to 8 byte boundaries (for speed), and
     * the 4 floats in a SIMD register to 16 byte boundaries. many other
     * platforms have similar behavior. setting a larger alignment can waste
     * a (very) small amount of memory. NOTE: this number must be a power of
     * two. this is set to 16 by default.
     */
    public static final int EFFICIENT_ALIGNMENT = 16;

    /* ********************************************************************************************************* */
    /* ************************* TZ: This has been move here from Common because it is mostly required here **** */
    /* ********************************************************************************************************* */

//  #define dEFFICIENT_SIZE(x) (((x)+(EFFICIENT_ALIGNMENT-1)) & ~((size_t)(EFFICIENT_ALIGNMENT-1)))
    static int dEFFICIENT_SIZE(int x) {
        return x;
        //return (((x)+(EFFICIENT_ALIGNMENT-1)) & ~((int)(EFFICIENT_ALIGNMENT-1)));
    }
//  #define dEFFICIENT_PTR(p) ((void *)dEFFICIENT_SIZE((size_t)(p)))
    static BlockPointer dEFFICIENT_PTR(BlockPointer p) {
    	//return new BlockPointer(dEFFICIENT_SIZE(p.toInt()));
    	//TZ reallocation not possible --> just return it
    	return p;
    }
    static BlockPointer dEFFICIENT_PTR(DxWorldProcessMemArena obj, int i) {
        //System.out.println("dEFFICIENT_PTR(Object obj, int i)");
    	//TODO arghhh!!!
    	//return NULL_BP;
        return new BlockPointer(obj, i);
    }
//  #define dOFFSET_EFFICIENTLY(p, b) ((void *)((size_t)(p) + dEFFICIENT_SIZE(b)))
    static BlockPointer dOFFSET_EFFICIENTLY(BlockPointer p, int b) {
        //return new BlockPointer((p.toInt() + dEFFICIENT_SIZE(b)));
    	//TZ reallocation not possible --> just return it
    	return p;
    }


    // #define dOVERALIGNED_SIZE(size, alignment) dEFFICIENT_SIZE((size) + ((alignment) - EFFICIENT_ALIGNMENT))
    //int dOVERALIGNED_SIZE(int size, int alignment) { return dEFFICIENT_SIZE(size + (alignment - EFFICIENT_ALIGNMENT)); }
    // #define dOVERALIGNED_PTR(buf_ptr, alignment) ((void *)(((uintptr_t)(buf_ptr) + ((alignment) - 1)) & (int)(~(alignment - 1)))) // Casting the mask to int ensures sign-extension to larger integer sizes
    // #define dOFFSET_OVERALIGNEDLY(buf_ptr, size, alignment) ((void *)((uintptr_t)(buf_ptr) + dOVERALIGNED_SIZE(size, alignment)))


    /* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
     * up to 15 bytes per allocation, depending on what alloca() returns.
     */
    //#define dALLOCA16(n) dEFFICIENT_PTR(alloca((n)+(EFFICIENT_ALIGNMENT)))

    //typedef void *(*alloc_block_fn_t)(size_t block_size);
    public interface alloc_block_fn_t {
        BlockPointer run(int block_size);
    }
    //typedef void *(*shrink_block_fn_t)(void *block_pointer, size_t block_current_size, size_t block_smaller_size);
    public interface shrink_block_fn_t {
        BlockPointer run(BlockPointer block_pointer, int block_current_size, int block_smaller_size);
    }
    //typedef void (*free_block_fn_t)(void *block_pointer, size_t block_current_size);
    public interface free_block_fn_t {
        void run(BlockPointer block_pointer, int block_current_size);
    }
    

    //TZ replacement for void*
    public static class BlockPointer {
        private int pointer;
        private Object o = null;
//        public BlockPointer(int pointer) {
//            this.pointer = pointer;
//        }
        public BlockPointer(DxWorldProcessMemArena obj, int pointer) {
        	this.pointer = pointer;
			o = obj;
		}
		int toInt() {
            return pointer;
        }
        public DxWorldProcessMemArena asDxWorldProcessMemArena() {
            if (o == null) {
                o = new DxWorldProcessMemArena();
            }
            return (DxWorldProcessMemArena) o;
        }
        void setTo(DxWorldProcessMemArena x) {
        	o = x;
        }
    }

    private DxUtil() {}
}
