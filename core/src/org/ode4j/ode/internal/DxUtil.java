/*
 * Created on Apr 14, 2012
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Generation - Code and Comments
 */
package org.ode4j.ode.internal;

import org.ode4j.ode.internal.DxUtil.BlockPointer;

public class DxUtil {

    //****************************************************************************
    // Malloc based world stepping memory manager

    /*extern */
    static DxWorldProcessMemoryManager g_WorldProcessMallocMemoryManager = 
        new DxWorldProcessMemoryManager(
                //null,//dAlloc, 
                new alloc_block_fn_t() {
                    @Override
                    public BlockPointer run(int block_size) {
                        System.err.println("FIXME: alloc_block_fn_t");
                        //return new BlockPointer(-1);
                        return null;
                    }
                },
                null,//dRealloc, 
                null);//dFree);
    /*extern */
    static DxWorldProcessMemoryReserveInfo g_WorldProcessDefaultReserveInfo = 
        new DxWorldProcessMemoryReserveInfo(
                1.2f, //dWORLDSTEP_RESERVEFACTOR_DEFAULT, 
                65536);// dWORLDSTEP_RESERVESIZE_DEFAULT);

    
    static final int sizeof(Class<?> cls) {
        if (cls == DxWorldProcessMemArena.class) {
            return -1;
        }
        return -1;
    }
    
    static final int sizeof(Object o) {
        if (o instanceof Class) {
            return sizeof((Class<?>)o);
        }
        return sizeof(o.getClass());
    }
    
    static final int EFFICIENT_ALIGNMENT = 16;
    
//  #define dEFFICIENT_SIZE(x) (((x)+(EFFICIENT_ALIGNMENT-1)) & ~((size_t)(EFFICIENT_ALIGNMENT-1)))
    static final int dEFFICIENT_SIZE(int x) {
        return -1;
        //return (((x)+(EFFICIENT_ALIGNMENT-1)) & ~((int)(EFFICIENT_ALIGNMENT-1)));
    }
//  #define dEFFICIENT_PTR(p) ((void *)dEFFICIENT_SIZE((size_t)(p)))
    static final BlockPointer dEFFICIENT_PTR(BlockPointer p) {
        return new BlockPointer(dEFFICIENT_SIZE(p.toInt()));
    }
    public static BlockPointer dEFFICIENT_PTR(Object obj, int i) {
        System.out.println("dEFFICIENT_PTR(Object obj, int i)");
        return new BlockPointer(i);
    }
//  #define dOFFSET_EFFICIENTLY(p, b) ((void *)((size_t)(p) + dEFFICIENT_SIZE(b)))
    static final BlockPointer dOFFSET_EFFICIENTLY(BlockPointer p, int b) {
        return new BlockPointer((p.toInt() + dEFFICIENT_SIZE(b)));
    }

    /* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
     * up to 15 bytes per allocation, depending on what alloca() returns.
     */

//    #ifndef SIZE_MAX
//    #define SIZE_MAX  ((size_t)(-1))
//    #endif
    static final int SIZE_MAX = Integer.MAX_VALUE; //TODO correct?

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
    

    //TZ replacemnent for void*
    static class BlockPointer {
        private final int pointer;
        public BlockPointer(int pointer) {
            this.pointer = pointer;
        }
        int toInt() {
            return pointer;
        }
        public Object getObject() {
            // TODO Auto-generated method stub
            return null;
        }
        
    }

    
}
