/*
 * Created on Apr 14, 2012
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Generation - Code and Comments
 */
package org.ode4j.ode.internal;

public class DxWorldProcessMemoryManager {
    //TODO extends dBase

    DxWorldProcessMemoryManager(DxUtil.alloc_block_fn_t fnAlloc, 
            DxUtil.shrink_block_fn_t fnShrink, DxUtil.free_block_fn_t fnFree)
            {
        Assign(fnAlloc, fnShrink, fnFree);
            }

    void Assign(DxUtil.alloc_block_fn_t fnAlloc, 
            DxUtil.shrink_block_fn_t fnShrink, DxUtil.free_block_fn_t fnFree)
    {
        m_fnAlloc = fnAlloc;
        m_fnShrink = fnShrink;
        m_fnFree = fnFree;
    }

    DxUtil.alloc_block_fn_t m_fnAlloc;
    DxUtil.shrink_block_fn_t m_fnShrink;
    DxUtil.free_block_fn_t m_fnFree;

    public void DESTRUCTOR() {
        // Nothing to do
    }
}
