/*
 * Created on Apr 14, 2012
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Generation - Code and Comments
 */
package org.ode4j.ode.internal;

import org.ode4j.ode.internal.DxWorldProcessIslandsInfo.dmemestimate_fn_t;

public class DxWorldProcessContext {

    //public:
    //public DxWorldProcessContext();
    //public DESTRUCTOR();

    //public boolean IsStructureValid();
    //public void CleanupContext();

    public DxWorldProcessMemArena GetIslandsMemArena() { 
        return m_pmaIslandsArena; 
    }
    public DxWorldProcessMemArena GetStepperMemArena() { 
        return m_pmaStepperArena; 
    }

    //    public DxWorldProcessMemArena ReallocateIslandsMemArena(int nMemoryRequirement, 
    //            final DxWorldProcessMemoryManager pmmMemortManager, float fReserveFactor, int uiReserveMinimum);
    //    public DxWorldProcessMemArena ReallocateStepperMemArena(int nMemoryRequirement, 
    //            final DxWorldProcessMemoryManager pmmMemortManager, float fReserveFactor, int uiReserveMinimum);

    //private:
    private void SetIslandsMemArena(DxWorldProcessMemArena pmaInstance) { 
        m_pmaIslandsArena = pmaInstance; 
    }
    private void SetStepperMemArena(DxWorldProcessMemArena pmaInstance) { 
        m_pmaStepperArena = pmaInstance; 
    }

    //private:
    private DxWorldProcessMemArena m_pmaIslandsArena;
    private DxWorldProcessMemArena m_pmaStepperArena;



    //****************************************************************************
    // dxWorldProcessContext

    DxWorldProcessContext() { //:
        m_pmaIslandsArena = null;//(NULL),
        m_pmaStepperArena = null;//(NULL)
        // Do nothing
    }

    //dxWorldProcessContext::~dxWorldProcessContext()
    public void DESTRUCTOR()
    {
        if (m_pmaIslandsArena!=null)
        {
            DxWorldProcessMemArena.FreeMemArena(m_pmaIslandsArena);
        }

        if (m_pmaStepperArena!=null)
        {
            DxWorldProcessMemArena.FreeMemArena(m_pmaStepperArena);
        }
    }

    boolean IsStructureValid() 
    {
        return (m_pmaIslandsArena==null || m_pmaIslandsArena.IsStructureValid()) 
        && (m_pmaStepperArena==null || m_pmaStepperArena.IsStructureValid()); 
    }

    void CleanupContext()
    {
        if (m_pmaIslandsArena!=null)
        {
            m_pmaIslandsArena.ResetState();
        }

        if (m_pmaStepperArena!=null)
        {
            m_pmaStepperArena.ResetState();
        }
    }

    DxWorldProcessMemArena ReallocateIslandsMemArena(int nMemoryRequirement, 
            final DxWorldProcessMemoryManager pmmMemortManager, 
            double fReserveFactor, int uiReserveMinimum)
    {
        DxWorldProcessMemArena pmaExistingArena = GetIslandsMemArena();
        DxWorldProcessMemArena pmaNewMemArena = DxWorldProcessMemArena.ReallocateMemArena(
                pmaExistingArena, nMemoryRequirement, pmmMemortManager, fReserveFactor, uiReserveMinimum);
        SetIslandsMemArena(pmaNewMemArena);
        return pmaNewMemArena;
    }

    DxWorldProcessMemArena ReallocateStepperMemArena(int nMemoryRequirement, 
            final DxWorldProcessMemoryManager pmmMemortManager, 
            double fReserveFactor, int uiReserveMinimum)
    {
        DxWorldProcessMemArena pmaExistingArena = GetStepperMemArena();
        DxWorldProcessMemArena pmaNewMemArena = DxWorldProcessMemArena.ReallocateMemArena(
                pmaExistingArena, nMemoryRequirement, pmmMemortManager, fReserveFactor, uiReserveMinimum);
        SetStepperMemArena(pmaNewMemArena);
        return pmaNewMemArena;
    }
}
