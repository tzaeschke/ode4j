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
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;


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


    //  bool dxReallocateWorldProcessContext (dxWorld *world, dxWorldProcessIslandsInfo &islandsinfo, 
    //  dReal stepsize, dmemestimate_fn_t stepperestimate);
    //void dxCleanupWorldProcessContext (dxWorld *world);
    public static boolean dxReallocateWorldProcessContext (DxWorld world, 
            DxWorldProcessIslandsInfo islandsinfo, 
            double stepsize, dmemestimate_fn_t stepperestimate)
    {
        //TZ DxStepWorkingMemory wmem = DxWorld.AllocateOnDemand(world.wmem);
        if (world.wmem == null) {
            world.wmem = new DxStepWorkingMemory();
        }
        DxStepWorkingMemory wmem = world.wmem;

        if (wmem == null) return false;

        DxWorldProcessContext context = wmem.SureGetWorldProcessingContext();
        if (context == null) return false;
        Common.dIASSERT (context.IsStructureValid());

        final DxWorldProcessMemoryReserveInfo reserveinfo = wmem.SureGetMemoryReserveInfo();
        final DxWorldProcessMemoryManager memmgr = wmem.SureGetMemoryManager();

        int islandsreq = world.EstimateIslandsProcessingMemoryRequirements();
        Common.dIASSERT(islandsreq == DxUtil.dEFFICIENT_SIZE(islandsreq));

        DxWorldProcessMemArena stepperarena = null;
        DxWorldProcessMemArena islandsarena = context.ReallocateIslandsMemArena(
                islandsreq, memmgr, 1.0f, reserveinfo.m_uiReserveMinimum);

        if (islandsarena != null)
        {
            int stepperreq = 
                DxWorldProcessIslandsInfo.BuildIslandsAndEstimateStepperMemoryRequirements(
                        islandsinfo, islandsarena, world, stepsize, stepperestimate);
            Common.dIASSERT(stepperreq == DxUtil.dEFFICIENT_SIZE(stepperreq));

            stepperarena = context.ReallocateStepperMemArena(stepperreq, 
                    memmgr, reserveinfo.m_fReserveFactor, reserveinfo.m_uiReserveMinimum);
        }

        return stepperarena != null;
    }

    public static void dxCleanupWorldProcessContext (DxWorld world)
    {
        DxStepWorkingMemory wmem = world.wmem;
        if (wmem != null)
        {
            DxWorldProcessContext context = wmem.GetWorldProcessingContext();
            if (context != null)
            {
                context.CleanupContext();
                Common.dIASSERT(context.IsStructureValid());
            }
        }
    }


}
