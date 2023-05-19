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

import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.gimpact.Gimpact;

/**
 * ODE initialization/finalization code.
 */
public class OdeInit {

     //****************************************************************************
    // Initialization tracking variables

    private static int g_uiODEInitCounter = 0;
    private static int g_uiODEInitModes = 0;

    //#if dTRIMESH_ENABLED && dTRIMESH_OPCODE
    //
    //    static
    //    void OPCODEAbort()
    //    {
    //        dICHECK(!"OPCODE Library Abort");
    //    }
    //
    //
    //#endif // #if dTRIMESH_ENABLED && dTRIMESH_OPCODE

    enum EODEINITMODE
    {
        //OIM__MIN,

        OIM_AUTOTLSCLEANUP,// = OIM__MIN,
        OIM_MANUALTLSCLEANUP,

//        OIM__MAX;
    };

//    #if dTLS_ENABLED
//    static const EODETLSKIND g_atkTLSKindsByInitMode[OIM__MAX] =
//    {
//        OTK_AUTOCLEANUP, // OIM_AUTOTLSCLEANUP,
//        OTK_MANUALCLEANUP, // OIM_MANUALTLSCLEANUP,
//    };
//    #endif // #if dTLS_ENABLED

    private static boolean IsODEModeInitialized(EODEINITMODE imInitMode)
    {
        return (g_uiODEInitModes & (1 << imInitMode.ordinal())) != 0;
    }

    private static void SetODEModeInitialized(EODEINITMODE imInitMode)
    {
        g_uiODEInitModes |= (1 << imInitMode.ordinal());
    }

    private static void ResetODEModeInitialized(EODEINITMODE imInitMode)
    {
        g_uiODEInitModes &= ~(1 << imInitMode.ordinal());
    }

    private static boolean IsODEAnyModeInitialized()
    {
        return g_uiODEInitModes != 0;
    }


//    enum TLD_ICA
//    {
//    private static final int
//        TLD_INTERNAL_COLLISIONDATA_ALLOCATED = 0x00000001;
//    };

    private static boolean AllocateThreadBasicDataIfNecessary(EODEINITMODE imInitMode)
    {
        boolean bResult = false;

        do
        {
//    #if dTLS_ENABLED
//            EODETLSKIND tkTlsKind = g_atkTLSKindsByInitMode[imInitMode];
//
//            const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags(tkTlsKind);
//
//            // If no flags are set it may mean that TLS slot is not allocated yet
//            if (uDataAllocationFlags == 0)
//            {
//                // Assign zero flags to make sure that TLS slot has been allocated
//                if (!COdeTls::AssignDataAllocationFlags(tkTlsKind, 0))
//                {
//                    break;
//                }
//            }
//
//    #else        	
//            (void)imInitMode; // unused
//    #endif // #if dTLS_ENABLED

            bResult = true;
        }
        while (false);

        return bResult;
    }

    private static void FreeThreadBasicDataOnFailureIfNecessary(EODEINITMODE imInitMode)
    {
//    #if dTLS_ENABLED
//
//        if (imInitMode == OIM_MANUALTLSCLEANUP)
//        {
//            EODETLSKIND tkTlsKind = g_atkTLSKindsByInitMode[imInitMode];
//
//            const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags(tkTlsKind);
//
//            if (uDataAllocationFlags == 0)
//            {
//                // So far, only free TLS slot, if no subsystems have data allocated
//                COdeTls::CleanupForThread();
//            }
//        }
//
//    	#else
//    	    (void)imInitMode; // unused
//    #endif // #if dTLS_ENABLED
    }

//    #if dTLS_ENABLED
//    static bool AllocateThreadCollisionData(EODETLSKIND tkTlsKind)
//    {
//        bool bResult = false;
//
//        do
//        {
//            dIASSERT(!(COdeTls::GetDataAllocationFlags(tkTlsKind) & TLD_INTERNAL_COLLISIONDATA_ALLOCATED));
//
//    #if dTRIMESH_ENABLED 
//
//            TrimeshCollidersCache *pccColliderCache = new TrimeshCollidersCache();
//            if (!COdeTls::AssignTrimeshCollidersCache(tkTlsKind, pccColliderCache))
//            {
//                delete pccColliderCache;
//                break;
//            }
//
//    #endif // dTRIMESH_ENABLED
//
//            COdeTls::SignalDataAllocationFlags(tkTlsKind, TLD_INTERNAL_COLLISIONDATA_ALLOCATED);
//
//            bResult = true;
//        }
//        while (false);
//
//        return bResult;
//    }
//    #endif // dTLS_ENABLED

    private static boolean AllocateThreadCollisionDataIfNecessary(EODEINITMODE imInitMode, RefBoolean bOutDataAllocated)
    {
        boolean bResult = false;
        bOutDataAllocated.set(false);

        do 
        {
//    #if dTLS_ENABLED
//            EODETLSKIND tkTlsKind = g_atkTLSKindsByInitMode[imInitMode];
//
//            const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags(tkTlsKind);
//
//            if ((uDataAllocationFlags & TLD_INTERNAL_COLLISIONDATA_ALLOCATED) == 0)
//            {
//                if (!AllocateThreadCollisionData(tkTlsKind))
//                {
//                    break;
//                }
//
//                bOutDataAllocated = true;
//            }
//
//        	#else
//                (void)imInitMode; // unused
//    #endif // #if dTLS_ENABLED

            bResult = true;
        }
        while (false);

        return bResult;
    }

    private static void FreeThreadCollisionData(EODEINITMODE imInitMode)
    {
//    #if dTLS_ENABLED
//
//        EODETLSKIND tkTlsKind = g_atkTLSKindsByInitMode[imInitMode];
//
//        COdeTls::DestroyTrimeshCollidersCache(tkTlsKind);
//
//        COdeTls::DropDataAllocationFlags(tkTlsKind, TLD_INTERNAL_COLLISIONDATA_ALLOCATED);
//
//    	#else
//    	    (void)imInitMode; // unused
//    #endif // dTLS_ENABLED
    }


    private static boolean InitODEForMode(EODEINITMODE imInitMode)
    {
        boolean bResult = false;

//    #if dOU_ENABLED
//        bool bOUCustomizationsDone = false;
//    #endif
//    #if dATOMICS_ENABLED
//        bool bAtomicsInitialized = false;
//    #endif
//    #if dTLS_ENABLED
//        EODETLSKIND tkTLSKindToInit = g_atkTLSKindsByInitMode[imInitMode];
//        bool bTlsInitialized = false;
//        #else
//            (void)imInitMode; // unused
//    #endif

        boolean bWorldThreadingInitialized = false;

        do
        {
            boolean bAnyModeAlreadyInitialized = IsODEAnyModeInitialized();

            if (!bAnyModeAlreadyInitialized)
            {
//    #if dOU_ENABLED
//                if (!COdeOu::DoOUCustomizations())
//                {
//                    break;
//                }
//
//                bOUCustomizationsDone = true;
//    #endif
//
//    #if dATOMICS_ENABLED
//                if (!COdeOu::InitializeAtomics())
//                {
//                    break;
//                }
//
//                bAtomicsInitialized = true;
//    #endif
            }

//    #if dTLS_ENABLED
//            if (!COdeTls::Initialize(tkTLSKindToInit))
//            {
//                break;
//            }
//
//            bTlsInitialized = true;
//    #endif

            if (!bAnyModeAlreadyInitialized)
            {
                bWorldThreadingInitialized = true;

//    #if dTRIMESH_ENABLED && dTRIMESH_OPCODE
//                if (!Opcode::InitOpcode(&OPCODEAbort))
//                {
//                    break;
//                }
//    #endif

    //#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT
                if (Common.dTRIMESH_ENABLED && Common.dTRIMESH_GIMPACT) {
                    Gimpact.gimpact_init();
                }//#endif

                DxGeom.dInitColliders();
            }

            bResult = true;
        }
        while (false);

        if (!bResult)
        {
            if (bWorldThreadingInitialized)
            {
            }

//    #if dTLS_ENABLED
//            if (bTlsInitialized)
//            {
//                COdeTls::Finalize(tkTLSKindToInit);
//            }
//    #endif
//
//    #if dATOMICS_ENABLED
//            if (bAtomicsInitialized)
//            {
//                COdeOu::FinalizeAtomics();
//            }
//    #endif
//
//    #if dOU_ENABLED
//            if (bOUCustomizationsDone)
//            {
//                COdeOu::UndoOUCustomizations();
//            }
//    #endif
        }

        return bResult;
    }


    private static boolean AllocateODEDataForThreadForMode(
            EODEINITMODE imInitMode, int uiAllocateFlags)
    {
        boolean bResult = false;

        final RefBoolean bCollisionDataAllocated = new RefBoolean(false);

        do
        {
            if (!AllocateThreadBasicDataIfNecessary(imInitMode))
            {
                break;
            }

            if ((uiAllocateFlags & dAllocateFlagCollisionData)!=0)
            {
                if (!AllocateThreadCollisionDataIfNecessary(imInitMode, bCollisionDataAllocated))
                {
                    break;
                }
            }

            bResult = true;
        }
        while (false);

        if (!bResult)
        {
            if (bCollisionDataAllocated.get())
            {
                FreeThreadCollisionData(imInitMode);
            }

            FreeThreadBasicDataOnFailureIfNecessary(imInitMode);
        }

        return bResult;
    }


    @SuppressWarnings("unused")
    static void CloseODEForMode(EODEINITMODE imInitMode)
    {
        boolean bAnyModeStillInitialized = IsODEAnyModeInitialized();

        if (!bAnyModeStillInitialized)
        {
            CollideSpaceGeom.dClearPosrCache();
            DxGeom.dFinitUserClasses();
            DxGeom.dFinitColliders();

            if (Common.dTRIMESH_ENABLED && Common.dTRIMESH_GIMPACT) {
                //#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT
                Gimpact.gimpact_terminate();
            } //#endif

            if (Common.dTRIMESH_ENABLED && Common.dTRIMESH_OPCODE) {
                //#if dTRIMESH_ENABLED && dTRIMESH_OPCODE
                throw new UnsupportedOperationException();
//                extern void opcode_collider_cleanup();
//            // Free up static allocations in opcode
//                opcode_collider_cleanup();
//
//            Opcode::CloseOpcode();
            } //#endif

            DxWorld.FinalizeDefaultThreading();
       }

//    #if dTLS_ENABLED
//        EODETLSKIND tkTLSKindToFinalize = g_atkTLSKindsByInitMode[imInitMode];
//        COdeTls::Finalize(tkTLSKindToFinalize);
//        #else
//            (void)imInitMode; // unused
//    #endif

        if (!bAnyModeStillInitialized)
        {
//    #if dATOMICS_ENABLED
//            COdeOu::FinalizeAtomics();
//    #endif
//
//    #if dOU_ENABLED
//            COdeOu::UndoOUCustomizations();
//    #endif
        }
    }


    //****************************************************************************
    // internal initialization and close routine implementations

    private static boolean InternalInitODE(int uiInitFlags)
    {
        boolean bResult = false;

        do 
        {
            EODEINITMODE imInitMode = 
                ((uiInitFlags & dInitFlagManualThreadCleanup)!=0) ? 
                        EODEINITMODE.OIM_MANUALTLSCLEANUP : 
                            EODEINITMODE.OIM_AUTOTLSCLEANUP;

            if (!IsODEModeInitialized(imInitMode))
            {
                if (!InitODEForMode(imInitMode))
                {
                    break;
                }

                SetODEModeInitialized(imInitMode);
            }

            ++g_uiODEInitCounter;
            bResult = true;
        }
        while (false);

        return bResult;
    }

    private static void InternalCloseODE()
    {
//        EODEINITMODE uiCurrentMode = (--g_uiODEInitCounter == 0) ? OIM__MIN : OIM__MAX;
//        for (; uiCurrentMode != OIM__MAX; ++uiCurrentMode)
        int max = (--g_uiODEInitCounter == 0) ? 0 : EODEINITMODE.values().length;
        for (; max != EODEINITMODE.values().length; ++max)
        {
            EODEINITMODE uiCurrentMode = EODEINITMODE.values()[max];
            if (IsODEModeInitialized(uiCurrentMode))
            {
                // Must be called before CloseODEForMode()
                ResetODEModeInitialized(uiCurrentMode);

                // Must be called after ResetODEModeInitialized()
                CloseODEForMode(uiCurrentMode);
            }
        }
    }

    private static boolean InternalAllocateODEDataForThread(int uiAllocateFlags)
    {
        boolean bAnyFailure = false;

        //for (EODEINITMODE uiCurrentMode = OIM__MIN; uiCurrentMode != OIM__MAX; ++uiCurrentMode)
        for (EODEINITMODE uiCurrentMode: EODEINITMODE.values())
        {
            if (IsODEModeInitialized(uiCurrentMode))
            {
                if (!AllocateODEDataForThreadForMode(uiCurrentMode, uiAllocateFlags))
                {
                    bAnyFailure = true;
                    break;
                }
            }
        }

        boolean bResult = !bAnyFailure;
        return bResult;
    }

    static void InternalCleanupODEAllDataForThread()
    {
//    #if dTLS_ENABLED
//        COdeTls::CleanupForThread();
//    #endif
    }

    //****************************************************************************
    // initialization and shutdown routines - allocate and initialize data,
    // cleanup before exiting

    public static void dInitODE()
    {
        boolean bInitResult = InternalInitODE(0);
        Common.dIVERIFY(bInitResult);

        boolean ibAllocResult = InternalAllocateODEDataForThread(dAllocateMaskAll);
        Common.dIVERIFY(ibAllocResult);
    }

    public static boolean dInitODE2(int uiInitFlags/*=0*/)
    {
        boolean bResult = false;

        boolean bODEInitialized = false;

        do
        {
            if (!InternalInitODE(uiInitFlags))
            {
                break;
            }

            bODEInitialized = true;

            if (!InternalAllocateODEDataForThread(dAllocateFlagBasicData))
            {
                break;
            }

            bResult = true;
        }
        while (false);

        if (!bResult)
        {
            if (bODEInitialized)
            {
                InternalCloseODE();
            }
        }

        return bResult;
    }


    public static boolean dAllocateODEDataForThread(int uiAllocateFlags)
    {
        Common.dUASSERT(g_uiODEInitCounter != 0, "Call dInitODE2 first");

        boolean bResult = InternalAllocateODEDataForThread(uiAllocateFlags);
        return bResult;
    }


    void dCleanupODEAllDataForThread()
    {
        Common.dUASSERT(g_uiODEInitCounter != 0, "Call dInitODE2 first or delay dCloseODE until all threads exit");

        InternalCleanupODEAllDataForThread();
    }


    public static void dCloseODE()
    {
        Common.dUASSERT(g_uiODEInitCounter != 0, "dCloseODE must not be called without dInitODE2 or if dInitODE2 fails"); // dCloseODE must not be called without dInitODE2 or if dInitODE2 fails

        InternalCloseODE();
    }

    
    /**
     * Library initialization flags.
     *
     * These flags define ODE library initialization options.
     *
     * {@code dInitFlagManualThreadCleanup} indicates that resources allocated in TLS for threads
     * using ODE are to be cleared by library client with explicit call to 
     * {@code dCleanupODEAllDataForThread}.
     * If this flag is not specified the automatic resource tracking algorithm is used.
     *
     * With automatic resource tracking, On Windows, memory allocated for a thread may 
     * remain not freed for some time after the thread exits. The resources may be 
     * released when one of other threads calls {@code dAllocateODEDataForThread}. Ultimately,
     * the resources are released when library is closed with {@code dCloseODE}. On other 
     * operating systems resources are always released by the thread itself on its exit
     * or on library closure with {@code dCloseODE}.
     *
     * With manual thread data cleanup mode every collision space object must be 
     * explicitly switched to manual cleanup mode with {@code dSpaceSetManualCleanup}
     * after creation. See description of the function for more details.
     *
     * If {@code dInitFlagManualThreadCleanup} was not specified during initialization,
     * calls to {@code dCleanupODEAllDataForThread} are not allowed.
     *
     * @see OdeInit#dInitODE2(int)
     * @see OdeInit#dAllocateODEDataForThread(int)
     * @see dSpaceSetManualCleanup
     * @see OdeInit#dCloseODE()
     */
    //enum dInitODEFlags {
    /** Thread local data is to be cleared explicitly on 
     * {@code dCleanupODEAllDataForThread function call.} */
    private static final int dInitFlagManualThreadCleanup = 0x00000001; 
    //};

   /**
     * ODE data allocation flags.
     *
     * These flags are used to indicate which data is to be pre-allocated in call to
     * {@code dAllocateODEDataForThread}.
     *
     * {@code dAllocateFlagBasicData} tells to allocate the basic data set required for
     * normal library operation. This flag is equal to zero and is always implicitly 
     * included.
     *
     * {@code dAllocateFlagCollisionData} tells that collision detection data is to be allocated.
     * Collision detection functions may not be called if the data has not be allocated 
     * in advance. If collision detection is not going to be used, it is not necessary
     * to specify this flag.
     *
     * {@code dAllocateMaskAll} is a mask that can be used for for allocating all possible 
     * data in cases when it is not known what exactly features of ODE will be used.
     * The mask may not be used in combination with other flags. It is guaranteed to
     * include all the current and future legal allocation flags. However, mature 
     * applications should use explicit flags they need rather than allocating everything.
     *
     * @see OdeInit#dAllocateODEDataForThread(int)
     */
    //enum dAllocateODEDataFlags {
    public final static int dAllocateFlagBasicData = 0; //@< Allocate basic data required for library to operate

    private static final int dAllocateFlagCollisionData = 0x00000001; //@< Allocate data for collision detection

    private static final int dAllocateMaskAll = ~0; //@< Allocate all the possible data that is currently defined or will be defined in the future.
    //};

    private OdeInit() {}
}
