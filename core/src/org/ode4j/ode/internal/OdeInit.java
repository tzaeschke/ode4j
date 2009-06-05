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

import org.cpp4j.java.RefBoolean;
import org.ode4j.ode.OdeConstants;

import static org.ode4j.ode.OdeMath.*;


/**
 * ODE initialization/finalization code
 */
public class OdeInit {
	//#include <ode/common.h>
	//#include <ode/odemath.h>
	//#include <ode/odeinit.h>
	//#include "config.h"
	//#include "collision_kernel.h"
	//#include "collision_trimesh_internal.h"
	//#include "odetls.h"
	//#include "odeou.h"


	//****************************************************************************
	// Thread local data allocators and providers

	//TODO
	//#if dTLS_ENABLED
	//#if dTRIMESH_ENABLED 
	//
	//#if dTRIMESH_OPCODE
	//
	//static Opcode::ThreadLocalData *ProvideOpcodeThreadLocalData()
	//{
	//	Opcode::ThreadLocalData *pldOpcodeData = (Opcode::ThreadLocalData *)COdeTls::GetTrimeshCollisionLibraryData();
	//	return pldOpcodeData;
	//}
	//
	//#endif // dTRIMESH_OPCODE
	//
	//#if dTRIMESH_GIMPACT
	//
	//// No thread local data provider for GIMPACT
	//
	//#endif // dTRIMESH_GIMPACT
	//
	//#endif // dTRIMESH_ENABLED
	//#endif // dTLS_ENABLED


	//enum
	//{
	private static final int
	TLD_INTERNAL_COLLISIONDATA_ALLOCATED = 0x00000001;
	//};

	private static boolean AllocateThreadBasicDataIfNecessary()
	{
		boolean bResult = false;

		do
		{
			//#if dTLS_ENABLED TODO
			//
			//		const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags();
			//
			//		// If no flags are set it may mean that TLS slot is not allocated yet
			//		if (uDataAllocationFlags == 0)
			//		{
			//			// Assign zero flags to make sure that TLS slot has been allocated
			//			if (!COdeTls::AssignDataAllocationFlags(0))
			//			{
			//				break;
			//			}
			//		}
			//
			//#endif // #if dTLS_ENABLED

			bResult = true;
		}
		while (false);

		return bResult;
	}

	private static void FreeThreadBasicDataOnFailureIfNecessary()
	{
		//#if dTLS_ENABLED TODO
		//
		//	const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags();
		//
		//	if (uDataAllocationFlags == 0)
		//	{
		//		// So far, only free TLS slot, if no subsystems have data allocated
		//		COdeTls::CleanupForThread();
		//	}
		//
		//#endif // #if dTLS_ENABLED
	}

	//#if dTLS_ENABLED TODO
	//static bool AllocateThreadCollisionData()
	//{
	//	bool bResult = false;
	//
	//	bool bCollidersCacheAllocated = false, bCollisionLibraryDataAllocated = false;
	//
	//	do
	//	{
	//		dIASSERT(!(COdeTls::GetDataAllocationFlags() & TLD_INTERNAL_COLLISIONDATA_ALLOCATED));
	//
	//#if dTRIMESH_ENABLED 
	//
	//		TrimeshCollidersCache *pccColliderCache = new TrimeshCollidersCache();
	//		if (!COdeTls::AssignTrimeshCollidersCache(pccColliderCache))
	//		{
	//			delete pccColliderCache;
	//			break;
	//		}
	//
	//		bCollidersCacheAllocated = true;
	//
	//#if dTRIMESH_OPCODE
	//
	//		Opcode::ThreadLocalData *pldOpcodeData = new Opcode::ThreadLocalData();
	//		if (!COdeTls::AssignTrimeshCollisionLibraryData((void *)pldOpcodeData))
	//		{
	//			delete pldOpcodeData;
	//			break;
	//		}
	//
	//#endif // dTRIMESH_OPCODE
	//
	//#if dTRIMESH_GIMPACT
	//
	//		// N thread local data for GIMPACT
	//
	//#endif // dTRIMESH_GIMPACT
	//
	//		bCollisionLibraryDataAllocated = true;
	//
	//#endif // dTRIMESH_ENABLED
	//
	//		COdeTls::SignalDataAllocationFlags(TLD_INTERNAL_COLLISIONDATA_ALLOCATED);
	//
	//		bResult = true;
	//	}
	//	while (false);
	//
	//	if (!bResult)
	//	{
	//		if (bCollisionLibraryDataAllocated)
	//		{
	//			COdeTls::DestroyTrimeshCollisionLibraryData();
	//		}
	//
	//		if (bCollidersCacheAllocated)
	//		{
	//			COdeTls::DestroyTrimeshCollidersCache();
	//		}
	//	}
	//	
	//	return bResult;
	//}
	//#endif // dTLS_ENABLED

	//static boolean AllocateThreadCollisionDataIfNecessary(boolean &bOutDataAllocated)
	private static boolean AllocateThreadCollisionDataIfNecessary(RefBoolean bOutDataAllocated)
	{
		boolean bResult = false;
		bOutDataAllocated.b = false;

		do 
		{
			//#if dTLS_ENABLED TODO
			//
			//		const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags();
			//
			//		if ((uDataAllocationFlags & TLD_INTERNAL_COLLISIONDATA_ALLOCATED) == 0)
			//		{
			//			if (!AllocateThreadCollisionData())
			//			{
			//				break;
			//			}
			//
			//			bOutDataAllocated = true;
			//		}
			//
			//#endif // #if dTLS_ENABLED

			bResult = true;
		}
		while (false);

		return bResult;
	}

	private static void FreeThreadCollisionData()
	{
		//#if dTLS_ENABLED TODO
		//
		//	COdeTls::DestroyTrimeshCollisionLibraryData();
		//	COdeTls::DestroyTrimeshCollidersCache();
		//
		//	COdeTls::DropDataAllocationFlags(TLD_INTERNAL_COLLISIONDATA_ALLOCATED);
		//
		//#endif // dTLS_ENABLED
	}


	//****************************************************************************
	// initialization and shutdown routines - allocate and initialize data,
	// cleanup before exiting

	private static boolean g_bODEInitialized = false;

	public static void dInitODE()
	{
		int bInitResult = dInitODE2(0);
		dIASSERT(bInitResult); dVARIABLEUSED(bInitResult);

		int ibAllocResult = dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);
		dIASSERT(ibAllocResult); dVARIABLEUSED(ibAllocResult);
	}

	//int dInitODE2(unsigned int uiInitFlags/*=0*/)
	public static int dInitODE2(int uiInitFlags/*=0*/)
	{
		dIASSERT(!g_bODEInitialized); // ODE can not be initialized twice

		int bResult = 0;

		//#if dOU_ENABLED TODO
		//	boolean bOUCustomizationsDone = false;
		//#endif
		//#if dATOMICS_ENABLED TODO
		//	boolean bAtomicsInitialized = false;
		//#endif
		//#if dTLS_ENABLED TODO
		//	boolean bTlsInitialized = false;
		//#endif

		do
		{
			//#if dOU_ENABLED TODO
			//		if (!COdeOu::DoOUCustomizations())
			//		{
			//			break;
			//		}
			//
			//		bOUCustomizationsDone = true;
			//#endif
			//
			//#if dATOMICS_ENABLED TODO
			//		if (!COdeOu::InitializeAtomics())
			//		{
			//			break;
			//		}
			//
			//		bAtomicsInitialized = true;
			//#endif
			//
			//#if dTLS_ENABLED TODO
			//		unsigned int uiTlsFlags = 0;
			//
			//		if (uiInitFlags & dInitFlagManualThreadCleanup)
			//		{
			//			uiTlsFlags |= COdeTls::MANUAL_DATA_CLEANUP;
			//		}
			//
			//		if (!COdeTls::Initialize(uiTlsFlags))
			//		{
			//			break;
			//		}
			//
			//		bTlsInitialized = true;
			//#endif

			//#if dTRIMESH_ENABLED && dTRIMESH_OPCODE TODO
			//		Opcode::ThreadLocalDataProviderProc pfnOpcodeDataProviderProc;
			//#if dTLS_ENABLED TODO
			//		pfnOpcodeDataProviderProc = &ProvideOpcodeThreadLocalData;
			//#else // dTLS_ENABLED
			//		pfnOpcodeDataProviderProc = NULL;
			//#endif // dTLS_ENABLED
			//		if (!Opcode::InitOpcode(pfnOpcodeDataProviderProc))
			//		{
			//			break;
			//		}
			//#endif

			//#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT TODO
			//		gimpact_init();
			//#endif

			//TODO put into a different class?
			DxGeom.dInitColliders();

			g_bODEInitialized = true;
			bResult = 1;
		}
		while (false);

		if (bResult==0)
		{
			//#if dTLS_ENABLED TODO
			//		if (bTlsInitialized)
			//		{
			//			COdeTls::Finalize();
			//		}
			//#endif

			//#if dATOMICS_ENABLED TODO
			//		if (bAtomicsInitialized)
			//		{
			//			COdeOu::FinalizeAtomics();
			//		}
			//#endif

			//#if dOU_ENABLED TODO
			//		if (bOUCustomizationsDone)
			//		{
			//			COdeOu::UndoOUCustomizations();
			//		}
			//#endif
		}

		return bResult;
	}


	// TODO remove ? TZ disabled for now, may not be required at all.
//	/**
//	 * @brief ODE data allocation flags.
//	 *
//	 * These flags are used to indicate which data is to be pre-allocated in call to
//	 * @c dAllocateODEDataForThread.
//	 *
//	 * @c dAllocateFlagBasicData tells to allocate the basic data set required for
//	 * normal library operation. This flag is equal to zero and is always implicitly
//	 * included.
//	 *
//	 * @c dAllocateFlagCollisionData tells that collision detection data is to be allocated.
//	 * Collision detection functions may not be called if the data has not be allocated
//	 * in advance. If collision detection is not going to be used, it is not necessary
//	 * to specify this flag.
//	 *
//	 * @c dAllocateMaskAll is a mask that can be used for for allocating all possible
//	 * data in cases when it is not known what exactly features of ODE will be used.
//	 * The mask may not be used in combination with other flags. It is guaranteed to
//	 * include all the current and future legal allocation flags. However, mature
//	 * applications should use explicit flags they need rather than allocating everything.
//	 *
//	 * @see dAllocateODEDataForThread
//	 * @ingroup init
//	 */
//	//public enum dAllocateODEDataFlags {
//	public static final int
//	dAllocateFlagBasicData = 0; //@< Allocate basic data required for library to operate
//
	/** @deprecated TZ: probably not required. */
	private static final int
	dAllocateFlagCollisionData = 0x00000001; //@< Allocate data for collision detection

	//int dAllocateODEDataForThread(unsigned int uiAllocateFlags)
	public static int dAllocateODEDataForThread(int uiAllocateFlags)
	{
		dIASSERT(g_bODEInitialized); // Call dInitODEEx first

		int bResult = 0;

		RefBoolean bCollisionDataAllocated = new RefBoolean(false);

		do
		{
			if (!AllocateThreadBasicDataIfNecessary())
			{
				break;
			}

			if ((uiAllocateFlags & dAllocateFlagCollisionData)!=0)
			{
				if (!AllocateThreadCollisionDataIfNecessary(bCollisionDataAllocated))
				{
					break;
				}
			}

			bResult = 1;
		}
		while (false);

		if (bResult == 0)
		{
			if (bCollisionDataAllocated.b)
			{
				FreeThreadCollisionData();
			}

			FreeThreadBasicDataOnFailureIfNecessary();
		}

		return bResult;
	}

	public static void dCleanupODEAllDataForThread()
	{
		dIASSERT(g_bODEInitialized); // Call dInitODEEx first or delay dCloseODE until all threads exit

		//#if dTLS_ENABLED TODO
		//	COdeTls::CleanupForThread();
		//#endif
	}


	public static void dCloseODE()
	{
		dIASSERT(g_bODEInitialized); // dCloseODE must not be called without dInitODEEx or if dInitODEEx fails

		g_bODEInitialized = false;

		CollideSpaceGeom.dClearPosrCache();
		DxGeom.dFinitUserClasses();
		DxGeom.dFinitColliders();

		//#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT TODO
		//	gimpact_terminate();
		//#endif

		//#if dTRIMESH_ENABLED && dTRIMESH_OPCODE TODO
		//	extern void opcode_collider_cleanup();
		//	// Free up static allocations in opcode
		//	opcode_collider_cleanup();
		//
		//	Opcode::CloseOpcode();
		//#endif

		//#if dTLS_ENABLED TODO
		//	COdeTls::Finalize();
		//#endif

		//#if dATOMICS_ENABLED TODO
		//	COdeOu::FinalizeAtomics();
		//#endif

		//#if dOU_ENABLED TODO
		//	COdeOu::UndoOUCustomizations();
		//#endif
	}
}

