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

import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.joints.OdeJointsFactoryImpl;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.ErrorHandler.*;
import static org.ode4j.ode.internal.cpp4j.Cstdio.*;


/**
 * this source file is mostly concerned with the data structures, not the
 * numerics.
 */
public class OdeFactoryImpl extends OdeJointsFactoryImpl {

	// misc defines
	//#define ALLOCA dALLOCA16
	//private static final String ALLOCA = "ALLOCA16";

	//****************************************************************************
	// utility

	//Moved to Objects_H.java

	

	//// remove the joint from neighbour lists of all connected bodies
	//
	//static void removeJointReferencesFromAttachedBodies (dxJoint j)
	//{
	//	for (int i=0; i<2; i++) {
	//		dxBody body = j.node[i].body;
	//		if (body) {
	//			dxJointNode n = body.firstjoint;
	//			dxJointNode last = null;
	//			while (n != null) {
	//				if (n.joint == j) {
	//					if (last) last->next = n.next;
	//					else body.firstjoint = n.next;
	//					break;
	//				}
	//				last = n;
	//				n = n.next;
	//			}
	//		}
	//	}
	//	j.node[0].body = 0;
	//	j.node[0].next = 0;
	//	j.node[1].body = 0;
	//	j.node[1].next = 0;
	//}
	//
	//****************************************************************************
	// debugging

	// see if an object list loops on itself (if so, it's bad).

	static <T extends DObject> boolean listHasLoops (Ref<T>  first)
	{
		//  if (first==null || first.next==null) return 0;
		//  dObject *a=first,*b=first->next;
		//  int skip=0;
		//  while (b) {
		//    if (a==b) return 1;
		//    b = b.next;
		//    if (skip != 0) a = a.next;
		//    skip ^= 1;
		//  }
		//  return 0;
		if (first.get()==null || first.get().getNext()==null) return false;
		DObject a=first.get(),b=first.get().getNext();
		int skip=0;
		while (b != null) {
			if (a==b) return true;
			b = b.getNext();
			if (skip != 0) a = a.getNext();
			skip ^= 1;
		}
		return false;
	}


	// check the validity of the world data structures

	private static int g_world_check_tag_generator = 0;

	static int generateWorldCheckTag()
	{
		// Atomicity is not necessary here
		return ++g_world_check_tag_generator;
	}

	@SuppressWarnings("unused")
	static void checkWorld (DxWorld w)
	{
		DxBody b;
		DxJoint j;

		// check there are no loops
		if (listHasLoops (w.firstbody)) dDebug (0,"body list has loops");
		if (listHasLoops (w.firstjoint)) dDebug (0,"joint list has loops");

		// check lists are well-formed (check `tome' pointers)
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) {
			if (b.getNext()!=null && b.getNext().getTome() != b.getNext())
				dDebug (0,"bad tome pointer in body list");
		}
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			if (j.getNext()!=null && j.getNext().getTome() != j.getNext())
				dDebug (0,"bad tome pointer in joint list");
		}

		// check counts
		int nn = 0;
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) nn++;
		if (w.nb != nn) dDebug (0,"body count incorrect");
		nn = 0;
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) nn++;
		if (w.nj != nn) dDebug (0,"joint count incorrect");

		// set all tag values to a known value
		int count = generateWorldCheckTag();
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) b.tag = count;
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) j.tag = count;

		// check all body/joint world pointers are ok
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) if (b.world != w)
			dDebug (0,"bad world pointer in body list");
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) if (j.world != w)
			dDebug (0,"bad world pointer in joint list");

		/*
  // check for half-connected joints - actually now these are valid
  for (j=w.firstjoint; j; j=(dxJoint*)j.next) {
    if (j.node[0].body || j.node[1].body) {
      if (!(j.node[0].body && j.node[1].body))
	dDebug (0,"half connected joint found");
    }
  }
		 */

		// check that every joint node appears in the joint lists of both bodies it
		// attaches
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			for (int i=0; i<2; i++) {
				if (j.node[i].body!=null) {
					int ok = 0;
					for (DxJointNode n=j.node[i].body.firstjoint.get(); n!=null; n=n.next) {
						if (n.joint == j) ok = 1;
					}
					if (ok==0) dDebug (0,"joint not in joint list of attached body");
				}
			}
		}

		// check all body joint lists (correct body ptrs)
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) {
			for (DxJointNode n=b.firstjoint.get(); n!=null; n=n.next) {
				if (n.joint.node[0] == n) {
					if (n.joint.node[1].body != b)
						dDebug (0,"bad body pointer in joint node of body list (1)");
				}
				else {
					if (n.joint.node[0].body != b)
						dDebug (0,"bad body pointer in joint node of body list (2)");
				}
				if (n.joint.tag != count) dDebug (0,"bad joint node pointer in body");
			}
		}

		// check all body pointers in joints, check they are distinct
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			if (j.node[0].body!=null && (j.node[0].body == j.node[1].body))
				dDebug (0,"non-distinct body pointers in joint");
			if ((j.node[0].body!=null && j.node[0].body.tag != count) ||
					(j.node[1].body!=null && j.node[1].body.tag != count))
				dDebug (0,"bad body pointer in joint");
		}
	}


	void dWorldCheck (DxWorld w)
	{
		checkWorld (w);
	}

	//****************************************************************************
	// body

	//TZ moved to dxBody



	//****************************************************************************
	// joints




	//****************************************************************************
	// testing

	//#define NUM 100
	private static final int NUM = 100; 

	//#define DO(x)
	private static boolean DO = false;
	private static void DO_printf(String msg, Object... args) {
		if (DO) {
			printf(msg, args);
		}
	}

	//extern "C" 
	public void dTestDataStructures()
	{
		int i;
		DO_printf ("testDynamicsStuff()\n");

		DxBody[] body = new DxBody[NUM];
		int nb = 0;
		DxJoint[] joint = new DxJoint[NUM];
		int nj = 0;

		for (i=0; i<NUM; i++) body[i] = null;
		for (i=0; i<NUM; i++) joint[i] = null;

		DO_printf ("creating world\n");
		DxWorld w = DxWorld.dWorldCreate();
		checkWorld (w);

		for (int round = 0; round < 1000; ++round) {
			if (nb < NUM && dRandReal() > 0.5) {
				DO_printf ("creating body\n");
				body[nb] = DxBody.dBodyCreate (w);
				DO_printf ("\t--> %s\n",body[nb].toString());
				nb++;
				checkWorld (w);
				DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
			if (nj < NUM && nb > 2 && dRandReal() > 0.5) {
				DxBody b1 = body [(int) (dRand() % nb)];
				DxBody b2 = body [(int) (dRand() % nb)];
				if (b1 != b2) {
					DO_printf ("creating joint, attaching to %s,%s\n",b1.toString(),b2.toString());
					joint[nj] = dJointCreateBall (w,null);
					DO_printf ("\t-->%s\n",joint[nj].toString());
					checkWorld (w);
					joint[nj].dJointAttach (b1,b2);
					nj++;
					checkWorld (w);
					DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
				}
			}
			if (nj > 0 && nb > 2 && dRandReal() > 0.5) {
				DxBody b1 = body [(int) (dRand() % nb)];
				DxBody b2 = body [(int) (dRand() % nb)];
				if (b1 != b2) {
					int k = (int) (dRand() % nj);
					DO_printf ("reattaching joint %s\n",joint[k].toString());
					joint[k].dJointAttach (b1,b2);
					checkWorld (w);
					DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
				}
			}
			if (nb > 0 && dRandReal() > 0.5) {
				int k = (int) (dRand() % nb);
				DO_printf ("destroying body %s\n",body[k].toString());
				body[k].dBodyDestroy ();
				checkWorld (w);
				for (; k < (NUM-1); k++) body[k] = body[k+1];
				nb--;
				DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
			if (nj > 0 && dRandReal() > 0.5) {
				int k = (int) (dRand() % nj);
				DO_printf ("destroying joint %s\n",joint[k].toString());
				dJointDestroy (joint[k]);
				checkWorld (w);
				for (; k < (NUM-1); k++) joint[k] = joint[k+1];
				nj--;
				DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
		}

		/*
  printf ("creating world\n");
  dWorld w = dWorldCreate();
  checkWorld (w);
  printf ("creating body\n");
  dBody b1 = dBodyCreate (w);
  checkWorld (w);
  printf ("creating body\n");
  dBody b2 = dBodyCreate (w);
  checkWorld (w);
  printf ("creating joint\n");
  dJoint j = dJointCreateBall (w);
  checkWorld (w);
  printf ("attaching joint\n");
  dJointAttach (j,b1,b2);
  checkWorld (w);
  printf ("destroying joint\n");
  dJointDestroy (j);
  checkWorld (w);
  printf ("destroying body\n");
  dBodyDestroy (b1);
  checkWorld (w);
  printf ("destroying body\n");
  dBodyDestroy (b2);
  checkWorld (w);
  printf ("destroying world\n");
  dWorldDestroy (w);
		 */
	}

	//****************************************************************************
	// configuration
//	#if 1
//	#define REGISTER_EXTENSION( __a )  #__a " "
//	#else
//		#define REGISTER_EXTENSION( __a )  "__a "
//	#endif
	//private Object REGISTER_EXTENSION;
	//TODO ?
//	if (true) {
//		REGISTER_EXTENSION( __a )  #__a " "
//	#else
//		#define REGISTER_EXTENSION( __a )  "__a "
//	#endif
	private static void REGISTER_EXTENSION(String s) {
		ode_configuration += s + " ";
	}

	//  String[] ?!?!?
	private static String ode_configuration = "ODE ";

		// EXTENSION LIST BEGIN
		//**********************************

//		#ifdef dNODEBUG
//		REGISTER_EXTENSION( ODE_EXT_no_debug )
//	#endif // dNODEBUG
//
//	#if dTRIMESH_ENABLED
//	REGISTER_EXTENSION( ODE_EXT_trimesh )
//
//	// tri-mesh extensions
//	#if dTRIMESH_OPCODE
//	REGISTER_EXTENSION( ODE_EXT_opcode )
//
//	// opcode extensions
//	#if dTRIMESH_16BIT_INDICES
//	REGISTER_EXTENSION( ODE_OPC_16bit_indices )
//	#endif
//
//	#if !dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER
//	REGISTER_EXTENSION( ODE_OPC_new_collider )
//	#endif
//
//	#endif // dTRIMESH_OPCODE
//
//	#if dTRIMESH_GIMPACT
//	REGISTER_EXTENSION( ODE_EXT_gimpact )
//
//	// gimpact extensions
//	#endif
//
//	#endif // dTRIMESH_ENABLED
//
//	#if dTLS_ENABLED
//	REGISTER_EXTENSION( ODE_EXT_mt_collisions )
//	#endif // dTLS_ENABLED
	static {
	if (dNODEBUG)
		REGISTER_EXTENSION( "ODE_EXT_no_debug" );

//	if (dUSE_MALLOC_FOR_ALLOCA)
//		REGISTER_EXTENSION( "ODE_EXT_malloc_not_alloca" );

	if (dTRIMESH_ENABLED) {
		REGISTER_EXTENSION( "ODE_EXT_trimesh" );

		// tri-mesh extensions
		if (dTRIMESH_OPCODE) {
			REGISTER_EXTENSION( "ODE_EXT_opcode" );

			// opcode extensions
			if (dTRIMESH_16BIT_INDICES)
				REGISTER_EXTENSION( "ODE_OPC_16bit_indices" );

			if (!dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER)
				REGISTER_EXTENSION( "ODE_OPC_new_collider" );
		} // dTRIMESH_OPCODE

		if (dTRIMESH_GIMPACT) {
			REGISTER_EXTENSION( "ODE_EXT_gimpact" );
		} // gimpact extensions

	} // dTRIMESH_ENABLED

//	if (dTLS_ENABLED) {
//		REGISTER_EXTENSION( "ODE_EXT_mt_collisions" );
//	} // dTLS_ENABLED

//	#if !dTHREADING_INTF_DISABLED
//	REGISTER_EXTENSION( ODE_EXT_threading )
//
//	#if dBUILTIN_THREADING_IMPL_ENABLED
//	REGISTER_EXTENSION( ODE_THR_builtin_impl )
//	#endif // #if dBUILTIN_THREADING_IMPL_ENABLED
//	#endif // #if !dTHREADING_INTF_DISABLED

	//**********************************
	// EXTENSION LIST END

	// These tokens are mutually exclusive, and always present
	//#ifdef dSINGLE
	//"ODE_single_precision"
	//#else
	//TZ TODO correct?
	ode_configuration +=
		"ODE_double_precision";
	//#endif // dDOUBLE

		//; // END
	}
	
	//const char* 
	public String _dGetConfiguration ()
	{
		return ode_configuration;
	}


	// Helper to check for a feature of ODE
//	int dCheckConfiguration( const char* extension )
	public boolean _dCheckConfiguration( final String extension )
	{
		//final String start;
		int start;
		//char * where, terminator;
		int where;
		int terminator;

		/* Feature names should not have spaces. */
//		where = (char*)strchr(extension, ' ');
//		if ( where || *extension == '\0')
//			return 1;
		if (extension.indexOf(' ') >= 0 || extension.length() == 0)
			return true;  //TODO TZ report. should this not return 'false' instead?

		final String config = OdeHelper.getConfiguration();

		final int ext_length = extension.length();//strlen(extension);

		/* It takes a bit of care to be fool-proof. Don't be fooled by sub-strings, etc. */
		start = 0;//config;
		for (  ; ;  )
		{
			//where = (char*)strstr((const char *) start, extension);
			where = config.indexOf(extension, start);
			if (where == -1)//if (!where)
				break;

			terminator = where + ext_length;

//			if ( (where == start || *(where - 1) == ' ') && 
//					(*terminator == ' ' || *terminator == '\0') )
			if ( (where == start || extension.charAt(where - 1) == ' ') && 
					(extension.charAt(terminator) == ' ' || 
							terminator == extension.length()) )
			{
				return true;
			}

			start = terminator;
		}

		return false;
	}

	public OdeFactoryImpl() {}
}
