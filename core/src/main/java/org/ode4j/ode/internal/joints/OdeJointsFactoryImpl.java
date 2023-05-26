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
package org.ode4j.ode.internal.joints;

import static org.ode4j.ode.internal.Common.*;

import java.util.LinkedList;
import java.util.List;

import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;


/**
 * Factory for Joints.
 */
public class OdeJointsFactoryImpl {


	//****************************************************************************
	// joints



	//template<class T>
	//dxJoint XXcreateJoint(dWorld w, dxJointGroup group)
	//{
	//    dxJoint j;
	//	if (group) {
	// 	   j = group->alloc<T>(w);
	//	} else {
	//	    j = new T(w);
	//	}
	//    return j;
	//}
	private <T extends DxJoint> T createJoint(T j, DJointGroup group)
	{
		//TODO move this into dxJoint constructor? (TZ)
		if (group != null) {
			((DxJointGroup)group).addJoint(j);
		}
		return j;
	}


	public DxJointBall dJointCreateBall (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointBall((DxWorld) w),group);
	}


	public DxJointConstrainedBall dJointCreateConstrainedBall (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointConstrainedBall(w),group);
	}


	public DxJointHinge dJointCreateHinge (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointHinge((DxWorld) w),group);
	}


	public DxJointSlider dJointCreateSlider (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointSlider((DxWorld) w),group);
	}


//	dxJoint dJointCreateContact (dWorld w, dJointGroup group,
//			final dContact[] c)
	public DxJointContact dJointCreateContact (DWorld w, DJointGroup group,
			final DContact c)
	{
		dAASSERT (w, c);
		DxJointContact j = createJoint(new DxJointContact((DxWorld) w), group);
		j.setContact(c);
		return j;
	}


	public DxJointHinge2 dJointCreateHinge2 (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointHinge2((DxWorld) w),group);
	}


	public DxJointUniversal dJointCreateUniversal (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointUniversal((DxWorld) w), group);
	}

	public DxJointPR dJointCreatePR (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPR((DxWorld) w),group);
	}

	public DxJointPU  dJointCreatePU (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPU((DxWorld) w),group);
	}

	public DxJointPiston  dJointCreatePiston (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPiston((DxWorld) w),group);
	}

	public DxJointFixed dJointCreateFixed (DWorld id, DJointGroup group)
	{
		dAASSERT (id);
		return createJoint( new DxJointFixed((DxWorld)id),group);
	}


	public DxJointNull dJointCreateNull (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointNull((DxWorld) w),group);
	}


	public DxJointAMotor dJointCreateAMotor (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointAMotor((DxWorld) w),group);
	}

	public DxJointLMotor dJointCreateLMotor (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointLMotor ((DxWorld) w),group);
	}

	public DxJointPlane2D dJointCreatePlane2D (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPlane2D((DxWorld) w),group);
	}

	public DxJointDBall dJointCreateDBall (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointDBall((DxWorld) w),group);
	}

	public DxJointDHinge dJointCreateDHinge (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointDHinge((DxWorld) w),group);
	}

	public DxJointTransmission dJointCreateTransmission (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointTransmission((DxWorld) w),group);
	}

	//TZ: Moved to DxJoint
//	static void FinalizeAndDestroyJointInstance(DxJoint j, boolean delete_it)
//	{
//	    // if any group joints have their world pointer set to 0, their world was
//	    // previously destroyed. no special handling is required for these joints.
//	    if (j.world != null) {
//	        j.removeJointReferencesFromAttachedBodies ();
//	        j.removeObjectFromList ();
//	        j.world.nj--;
//	    }
//	    if (delete_it) { 
//	        //delete j;
//	    } else {
//	        j.DESTRUCTOR();//j->~dxJoint();
//	    }
//	}
	
	protected static void dJointDestroy (DxJoint j)
	{
	    //dAASSERT (j);
	    if ((j.flags & DxJoint.dJOINT_INGROUP)==0) {
	        j.FinalizeAndDestroyJointInstance(true);
	    }
	}


	//dJointGroup dJointGroupCreate (int max_size)
	//{
	//    // not any more ... dUASSERT (max_size > 0,"max size must be > 0");
	//    dxJointGroup *group = new dxJointGroup;
	//    group->num = 0;
	//    return group;
	//}
	//
	//
	//void dJointGroupDestroy (dJointGroup group)
	//{
	//    dAASSERT (group);
	//    dJointGroupEmpty (group);
	//    delete group;
	//}
	//
	//
	//void dJointGroupEmpty (dJointGroup group)
	//{
	//    // the joints in this group are detached starting from the most recently
	//    // added (at the top of the stack). this helps ensure that the various
	//    // linked lists are not traversed too much, as the joints will hopefully
	//    // be at the start of those lists.
	//    // if any group joints have their world pointer set to 0, their world was
	//    // previously destroyed. no special handling is required for these joints.
	//    
	//    dAASSERT (group);
	//    int i;
	//    dxJoint **jlist = (dxJoint**) ALLOCA (group->num * sizeof(dxJoint*));
	//    dxJoint *j = (dxJoint*) group->stack.rewind();
	//    for (i=0; i < group->num; i++) {
	//        jlist[i] = j;
	//        j = (dxJoint*) (group->stack.next (j->size()));
	//    }
	//    for (i=group->num-1; i >= 0; i--) {
	//        if (jlist[i]->world) {
	//            removeJointReferencesFromAttachedBodies (jlist[i]);
	//            removeObjectFromList (jlist[i]);
	//            jlist[i]->world->nj--;
	//            jlist[i]->~dxJoint();
	//        }
	//    }
	//    group->num = 0;
	//    group->stack.freeAll();
	//}
	//
	//int dJointGetNumBodies(dxJoint joint)
	//{
	//    // check arguments
	//    dUASSERT (joint,"bad joint argument");
	//
	//    if ( !joint.node[0].body )
	//        return 0;
	//    else if ( !joint.node[1].body )
	//        return 1;
	//    else
	//        return 2;
	//}
	//
	//

	public void dJointAttach (DxJoint joint, DxBody body1, DxBody body2)
	{	
		joint.dJointAttach(body1, body2);
	}

	//void dJointAttach (dxJoint joint, dxBody body1, dxBody body2)
	//{
	//  // check arguments
	//  dUASSERT (joint,"bad joint argument");
	//  dUASSERT (body1 == 0 || body1 != body2,"can't have body1==body2");
	//  dxWorld world = joint.world;
	//  dUASSERT ( (!body1 || body1.world == world) &&
	//	     (!body2 || body2.world == world),
	//	     "joint and bodies must be in same world");
	//
	//  // check if the joint can not be attached to just one body
	//  dUASSERT (!((jointflags & dJOINT_TWOBODIES) &&
	//	      ((body1 != 0) ^ (body2 != 0))),
	//	    "joint can not be attached to just one body");
	//
	//  // remove any existing body attachments
	//  if (joint.node[0].body || joint.node[1].body) {
	//    removeJointReferencesFromAttachedBodies (joint);
	//  }
	//
	//  // if a body is zero, make sure that it is body2, so 0 --> node[1].body
	//  if (body1==null) {
	//    body1 = body2;
	//    body2 = null;
	//    joint.flags |= dJOINT_REVERSE;
	//  }
	//  else {
	//    joint.flags &= (~dJOINT_REVERSE);
	//  }
	//
	//  // attach to new bodies
	//  joint.node[0].body = body1;
	//  joint.node[1].body = body2;
	//  if (body1!=null) {
	//    joint.node[1].next = body1.firstjoint;
	//    body1.firstjoint = joint.node[1];
	//  }
	//  else joint.node[1].next = 0;
	//  if (body2!=null) {
	//    joint.node[0].next = body2.firstjoint;
	//    body2.firstjoint = joint.node[0];
	//  }
	//  else {
	//    joint.node[0].next = 0;
	//  }
	//}
	//
	//
	//void dJointSetData (dxJoint joint, void data)
	//{
	//  dAASSERT (joint);
	//  joint.userdata = data;
	//}
	//
	//
	//void dJointGetData (dxJoint joint)
	//{
	//  dAASSERT (joint);
	//  return joint.userdata;
	//}
	//
	//
	//dJointType dJointGetType (dxJoint joint)
	//{
	//  dAASSERT (joint);
	//  return joint.type();
	//}
	//
	//
	//dBody dJointGetBody (dxJoint joint, int index)
	//{
	//  dAASSERT (joint);
	//  if (index == 0 || index == 1) {
	//    if (joint.flags & dJOINT_REVERSE) return joint.node[1-index].body;
	//    else return joint.node[index].body;
	//  }
	//  else return 0;
	//}
	//
	//
	//void dJointSetFeedback (dxJoint joint, dJointFeedback f)
	//{
	//  dAASSERT (joint);
	//  joint.feedback = f;
	//}
	//
	//
	//dJointFeedback dJointGetFeedback (dxJoint joint)
	//{
	//  dAASSERT (joint);
	//  return joint.feedback;
	//}
	//
	//
	//
	public static DJoint dConnectingJoint (DBody in_b1, DBody in_b2)
	{
		dAASSERT (in_b1!=null || in_b2!=null);

		DxBody b1, b2;

		if (in_b1 == null) {
			b1 = (DxBody) in_b2;
			b2 = (DxBody) in_b1;
		}
		else {
			b1 = (DxBody) in_b1;
			b2 = (DxBody) in_b2;
		}

		// look through b1's neighbour list for b2
		for (DxJointNode n=b1.firstjoint.get(); n!=null; n=n.next) {
			if (n.body == b2) return n.joint;
		}

		return null;
	}



	//int dConnectingJointList (DxBody in_b1, DxBody in_b2, DxJoint[] out_list)
	public static List<DJoint> dConnectingJointList (DxBody in_b1, DxBody in_b2)
	{
		dAASSERT (in_b1!=null || in_b2!=null);

		List<DJoint> out_list = new LinkedList<>();
		
		DxBody b1, b2;

		if (in_b1 == null) {
			b1 = in_b2;
			b2 = in_b1;
		}
		else {
			b1 = in_b1;
			b2 = in_b2;
		}

		// look through b1's neighbour list for b2
		//int numConnectingJoints = 0;
		for (DxJointNode n=b1.firstjoint.get(); n!=null; n=n.next) {
			if (n.body == b2)
				//out_list[numConnectingJoints++] = n.joint;
				out_list.add(n.joint);
		}

		return out_list;//numConnectingJoints;
	}


	public boolean _dAreConnected (DBody b1, DBody b2)
	{
		//dAASSERT (b1!=null);// b2 can be null
		// look through b1's neighbour list for b2
		for (DxJointNode n=((DxBody)b1).firstjoint.get(); n!=null; n=n.next) {
			if (n.body == b2) return true;
		}
		return false;
	}


	@SafeVarargs
	public final boolean _dAreConnectedExcluding(DBody b1, DBody b2, Class<? extends DJoint>... jointTypes)
	{
		//dAASSERT (b1!=null);// b2 can be null
		// look through b1's neighbour list for b2
		for (DxJointNode n=((DxBody)b1).firstjoint.get(); n!=null; n=n.next) {
			if ( n.body == b2) {
				boolean found = false;
				Class<?> clsJoint = n.joint.getClass();
				for (Class<?> cls: jointTypes) {
					if ( cls == clsJoint )  {
						found = true;
						break;
					}
				}
				if (!found) return true;
			}
		}
		return false;
	}

	protected OdeJointsFactoryImpl() {}
}
