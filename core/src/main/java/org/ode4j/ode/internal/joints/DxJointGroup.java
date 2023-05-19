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

import java.util.ArrayList;

import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.internal.DBase;

/** 
 * joint group. NOTE: any joints in the group that have their world destroyed
 * will have their world pointer set to 0.
 */
public class DxJointGroup extends DBase implements DJointGroup
{
//    private int num;        // number of joints on the stack
//    private dObStack<dxJoint> stack = new dObStack<dxJoint>(); // a stack of (possibly differently sized) dxJoint
//                  // objects.
    
	//TODO use LinkedList?
	private final ArrayList<DxJoint> _stack = new ArrayList<DxJoint>(); 

	public DxJointGroup() {}
	
//    template<class T>
//    T *alloc(dWorldID w)
//    {
//        T *j = (T *)m_stack.alloc(sizeof(T));
//        if (j != NULL) {
//            ++m_num;
//            new(j) T(w);
//            j->flags |= dJOINT_INGROUP;
//        }
//        return j;
//    }
//
	//TODO
	int getJointCount() { return _stack.size(); }
//
	//TODO
//    void *beginEnum() { return m_stack.rewind(); }
	//TODO
//    void *continueEnum(size_t num_bytes) { return m_stack.next(num_bytes); }
//
	//TODO
//    void freeAll();

	//  size_t exportJoints(dxJoint **jlist);
	int exportJoints(ArrayList<DxJoint> jlist)
	{
//		size_t i=0;
//		dxJoint *j = (dxJoint*) m_stack.rewind();
//		while (j != NULL) {
//			jlist[i++] = j;
//			j = (dxJoint*) (m_stack.next (j->size()));
//		}
//		return i;
		jlist.addAll(_stack);
		return _stack.size();
	}

	void freeAll()
	{
		//m_num = 0;
		//m_stack.freeAll();
		_stack.clear();
	}



	
	
    public static DxJointGroup dJointGroupCreate (int max_size)
    {
        // not any more ... dUASSERT (max_size > 0,"max size must be > 0");
        DxJointGroup group = new DxJointGroup();
//        group.num = 0;
        return group;
    }


    public void dJointGroupDestroy ()
    {
//        COM.dAASSERT (group);
        dJointGroupEmpty ();
//TZ        delete group;
//        stack = null;
        DESTRUCTOR();
    }


//    public void dJointGroupEmpty (dxJointGroup group)
    void dJointGroupEmpty ()
    {
//        final int num_joints = getJointCount();
//        if (num_joints != 0) {
//            // Local array is used since ALLOCA leads to mysterious NULL values in first array element and crashes under VS2005 :)
//            final int max_stack_jlist_size = 1024;
////            DxJoint [][]stack_jlist[max_stack_jlist_size];
////
////            final int jlist_size = num_joints * sizeof(dxJoint*);
////            dxJoint **jlist = num_joints <= max_stack_jlist_size ? stack_jlist : (dxJoint **)dAlloc(jlist_size);
//
//            if (jlist != NULL) {
//                // the joints in this group are detached starting from the most recently
//                // added (at the top of the stack). this helps ensure that the various
//                // linked lists are not traversed too much, as the joints will hopefully
//                // be at the start of those lists.
//                int num_exported = exportJoints(jlist);
//                Common.dIVERIFY(num_exported == num_joints);
//
//                for (int i = num_joints; i != 0; ) {
//                    --i;
//                    DxJoint j = jlist[i];
//                    FinalizeAndDestroyJointInstance(j, false);
//                }
//            } else {
//                // ...else if there is no memory, go on detaching the way it is possible
//                int joint_bytes;
//                for (DxJoint j = (dxJoint *)group->beginEnum(); j != NULL; j = (dxJoint *)group->continueEnum(joint_bytes)) {
//                    joint_bytes = j->size(); // Get size before object is destroyed!
//                    FinalizeAndDestroyJointInstance(j, false);
//                }
//            }
//
//            freeAll();
//
//            if (jlist != stack_jlist && jlist != NULL) {
//                dFree(jlist, jlist_size);
//            }
//        }
        for (int i = _stack.size()-1; i >= 0; i--) {
        	DxJoint j = _stack.get(i);
        	j.FinalizeAndDestroyJointInstance(false);
        }
        _stack.clear();
    }


    void addJoint(DxJoint j) {
//		stack.add(j);
//		num++;
    	_stack.add(j);
		j.setFlagsInGroup();//j.flags |= DxJoint.dJOINT_INGROUP;
    }
    
    
    // ********************************************
    // API dJointGroup
    // ********************************************
    
	@Override
	public void empty()
	{ dJointGroupEmpty (); }
	@Override
	public void clear()
	{ empty(); }


	@Override
	public void destroy() {
		dJointGroupDestroy();
	}
    
}
