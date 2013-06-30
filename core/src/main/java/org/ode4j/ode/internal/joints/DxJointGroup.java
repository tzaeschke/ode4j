/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2013 Tilmann Zaeschke     *
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
    public void dJointGroupEmpty ()
    {
        // the joints in this group are detached starting from the most recently
        // added (at the top of the stack). this helps ensure that the various
        // linked lists are not traversed too much, as the joints will hopefully
        // be at the start of those lists.
        // if any group joints have their world pointer set to 0, their world was
        // previously destroyed. no special handling is required for these joints.
        
        //COM.dAASSERT (group);
//        int i;
//        dxJoint **jlist = (dxJoint**) ALLOCA (group.num * sizeof(dxJoint*));
//        dxJoint *j = (dxJoint*) group.stack.rewind();
//        dxJoint[]jlist = new dxJoint[num]; //(dxJoint**) ALLOCA (group.num * sizeof(dxJoint*));
//        dxJoint j = stack.rewind();
//        for (i=0; i < num; i++) {
//            jlist[i] = j;
//            j = (dxJoint) (stack.next());//j.size()));
//        }
//        for (i=num-1; i >= 0; i--) {
//            if (jlist[i].world != null) {
//            	jlist[i].removeJointReferencesFromAttachedBodies ();
//                jlist[i].removeObjectFromList ();
//                jlist[i].world.nj--;
//                jlist[i].DESTRUCTOR();//~dxJoint();
//            }
//        }
//        num = 0;
//        stack.freeAll();
        for (int i = _stack.size()-1; i >= 0; i--) {
        	DxJoint j = _stack.get(i);
            if (j.world != null) {
            	j.removeJointReferencesFromAttachedBodies ();
                j.removeObjectFromList ();
                j.world.nj--;
                j.DESTRUCTOR();//~dxJoint();
            }
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
