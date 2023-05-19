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
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import static org.ode4j.ode.internal.ErrorHandler.*;

public class DxWorldProcessIslandsInfo {

    void AssignInfo(int islandcount, int[] islandsizes, DxBody[] bodies, DxJoint[] joints)
    {
        m_IslandCount = islandcount;
        m_pIslandSizes = islandsizes;
        m_pBodies = bodies;
        m_pBodiesP = 0;
        m_pJoints = joints;
        m_pJointsP = 0;
    }

    public int GetIslandsCount() { return m_IslandCount; }
    public int[] GetIslandSizes() { return m_pIslandSizes; }
    public DxBody[] GetBodiesArray() { return m_pBodies; }
    public int GetBodiesArrayP() { return m_pBodiesP; }
    public DxJoint[] GetJointsArray() { return m_pJoints; }
    public int GetJointsArrayP() { return m_pJointsP; }

    //private:
    private int m_IslandCount;
    private int[] m_pIslandSizes;
    private DxBody[] m_pBodies;
    private int m_pBodiesP;
    private DxJoint[] m_pJoints;
    private int m_pJointsP;

    // *******************************
    // from util.cpp (TZ)
    // *******************************

    //typedef size_t (*dmemestimate_fn_t) (dxBody * const *body, unsigned int nb, 
    //        dxJoint * const *_joint, unsigned int _nj);
    public interface dmemestimate_fn_t {
        public int dxEstimateMemoryRequirements(DxBody[] body, int bodyOfs, int nb, 
                DxJoint[] _joint, int jointOfs, int _nj);
    };


	//private enum dxISLANDSIZESELEMENT
	//{
	static final int dxISE_BODIES_COUNT = 0;
	static final int dxISE_JOINTS_COUNT = 1;
	static final int dxISE__MAX = 2;
	//};
	
    static int BuildIslandsAndEstimateStepperMemoryRequirements(
            DxWorldProcessIslandsInfo islandsinfo, DxWorldProcessMemArena memarena, 
            DxWorld world, double stepsize, dmemestimate_fn_t stepperestimate)
    {
        int maxreq = 0;

        // handle auto-disabling of bodies
        world.dInternalHandleAutoDisabling (stepsize);

        int nb = world.nb, nj = world.nj;
        // Make array for island body/joint counts
        int[] islandsizes = memarena.AllocateArrayInt(2 * nb);
        int sizescurrP;

        // make arrays for body and joint lists (for a single island) to go into
        DxBody[] body = memarena.AllocateArrayDxBody(nb);
        DxJoint[] joint = memarena.AllocateArrayDxJoint(nj);

        BlockPointer stackstate = memarena.BEGIN_STATE_SAVE();
        {
            // allocate a stack of unvisited bodies in the island. the maximum size of
            // the stack can be the lesser of the number of bodies or joints, because
            // new bodies are only ever added to the stack by going through untagged
            // joints. all the bodies in the stack must be tagged!
            int stackalloc = (nj < nb) ? nj : nb;
            DxBody[] stack = memarena.AllocateArrayDxBody(stackalloc);

            {
                // set all body/joint tags to 0
                for (DxBody b=world.firstbody.get(); b!=null; b=(DxBody)b.getNext()) b.tag = 0;
                for (DxJoint j=world.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) j.tag = 0;
            }

            sizescurrP = 0;//islandsizes;
            int bodystart = 0;//body;
            int jointstart = 0;//joint;
            for (DxBody bb=world.firstbody.get(); bb!=null; bb=(DxBody) bb.getNext()) {
                // get bb = the next enabled, untagged body, and tag it
                if (bb.tag==0) {
                    if (bb.dBodyIsEnabled()) {
                        bb.tag = 1;

                        int bodycurr = bodystart;
                        int jointcurr = jointstart;

                        // tag all bodies and joints starting from bb.
                        //*bodycurr++ = bb;
                        body[bodycurr++] = bb;

                        int stacksize = 0;
                        DxBody b = bb;

                        while (true) {
                            // traverse and tag all body's joints, add untagged connected bodies
                            // to stack
                            for (DxJointNode n=b.firstjoint.get(); n!=null; n=n.next) {
                                DxJoint njoint = n.joint;
                                if (njoint.tag==0) {
                                    if (njoint.isEnabled()) {
                                        njoint.tag = 1;
                                        //*jointcurr++ = njoint;
                                        joint[jointcurr++] = njoint;

                                        DxBody nbody = n.body;
                                        // Body disabled flag is not checked here. This is how auto-enable works.
                                        if (nbody!=null && nbody.tag <= 0) {
                                            nbody.tag = 1;
                                            // Make sure all bodies are in the enabled state.
                                            nbody.dBodyEnable_noAdis();
                                            //nbody.flags &= ~DxBody.dxBodyDisabled;
                                            stack[stacksize++] = nbody;
                                        }
                                    } else {
                                        njoint.tag = -1; // Used in Step to prevent search over disabled joints (not needed for QuickStep so far)
                                    }
                                }
                            }
                            Common.dIASSERT(stacksize <= world.nb);
                            Common.dIASSERT(stacksize <= world.nj);

                            if (stacksize == 0) {
                                break;
                            }

                            b = stack[--stacksize]; // pop body off stack
                            body[bodycurr++] = b;//*bodycurr++ = b;    // put body on body list
                        }

                        int bcount = bodycurr - bodystart;
                        int jcount = jointcurr - jointstart;
                        Common.dIASSERT((bodycurr - bodystart) <= Integer.MAX_VALUE);//UINT_MAX);
                        Common.dIASSERT((jointcurr - jointstart) <= Integer.MAX_VALUE);//UINT_MAX);

                        islandsizes[sizescurrP+dxISE_BODIES_COUNT] = bcount;
                        islandsizes[sizescurrP+dxISE_JOINTS_COUNT] = jcount;
                        sizescurrP += dxISE__MAX;

                        int islandreq = stepperestimate.dxEstimateMemoryRequirements(
                                body, bodystart, bcount, joint, jointstart, jcount);
                        maxreq = (maxreq > islandreq) ? maxreq : islandreq;

                        bodystart = bodycurr;
                        jointstart = jointcurr;
                    } else {
                        bb.tag = -1; // Not used so far (assigned to retain consistency with joints)
                    }
                }
            }
        } //END_STATE_SAVE(memarena, stackstate);
        memarena.END_STATE_SAVE(stackstate);

        if (!Common.dNODEBUG) { //# ifndef dNODEBUG
            // if debugging, check that all objects (except for disabled bodies,
            // unconnected joints, and joints that are connected to disabled bodies)
            // were tagged.
            {
                for (DxBody b=world.firstbody.get(); b!=null; b=(DxBody)b.getNext()) {
                    if (!b.isEnabled()) {
                        if (b.tag > 0) dDebug (0,"disabled body tagged");
                    }
                    else {
                        if (b.tag <= 0) dDebug (0,"enabled body not tagged");
                    }
                }
                for (DxJoint j=world.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
                    if ( (( j.node[0].body!=null && j.node[0].body.isEnabled() ) ||
                            (j.node[1].body!=null && j.node[1].body.isEnabled() ))
                            && 
                            j.isEnabled() ) {
                        if (j.tag <= 0) dDebug (0,"attached enabled joint not tagged");
                    }
                    else {
                        if (j.tag > 0) dDebug (0,"unattached or disabled joint tagged");
                    }
                }
            }
        }//# endif

        //int islandcount = ((size_t)(sizescurr - islandsizes) / sizeelements);
        int islandcount = sizescurrP / dxISE__MAX;
        islandsinfo.AssignInfo(islandcount, islandsizes, body, joint);

        return maxreq;
    }

    public DxWorldProcessIslandsInfo() {}
}
