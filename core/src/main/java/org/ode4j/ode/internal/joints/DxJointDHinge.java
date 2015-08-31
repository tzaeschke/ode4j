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

import static org.ode4j.ode.OdeMath.dCalcVectorCross3;
import static org.ode4j.ode.OdeMath.dPlaneSpace;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DDoubleHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.DxWorld;

/**
 * Double Hinge joint.
 */
public class DxJointDHinge extends DxJointDBall implements DDoubleHingeJoint {
	
    private final DVector3 axis1 = new DVector3();
    private final DVector3 axis2 = new DVector3();
    
//    dxJointDHinge(dxWorld *w);
//
//    virtual void getSureMaxInfo( SureMaxInfo* info );
//    virtual void getInfo1( Info1* info );
//    virtual void getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr* info );
//    virtual dJointType type() const;
//    virtual size_t size() const;

    
    DxJointDHinge(DxWorld w) {
    	super(w);
    }


	@Override
    void
    getSureMaxInfo( SureMaxInfo info )
    {
        info.max_m = 4;
    }


	@Override
    public void
    getInfo1( DxJoint.Info1 info )
    {
        info.m = 4;
        info.nub = 4;
    }


	@Override
    public void
    getInfo2( double worldFPS, double worldERP, Info2Descr info )
    {
        super.getInfo2( worldFPS, worldERP, info ); // sets row0
        
        final int skip = info.rowskip();
        final int row1 = skip;
        final int row2 = 2*skip;
        final int row3 = 3*skip;
        
        DVector3 globalAxis1 = new DVector3();
        //dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], globalAxis1);
        node[0].body.vectorToWorld(axis1, globalAxis1);

        // angular constraints, perpendicular to axis
        DVector3 p = new DVector3(), q = new DVector3();
        dPlaneSpace(globalAxis1, p, q);
        info._J[info.J1ap+row1+0] = p.get0();
        info._J[info.J1ap+row1+1] = p.get1();
        info._J[info.J1ap+row1+2] = p.get2();
        info._J[info.J1ap+row2+0] = q.get0();
        info._J[info.J1ap+row2+1] = q.get1();
        info._J[info.J1ap+row2+2] = q.get2();

        if ( node[1].body != null ) {
            info._J[info.J2ap+row1+0] = -p.get0();
            info._J[info.J2ap+row1+1] = -p.get1();
            info._J[info.J2ap+row1+2] = -p.get2();
            info._J[info.J2ap+row2+0] = -q.get0();
            info._J[info.J2ap+row2+1] = -q.get1();
            info._J[info.J2ap+row2+2] = -q.get2();
        }

        DVector3 globalAxis2 = new DVector3();
        if ( node[1].body != null ) {
            //dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], globalAxis2);
        	node[1].body.vectorToWorld(axis2, globalAxis2);
        } else {
        	//dCopyVector3(globalAxis2, axis2);
        	globalAxis2.set(axis2);
        }
        
        // similar to the hinge joint
        DVector3 u = new DVector3();
        dCalcVectorCross3(u, globalAxis1, globalAxis2);
        final double k = worldFPS * super.erp();

        info.setC(1, k * u.dot(p) );
        info.setC(2, k * u.dot(q) );




        /*
         * Constraint along the axis: translation along it should couple angular movement.
         * This is just the ball-and-socket derivation, projected onto the hinge axis,
         * producing a single constraint at the end.
         *
         * The choice of "ball" position can be arbitrary; we could place it at the center
         * of one of the bodies, canceling out its rotational jacobian; or we could make
         * everything symmetrical by just placing at the midpoint between the centers.
         *
         * I like symmetry, so I'll use the second approach here. I'll call the midpoint h.
         *
         * Of course, if the second body is NULL, the first body is pretty much locked
         * along this axis, and the linear constraint is enough.
         */

        //info.J1l[row3+0] = globalAxis1[0];
        //info.J1l[row3+1] = globalAxis1[1];
        //info.J1l[row3+2] = globalAxis1[2];
        globalAxis1.wrapSet(info._J, info.J1lp+row3);

        if ( node[1].body != null ) {

            DVector3 h = new DVector3();
            //dAddScaledVectors3(h, node[0].body.posr().pos(), node[1].body.posr().pos, -0.5, 0.5);
            h.eqSum(node[0].body.posr().pos(), -0.5, node[1].body.posr().pos(), 0.5);
            
            DVector3 omega = new DVector3();
            dCalcVectorCross3(omega, h, globalAxis1);
            //info.J1a[row3+0] = omega[0];
            //info.J1a[row3+1] = omega[1];
            //info.J1a[row3+2] = omega[2];
            omega.wrapSet(info._J, info.J1ap+row3);

            //info.J2l[row3+0] = -globalAxis1[0];
            //info.J2l[row3+1] = -globalAxis1[1];
            //info.J2l[row3+2] = -globalAxis1[2];
            globalAxis1.wrapSet(info._J,  info.J2lp+row3);

            //info.J2a[row3+0] = omega[0];
            //info.J2a[row3+1] = omega[1];
            //info.J2a[row3+2] = omega[2];
            omega.wrapSet(info._J, info.J2ap+row3);
        }

        // error correction: both anchors should lie on the same plane perpendicular to the axis
        DVector3 globalA1 = new DVector3(), globalA2 = new DVector3();
        //dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], globalA1);
        node[0].body.getRelPointPos(anchor1(), globalA1);
        if ( node[1].body != null ) {
            //dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], globalA2);
        	node[1].body.getRelPointPos(anchor2(), globalA2);
        } else {
        	//dCopyVector3(globalA2, anchor2);
        	globalA2.set(anchor2());
        }

        DVector3 d = new DVector3();
        //dSubtractVectors3(d, globalA1, globalA2); // displacement error
        d.eqDiff(globalA1, globalA2); // displacement error
        info.setC(3, -k * globalAxis1.dot(d) );
    }

    void dJointSetDHingeAxis( DVector3C xyz )
    {
        //dBodyVectorFromWorld(node[0].body, x, y, z, axis1);
    	node[0].body.vectorFromWorld(xyz, axis1);
        if (node[1].body != null) {
            //dBodyVectorFromWorld(node[1].body, x, y, z, axis2);
        	node[1].body.vectorFromWorld(xyz, axis2);
        } else {
//            joint->axis2[0] = x;
//            joint->axis2[1] = y;
//            joint->axis2[2] = z;
        	axis2.set(xyz);
        }
        axis1.normalize();
        axis1.normalize();
    }

    void dJointGetDHingeAxis( DVector3 result )
    {
        //dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], result);
    	node[0].body.vectorToWorld(axis1, result);
    }


    void dJointSetDHingeAnchor1( DVector3C xyz )
    {
        dJointSetDBallAnchor1(xyz);
    }


    void dJointSetDHingeAnchor2( DVector3C xyz )
    {
        dJointSetDBallAnchor2(xyz);
    }

    double dJointGetDHingeDistance()
    {
        return dJointGetDBallDistance();
    }


    void dJointGetDHingeAnchor1( DVector3 result )
    {
        dJointGetDBallAnchor1(result);
    }


    void dJointGetDHingeAnchor2( DVector3 result )
    {
        dJointGetDBallAnchor2(result);
    }


    void dJointSetDHingeParam( DJoint.PARAM parameter, double value )
    {
        dJointSetDBallParam(parameter, value);
    }


    double dJointGetDHingeParam( DJoint.PARAM parameter )
    {
        return dJointGetDBallParam(parameter);
    }


	// *******************************
	// API dDHingeJoint
	// *******************************

	@Override
	public void setAxis(double x, double y, double z) {
		dJointSetDHingeAxis(new DVector3(x, y, z));
	}

	@Override
	public void getAxis(DVector3 result) {
		dJointGetDHingeAxis(result);
	}

	@Override
	public void setAxis(DVector3C xyz) {
		dJointSetDHingeAxis(xyz);
	}

}
