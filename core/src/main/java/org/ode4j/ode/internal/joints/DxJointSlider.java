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

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.dUASSERT;
import static org.ode4j.ode.internal.Rotation.dQMultiply1;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;


/**
 * **************************************************************************
 * slider
 * slider. if body2 is 0 then qrel is the absolute rotation of body1 and
 * offset is the position of body1 center along axis1.
 */
public class DxJointSlider extends DxJoint implements DSliderJoint {
    DVector3 axis1;     // axis w.r.t first body
    public DQuaternion qrel;   // initial relative rotation body1 -> body2
    DVector3 offset;    // point relative to body2 that should be
    // aligned with body1 center along axis1
    DxJointLimitMotor limot; // limit and motor information

    DxJointSlider(DxWorld w)
    //dxJoint ( w )
    {
        super(w);
        axis1 = new DVector3(1, 0, 0);
        qrel = new DQuaternion();
        offset = new DVector3();
        limot = new DxJointLimitMotor();
        limot.init(world);
    }


    //double dJointGetSliderPosition ( dJoint j )
    public double dJointGetSliderPosition() {
        // get axis1 in global coordinates
        DVector3 ax1 = new DVector3(), q = new DVector3();
        dMultiply0_331(ax1, node[0].body.posr().R(), axis1);

        if (node[1].body != null) {
            // get body2 + offset point in global coordinates
            dMultiply0_331(q, node[1].body.posr().R(), offset);
            //			for ( int i = 0; i < 3; i++ )
            //				q.v[i] = node[0].body._posr.pos.v[i]
            //				         - q.v[i] - node[1].body._posr.pos.v[i];
            q.eqDiff(node[0].body.posr().pos(), q);
            q.sub(node[1].body.posr().pos());

        } else {
            //			q.v[0] = node[0].body._posr.pos.v[0] - offset.v[0];
            //			q.v[1] = node[0].body._posr.pos.v[1] - offset.v[1];
            //			q.v[2] = node[0].body._posr.pos.v[2] - offset.v[2];
            q.eqDiff(node[0].body.posr().pos(), offset);

            if (isFlagsReverse()) {   // N.B. it could have been simplier to only inverse the sign of
                //      the dCalcVectorDot3 result but this case is exceptional and doing
                //      the check for all case can decrease the performance.
                //				ax1.v[0] = -ax1.v[0];
                //				ax1.v[1] = -ax1.v[1];
                //				ax1.v[2] = -ax1.v[2];
                ax1.scale(-1);
            }
        }

        return dCalcVectorDot3(ax1, q);
    }


    //double dJointGetSliderPositionRate ( dJoint j )
    public double dJointGetSliderPositionRate() {
        // get axis1 in global coordinates
        DVector3 ax1 = new DVector3();
        dMultiply0_331(ax1, node[0].body.posr().R(), axis1);

        if (node[1].body != null) {
            return dCalcVectorDot3(ax1, node[0].body.lvel) - dCalcVectorDot3(ax1, node[1].body.lvel);
        } else {
            double rate = dCalcVectorDot3(ax1, node[0].body.lvel);
            if (isFlagsReverse()) rate = -rate;
            return rate;
        }
    }


    @Override
    void getSureMaxInfo(SureMaxInfo info) {
        info.max_m = 6;
    }


    @Override
    public void getInfo1(DxJoint.Info1 info) {
        info.setNub(5);

        // see if joint is powered
        if (limot.fmax > 0) info.setM(6); // powered slider needs an extra constraint row
        else info.setM(5);

        // see if we're at a joint limit.
        limot.limit = 0;
        if ((limot.lostop > -dInfinity || limot.histop < dInfinity) && limot.lostop <= limot.histop) {
            // measure joint position
            double pos = dJointGetSliderPosition();
            if (pos <= limot.lostop) {
                limot.limit = 1;
                limot.limit_err = pos - limot.lostop;
                info.setM(6);
            } else if (pos >= limot.histop) {
                limot.limit = 2;
                limot.limit_err = pos - limot.histop;
                info.setM(6);
            }
        }
    }


    /**
     * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
     */
    @Override
    public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
                         int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
                         int pairLoHiOfs, int[] findexA, int findexOfs) {
        // 3 rows to make body rotations equal
        setFixedOrientation(this, worldFPS, worldERP, rowskip, J1A, J1Ofs, J2A, J2Ofs, pairskip, pairRhsCfmA,
                pairRhsCfmOfs, qrel);

        // pull out pos and R for both bodies. also get the `connection'
        // vector pos2-pos1.

        //double *pos1, *pos2, *R1, *R2;
        DVector3 c = new DVector3();
        DVector3C pos2 = null;
        DMatrix3C R2 = null;

        DVector3C pos1 = node[0].body.posr().pos();
        DMatrix3C R1 = node[0].body.posr().R();

        DVector3 ax1 = new DVector3(); // joint axis in global coordinates (unit length)
        DVector3 p = new DVector3(), q = new DVector3(); // plane space of ax1
        dMultiply0_331 ( ax1, R1, axis1 );
        dPlaneSpace ( ax1, p, q );

        DxBody body1 = node[1].body;

        if ( body1 != null )
        {
            R2 = body1.posr().R();
            pos2 = body1.posr().pos();
            dSubtractVectors3( c, pos2, pos1 );
        }

        // remaining two rows. we want: vel2 = vel1 + w1 x c ... but this would
        // result in three equations, so we project along the planespace vectors
        // so that sliding along the slider axis is disregarded. for symmetry we
        // also substitute (w1+w2)/2 for w1, as w1 is supposed to equal w2.

        int currRowSkip = 3 * rowskip, currPairSkip = 3 * pairskip;
        {
            dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, p);

            if (body1 != null) {
                DVector3 tmp = new DVector3();

                dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, p);

                dCalcVectorCross3(tmp, c, p);
                dCopyScaledVector3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, tmp, 0.5);
                dCopyVector3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, J1A, J1Ofs + currRowSkip + GI2__JA_MIN);
            }
        }

        currRowSkip += rowskip;
        {
            dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, q);

            if (body1 != null) {
                DVector3 tmp = new DVector3();

                dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, q);

                dCalcVectorCross3(tmp, c, q);
                dCopyScaledVector3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, tmp, 0.5);
                dCopyVector3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, J1A, J1Ofs + currRowSkip + GI2__JA_MIN);
            }
        }

        // compute last two elements of right hand side. we want to align the offset
        // point (in body 2's frame) with the center of body 1.
        double k = worldFPS * worldERP;

        if (body1 != null) {
            DVector3 ofs = new DVector3();  // offset point in global coordinates
            dMultiply0_331(ofs, R2, offset);
            dAddVectors3(c, c, ofs);

            pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(p, c);

            currPairSkip += pairskip;
            pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(q, c);
        } else {
            DVector3 ofs = new DVector3();  // offset point in global coordinates
            dSubtractVectors3(ofs, offset, pos1);

            pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(p, ofs);

            currPairSkip += pairskip;
            pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(q, ofs);

            if ((flags & dJOINT_REVERSE) != 0) {
                dNegateVector3(ax1);
            }
        }

        // if the slider is powered, or has joint limits, add in the extra row
        currRowSkip += rowskip;
        currPairSkip += pairskip;
        limot.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
                pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, false);
    }


    //void dJointSetSliderAxis ( dJoint j, double x, double y, double z )
    public void dJointSetSliderAxis(double x, double y, double z) {
        setAxes(x, y, z, axis1, null);

        computeOffset();

        computeInitialRelativeRotation();
    }


    //	void dJointSetSliderAxisDelta ( dJoint j, double x, double y, double z,
    //	double dx, double dy, double dz )
    void dJointSetSliderAxisDelta(double x, double y, double z, double dx, double dy, double dz) {
        setAxes(x, y, z, axis1, null);

        computeOffset();

        // compute initial relative rotation body1 . body2, or env . body1
        // also compute center of body1 w.r.t body 2
        if (!(node[1].body != null)) {
            //	        offset[0] += dx;
            //	        offset[1] += dy;
            //	        offset[2] += dz;
            offset.add(dx, dy, dz);
        }

        computeInitialRelativeRotation();
    }


    //	void dJointGetSliderAxis ( dJoint j, dVector3 result )
    void dJointGetSliderAxis(DVector3 result) {
        //		dxJointSlider joint = ( dxJointSlider ) j;
        //		dUASSERT ( joint, "bad joint argument" );
        dUASSERT(result, "bad result argument");
        //		checktype ( joint, dxJointSlider.class );
        getAxis(result, axis1);
    }


    //void dJointSetSliderParam ( dJoint j, D_PARAM_NAMES parameter, double value )
    public void dJointSetSliderParam(PARAM_N parameter, double value) {
        limot.set(parameter.toSUB(), value);
    }


    double dJointGetSliderParam(PARAM_N parameter) {
        return limot.get(parameter.toSUB());
    }


    //	void dJointAddSliderForce ( dJoint j, double force )
    public void dJointAddSliderForce(double force) {
        DVector3 axis = new DVector3();

        if (isFlagsReverse()) force = -force;

        getAxis(axis, axis1);
        //		axis.v[0] *= force;
        //		axis.v[1] *= force;
        //		axis.v[2] *= force;
        axis.scale(force);

        if (node[0].body != null) node[0].body.dBodyAddForce(axis.get0(), axis.get1(), axis.get2());
        if (node[1].body != null) node[1].body.dBodyAddForce(-axis.get0(), -axis.get1(), -axis.get2());

        if (node[0].body != null && node[1].body != null) {
            // linear torque decoupling:
            // we have to compensate the torque, that this slider force may generate
            // if body centers are not aligned along the slider axis

            DVector3 ltd = new DVector3(); // Linear Torque Decoupling vector (a torque)

            DVector3 c = new DVector3();
            //			c.v[0] = 0.5 * ( joint.node[1].body._posr.pos.v[0] - joint.node[0].body._posr.pos.v[0] );
            //			c.v[1] = 0.5 * ( joint.node[1].body._posr.pos.v[1] - joint.node[0].body._posr.pos.v[1] );
            //			c.v[2] = 0.5 * ( joint.node[1].body._posr.pos.v[2] - joint.node[0].body._posr.pos.v[2] );
            c.eqDiff(node[1].body.posr().pos(), node[0].body.posr().pos()).scale(0.5);
            dCalcVectorCross3(ltd, c, axis);

            node[0].body.dBodyAddTorque(ltd);
            node[1].body.dBodyAddTorque(ltd);
        }
    }


    @Override
    void setRelativeValues() {
        computeOffset();
        computeInitialRelativeRotation();
    }


    /// Compute initial relative rotation body1 -> body2, or en.-> body1
    void computeInitialRelativeRotation() {
        if (node[0].body != null) {
            // compute initial relative rotation body1 -> body2, or env -> body1
            // also compute center of body1 w.r.t body 2
            if (node[1].body != null) {
                dQMultiply1(qrel, node[0].body._q, node[1].body._q);
            } else {
                // set qrel to the transpose of the first body's q
                //				qrel.v[0] =  node[0].body._q.v[0];
                //				qrel.v[1] = -node[0].body._q.v[1];
                //				qrel.v[2] = -node[0].body._q.v[2];
                //				qrel.v[3] = -node[0].body._q.v[3];
                qrel.set(node[0].body._q).scale(-1);
            }
        }
    }


    /// Compute center of body1 w.r.t body 2
    private void computeOffset() {
        if (node[1].body != null) {
            DVector3 c = new DVector3();
            //	        c[0] = node[0].body->posr.pos[0] - node[1].body->posr.pos[0];
            //	        c[1] = node[0].body->posr.pos[1] - node[1].body->posr.pos[1];
            //	        c[2] = node[0].body->posr.pos[2] - node[1].body->posr.pos[2];
            c.eqDiff(node[0].body.posr().pos(), node[1].body.posr().pos());

            dMultiply1_331(offset, node[1].body.posr().R(), c);
        } else if (node[0].body != null) {
            //	        offset[0] = node[0].body->posr.pos[0];
            //	        offset[1] = node[0].body->posr.pos[1];
            //	        offset[2] = node[0].body->posr.pos[2];
            offset.set(node[0].body.posr().pos());
        }
    }


    // ***********************************
    // API dSliderJoint
    // ***********************************

    @Override
    public void setAxis(double x, double y, double z) {
        dJointSetSliderAxis(x, y, z);
    }

    @Override
    public void setAxis(DVector3C a)
    //TODO use dVector3
    {
        dJointSetSliderAxis(a.get0(), a.get1(), a.get2());
    }

    @Override
    public void getAxis(DVector3 result) {
        dJointGetSliderAxis(result);
    }

    @Override
    public double getPosition() {
        return dJointGetSliderPosition();
    }

    @Override
    public double getPositionRate() {
        return dJointGetSliderPositionRate();
    }

    @Override
    public void setParam(PARAM_N parameter, double value) {
        dJointSetSliderParam(parameter, value);
    }

    @Override
    public double getParam(PARAM_N parameter) {
        return dJointGetSliderParam(parameter);
    }
    // TODO: expose params through methods

    @Override
    public void addForce(double force) {
        dJointAddSliderForce(force);
    }


    @Override
    public void setParamFMax(double d) {
        dJointSetSliderParam(PARAM_N.dParamFMax1, d);
    }


    @Override
    public void setParamHiStop(double d) {
        dJointSetSliderParam(PARAM_N.dParamHiStop1, d);
    }


    @Override
    public void setParamLoStop(double d) {
        dJointSetSliderParam(PARAM_N.dParamLoStop1, d);
    }


    @Override
    public void setParamVel(double d) {
        dJointSetSliderParam(PARAM_N.dParamVel1, d);
    }


    @Override
    public void setParamBounce(double d) {
        dJointSetSliderParam(PARAM_N.dParamBounce1, d);
    }


    @Override
    public void setAxisDelta(double x, double y, double z, double dx, double dy, double dz) {
        dJointSetSliderAxisDelta(x, y, z, dx, dy, dz);
    }
}
