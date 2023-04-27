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

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.Rotation.dQFromAxisAndAngle;
import static org.ode4j.ode.internal.Rotation.dQMultiply0;
import static org.ode4j.ode.internal.Rotation.dQMultiply1;
import static org.ode4j.ode.internal.Rotation.dQMultiply2;
import static org.ode4j.ode.internal.Rotation.dQfromR;
import static org.ode4j.ode.internal.Rotation.dRFrom2Axes;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


/**
 * ****************************************************************************
 * universal
 * <p>
 * I just realized that the universal joint is equivalent to a hinge 2 joint with
 * perfectly stiff suspension.  By comparing the hinge 2 implementation to
 * the universal implementation, you may be able to improve this
 * implementation (or, less likely, the hinge2 implementation).
 */
public class DxJointUniversal extends DxJoint implements DUniversalJoint {
    DVector3 _anchor1 = new DVector3();   // anchor w.r.t first body
    DVector3 _anchor2 = new DVector3();   // anchor w.r.t second body
    DVector3 _axis1;     // axis w.r.t first body
    DVector3 _axis2;     // axis w.r.t second body
    DQuaternion qrel1;  // initial relative rotation body1 -> virtual cross piece
    DQuaternion qrel2;  // initial relative rotation virtual cross piece -> body2
    DxJointLimitMotor limot1; // limit and motor information for axis1
    DxJointLimitMotor limot2; // limit and motor information for axis2

    DxJointUniversal(DxWorld w) {
        super(w);
        _axis1 = new DVector3(1, 0, 0);
        _axis2 = new DVector3(0, 1, 0);
        qrel1 = new DQuaternion();
        qrel2 = new DQuaternion();
        limot1 = new DxJointLimitMotor();
        limot1.init(world);
        limot2 = new DxJointLimitMotor();
        limot2.init(world);
    }


    void getAxes(DVector3 ax1, DVector3 ax2) {
        // This says "ax1 = joint->node[0].body->posr.R * joint->axis1"
        dMultiply0_331(ax1, node[0].body.posr().R(), _axis1);

        if (node[1].body != null) {
            dMultiply0_331(ax2, node[1].body.posr().R(), _axis2);
        } else {
            //			ax2[0] = axis2[0];
            //			ax2[1] = axis2[1];
            //			ax2[2] = axis2[2];
            ax2.set(_axis2);
        }
    }

    void
        //getAngles( dReal *angle1, dReal *angle2 )
    getAngles(RefDouble angle1, RefDouble angle2) {
        if (node[0].body != null) {
            // length 1 joint axis in global coordinates, from each body
            DVector3 ax1 = new DVector3(), ax2 = new DVector3();
            DMatrix3 R = new DMatrix3();
            DQuaternion qcross = new DQuaternion(), qq = new DQuaternion(), qrel = new DQuaternion();

            getAxes(ax1, ax2);

            // It should be possible to get both angles without explicitly
            // constructing the rotation matrix of the cross.  Basically,
            // orientation of the cross about axis1 comes from body 2,
            // about axis 2 comes from body 1, and the perpendicular
            // axis can come from the two bodies somehow.  (We don't really
            // want to assume it's 90 degrees, because in general the
            // constraints won't be perfectly satisfied, or even very well
            // satisfied.)
            //
            // However, we'd need a version of getHingeAngleFromRElativeQuat()
            // that CAN handle when its relative quat is rotated along a direction
            // other than the given axis.  What I have here works,
            // although it's probably much slower than need be.

            dRFrom2Axes(R, ax1, ax2);

            dQfromR(qcross, R);


            // This code is essentialy the same as getHingeAngle(), see the comments
            // there for details.

            // get qrel = relative rotation between node[0] and the cross
            dQMultiply1(qq, node[0].body._q, qcross);
            dQMultiply2(qrel, qq, qrel1);

            //*angle1 = getHingeAngleFromRelativeQuat( qrel, axis1 );
            angle1.set(getHingeAngleFromRelativeQuat(qrel, _axis1));

            // This is equivalent to
            // dRFrom2Axes(R, ax2[0], ax2[1], ax2[2], ax1[0], ax1[1], ax1[2]);
            // You see that the R is constructed from the same 2 axis as for angle1
            // but the first and second axis are swapped.
            // So we can take the first R and rapply a rotation to it.
            // The rotation is around the axis between the 2 axes (ax1 and ax2).
            // We do a rotation of 180deg.

            DQuaternion qcross2 = new DQuaternion();
            // Find the vector between ax1 and ax2 (i.e. in the middle)
            // We need to turn around this vector by 180deg

            // The 2 axes should be normalize so to find the vector between the 2.
            // Add and devide by 2 then normalize or simply normalize
            //    ax2
            //    ^
            //    |
            //    |
            ///   *------------> ax1
            //    We want the vector a 45deg
            //
            // N.B. We don't need to normalize the ax1 and ax2 since there are
            //      normalized when we set them.

            // We set the quaternion q = [cos(theta), dir*sin(theta)] = [w, x, y, Z]
            qrel.set0(0);                // equivalent to cos(Pi/2)
            qrel.set1(ax1.get0() + ax2.get0());  // equivalent to x*sin(Pi/2); since sin(Pi/2) = 1
            qrel.set2(ax1.get1() + ax2.get1());
            qrel.set3(ax1.get2() + ax2.get2());

            double l =
                    dRecip(Math.sqrt(qrel.get1() * qrel.get1() + qrel.get2() * qrel.get2() + qrel.get3() * qrel.get3()));
            qrel.scale(1, l);
            qrel.scale(2, l);
            qrel.scale(3, l);

            dQMultiply0(qcross2, qrel, qcross);

            if (node[1].body != null) {
                dQMultiply1(qq, node[1].body._q, qcross2);
                dQMultiply2(qrel, qq, qrel2);
            } else {
                // pretend joint->node[1].body->q is the identity
                dQMultiply2(qrel, qcross2, qrel2);
            }

            //*angle2 = - getHingeAngleFromRelativeQuat( qrel, axis2 );
            angle2.set(-getHingeAngleFromRelativeQuat(qrel, _axis2));
        } else {
            //			*angle1 = 0;
            //			*angle2 = 0;
            angle1.set(0);
            angle2.set(0);
        }
    }

    protected double getAngle1Internal() {
        if (node[0].body != null) {
            // length 1 joint axis in global coordinates, from each body
            DVector3 ax1 = new DVector3(), ax2 = new DVector3();
            DMatrix3 R = new DMatrix3();
            DQuaternion qcross = new DQuaternion(), qq = new DQuaternion(), qrel = new DQuaternion();

            getAxes(ax1, ax2);

            // It should be possible to get both angles without explicitly
            // constructing the rotation matrix of the cross.  Basically,
            // orientation of the cross about axis1 comes from body 2,
            // about axis 2 comes from body 1, and the perpendicular
            // axis can come from the two bodies somehow.  (We don't really
            // want to assume it's 90 degrees, because in general the
            // constraints won't be perfectly satisfied, or even very well
            // satisfied.)
            //
            // However, we'd need a version of getHingeAngleFromRElativeQuat()
            // that CAN handle when its relative quat is rotated along a direction
            // other than the given axis.  What I have here works,
            // although it's probably much slower than need be.

            dRFrom2Axes(R, ax1, ax2);
            dQfromR(qcross, R);

            // This code is essential the same as getHingeAngle(), see the comments
            // there for details.

            // get qrel = relative rotation between node[0] and the cross
            dQMultiply1(qq, node[0].body._q, qcross);
            dQMultiply2(qrel, qq, qrel1);

            return getHingeAngleFromRelativeQuat(qrel, _axis1);
        }
        return 0;
    }


    protected double getAngle2Internal() {
        if (node[0].body != null) {
            // length 1 joint axis in global coordinates, from each body
            DVector3 ax1 = new DVector3(), ax2 = new DVector3();
            DMatrix3 R = new DMatrix3();
            DQuaternion qcross = new DQuaternion(), qq = new DQuaternion(), qrel = new DQuaternion();

            getAxes(ax1, ax2);

            // It should be possible to get both angles without explicitly
            // constructing the rotation matrix of the cross.  Basically,
            // orientation of the cross about axis1 comes from body 2,
            // about axis 2 comes from body 1, and the perpendicular
            // axis can come from the two bodies somehow.  (We don't really
            // want to assume it's 90 degrees, because in general the
            // constraints won't be perfectly satisfied, or even very well
            // satisfied.)
            //
            // However, we'd need a version of getHingeAngleFromRElativeQuat()
            // that CAN handle when its relative quat is rotated along a direction
            // other than the given axis.  What I have here works,
            // although it's probably much slower than need be.

            dRFrom2Axes(R, ax2, ax1);
            dQfromR(qcross, R);

            if (node[1].body != null) {
                dQMultiply1(qq, node[1].body._q, qcross);
                dQMultiply2(qrel, qq, qrel2);
            } else {
                // pretend joint->node[1].body->q is the identity
                dQMultiply2(qrel, qcross, qrel2);
            }

            return -getHingeAngleFromRelativeQuat(qrel, _axis2);
        }
        return 0;
    }


    @Override
    void getSureMaxInfo(SureMaxInfo info) {
        info.max_m = 6;
    }


    @Override
    public void getInfo1(DxJoint.Info1 info) {
        info.setNub(4);
        info.setM(4);

        boolean limiting1 = (limot1.lostop >= -M_PI || limot1.histop <= M_PI) && limot1.lostop <= limot1.histop;
        boolean limiting2 = (limot2.lostop >= -M_PI || limot2.histop <= M_PI) && limot2.lostop <= limot2.histop;

        // We need to call testRotationLimit() even if we're motored, since it
        // records the result.
        limot1.limit = 0;
        limot2.limit = 0;

        if (limiting1 || limiting2) {
            //double angle1, angle2;
            RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
            //getAngles( &angle1, &angle2 );
            getAngles(angle1, angle2);
            if (limiting1) limot1.testRotationalLimit(angle1.get());
            if (limiting2) limot2.testRotationalLimit(angle2.get());
        }
        if (limot1.limit != 0 || limot1.fmax > 0) info.incM();
        if (limot2.limit != 0 || limot2.fmax > 0) info.incM();
    }


    /**
     * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
     */
    @Override
    public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
                         int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
                         int pairLoHiOfs, int[] findexA, int findexOfs) {
        // set the three ball-and-socket rows
        setBall(this, worldFPS, worldERP, rowskip, J1A, J1Ofs, J2A, J2Ofs, pairskip, pairRhsCfmA, pairRhsCfmOfs,
                _anchor1, _anchor2);

        // set the universal joint row. the angular velocity about an axis
        // perpendicular to both joint axes should be equal. thus the constraint
        // equation is
        //    p*w1 - p*w2 = 0
        // where p is a vector normal to both joint axes, and w1 and w2
        // are the angular velocity vectors of the two bodies.

        // length 1 joint axis in global coordinates, from each body
        DVector3 ax1 = new DVector3(), ax2 = new DVector3();
        // length 1 vector perpendicular to ax1 and ax2. Neither body can rotate
        // about this.
        DVector3 p = new DVector3();

        // Since axis1 and axis2 may not be perpendicular
        // we find a axis2_tmp which is really perpendicular to axis1
        // and in the plane of axis1 and axis2
        getAxes(ax1, ax2);

        double k = dCalcVectorDot3( ax1, ax2 );

        DVector3 ax2_temp = new DVector3();
        dAddVectorScaledVector3(ax2_temp, ax2, ax1, -k);
        dCalcVectorCross3(p, ax1, ax2_temp);
        dNormalize3(p);

        int currRowSkip = 3 * rowskip;
        {
            dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, p);

            if (node[1].body != null) {
                dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, p);
            }
        }

        // compute the right hand side of the constraint equation. set relative
        // body velocities along p to bring the axes back to perpendicular.
        // If ax1, ax2 are unit length joint axes as computed from body1 and
        // body2, we need to rotate both bodies along the axis p.  If theta
        // is the angle between ax1 and ax2, we need an angular velocity
        // along p to cover the angle erp * (theta - Pi/2) in one step:
        //
        //   |angular_velocity| = angle/time = erp*(theta - Pi/2) / stepsize
        //                      = (erp*fps) * (theta - Pi/2)
        //
        // if theta is close to Pi/2,
        // theta - Pi/2 ~= cos(theta), so
        //    |angular_velocity|  ~= (erp*fps) * (ax1 dot ax2)

        int currPairSkip = 3 * pairskip;
        {
            pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = worldFPS * worldERP * (-k);
        }

        currRowSkip += rowskip;
        currPairSkip += pairskip;

        // if the first angle is powered, or has joint limits, add in the stuff
        if (limot1.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
                pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, true)) {
            currRowSkip += rowskip;
            currPairSkip += pairskip;
        }

        // if the second angle is powered, or has joint limits, add in more stuff
        limot2.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
                pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax2, true);
    }


    void computeInitialRelativeRotations() {
        if (node[0].body != null) {
            DVector3 ax1 = new DVector3(), ax2 = new DVector3();
            DMatrix3 R = new DMatrix3();
            DQuaternion qcross = new DQuaternion();

            getAxes(ax1, ax2);

            // Axis 1.
            dRFrom2Axes(R, ax1, ax2);
            dQfromR(qcross, R);
            dQMultiply1(qrel1, node[0].body._q, qcross);

            // Axis 2.
            dRFrom2Axes(R, ax2, ax1);
            dQfromR(qcross, R);
            if (node[1].body != null) {
                dQMultiply1(qrel2, node[1].body._q, qcross);
            } else {
                // set joint->qrel to qcross
                //for ( int i = 0; i < 4; i++ ) qrel2.v[i] = qcross.v[i];
                qrel2.set(qcross);
            }
        }
    }


    //	public void dJointSetUniversalAnchor( dJoint j, double x, double y, double z )
    public void dJointSetUniversalAnchor(double x, double y, double z) {
        dJointSetUniversalAnchor(new DVector3(x, y, z));
    }

    public void dJointSetUniversalAnchor(DVector3C xyz) {
        setAnchors(xyz, _anchor1, _anchor2);
        computeInitialRelativeRotations();
    }


    //	private void dJointSetUniversalAxis1( dJoint j, double x, double y, double z )
    public void dJointSetUniversalAxis1(double x, double y, double z) {
        if (isFlagsReverse()) setAxes(x, y, z, null, _axis2);
        else setAxes(x, y, z, _axis1, null);
        computeInitialRelativeRotations();
    }


    void dJointSetUniversalAxis1Offset(double x, double y, double z, double offset1, double offset2) {
        //		dxJointUniversal* joint = ( dxJointUniversal* )j;
        //		dUASSERT( joint, "bad joint argument" );
        //		checktype( joint, Universal );
        DVector3C xyz = new DVector3(x, y, z);

        if (isFlagsReverse()) {
            setAxes(xyz, null, _axis2);
            offset1 = -offset1;
            offset2 = -offset2;
        } else setAxes(xyz, _axis1, null);

        computeInitialRelativeRotations();

        DVector3 ax2 = new DVector3();
        getAxis2(ax2, _axis2);

        {
            DVector3 ax1 = new DVector3();
            getAxes(ax1, ax2);
        }


        DQuaternion qAngle = new DQuaternion();
        dQFromAxisAndAngle(qAngle, xyz, offset1);

        DMatrix3 R = new DMatrix3();
        dRFrom2Axes(R, xyz, ax2);

        DQuaternion qcross = new DQuaternion();
        dQfromR(qcross, R);

        DQuaternion qOffset = new DQuaternion();
        dQMultiply0(qOffset, qAngle, qcross);

        dQMultiply1(qrel1, node[0].body._q, qOffset);

        // Calculating the second offset
        dQFromAxisAndAngle(qAngle, ax2, offset2);

        dRFrom2Axes(R, ax2, xyz);
        dQfromR(qcross, R);

        dQMultiply1(qOffset, qAngle, qcross);
        if (node[1].body != null) {
            dQMultiply1(qrel2, node[1].body._q, qOffset);
        } else {
            //			joint->qrel2[0] = qcross[0];
            //			joint->qrel2[1] = qcross[1];
            //			joint->qrel2[2] = qcross[2];
            //			joint->qrel2[3] = qcross[3];
            qrel2.set(qcross);
        }
    }


    //	private void dJointSetUniversalAxis2( dJoint j, double x, double y, double z )
    public void dJointSetUniversalAxis2(double x, double y, double z) {
        if (isFlagsReverse()) setAxes(x, y, z, _axis1, null);
        else setAxes(x, y, z, null, _axis2);
        computeInitialRelativeRotations();
    }


    void dJointSetUniversalAxis2Offset(double x, double y, double z, double offset1, double offset2) {
        DVector3C xyz = new DVector3(x, y, z);

        if (isFlagsReverse()) {
            setAxes(xyz, _axis1, null);
            offset1 = -offset2;
            offset2 = -offset1;
        } else setAxes(xyz, null, _axis2);


        computeInitialRelativeRotations();

        // It is easier to retreive the 2 axes here since
        // when there is only one body B2 (the axes switch position)
        // Doing this way eliminate the need to write the code differently
        // for both case.
        DVector3 ax1 = new DVector3(), ax2 = new DVector3();
        getAxes(ax1, ax2);


        DQuaternion qAngle = new DQuaternion();
        dQFromAxisAndAngle(qAngle, ax1, offset1);

        DMatrix3 R = new DMatrix3();
        dRFrom2Axes(R, ax1, ax2);

        DQuaternion qcross = new DQuaternion();
        dQfromR(qcross, R);

        DQuaternion qOffset = new DQuaternion();
        dQMultiply0(qOffset, qAngle, qcross);


        dQMultiply1(qrel1, node[0].body._q, qOffset);


        // Calculating the second offset
        dQFromAxisAndAngle(qAngle, ax2, offset2);

        dRFrom2Axes(R, ax2, ax1);
        dQfromR(qcross, R);

        dQMultiply1(qOffset, qAngle, qcross);
        if (node[1].body != null) {
            dQMultiply1(qrel2, node[1].body._q, qOffset);
        } else {
            //			joint->qrel2[0] = qcross[0];
            //			joint->qrel2[1] = qcross[1];
            //			joint->qrel2[2] = qcross[2];
            //			joint->qrel2[3] = qcross[3];
            qrel2.set(qcross);
        }
    }


    //	public void dJointGetUniversalAnchor( dJoint j, dVector3 result )
    public void dJointGetUniversalAnchor(DVector3 result) {
        if (isFlagsReverse()) getAnchor2(result, _anchor2);
        else getAnchor(result, _anchor1);
    }


    //	private void dJointGetUniversalAnchor2( dJoint j, dVector3 result )
    private void dJointGetUniversalAnchor2(DVector3 result) {
        if (isFlagsReverse()) getAnchor(result, _anchor1);
        else getAnchor2(result, _anchor2);
    }


    public void dJointGetUniversalAxis1(DVector3 result) {
        if (isFlagsReverse()) getAxis2(result, _axis2);
        else getAxis(result, _axis1);
    }


    public void dJointGetUniversalAxis2(DVector3 result) {
        if (isFlagsReverse()) getAxis(result, _axis1);
        else getAxis2(result, _axis2);
    }


    //	private void dJointSetUniversalParam( dJoint j, int parameter, double value )
    public void dJointSetUniversalParam(PARAM_N parameter, double value) {
        if (parameter.isGroup2()) //and( 0xff00 ).eq( 0x100 ))
        {
            limot2.set(parameter.toSUB(), value);//parameter.and( 0xff ), value );
        } else {
            limot1.set(parameter.toSUB(), value);
        }
    }


    //	private double dJointGetUniversalParam( dJoint j, D_PARAM_NAMES parameter )
    private double dJointGetUniversalParam(PARAM_N parameter) {
        if (parameter.isGroup2())//and( 0xff00 ).eq( 0x100 ))
        {
            return limot2.get(parameter.toSUB());//and( 0xff) );
        } else {
            return limot1.get(parameter.toSUB());
        }
    }

    //TZ removed to avoid RefDouble usage
    //	private void dJointGetUniversalAngles( RefDouble angle1,
    //			RefDouble angle2 )
    //	{
    ////		dxJointUniversal joint = ( dxJointUniversal )j;
    ////		dUASSERT( joint, "bad joint argument" );
    ////		checktype( joint, dxJointUniversal.class );
    //		if ( (flags & dJOINT_REVERSE) != 0 )
    //			getAngles( angle2, angle1 );
    //			angle2.d = -angle2.d;
    //		else
    //			getAngles( angle1, angle2 );
    //	}


    public double dJointGetUniversalAngle1() {
        if (isFlagsReverse()) return getAngle2Internal();
        else return getAngle1Internal();
    }


    public double dJointGetUniversalAngle2() {
        if (isFlagsReverse()) return -getAngle1Internal();
        else return getAngle2Internal();
    }


    public double dJointGetUniversalAngle1Rate() {
        if (node[0].body != null) {
            DVector3 axis = new DVector3();

            if (isFlagsReverse()) getAxis2(axis, _axis2);
            else getAxis(axis, _axis1);

            double rate = dCalcVectorDot3(axis, node[0].body.avel);
            if (node[1].body != null) rate -= dCalcVectorDot3(axis, node[1].body.avel);
            return rate;
        }
        return 0;
    }


    public double dJointGetUniversalAngle2Rate() {
        if (node[0].body != null) {
            DVector3 axis = new DVector3();

            if (isFlagsReverse()) getAxis(axis, _axis1);
            else getAxis2(axis, _axis2);

            double rate = dCalcVectorDot3(axis, node[0].body.avel);
            if (node[1].body != null) rate -= dCalcVectorDot3(axis, node[1].body.avel);
            return rate;
        }
        return 0;
    }


    private void dJointAddUniversalTorques(double torque1, double torque2) {
        DVector3 axis1 = new DVector3(), axis2 = new DVector3();

        if (isFlagsReverse()) {
            double temp = torque1;
            torque1 = -torque2;
            torque2 = -temp;
        }

        getAxis(axis1, _axis1);
        getAxis2(axis2, _axis2);
        //		axis1.v[0] = axis1.v[0] * torque1 + axis2.v[0] * torque2;
        //		axis1.v[1] = axis1.v[1] * torque1 + axis2.v[1] * torque2;
        //		axis1.v[2] = axis1.v[2] * torque1 + axis2.v[2] * torque2;
        axis1.eqSum(axis1, torque1, axis2, torque2);

        if (node[0].body != null) node[0].body.dBodyAddTorque(axis1);
        if (node[1].body != null) node[1].body.dBodyAddTorque(axis1.scale(-1));
    }


    @Override
    void setRelativeValues() {
        DVector3 anchor = new DVector3();
        dJointGetUniversalAnchor(anchor);
        setAnchors(anchor, _anchor1, _anchor2);

        DVector3 ax1 = new DVector3(), ax2 = new DVector3();
        dJointGetUniversalAxis1(ax1);
        dJointGetUniversalAxis2(ax2);

        if (isFlagsReverse()) {
            setAxes(ax1, null, _axis2);
            setAxes(ax2, _axis1, null);
        } else {
            setAxes(ax1, _axis1, null);
            setAxes(ax2, null, _axis2);
        }

        computeInitialRelativeRotations();
    }


    public DxJointLimitMotor getLimot1() {
        return limot1;
    }

    public DxJointLimitMotor getLimot2() {
        return limot2;
    }


    // ***********************************
    // API dUniversalJoint
    // ***********************************

    @Override
    public void setAnchor(double x, double y, double z) {
        dJointSetUniversalAnchor(x, y, z);
    }

    @Override
    public void setAnchor(DVector3C a) {
        dJointSetUniversalAnchor(a);
    }

    @Override
    public void setAxis1(double x, double y, double z) {
        dJointSetUniversalAxis1(x, y, z);
    }

    @Override
    public void setAxis1(DVector3C a)
    //TODO use dVector3
    {
        setAxis1(a.get0(), a.get1(), a.get2());
    }

    @Override
    public void setAxis2(double x, double y, double z) {
        dJointSetUniversalAxis2(x, y, z);
    }

    @Override
    public void setAxis2(DVector3C a)
    //TODO use dVector3
    {
        setAxis2(a.get0(), a.get1(), a.get2());
    }

    @Override
    public void getAnchor(DVector3 result) {
        dJointGetUniversalAnchor(result);
    }

    @Override
    public void getAnchor2(DVector3 result) {
        dJointGetUniversalAnchor2(result);
    }

    @Override
    public void getAxis1(DVector3 result) {
        dJointGetUniversalAxis1(result);
    }

    @Override
    public void getAxis2(DVector3 result) {
        dJointGetUniversalAxis2(result);
    }

    @Override
    public void setParam(PARAM_N parameter, double value) {
        dJointSetUniversalParam(parameter, value);
    }

    @Override
    public double getParam(PARAM_N parameter) {
        return dJointGetUniversalParam(parameter);
    }

    //	  public void getAngles(double *angle1, double *angle2)
    //	    { dJointGetUniversalAngles (angle1, angle2); }

    /**
     * TZ Take care to call getAngle1Internal() from dx-classes.
     */
    @Override
    public double getAngle1() {
        return dJointGetUniversalAngle1();
    }

    @Override
    public double getAngle1Rate() {
        return dJointGetUniversalAngle1Rate();
    }

    /**
     * TZ Take care to call getAngle2Internal() from dx-classes.
     */
    @Override
    public double getAngle2() {
        return dJointGetUniversalAngle2();
    }

    @Override
    public double getAngle2Rate() {
        return dJointGetUniversalAngle2Rate();
    }

    @Override
    public void addTorques(double torque1, double torque2) {
        dJointAddUniversalTorques(torque1, torque2);
    }


    @Override
    public void setAxis1Offset(double x, double y, double z, double offset1, double offset2) {
        dJointSetUniversalAxis1Offset(x, y, z, offset1, offset2);
    }


    @Override
    public void setAxis2Offset(double x, double y, double z, double offset1, double offset2) {
        dJointSetUniversalAxis2Offset(x, y, z, offset1, offset2);
    }
}

