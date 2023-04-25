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
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.M_PI_2;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dSqrt;
import static org.ode4j.ode.internal.Common.dUASSERT;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DTransmissionJoint;
import org.ode4j.ode.internal.DxWorld;

/**
 * Double Hinge joint.
 */
public class DxJointTransmission extends DxJoint implements DTransmissionJoint {
	
    private TRANSMISSION mode; 
    private boolean update;
    private final DVector3[] contacts = new DVector3[]{new DVector3(), new DVector3()};
    private final DVector3[] axes = new DVector3[]{new DVector3(), new DVector3()};
    private final DVector3[] anchors = new DVector3[]{new DVector3(), new DVector3()};
    private final DMatrix3[] reference = new DMatrix3[]{new DMatrix3(), new DMatrix3()};
    private final double[] phase = new double[2];
    private final double[] radii = new double[2];
    private double backlash;
    double ratio;        // transmission ratio
    double erp;          // error reduction
    double cfm;          // constraint force mix in
    
//    dxJointTransmission(dxWorld *w);
//
//    virtual void getSureMaxInfo( SureMaxInfo* info );
//    virtual void getInfo1( Info1* info );
//    virtual void getInfo2( dReal worldFPS, dReal worldERP,
//                           int rowskip, dReal *J1, dReal *J2,
//                           int pairskip, dReal *pairRhsCfm, dReal *pairLoHi,
//                           int *findex );
//    virtual dJointType type() const;
//    virtual size_t size() const;

    
//    namespace {
//        inline dReal clamp(dReal x, dReal minX, dReal maxX)
//        {
//            return x < minX ? minX : (x > maxX ? maxX : x);
//        }
//    }
    private static double clamp(double x, double minX, double maxX) {
    	return x < minX ? minX : (Math.min(x, maxX));
    }

    /*
     * Transmission joint
     */

    DxJointTransmission(DxWorld w) {
    	super(w);
        int i;
        
        flags |= dJOINT_TWOBODIES;
        mode = TRANSMISSION.dTransmissionParallelAxes;

        cfm = world.getCFM();
        erp = world.getERP();
        
        for (i = 0 ; i < 2 ; i += 1) {
            //dSetZero( anchors[i], 4 );
            //dSetZero( axes[i], 4 );
            axes[i].set0(1);

            radii[i] = 0;
        }
        
        backlash = 0;
        ratio = 1;
        update = true;
    }

	@Override
    void
    getSureMaxInfo( SureMaxInfo info )
    {
        info.max_m = 1;
    }

	@Override
    public void
    getInfo1( DxJoint.Info1 info )
    {
        // If there's backlash in the gears then constraint must be
        // unilateral, that is the driving gear can only push the driven
        // gear in one direction.  In order to push it in the other it
        // first needs to traverse the backlash gap.

        info.m = 1;
        info.nub = backlash > 0 ? 0 : 1;
    }

    /**
     * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
     */
    @Override
    public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
                         int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
                         int pairLoHiOfs, int[] findexA, int findexOfs) {
        DVector3[] a = new DVector3[]{new DVector3(), new DVector3()}; 
        DVector3[] n = new DVector3[]{new DVector3(), new DVector3()}; 
        DVector3[] l = new DVector3[]{new DVector3(), new DVector3()}; 
        DVector3[] r = new DVector3[]{new DVector3(), new DVector3()}; 
        DVector3[] c = new DVector3[]{new DVector3(), new DVector3()}; 
        DVector3 s = new DVector3(), t = new DVector3(), O = new DVector3();
        DVector3 d = new DVector3(), z = new DVector3(), u = new DVector3(), v = new DVector3();
        double theta, delta, nn, na_0, na_1, cosphi, sinphi, m;
        //final double *p[2], *omega[2];
        DVector3C[] p = new DVector3C[2];
        DVector3C[] omega = new DVector3C[2];
        int i;

        // Transform all needed quantities to the global frame.

        for (i = 0 ; i < 2 ; i += 1) {
//            dBodyGetRelPointPos(node[i].body,
//                                anchors[i][0], anchors[i][1], anchors[i][2],
//                                a[i]);
        	node[i].body.getRelPointPos(anchors[i], a[i]);

//            dBodyVectorToWorld(node[i].body, axes[i][0], axes[i][1], axes[i][2],
//                               n[i]);
        	node[i].body.vectorToWorld(axes[i],  n[i]);

            p[i] = node[i].body.getPosition();//dBodyGetPosition(node[i].body);
            omega[i] =  node[i].body.getAngularVel();//dBodyGetAngularVel(node[i].body);
        }

        if (update) {
            // Make sure both gear reference frames end up with the same
            // handedness.
        
            if (dCalcVectorDot3(n[0], n[1]) < 0) {
                axes[0].scale(-1);//dNegateVector3(axes[0]);
                n[0].scale(-1);//dNegateVector3(n[0]);
            }
        }

        // Calculate the mesh geometry based on the current mode.
        
        switch (mode) {
        case dTransmissionParallelAxes:
            // Simply calculate the contact point as the point on the
            // baseline that will yield the correct ratio.

            dIASSERT (ratio > 0);

            d.eqDiff(a[1],  a[0]);//dSubtractVectors3(d, a[1], a[0]);
            dAddVectorScaledVector3(c[0], a[0], d, ratio / (1 + ratio));
            c[1].set(c[0]);//dCopyVector3(c[1], c[0]);
            
            dNormalize3(d);
            
            for (i = 0 ; i < 2 ; i += 1) {
                dCalcVectorCross3(l[i], d, n[i]);
            }

            break;
        case dTransmissionIntersectingAxes:
            // Calculate the line of intersection between the planes of the
            // gears.

            dCalcVectorCross3(l[0], n[0], n[1]);
            l[1].set(l[0]);//dCopyVector3(l[1], l[0]);

            nn = dCalcVectorDot3(n[0], n[1]);
            dIASSERT(Math.abs(nn) != 1);
            
            na_0 = dCalcVectorDot3(n[0], a[0]);
            na_1 = dCalcVectorDot3(n[1], a[1]);

            dAddScaledVectors3(O, n[0], n[1],
                               (na_0 - na_1 * nn) / (1 - nn * nn),
                               (na_1 - na_0 * nn) / (1 - nn * nn));

            // Find the contact point as:
            //
            // c = ((r_a - O) . l) l + O
            //
            // where r_a the anchor point of either gear and l, O the tangent
            // line direction and origin.

            for (i = 0 ; i < 2 ; i += 1) {
                d.eqDiff(a[i], O);//dSubtractVectors3(d, a[i], O);
                m = dCalcVectorDot3(d, l[i]);
                dAddVectorScaledVector3(c[i], O, l[i], m);
            }

            break;
        case dTransmissionChainDrive:
            d.eqDiff(a[0],  a[1]);//dSubtractVectors3(d, a[0], a[1]);
            m = d.length();//dCalcVectorLength3(d);

            dIASSERT(m > 0);
            
            // Caclulate the angle of the contact point relative to the
            // baseline.

            cosphi = clamp((radii[1] - radii[0]) / m, -1.0, 1.0); // Force into range to fix possible computation errors
            sinphi = dSqrt (1.0 - cosphi * cosphi);

            d.normalize();

            for (i = 0 ; i < 2 ; i += 1) {
                // Calculate the contact radius in the local reference
                // frame of the chain.  This has axis x pointing along the
                // baseline, axis y pointing along the sprocket axis and
                // the remaining axis normal to both.

//                u[0] = radii[i] * cosphi;
//                u[1] = 0;
//                u[2] = radii[i] * sinphi;
            	u.set( radii[i] * cosphi, 0, radii[i] * sinphi );

                // Transform the contact radius into the global frame.

                dCalcVectorCross3(z, d, n[i]);
                
                v.set0( dCalcVectorDot3(d, u) );
                v.set1( dCalcVectorDot3(n[i], u) );
                v.set2( dCalcVectorDot3(z, u) );

                // Finally calculate contact points and l.
                
                c[i].eqSum(a[i], v);//dAddVectors3(c[i], a[i], v);
                dCalcVectorCross3(l[i], v, n[i]);
                dNormalize3(l[i]);

                // printf ("%d: %f, %f, %f\n",
                //      i, l[i][0], l[i][1], l[i][2]);
            }

            break;
        }

        if (update) {
            // We need to calculate an initial reference frame for each
            // wheel which we can measure the current phase against.  This
            // frame will have the initial contact radius as the x axis,
            // the wheel axis as the z axis and their cross product as the
            // y axis.

            for (i = 0 ; i < 2 ; i += 1) {
                r[i].eqDiff(c[i], a[i]); //dSubtractVectors3 (r[i], c[i], a[i]);
                radii[i] = r[i].length();//dCalcVectorLength3(r[i]);
                dIASSERT(radii[i] > 0);
                
                //dBodyVectorFromWorld(node[i].body, r[i][0], r[i][1], r[i][2],
                //                     reference[i]);
                DVector3 dummyRef0 = new DVector3();
                DVector3 dummyRef4 = new DVector3();
                //node[i].body.vectorFromWorld(r[i], reference[i]);
                node[i].body.vectorFromWorld(r[i], dummyRef0);
                dummyRef0.normalize();//dNormalize3(reference[i]);
                reference[i].setOfs(0, dummyRef0);
                reference[i].setOfs(8, axes[i]);//dCopyVector3(reference[i] + 8, axes[i]);
                //dCalcVectorCross3(reference[i] + 4, reference[i] + 8, reference[i]);
                dCalcVectorCross3(dummyRef4, axes[i], dummyRef0);
                reference[i].setOfs(4, dummyRef4);

                // printf ("%f\n", dDOT(r[i], n[i]));
                // printf ("(%f, %f, %f,\n %f, %f, %f,\n %f, %f, %f)\n",
                //      reference[i][0],reference[i][1],reference[i][2],
                //      reference[i][4],reference[i][5],reference[i][6],
                //      reference[i][8],reference[i][9],reference[i][10]);

                phase[i] = 0;
            }

            ratio = radii[0] / radii[1];
            update = false;
        }
        
        for (i = 0 ; i < 2 ; i += 1) {
        	double phase_hat;

            r[i].eqDiff(c[i], a[i]);//dSubtractVectors3 (r[i], c[i], a[i]);
            
            // Transform the (global) contact radius into the gear's
            // reference frame.

            //dBodyVectorFromWorld (node[i].body, r[i][0], r[i][1], r[i][2], s);
            node[i].body.vectorFromWorld(r[i], s);
            dMultiply0_331(t, reference[i], s);

            // Now simply calculate its angle on the plane relative to the
            // x-axis which is the initial contact radius.  This will be
            // an angle between -pi and pi that is coterminal with the
            // actual phase of the wheel.  To find the real phase we
            // estimate it by adding omega * dt to the old phase and then
            // find the closest angle to that, that is coterminal to
            // theta.

            theta = Math.atan2(t.get1(), t.get0());
            phase_hat = phase[i] + dCalcVectorDot3(omega[i], n[i]) / worldFPS;

            if (phase_hat > M_PI_2) {
                if (theta < 0) {
                    theta += 2 * M_PI;
                }

                theta += Math.floor(phase_hat / (2 * M_PI)) * (2 * M_PI);
            } else if (phase_hat < -M_PI_2) {
                if (theta > 0) {
                    theta -= 2 * M_PI;
                }

                theta += Math.ceil(phase_hat / (2 * M_PI)) * (2 * M_PI);
            }
                    
            if (phase_hat - theta > M_PI) {
                phase[i] = theta + (2 * M_PI);
            } else if (phase_hat - theta < -M_PI) {
                phase[i] = theta - (2 * M_PI);
            } else {
                phase[i] = theta;
            }

            dIASSERT(Math.abs(phase_hat - phase[i]) < M_PI);
        }

        // Calculate the phase error.  Depending on the mode the condition
        // is that the distances traveled by each contact point must be
        // either equal (chain and sprockets) or opposite (gears).

        if (mode == TRANSMISSION.dTransmissionChainDrive) {
            delta = (r[0].length() * phase[0] -
                     r[1].length() * phase[1]);
        } else {
            delta = (r[0].length() * phase[0] +
                     r[1].length() * phase[1]);
        }

        // When in chain mode a torque reversal, signified by the change
        // in sign of the wheel phase difference, has the added effect of
        // switching the active chain branch.  We must therefore reflect
        // the contact points and tangents across the baseline.
        
        if (mode == TRANSMISSION.dTransmissionChainDrive && delta < 0) {
            DVector3 d2 = new DVector3();

            d2.eqDiff(a[0], a[1]);//dSubtractVectors3(d, a[0], a[1]);
            
            for (i = 0 ; i < 2 ; i += 1) {
                DVector3 nn2 = new DVector3();
                double a2;
                
                dCalcVectorCross3(nn2, n[i], d2);
                a2 = dCalcVectorDot3(nn2, nn2);
                dIASSERT(a2 > 0);
                
                dAddScaledVectors3(c[i], c[i], nn2,
                                   1, -2 * dCalcVectorDot3(c[i], nn2) / a2);
                dAddScaledVectors3(l[i], l[i], nn2,
                                   -1, 2 * dCalcVectorDot3(l[i], nn2) / a2);
            }
        }

        // Do not add the constraint if there's backlash and we're in the
        // backlash gap.

        if (backlash == 0 || Math.abs(delta) > backlash) {
            // The constraint is satisfied if the absolute velocity of the
            // contact point projected onto the tangent of the wheels is equal
            // for both gears.  This velocity can be calculated as:
            // 
            // u = v + omega x r_c
            // 
            // The constraint therefore becomes:
            // (v_1 + omega_1 x r_c1) . l = (v_2 + omega_2 x r_c2) . l <=>
            // (v_1 . l + (r_c1 x l) . omega_1 = v_2 . l + (r_c2 x l) . omega_2

            for (i = 0 ; i < 2 ; i += 1) {
                r[i].eqDiff(c[i], p[i]);//dSubtractVectors3 (r[i], c[i], p[i]);
            }

            dCopyVector3(J1A, J1Ofs + GI2__JL_MIN, l[0]);
            dCalcVectorCross3(J1A, J1Ofs + GI2__JA_MIN, r[0], l[0]);

            dCopyNegatedVector3(J2A, J2Ofs + GI2__JL_MIN, l[1]);
            dCalcVectorCross3(J2A, J2Ofs + GI2__JA_MIN, l[1], r[1]);

            if (delta > 0) {
                if (backlash > 0) {
                    pairLoHiA[pairLoHiOfs + GI2_LO] = -dInfinity;
                    pairLoHiA[pairLoHiOfs + GI2_HI] = 0;
                }

                pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = -worldFPS * erp * (delta - backlash);
            } else {
                if (backlash > 0) {
                    pairLoHiA[pairLoHiOfs + GI2_LO] = 0;
                    pairLoHiA[pairLoHiOfs + GI2_HI] = dInfinity;
                }

                pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = -worldFPS * erp * (delta + backlash);
            }
        }

        pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = cfm;

        // printf ("%f, %f, %f, %f, %f\n", delta, phase[0], phase[1], -phase[1] / phase[0], ratio);

        // Cache the contact point (in world coordinates) to avoid
        // recalculation if requested by the user.

        contacts[0].set(c[0]);//dCopyVector3(contacts[0], c[0]);
        contacts[1].set(c[1]);//dCopyVector3(contacts[1], c[1]);
    }

    void dJointSetTransmissionAxis( DVector3C xyz )
    {
        int i;
        dUASSERT(mode == TRANSMISSION.dTransmissionParallelAxes ||
                 mode == TRANSMISSION.dTransmissionChainDrive ,
                 "axes must be set individualy in current mode" );

        for (i = 0 ; i < 2 ; i += 1) {
            if (node[i].body != null) {
                //dBodyVectorFromWorld(joint->node[i].body, x, y, z, joint->axes[i]);
            	node[i].body.vectorFromWorld(xyz, axes[i]);
                axes[i].normalize();
            }
        }

        update = true;
    }

    void dJointSetTransmissionAxis1( DVector3C xyz )
    {
        dUASSERT(mode == TRANSMISSION.dTransmissionIntersectingAxes,
                 "can't set individual axes in current mode" );

        if (node[0].body != null) {
            //dBodyVectorFromWorld(joint->node[0].body, x, y, z, joint->axes[0]);
        	node[0].body.vectorFromWorld(xyz, axes[0]);
            axes[0].normalize();
        }

        update = true;
    }

    void dJointSetTransmissionAxis2( DVector3C xyz )
    {    
        dUASSERT(mode == TRANSMISSION.dTransmissionIntersectingAxes,
                 "can't set individual axes in current mode" );

        if (node[1].body != null) {
            //dBodyVectorFromWorld(joint->node[1].body, x, y, z, joint->axes[1]);
        	node[1].body.vectorFromWorld(xyz, axes[1]);
            axes[1].normalize();
        }
        
        update = true;
    }

    void dJointSetTransmissionAnchor1( DVector3C xyz )
    {
        if (node[0].body != null) {
            //dBodyGetPosRelPoint(joint->node[0].body, x, y, z, joint->anchors[0]);
        	node[0].body.getPosRelPoint(xyz, anchors[0]);
        }
        
        update = true;
    }

    void dJointSetTransmissionAnchor2( DVector3C xyz )
    {
        if (node[1].body != null) {
            //dBodyGetPosRelPoint(joint->node[1].body, x, y, z, joint->anchors[1]);
        	node[1].body.getPosRelPoint(xyz, anchors[1]);
        }
        
        update = true;
    }

    void dJointGetTransmissionContactPoint1( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );

        //dCopyVector3(result, contacts[0]);
        result.set(contacts[0]);
    }

    void dJointGetTransmissionContactPoint2( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );

        //dCopyVector3(result, contacts[1]);
        result.set(contacts[1]);
    }

    void dJointGetTransmissionAxis( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );
        dUASSERT(mode == TRANSMISSION.dTransmissionParallelAxes,
                 "axes must be queried individualy in current mode" );

        if (node[0].body != null) {
//            dBodyVectorToWorld(joint->node[0].body,
//                               joint->axes[0][0],
//                               joint->axes[0][1],
//                               joint->axes[0][2],
//                               result);
        	node[0].body.vectorToWorld(axes[0], result);
        }
    }

    void dJointGetTransmissionAxis1( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );

        if (node[0].body != null) {
//            dBodyVectorToWorld(joint->node[0].body,
//                               joint->axes[0][0],
//                               joint->axes[0][1],
//                               joint->axes[0][2],
//                               result);
        	node[0].body.vectorToWorld(axes[0], result);
        }
    }

    void dJointGetTransmissionAxis2( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );

        if (node[1].body != null) {
//            dBodyVectorToWorld(joint->node[1].body,
//                               joint->axes[1][0],
//                               joint->axes[1][1],
//                               joint->axes[1][2],
//                               result);
        	node[1].body.vectorToWorld(axes[1], result);
        }
    }

    void dJointGetTransmissionAnchor1( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );

        if (node[0].body != null) {
//            dBodyGetRelPointPos(node[0].body,
//                                anchors[0][0],
//                                anchors[0][1],
//                                anchors[0][2],
//                                result);
        	node[0].body.getRelPointPos(anchors[0], result);
        }
    }

    void dJointGetTransmissionAnchor2( DVector3 result )
    {
        dUASSERT( result, "bad result argument" );

        if (node[1].body != null) {
//            dBodyGetRelPointPos(node[1].body,
//                                anchors[1][0], 
//                                anchors[1][1], 
//                                anchors[1][2],
//                                result);
        	node[1].body.getRelPointPos(anchors[1], result);
        }
    }

    void dJointSetTransmissionParam( DJoint.PARAM parameter, double value )
    {
        switch ( parameter ) {
            case dParamCFM:
                cfm = value;
                break;
            case dParamERP:
                erp = value;
                break;
    		default:
    			//ignore
        }
    }


    double dJointGetTransmissionParam( DJoint.PARAM parameter )
    {
        switch ( parameter ) {
            case dParamCFM:
                return cfm;
            case dParamERP:
                return erp;
            default:
                return 0;
        }
    }

    void dJointSetTransmissionMode( TRANSMISSION mode )
    {
        dUASSERT( mode == TRANSMISSION.dTransmissionParallelAxes ||
                  mode == TRANSMISSION.dTransmissionIntersectingAxes ||
                  mode == TRANSMISSION.dTransmissionChainDrive, "invalid joint mode" );

        this.mode = mode;
    }


    TRANSMISSION dJointGetTransmissionMode()
    {
        return mode;
    }

    void dJointSetTransmissionRatio(double ratio )
    {
        dUASSERT( mode == TRANSMISSION.dTransmissionParallelAxes,
                  "can't set ratio explicitly in current mode" );
        dUASSERT( ratio > 0, "ratio must be positive" );

        this.ratio = ratio;
    }


    double dJointGetTransmissionRatio()
    {
        return ratio;
    }

    double dJointGetTransmissionAngle1()
    {
        return phase[0];
    }

    double dJointGetTransmissionAngle2()
    {
        return phase[1];
    }

    double dJointGetTransmissionRadius1()
    {
        return radii[0];
    }

    double dJointGetTransmissionRadius2()
    {
        return radii[1];
    }

    void dJointSetTransmissionRadius1( double radius )
    {
        dUASSERT( mode == TRANSMISSION.dTransmissionChainDrive,
                  "can't set wheel radius explicitly in current mode" );

        radii[0] = radius;
    }

    void dJointSetTransmissionRadius2( double radius )
    {
        dUASSERT( mode == TRANSMISSION.dTransmissionChainDrive,
                  "can't set wheel radius explicitly in current mode" );

        radii[1] = radius;
    }

    double dJointGetTransmissionBacklash()
    {
        return backlash;
    }

    void dJointSetTransmissionBacklash( double backlash )
    {
        this.backlash = backlash;
    }

	// *******************************
	// API dTransmission
	// *******************************

	@Override
	public void setAxis(double x, double y, double z) {
		dJointSetTransmissionAxis(new DVector3(x, y, z));
	}

	@Override
	public void setAxis(DVector3C xyz) {
		dJointSetTransmissionAxis(xyz);
	}

	@Override
	public void getAxis(DVector3 result) {
		dJointGetTransmissionAxis(result);
	}

	@Override
	public void getContactPoint1(DVector3 result) {
		dJointGetTransmissionContactPoint1(result);
	}

	@Override
	public void getContactPoint2(DVector3 result) {
		dJointGetTransmissionContactPoint2(result);
	}

	@Override
	public void setAxis1(double x, double y, double z) {
		dJointSetTransmissionAxis1(new DVector3(x, y, z));
	}

	@Override
	public void setAxis1(DVector3C xyz) {
		dJointSetTransmissionAxis1(xyz);
	}

	@Override
	public void getAxis1(DVector3 result) {
		dJointGetTransmissionAxis1(result);
	}

	@Override
	public void setAxis2(double x, double y, double z) {
		dJointSetTransmissionAxis2(new DVector3(x, y, z));
	}

	@Override
	public void setAxis2(DVector3C xyz) {
		dJointSetTransmissionAxis2(xyz);
	}

	@Override
	public void getAxis2(DVector3 result) {
		dJointGetTransmissionAxis2(result);
	}

	@Override
	public void setAnchor1(double x, double y, double z) {
		dJointSetTransmissionAnchor1(new DVector3(x, y, z));
	}

	@Override
	public void setAnchor1(DVector3C xyz) {
		dJointSetTransmissionAnchor1(xyz);
	}

	@Override
	public void getAnchor1(DVector3 result) {
		dJointGetTransmissionAnchor1(result);
	}

	@Override
	public void setAnchor2(double x, double y, double z) {
		dJointSetTransmissionAnchor2(new DVector3(x, y, z));
	}

	@Override
	public void setAnchor2(DVector3C xyz) {
		dJointSetTransmissionAnchor2(xyz);
	}

	@Override
	public void getAnchor2(DVector3 result) {
		dJointGetTransmissionAnchor2(result);
	}

	@Override
	public void setMode(TRANSMISSION mode) {
		dJointSetTransmissionMode(mode);
	}

	@Override
	public TRANSMISSION getMode() {
		return dJointGetTransmissionMode();
	}

	@Override
	public void setRatio(double ratio) {
		dJointSetTransmissionRatio(ratio);
	}

	@Override
	public double getRatio() {
		return dJointGetTransmissionRatio();
	}

	@Override
	public double getAngle1() {
		return dJointGetTransmissionAngle1();
	}

	@Override
	public double getAngle2() {
		return dJointGetTransmissionAngle2();
	}

	@Override
	public double getRadius1() {
		return dJointGetTransmissionRadius1();
	}

	@Override
	public double getRadius2() {
		return dJointGetTransmissionRadius2();
	}

	@Override
	public void setRadius1(double radius) {
		dJointSetTransmissionRadius1(radius);
	}

	@Override
	public void setRadius2(double radius) {
		dJointSetTransmissionRadius2(radius);
	}

	@Override
	public double getBacklash() {
		return dJointGetTransmissionBacklash();
	}

	@Override
	public void setBacklash(double backlash) {
		dJointSetTransmissionBacklash(backlash);
	}

	@Override
	public void setParam(PARAM_N parameter, double value) {
		dJointSetTransmissionParam(parameter.toSUB(), value);
	}

	@Override
	public double getParam(PARAM_N parameter) {
		return dJointGetTransmissionParam(parameter.toSUB());
	}

}
