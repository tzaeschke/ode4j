/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Z�schke      *
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

import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.libccd.CCDQuat.ccdQuatRotVec;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.CollisionLibccd.ccd_cyl_t;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

class CollisionLibccdCylinderStacking {

    public static int collideCylCyl(DGeom o1, DGeom o2, ccd_cyl_t cyl1, ccd_cyl_t cyl2, int flags,
                                    DContactGeomBuffer contacts) {
        int maxContacts = Math.min(flags & DxGeom.NUMC_MASK, 8);
        dAASSERT(maxContacts != 0);

        maxContacts = maxContacts > 8 ? 8 : maxContacts;
        double axesProd = Math.abs(ccdVec3Dot(cyl1.axis, cyl2.axis));
        // Check if cylinders' axes are in line
        if (1.0 - axesProd < 1e-3) {
            ccd_vec3_t p = new ccd_vec3_t();
            double r1 = ((DxCylinder) o1).getRadius();
            double r2 = ((DxCylinder) o2).getRadius();
            double l1 = 0.5 * ((DxCylinder) o1).getLength();
            double l2 = 0.5 * ((DxCylinder) o2).getLength();
            // Determine the cylinder with smaller radius (minCyl) and bigger
            // radius (maxCyl) and their respective properties: radius, length
            boolean r1IsMin;
            double rmin, rmax;
            ccd_cyl_t minCyl, maxCyl;
            if (r1 <= r2) {
                rmin = r1;
                rmax = r2;
                minCyl = cyl1;
                maxCyl = cyl2;
                r1IsMin = true;
            } else {
                rmin = r2;
                rmax = r1;
                minCyl = cyl2;
                maxCyl = cyl1;
                r1IsMin = false;
            }

            double lSum = l1 + l2;

            ccdVec3Copy(p, minCyl.pos);
            ccdVec3Sub(p, maxCyl.pos);
            double dot = ccdVec3Dot(p, maxCyl.axis);

            // Maximum possible contact depth
            double depth_v = lSum - Math.abs(dot) + Math.sqrt(Math.max(0, 1.0 - axesProd * axesProd)) * rmin;
            if (depth_v < 0.0) {
                return 0;
            }

            // Project the smaller cylinder's center onto the larger cylinder's
            // plane
            ccd_vec3_t proj = new ccd_vec3_t();
            ccdVec3Copy(proj, maxCyl.axis);
            ccdVec3Scale(proj, -dot);
            ccdVec3Add(proj, p);
            double radiiDif = Math.sqrt(ccdVec3Len2(proj));
            double depth_h = r1 + r2 - radiiDif;
            // Check the distance between cylinders' centers
            if (depth_h < 0.0) {
                return 0;
            }
            // Check if "vertical" contact depth is less than "horizontal"
            // contact depth
            if (depth_v < depth_h) {
                double dot2 = -ccdVec3Dot(p, minCyl.axis);
                // lmin, lmax - distances from cylinders' centers to potential
                // contact points relative to cylinders' axes
                double lmax = r1IsMin ? l2 : l1;
                double lmin = r1IsMin ? l1 : l2;
                lmin = dot2 < 0 ? -lmin : lmin;
                lmax = dot < 0 ? -lmax : lmax;
                // Contact normal direction, relative to o1's axis
                double normaldir = (dot < 0 != r1IsMin) ? 1 : -1;
                int contactCount = 0;
                if (rmin + radiiDif <= rmax) {
                    // Case 1: The smaller disc is fully contained within the
                    // larger one
                    // Simply generate N points on the rim of the smaller disc
                    double maxContactRecip = Math.PI * 2 / maxContacts;
                    double maxContactsRecip = (0 < maxContacts ? (2.0 * Math.PI / maxContacts) : (2.0 * Math.PI)); // The 'else' value does not matter. Just try helping the optimizer.
                    for (int i = 0; i < maxContacts; i++) {
                        RefDouble depth = new RefDouble();
                        double a = maxContactRecip * i;
                        if (testAndPrepareDiscContactForAngle(a, rmin, lmin, lSum, minCyl, maxCyl, p, depth)) {
                            contactCount = addCylCylContact(o1, o2, maxCyl.axis, contacts, p, normaldir, depth.get(), contactCount, flags);
                            if ((flags & CONTACTS_UNIMPORTANT) != 0) {
                                dIASSERT(contactCount != 0);
                                break;
                            }
                        }
                    }
                    return contactCount;
                } else {
                    // Case 2: Discs intersect
                    // Firstly, find intersections assuming the larger cylinder
                    // is placed at (0,0,0)
                    // http://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect
                    ccd_vec3_t proj2 = new ccd_vec3_t();
                    ccdVec3Copy(proj2, proj);
                    ccdQuatRotVec(proj, maxCyl.rot_inv);
                    double d = Math.sqrt(proj.get0() * proj.get0() + proj.get1() * proj.get1());
                    dIASSERT(d != 0.0);

                    double dRecip = 1.0 / d;
                    double rmaxSquare = rmax * rmax, rminSquare = rmin * rmin, dSquare = d * d;

                    double minA, diffA, minB, diffB;
                    {
                        double l = (rmaxSquare - rminSquare + dSquare) * (0.5 * dRecip);
                        double h = dSqrt(rmaxSquare - l * l);
                        double divLbyD = l * dRecip, divHbyD = h * dRecip;
                        double x1 = divLbyD * ccdVec3X(proj) + divHbyD * ccdVec3Y(proj);
                        double y1 = divLbyD * ccdVec3Y(proj) - divHbyD * ccdVec3X(proj);
                        double x2 = divLbyD * ccdVec3X(proj) - divHbyD * ccdVec3Y(proj);
                        double y2 = divLbyD * ccdVec3Y(proj) + divHbyD * ccdVec3X(proj);
                        // Map the intersection points to angles
                        double ap1 = dAtan2(y1, x1);
                        double ap2 = dAtan2(y2, x2);
                        minA = dMin(ap1, ap2);
                        double maxA = dMax(ap1, ap2);
                        // If the segment connecting cylinders' centers does not intersect the arc, change the angles
                        double a = dAtan2(ccdVec3Y(proj), ccdVec3X(proj));
                        if (a < minA || a > maxA) {
                            a = maxA;
                            maxA = minA + Math.PI * 2.0;
                            minA = a;
                        }
                        diffA = maxA - minA;
                    }

                    // Do the same for the smaller cylinder assuming it is
                    // placed at (0,0,0) now
                    ccdVec3Copy(proj, proj2);
                    ccdVec3Scale(proj, -1);
                    ccdQuatRotVec(proj, minCyl.rot_inv);
                    {
                        double l = (rminSquare - rmaxSquare + dSquare) * (0.5 * dRecip);
                        double h = dSqrt(rminSquare - l * l);
                        double divLbyD = l * dRecip, divHbyD = h * dRecip;
                        double x1 = divLbyD * ccdVec3X(proj) + divHbyD * ccdVec3Y(proj);
                        double y1 = divLbyD * ccdVec3Y(proj) - divHbyD * ccdVec3X(proj);
                        double x2 = divLbyD * ccdVec3X(proj) - divHbyD * ccdVec3Y(proj);
                        double y2 = divLbyD * ccdVec3Y(proj) + divHbyD * ccdVec3X(proj);
                        double ap1 = dAtan2(y1, x1);
                        double ap2 = dAtan2(y2, x2);
                        minB = dMin(ap1, ap2);
                        double maxB = dMax(ap1, ap2);
                        double a = dAtan2(ccdVec3Y(proj), ccdVec3X(proj));
                        if (a < minB || a > maxB) {
                            a = maxB;
                            maxB = (minB + Math.PI * 2.0);
                            minB = a;
                        }
                        diffB = maxB - minB;
                    }

                    // Find contact point distribution ratio based on arcs
                    // lengths
                    double ratio = diffA * rmax / (diffA * rmax + diffB * rmin);
                    dIASSERT(ratio <= 1.0);
                    dIASSERT(ratio >= 0.0);
                    ratio = dMin(ratio, 1.0);
                    ratio = dMax(ratio, 0.0);

                    int nMax = (int) dFloor(ratio * maxContacts + 0.5);
                    int nMin = maxContacts - nMax;
                    dIASSERT(nMax <= maxContacts);

                    // Make sure there is at least one point on the smaller radius rim
                    if (nMin < 1) {
                        nMin = 1;
                        nMax -= 1;
                    }
                    // Otherwise transfer one point to the larger radius rim as it is going to fill the rim intersection points
                    else if (nMin > 1) {
                        nMin -= 1;
                        nMax += 1;
                    }

                    // Smaller disc first, skipping the overlapping points
                    double nMinRecip = 0 < nMin ? diffB / (nMin + 1) : diffB; // The 'else' value does not matter. Just try helping the optimizer.
                    for (int i = 1; i <= nMin; i++) {
                        RefDouble depth = new RefDouble();
                        double a = minB + nMinRecip * i;
                        if (testAndPrepareDiscContactForAngle(a, rmin, lmin, lSum, minCyl, maxCyl, p, depth)) {
                            contactCount = addCylCylContact(o1, o2, maxCyl.axis, contacts, p, normaldir, depth.get(), contactCount, flags);
                            if ((flags & CONTACTS_UNIMPORTANT) != 0) {
                                dIASSERT(contactCount != 0);
                                break;
                            }
                        }
                    }

                    if (contactCount == 0 || (flags & CONTACTS_UNIMPORTANT) == 0) {
                        // Then the larger disc, + additional point as the start/end points of arcs overlap
                        // (or a single contact at the arc middle point if just one is required)
                        double nMaxRecip = nMax > 1 ? diffA / (nMax - 1) : diffA; // The 'else' value does not matter. Just try helping the optimizer.
                        double adjustedMinA = nMax == 1 ? minA + 0.5 * diffA : minA;

                        for (int i = 0; i < nMax; i++) {
                            RefDouble depth = new RefDouble();
                            double a = adjustedMinA + nMaxRecip * i;
                            if (testAndPrepareDiscContactForAngle(a, rmax, lmax, lSum, maxCyl, minCyl, p, depth)) {
                                contactCount = addCylCylContact(o1, o2, maxCyl.axis, contacts, p, normaldir, depth.get(), contactCount, flags);
                                if ((flags & CONTACTS_UNIMPORTANT) != 0) {
                                    dIASSERT(contactCount != 0);
                                    break;
                                }
                            }
                        }
                    }

                    return contactCount;
                }
            }
        }
        return -1;
    }

    private static boolean testAndPrepareDiscContactForAngle(double angle, double radius, double length, double lSum,
                                                             ccd_cyl_t priCyl, ccd_cyl_t secCyl, ccd_vec3_t p, RefDouble out_depth) {
        boolean ret = false;

        ccd_vec3_t p2 = new ccd_vec3_t();
        ccdVec3Set(p, dCos(angle) * radius, dSin(angle) * radius, 0);
        ccdQuatRotVec(p, priCyl.rot);
        ccdVec3Add(p, priCyl.pos);
        ccdVec3Copy(p2, p);
        ccdVec3Sub(p2, secCyl.pos);
        double depth = lSum - dFabs(ccdVec3Dot(p2, secCyl.axis));

        if (depth >= 0) {
            ccdVec3Copy(p2, priCyl.axis);
            ccdVec3Scale(p2, length);
            ccdVec3Add(p, p2);

            out_depth.set(depth);
            ret = true;
        }

        return ret;
    }

    // Adds a contact between 2 cylinders provided its depth is >= 0
    private static int addCylCylContact(DGeom o1, DGeom o2, ccd_vec3_t axis, DContactGeomBuffer contacts, ccd_vec3_t p,
                                        double normaldir, double depth, int j, int flags) {
        dIASSERT(depth >= 0);

        DContactGeom contact = contacts.getSafe(flags, j);
        contact.g1 = o1;
        contact.g2 = o2;
        contact.side1 = -1;
        contact.side2 = -1;
        contact.normal.set(normaldir * ccdVec3X(axis), normaldir * ccdVec3Y(axis), normaldir * ccdVec3Z(axis));
        contact.depth = depth;
        contact.pos.set(ccdVec3X(p), ccdVec3Y(p), ccdVec3Z(p));

        return j + 1;
    }

}