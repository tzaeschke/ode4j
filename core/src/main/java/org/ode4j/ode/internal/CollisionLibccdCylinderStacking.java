package org.ode4j.ode.internal;

import static org.ode4j.ode.internal.libccd.CCDQuat.ccdQuatRotVec;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Add;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Copy;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Dot;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Len2;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Scale;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Sub;

import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.internal.CollisionLibccd.ccd_cyl_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

public class CollisionLibccdCylinderStacking {

	public static int collideCylCyl(DGeom o1, DGeom o2, ccd_cyl_t cyl1, ccd_cyl_t cyl2, int flags,
			DContactGeomBuffer contacts) {
		int maxContacts = Math.min(flags & DxGeom.NUMC_MASK, 8);
		if (maxContacts == 0) {
			return 0;
		}
		double axesProd = Math.abs(ccdVec3Dot(cyl1.axis, cyl2.axis));
		// Check if cylinders' axes are in line
		if (1.0 - axesProd < 1e-3) {
			ccd_vec3_t p = new ccd_vec3_t();
			double r1 = ((DxCylinder) o1).getRadius();
			double r2 = ((DxCylinder) o2).getRadius();
			// Determine the cylinder with smaller radius (minCyl) and bigger
			// radius (maxCyl) and their respective properties: radius, length
			double l1 = 0.5 * ((DxCylinder) o1).getLength();
			double l2 = 0.5 * ((DxCylinder) o2).getLength();
			double rmin = Math.min(r1, r2);
			double rmax = Math.max(r1, r2);
			boolean r1isMin = r1 == rmin;
			ccd_cyl_t minCyl = r1isMin ? cyl1 : cyl2;
			ccd_cyl_t maxCyl = r1isMin ? cyl2 : cyl1;
			ccdVec3Copy(p, minCyl.pos);
			ccdVec3Sub(p, maxCyl.pos);
			double dot = ccdVec3Dot(p, maxCyl.axis);
			double lSum = l1 + l2;
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
				double lmax = r1isMin ? l2 : l1;
				double lmin = r1isMin ? l1 : l2;
				lmax = dot < 0 ? -lmax : lmax;
				lmin = dot2 < 0 ? -lmin : lmin;
				// Contact normal direction, relative to o1's axis
				double normaldir = (dot < 0 != r1isMin) ? 1 : -1;
				int contactCount = 0;
				if (rmin + radiiDif <= rmax) {
					// Case 1: The smaller disc is fully contained within the
					// larger one
					// Simply generate N points on the rim of the smaller disc
					double maxContactRecip = Math.PI * 2 / maxContacts;
					for (int i = 0; i < maxContacts; i++) {
						double a = i * maxContactRecip;
						contactCount = testAndAddDiscContact(a, rmin, lmin, lSum, minCyl, maxCyl, normaldir, o1, o2,
								contacts, contactCount);
						if (contactCount > 0 && (flags & OdeConstants.CONTACTS_UNIMPORTANT) != 0) {
							break;
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
					double dRecip = 1.0 / d;
					double rmaxSquare = rmax * rmax;
					double rminSquare = rmin * rmin;
					double dSquare = d * d;
					double l = (rmaxSquare - rminSquare + dSquare) * 0.5 * dRecip;
					double h = Math.sqrt(rmaxSquare - l * l);
					double divLbyD = l * dRecip;
					double divHbyD = h * dRecip;
					double x1 = divLbyD * proj.get0() + divHbyD * proj.get1();
					double y1 = divLbyD * proj.get1() - divHbyD * proj.get0();
					double x2 = divLbyD * proj.get0() - divHbyD * proj.get1();
					double y2 = divLbyD * proj.get1() + divHbyD * proj.get0();
					// Map the intersection points to angles
					double ap1 = Math.atan2(y1, x1);
					double ap2 = Math.atan2(y2, x2);
					double minA = Math.min(ap1, ap2);
					double maxA = Math.max(ap1, ap2);
					// If the segment connecting cylinders' centers does not
					// intersect the arc, change the angles
					double a = Math.atan2(proj.get1(), proj.get0());
					if (a < minA || a > maxA) {
						a = maxA;
						maxA = minA + Math.PI * 2;
						minA = a;
					}
					double diffA = maxA - minA;
					// Do the same for the smaller cylinder assuming it is
					// placed at (0,0,0) now
					ccdVec3Copy(proj, proj2);
					ccdVec3Scale(proj, -1);
					ccdQuatRotVec(proj, minCyl.rot_inv);
					l = (rminSquare - rmaxSquare + dSquare) * 0.5 * dRecip;
					h = Math.sqrt(rminSquare - l * l);
					divLbyD = l * dRecip;
					divHbyD = h * dRecip;
					x1 = divLbyD * proj.get0() + divHbyD * proj.get1();
					y1 = divLbyD * proj.get1() - divHbyD * proj.get0();
					x2 = divLbyD * proj.get0() - divHbyD * proj.get1();
					y2 = divLbyD * proj.get1() + divHbyD * proj.get0();
					ap1 = Math.atan2(y1, x1);
					ap2 = Math.atan2(y2, x2);
					double minB = Math.min(ap1, ap2);
					double maxB = Math.max(ap1, ap2);
					a = Math.atan2(proj.get1(), proj.get0());
					if (a < minB || a > maxB) {
						a = maxB;
						maxB = minB + Math.PI * 2;
						minB = a;
					}
					double diffB = maxB - minB;
					// Find contact point distribution ratio based on arcs
					// lengths
					double ratio = diffA * rmax / (diffA * rmax + diffB * rmin);
					int nMax = (int) Math.round(ratio * maxContacts);
					int nMin = maxContacts - nMax;
					// Make sure there is at least one point on the smaller
					// radius rim
					if (nMin < 1) {
						nMin = 1;
						nMax -= 1;
					}
					// Otherwise transfer one point to the larger radius rim as
					// it is going to fill the rim intersection points
					else if (nMin > 1) {
						nMin -= 1;
						nMax += 1;
					}

					// Smaller disc first, skipping the overlapping points
					double nMinRecip = 0 < nMin ? diffB / (nMin + 1) : diffB;
					for (int i = 1; i <= nMin; i++) {
						a = minB + nMinRecip * i;
						contactCount = testAndAddDiscContact(a, rmin, lmin, lSum, minCyl, maxCyl, normaldir, o1, o2,
								contacts, contactCount);
						if (contactCount > 0 && (flags & OdeConstants.CONTACTS_UNIMPORTANT) != 0) {
							break;
						}
					}
					if (contactCount == 0 || (flags & OdeConstants.CONTACTS_UNIMPORTANT) == 0) {
						double nMaxRecip = nMax > 1 ? diffA / (nMax - 1) : diffA;
						double adjustedMinA = nMax == 1 ? minA + 0.5 * diffA : minA;
						// Then the larger disc, + additional point as the
						// start/end points of arcs overlap
						// (or a single contact at the arc middle point if just
						// one is required)
						for (int i = 0; i < nMax; i++) {
							a = adjustedMinA + nMaxRecip * i;
							contactCount = testAndAddDiscContact(a, rmax, lmax, lSum, maxCyl, minCyl, normaldir, o1, o2,
									contacts, contactCount);
							if (contactCount > 0 && (flags & OdeConstants.CONTACTS_UNIMPORTANT) != 0) {
								break;
							}
						}
					}
					return contactCount;
				}
			}
		}
		return -1;
	}

	private static int testAndAddDiscContact(double angle, double radius, double length, double lSum, ccd_cyl_t c1,
			ccd_cyl_t maxCyl, double normaldir, DGeom o1, DGeom o2, DContactGeomBuffer contacts, int contactCount) {
		ccd_vec3_t p = new ccd_vec3_t();
		p.set(Math.cos(angle) * radius, Math.sin(angle) * radius, 0);
		ccd_vec3_t p2 = new ccd_vec3_t();
		// Transform to world coordinates
		ccdQuatRotVec(p, c1.rot);
		ccdVec3Add(p, c1.pos);
		ccdVec3Copy(p2, p);
		ccdVec3Sub(p2, maxCyl.pos);
		// Calculate contact depth
		double depth = lSum - Math.abs(ccdVec3Dot(p2, maxCyl.axis));
		if (depth >= 0) {
			// Contact point
			ccdVec3Copy(p2, c1.axis);
			ccdVec3Scale(p2, length);
			ccdVec3Add(p, p2);
			contactCount = addContact(o1, o2, maxCyl.axis, contacts, p, normaldir, depth, contactCount);
		}
		return contactCount;
	}

	// Adds a contact between 2 cylinders provided its depth is >= 0
	private static int addContact(DGeom o1, DGeom o2, ccd_vec3_t axis, DContactGeomBuffer contacts, ccd_vec3_t p,
			double normaldir, double depth, int j) {
		DContactGeom contact = contacts.get(j++);
		contact.g1 = o1;
		contact.g2 = o2;
		contact.side1 = -1;
		contact.side2 = -1;
		contact.normal.set0(normaldir * axis.get0());
		contact.normal.set1(normaldir * axis.get1());
		contact.normal.set2(normaldir * axis.get2());
		contact.depth = depth;
		contact.pos.set(p.get0(), p.get1(), p.get2());
		return j;
	}

}