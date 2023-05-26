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

import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.dIASSERT;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;


/** 
 * ****************************************************************************
 * contact
 */
public class DxJointContact extends DxJoint implements DContactJoint
{
	int the_m;   // number of rows computed by getInfo1
	private final DContact contact = new DContact();

	DxJointContact( DxWorld w ) 
	//        dxJoint( w )
	{
		super (w);
	}

	@Override
	void getSureMaxInfo( SureMaxInfo info )
	{
	    info.max_m = 3; // ...as the actual m is very likely to hit the maximum
	    // ...as the actual m is very likely to hit the maximum
	    info.max_m = (contact.surface.mode&OdeConstants.dContactRolling)!=0?6:3; 
	}


	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		// make sure mu's >= 0, then calculate number of constraint rows and number
		// of unbounded rows.
		int m = 1, nub = 0;

		// Anisotropic sliding and rolling and spinning friction
		if ((contact.surface.mode & dContactAxisDep) != 0) {
			if (contact.surface.mu < 0) {
				contact.surface.mu = 0;
			}
			else if (contact.surface.mu > 0) {
				if (contact.surface.mu == dInfinity) { nub++; }
				m++;
			}

			if (contact.surface.mu2 < 0) {
				contact.surface.mu2 = 0;
			}
			else if (contact.surface.mu2 > 0) {
				if (contact.surface.mu2 == dInfinity) { nub++; }
				m++;
			}

			if ((contact.surface.mode & dContactRolling) != 0) {
				if (contact.surface.rho < 0) {
					contact.surface.rho = 0;
				}
				else {
					if (contact.surface.rho == dInfinity) { nub++; }
					m++;
				}

				if (contact.surface.rho2 < 0) {
					contact.surface.rho2 = 0;
				}
				else {
					if (contact.surface.rho2 == dInfinity) { nub++; }
					m++;
				}

				if (contact.surface.rhoN < 0) {
					contact.surface.rhoN = 0;
				}
				else {
					if (contact.surface.rhoN == dInfinity) { nub++; }
					m++;
				}
			}
		}
		else {
			if (contact.surface.mu < 0) {
				contact.surface.mu = 0;
			}
			else if (contact.surface.mu > 0) {
				if (contact.surface.mu == dInfinity) { nub += 2; }
				m += 2;
			}

			if ((contact.surface.mode & dContactRolling) != 0) {
				if (contact.surface.rho < 0) {
					contact.surface.rho = 0;
				}
				else {
					if (contact.surface.rho == dInfinity) { nub += 3; }
					m += 3;
				}
			}
		}

		the_m = m;
		info.setM(m);
		info.setNub(nub);
	}

	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		final int ROW_NORMAL = 0;
		final int ROW__OPTIONAL_MIN = 1;

		final int surface_mode = contact.surface.mode;

		// set right hand side and cfm value for normal
		double erp = (surface_mode & dContactSoftERP) != 0 ? contact.surface.soft_erp : worldERP;
		double k = worldFPS * erp;

		double depth = contact.geom.depth - world.contactp.min_depth;
		if (depth < 0) depth = 0;

		double motionN = (surface_mode & dContactMotionN) != 0 ? contact.surface.motionN : 0.0;
		final double pushout = k * depth + motionN;

		boolean apply_bounce = (surface_mode & dContactBounce) != 0 && contact.surface.bounce_vel >= 0;
		double outgoing = 0;

		// note: this cap should not limit bounce velocity
		final double maxvel = world.contactp.max_vel;
		double c = Math.min(pushout, maxvel);

		// c1,c2 = contact points with respect to body PORs
		DVector3 c1 = new DVector3(), c2 = new DVector3();

		// get normal, with sign adjusted for body1/body2 polarity
		DVector3 normal = new DVector3();
		if ((flags & dJOINT_REVERSE) != 0) {
			dCopyNegatedVector3(normal, contact.geom.normal);
		}
		else {
			dCopyVector3(normal, contact.geom.normal);
		}

		DxBody b1 = node[1].body;
		if (b1 != null) {
			dSubtractVectors3(c2, contact.geom.pos, b1.posr().pos());
			// set Jacobian for b1 normal
			dCopyNegatedVector3(J2A, J2Ofs + ROW_NORMAL * rowskip + GI2__JL_MIN, normal);
			dCalcVectorCross3(J2A, J2Ofs + ROW_NORMAL * rowskip + GI2__JA_MIN, normal, c2); //== dCalcVectorCross3( J2 + GI2__JA_MIN, c2, normal ); dNegateVector3( J2 + GI2__JA_MIN );
			if (apply_bounce) {
				outgoing /*+*/= dCalcVectorDot3(J2A, J2Ofs + ROW_NORMAL * rowskip + GI2__JA_MIN, node[1].body.avel)
				- dCalcVectorDot3(normal, node[1].body.lvel);
			}
		}

		DxBody b0 = node[0].body;
		dSubtractVectors3(c1, contact.geom.pos, b0.posr().pos());
		// set Jacobian for b0 normal
		dCopyVector3(J1A, J1Ofs + ROW_NORMAL * rowskip + GI2__JL_MIN, normal);
		dCalcVectorCross3(J1A, J1Ofs + ROW_NORMAL * rowskip + GI2__JA_MIN, c1, normal);
		if (apply_bounce) {
			// calculate outgoing velocity (-ve for incoming contact)
			outgoing += dCalcVectorDot3(J1A, J1Ofs + ROW_NORMAL * rowskip + GI2__JA_MIN, node[0].body.avel)
			+ dCalcVectorDot3(normal, node[0].body.lvel);
		}

		// deal with bounce
		if (apply_bounce) {
			double negated_outgoing = motionN - outgoing;
			// only apply bounce if the outgoing velocity is greater than the
			// threshold, and if the resulting c[rowNormal] exceeds what we already have.
			dIASSERT(contact.surface.bounce_vel >= 0);
			if (/*contact.surface.bounce_vel >= 0 &&*/
					negated_outgoing > contact.surface.bounce_vel) {
            final double newc = contact.surface.bounce * negated_outgoing + motionN;
				if (newc > c) { c = newc; }
			}
		}

		pairRhsCfmA[pairRhsCfmOfs + ROW_NORMAL * pairskip + GI2_RHS] = c;

		if ((surface_mode & dContactSoftCFM) != 0) {
			pairRhsCfmA[pairRhsCfmOfs + ROW_NORMAL * pairskip + GI2_CFM] = contact.surface.soft_cfm;
		}

		// set LCP limits for normal
		pairLoHiA[pairLoHiOfs + ROW_NORMAL * pairskip + GI2_LO] = 0;
		pairLoHiA[pairLoHiOfs + ROW_NORMAL * pairskip + GI2_HI] = dInfinity;


		if (the_m > 1) { // if no friction, there is nothing else to do
			// now do jacobian for tangential forces
			DVector3 t1 = new DVector3(), t2 = new DVector3(); // two vectors tangential to normal

			if ((surface_mode & dContactFDir1) != 0) {   // use fdir1 ?
				dCopyVector3(t1, contact.fdir1);
				dCalcVectorCross3(t2, normal, t1);
			} else {
				dPlaneSpace(normal, t1, t2);
			}

			int row = ROW__OPTIONAL_MIN;
			int currRowSkip = row * rowskip, currPairSkip = row * pairskip;

			// first friction direction
      		final double mu = contact.surface.mu;

			if (mu > 0) {
				dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, t1);
				dCalcVectorCross3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, c1, t1);

				if (node[1].body != null) {
					dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, t1);
					dCalcVectorCross3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, t1, c2); //== dCalcVectorCross3( J2 +
					// rowskip + GI2__JA_MIN, c2, t1 ); dNegateVector3( J2 + rowskip + GI2__JA_MIN );
				}

				// set right hand side
				if ((surface_mode & dContactMotion1) != 0) {
					pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = contact.surface.motion1;
				}
				// set slip (constraint force mixing)
				if ((surface_mode & dContactSlip1) != 0) {
					pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_CFM] = contact.surface.slip1;
				}

				// set LCP bounds and friction index. this depends on the approximation
				// mode
				pairLoHiA[pairLoHiOfs + currPairSkip + GI2_LO] = -mu;
				pairLoHiA[pairLoHiOfs + currPairSkip + GI2_HI] = mu;

				if ((surface_mode & dContactApprox1_1) != 0) {
					findexA[findexOfs + row] = 0;
				}

				++row;
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}

			// second friction direction
			final double mu2 = (surface_mode & dContactMu2) != 0 ? contact.surface.mu2 : mu;
			if (mu2 > 0) {
				dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, t2);
				dCalcVectorCross3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, c1, t2);

				if (node[1].body != null) {
					dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, t2);
					dCalcVectorCross3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, t2, c2); //== dCalcVectorCross3( J2 +
					// currRowSkip + GI2__JA_MIN, c2, t2 ); dNegateVector3( J2 + currRowSkip + GI2__JA_MIN );
				}

				// set right hand side
				if ((surface_mode & dContactMotion2) != 0) {
					pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = contact.surface.motion2;
				}
				// set slip (constraint force mixing)
				if ((surface_mode & dContactSlip2) != 0) {
					pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_CFM] = contact.surface.slip2;
				}

				// set LCP bounds and friction index. this depends on the approximation
				// mode
				pairLoHiA[pairLoHiOfs + currPairSkip + GI2_LO] = -mu2;
				pairLoHiA[pairLoHiOfs + currPairSkip + GI2_HI] = mu2;

				if ((surface_mode & dContactApprox1_2) != 0) {
					findexA[findexOfs + row] = 0;
				}

				++row;
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}

			// Handle rolling/spinning friction
			if ((surface_mode & dContactRolling) != 0) {

				DVector3C[] ax = {
						t1, // Rolling around t1 creates movement parallel to t2
						t2,
						normal // Spinning axis
				};

				final int[] approx_bits = { dContactApprox1_1, dContactApprox1_2, dContactApprox1_N };

				// Get the coefficients
				double[] rho = new double[3];
				rho[0] = contact.surface.rho;
				if ((surface_mode & dContactAxisDep) != 0) {
					rho[1] = contact.surface.rho2;
					rho[2] = contact.surface.rhoN;
				} else {
					rho[1] = rho[0];
					rho[2] = rho[0];
				}

				for (int i = 0; i != 3; ++i) {
					if (rho[i] > 0) {
						// Set the angular axis
						dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, ax[i]);

						if (b1 != null) {
							dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, ax[i]);
						}

						// Set the lcp limits
						pairLoHiA[pairLoHiOfs + currPairSkip + GI2_LO] = -rho[i];
						pairLoHiA[pairLoHiOfs + currPairSkip + GI2_HI] = rho[i];

						// Should we use proportional force?
						if ((surface_mode & approx_bits[i]) != 0) {
							// Make limits proportional to normal force
							findexA[findexOfs + row] = 0;
						}

						++row;
						currRowSkip += rowskip;
						currPairSkip += pairskip;
					}
				}
			}
		}
	}


	@Override
	public double getParam(PARAM_N parameter) {
		throw new UnsupportedOperationException();
	}


	@Override
	public void setParam(PARAM_N parameter, double value) {
		throw new UnsupportedOperationException();
	}

	@Override
	public DContact getContact() {
		return contact;
	}

	public void setContact(DContact contact) {
		this.contact.set(contact);
	}
}

