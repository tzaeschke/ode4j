/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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

import org.ode4j.math.DVector3;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.internal.DxWorld;
import static org.ode4j.ode.OdeMath.*;


/** 
 * ****************************************************************************
 * contact
 */
public class DxJointContact extends DxJoint implements DContactJoint
{
	int the_m;   // number of rows computed by getInfo1
	public DContact contact;

	DxJointContact( DxWorld w ) 
	//        dxJoint( w )
	{
		super (w);
	}


	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		// make sure mu's >= 0, then calculate number of constraint rows and number
		// of unbounded rows.
		int m = 1, nub = 0;
		if ( contact.surface.mu < 0 ) contact.surface.mu = 0;
		if ( (contact.surface.mode & dContactMu2) != 0 )
		{
			if ( contact.surface.mu > 0 ) m++;
			if ( contact.surface.mu2 < 0 ) contact.surface.mu2 = 0;
			if ( contact.surface.mu2 > 0 ) m++;
			if ( contact.surface.mu  == dInfinity ) nub ++;
			if ( contact.surface.mu2 == dInfinity ) nub ++;
		}
		else
		{
			if ( contact.surface.mu > 0 ) m += 2;
			if ( contact.surface.mu == dInfinity ) nub += 2;
		}

		the_m = m;
		info.setM(m);
		info.setNub(nub);
	}


	@Override
	public void
	getInfo2( DxJoint.Info2 info )
	{
		int s = info.rowskip();
		int s2 = 2 * s;

		// get normal, with sign adjusted for body1/body2 polarity
		DVector3 normal = new DVector3();
		if ( isFlagsReverse() )
		{
//			normal.v[0] = - contact.geom.normal.v[0];
//			normal.v[1] = - contact.geom.normal.v[1];
//			normal.v[2] = - contact.geom.normal.v[2];
			normal.set( contact.geom.normal ).scale(-1);
		}
		else
		{
//			normal.v[0] = contact.geom.normal.v[0];
//			normal.v[1] = contact.geom.normal.v[1];
//			normal.v[2] = contact.geom.normal.v[2];
			normal.set( contact.geom.normal );
		}
		//normal.v[3] = 0; // @@@ hmmm

		// c1,c2 = contact points with respect to body PORs
		DVector3 c1 = new DVector3(), c2 = new DVector3(); //{0,0,0};
//		c1.v[0] = contact.geom.pos.v[0] - node[0].body._posr.pos.v[0];
//		c1.v[1] = contact.geom.pos.v[1] - node[0].body._posr.pos.v[1];
//		c1.v[2] = contact.geom.pos.v[2] - node[0].body._posr.pos.v[2];
		c1.eqDiff( contact.geom.pos, node[0].body.posr().pos() );

		// set jacobian for normal
		info._J[info.J1lp+0] = normal.get0();
		info._J[info.J1lp+1] = normal.get1();
		info._J[info.J1lp+2] = normal.get2();
		dCROSS( info._J, info.J1ap, OP.EQ , c1, normal );
		if ( node[1].body != null)
		{
//			c2.v[0] = contact.geom.pos.v[0] - node[1].body._posr.pos.v[0];
//			c2.v[1] = contact.geom.pos.v[1] - node[1].body._posr.pos.v[1];
//			c2.v[2] = contact.geom.pos.v[2] - node[1].body._posr.pos.v[2];
			c2.eqDiff( contact.geom.pos, node[1].body.posr().pos() );
			info._J[info.J2lp+0] = -normal.get0();
			info._J[info.J2lp+1] = -normal.get1();
			info._J[info.J2lp+2] = -normal.get2();
			dCROSS( info._J, info.J2ap, OP.EQ_SUB, c2, normal );
		}

		// set right hand side and cfm value for normal
		double erp = info.erp;
		if (( contact.surface.mode & dContactSoftERP ) != 0)
			erp = contact.surface.soft_erp;
		double k = info.fps * erp;
		double depth = contact.geom.depth - world.contactp.min_depth;
		if ( depth < 0 ) depth = 0;

		if (( contact.surface.mode & dContactSoftCFM ) != 0)
			info.setCfm(0, contact.surface.soft_cfm);


		double motionN = 0;
		if (( contact.surface.mode & dContactMotionN ) != 0)
			motionN = contact.surface.motionN;

		final double pushout = k * depth + motionN;
		info.setC(0, pushout);

		// note: this cap should not limit bounce velocity
		final double maxvel = world.contactp.max_vel;
		if ( info.getC(0) > maxvel )
			info.setC(0, maxvel);

		// deal with bounce
		if (( contact.surface.mode & dContactBounce) != 0)
		{
			// calculate outgoing velocity (-ve for incoming contact)
			double outgoing = 
				dDOT( info._J, info.J1lp, node[0].body.lvel )
				+ dDOT( info._J, info.J1ap, node[0].body.avel );
			if ( node[1].body != null)
			{
				outgoing += 
					dDOT( info._J, info.J2lp, node[1].body.lvel )
					+ dDOT( info._J, info.J2ap, node[1].body.avel );
			}
			outgoing -= motionN;
			// only apply bounce if the outgoing velocity is greater than the
			// threshold, and if the resulting c[0] exceeds what we already have.
			if ( contact.surface.bounce_vel >= 0 &&
					( -outgoing ) > contact.surface.bounce_vel )
			{
				double newc = - contact.surface.bounce * outgoing + motionN;
				if ( newc > info.getC(0) ) info.setC(0, newc);
			}
		}

		// set LCP limits for normal
		info.setLo(0, 0);
		info.setHi(0, dInfinity);

		// now do jacobian for tangential forces
		DVector3 t1 = new DVector3(), t2 = new DVector3(); // two vectors tangential to normal

		// first friction direction
		if ( the_m >= 2 )
		{
			if (( contact.surface.mode & dContactFDir1) != 0)   // use fdir1 ?
			{
//				t1.v[0] = contact.fdir1.v[0];
//				t1.v[1] = contact.fdir1.v[1];
//				t1.v[2] = contact.fdir1.v[2];
				t1.set( contact.fdir1 );
				dCROSS( t2, OP.EQ , normal, t1 );
			}
			else
			{
				dPlaneSpace( normal, t1, t2 );
			}
			info._J[info.J1lp+s+0] = t1.get0();
			info._J[info.J1lp+s+1] = t1.get1();
			info._J[info.J1lp+s+2] = t1.get2();
			dCROSS( info._J, info.J1ap + s, OP.EQ , c1, t1 );
			if ( node[1].body != null)
			{
				info._J[info.J2lp+s+0] = -t1.get0();
				info._J[info.J2lp+s+1] = -t1.get1();
				info._J[info.J2lp+s+2] = -t1.get2();
				dCROSS( info._J, info.J2ap + s, OP.EQ_SUB, c2, t1 );
			}
			// set right hand side
			if (( contact.surface.mode & dContactMotion1) != 0)
			{
				info.setC(1, contact.surface.motion1);
			}
			// set LCP bounds and friction index. this depends on the approximation
			// mode
			info.setLo(1, -contact.surface.mu);
			info.setHi(1, contact.surface.mu);
			if (( contact.surface.mode & dContactApprox1_1) != 0)
				//info._findexA[info._findexP+1] = 0;//info.findex[1] = 0;
				info.setFindex(1, 0);

			// set slip (constraint force mixing)
			if (( contact.surface.mode & dContactSlip1) != 0)
				info.setCfm(1, contact.surface.slip1);
		}

		// second friction direction
		if ( the_m >= 3 )
		{
			info._J[info.J1lp+s2+0] = t2.get0();
			info._J[info.J1lp+s2+1] = t2.get1();
			info._J[info.J1lp+s2+2] = t2.get2();
			dCROSS( info._J, info.J1ap + s2, OP.EQ , c1, t2 );
			if ( node[1].body != null)
			{
				info._J[info.J2lp+s2+0] = -t2.get0();
				info._J[info.J2lp+s2+1] = -t2.get1();
				info._J[info.J2lp+s2+2] = -t2.get2();
				dCROSS( info._J, info.J2ap + s2, OP.EQ_SUB, c2, t2 );
			}
			// set right hand side
			if (( contact.surface.mode & dContactMotion2) != 0)
			{
				info.setC(2, contact.surface.motion2);
			}
			// set LCP bounds and friction index. this depends on the approximation
			// mode
			if (( contact.surface.mode & dContactMu2) != 0)
			{
				info.setLo(2, -contact.surface.mu2);
				info.setHi(2, contact.surface.mu2);
			}
			else
			{
				info.setLo(2, -contact.surface.mu);
				info.setHi(2, contact.surface.mu);
			}
			if (( contact.surface.mode & dContactApprox1_2) != 0)
				info.setFindex(2, 0);//info.findex[2] = 0;

			// set slip (constraint force mixing)
			if (( contact.surface.mode & dContactSlip2) != 0)
				info.setCfm(2, contact.surface.slip2);
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
}

