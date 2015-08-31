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

import static org.ode4j.ode.OdeConstants.dContactApprox1_1;
import static org.ode4j.ode.OdeConstants.dContactApprox1_2;
import static org.ode4j.ode.OdeConstants.dContactApprox1_N;
import static org.ode4j.ode.OdeConstants.dContactBounce;
import static org.ode4j.ode.OdeConstants.dContactFDir1;
import static org.ode4j.ode.OdeConstants.dContactMotion1;
import static org.ode4j.ode.OdeConstants.dContactMotion2;
import static org.ode4j.ode.OdeConstants.dContactMotionN;
import static org.ode4j.ode.OdeConstants.dContactMu2;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.dCalcVectorCross3;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.OdeMath.dPlaneSpace;
import static org.ode4j.ode.OdeMath.dNegateVector3;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;


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
	    boolean roll = (contact.surface.mode&OdeConstants.dContactRolling)!=0;
	    
		if ( contact.surface.mu < 0 ) contact.surface.mu = 0;

	    // Anisotropic sliding and rolling and spinning friction 
	    if ( (contact.surface.mode & OdeConstants.dContactAxisDep) != 0 )
		{
			if ( contact.surface.mu2 < 0 ) contact.surface.mu2 = 0;
			if ( contact.surface.mu > 0 ) m++;
			if ( contact.surface.mu2 > 0 ) m++;
			if ( contact.surface.mu  == dInfinity ) nub ++;
			if ( contact.surface.mu2 == dInfinity ) nub ++;
	        if (roll) {
	            if ( contact.surface.rho < 0 ) contact.surface.rho = 0;
	            else m++;
	            if ( contact.surface.rho2 < 0 ) contact.surface.rho2 = 0;
	            else m++;
	            if ( contact.surface.rhoN < 0 ) contact.surface.rhoN = 0;
	            else m++;

	            if ( contact.surface.rho  == dInfinity ) nub++;
	            if ( contact.surface.rho2 == dInfinity ) nub++;
	            if ( contact.surface.rhoN == dInfinity ) nub++;
	          }
		}
		else
		{
			if ( contact.surface.mu > 0 ) m += 2;
			if ( contact.surface.mu == dInfinity ) nub += 2;
	        if (roll) {
	            if ( contact.surface.rho < 0 ) contact.surface.rho = 0;
	            else m+=3;
	            if ( contact.surface.rho == dInfinity ) nub += 3;
	          }
		}

		the_m = m;
		info.setM(m);
		info.setNub(nub);
	}


	@Override
	public void
	getInfo2( double worldFPS, double worldERP, DxJoint.Info2Descr info )
	{
		int s = info.rowskip();
		int s2 = 2 * s;

	    final int rowNormal = 0;
	    final int rowFriction1 = 1;
	    int rowFriction2 = 2; // we might decrease it to 1, so no const
	    int rollRow=3;

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
		dCalcVectorCross3( info._J, info.J1ap, c1, normal );

		DxBody b1 = node[1].body;
		if ( b1 != null)
		{
//			c2.v[0] = contact.geom.pos.v[0] - node[1].body._posr.pos.v[0];
//			c2.v[1] = contact.geom.pos.v[1] - node[1].body._posr.pos.v[1];
//			c2.v[2] = contact.geom.pos.v[2] - node[1].body._posr.pos.v[2];
			c2.eqDiff( contact.geom.pos, b1.posr().pos() );
			info._J[info.J2lp+0] = -normal.get0();
			info._J[info.J2lp+1] = -normal.get1();
			info._J[info.J2lp+2] = -normal.get2();
	        dCalcVectorCross3( info._J, info.J2ap, c2, normal );
	        dNegateVector3( info._J, info.J2ap );
		}

		// set right hand side and cfm value for normal
		double erp = worldERP;
		if (( contact.surface.mode & dContactSoftERP ) != 0)
			erp = contact.surface.soft_erp;
		double k = worldFPS * erp;
		double depth = contact.geom.depth - world.contactp.min_depth;
		if ( depth < 0 ) depth = 0;

		if (( contact.surface.mode & dContactSoftCFM ) != 0)
			info.setCfm(rowNormal, contact.surface.soft_cfm);


		double motionN = 0;
		if (( contact.surface.mode & dContactMotionN ) != 0)
			motionN = contact.surface.motionN;

		final double pushout = k * depth + motionN;
		info.setC(rowNormal, pushout);

		// note: this cap should not limit bounce velocity
		final double maxvel = world.contactp.max_vel;
		if ( info.getC(rowNormal) > maxvel )
			info.setC(rowNormal, maxvel);

		// deal with bounce
		if (( contact.surface.mode & dContactBounce) != 0)
		{
			// calculate outgoing velocity (-ve for incoming contact)
			double outgoing = 
				dCalcVectorDot3( info._J, info.J1lp, node[0].body.lvel )
				+ dCalcVectorDot3( info._J, info.J1ap, node[0].body.avel );
			if ( b1 != null)
			{
				outgoing += 
					dCalcVectorDot3( info._J, info.J2lp, node[1].body.lvel )
					+ dCalcVectorDot3( info._J, info.J2ap, node[1].body.avel );
			}
			outgoing -= motionN;
			// only apply bounce if the outgoing velocity is greater than the
			// threshold, and if the resulting c[0] exceeds what we already have.
			if ( contact.surface.bounce_vel >= 0 &&
					( -outgoing ) > contact.surface.bounce_vel )
			{
				double newc = - contact.surface.bounce * outgoing + motionN;
				if ( newc > info.getC(rowNormal) ) info.setC(rowNormal, newc);
			}
		}

		// set LCP limits for normal
		info.setLo(0, 0);
		info.setHi(0, dInfinity);

		if ( the_m == 1 ) // no friction, there is nothing else to do
		    return;

		// now do jacobian for tangential forces
		DVector3 t1 = new DVector3(), t2 = new DVector3(); // two vectors tangential to normal

		if (( contact.surface.mode & dContactFDir1) != 0)   // use fdir1 ?
		{
//				t1.v[0] = contact.fdir1.v[0];
//				t1.v[1] = contact.fdir1.v[1];
//				t1.v[2] = contact.fdir1.v[2];
			t1.set( contact.fdir1 );
			dCalcVectorCross3( t2, normal, t1 );
		}
		else
		{
			dPlaneSpace( normal, t1, t2 );
		}

		// first friction direction
	    if ( contact.surface.mu > 0 )
	    {
	    	info._J[info.J1lp+s+0] = t1.get0();
	    	info._J[info.J1lp+s+1] = t1.get1();
	    	info._J[info.J1lp+s+2] = t1.get2();
	    	dCalcVectorCross3( info._J, info.J1ap + s, c1, t1 );
	    	
	    	if ( node[1].body != null)
	    	{
	    		info._J[info.J2lp+s+0] = -t1.get0();
	    		info._J[info.J2lp+s+1] = -t1.get1();
	    		info._J[info.J2lp+s+2] = -t1.get2();
	    		//			    dReal *J2a_plus_s = info->J2a + s;
	    		//	            dCalcVectorCross3( J2a_plus_s, c2, t1 );
	    		//	            dNegateVector3( J2a_plus_s );
	    		dCalcVectorCross3( info._J, info.J2ap+s, c2, t1 );
	    		dNegateVector3( info._J, info.J2ap+s );
	    	}
	    	
	    	// set right hand side
	    	if (( contact.surface.mode & dContactMotion1) != 0)
	    	{
	    		info.setC(rowFriction1, contact.surface.motion1);
	    	}
	    	// set LCP bounds and friction index. this depends on the approximation
	    	// mode
	    	info.setLo(rowFriction1, -contact.surface.mu);
	    	info.setHi(rowFriction1, contact.surface.mu);
	    	if (( contact.surface.mode & dContactApprox1_1) != 0)
	    		//info._findexA[info._findexP+1] = 0;//info.findex[1] = 0;
	    		info.setFindex(rowFriction1, 0);

	    	// set slip (constraint force mixing)
	    	if (( contact.surface.mode & dContactSlip1) != 0)
	    		info.setCfm(1, contact.surface.slip1);
	    } else {
	        // there was no friction for direction 1, so the second friction constraint
	        // has to be on this line instead
	        s2 = s;
	        rowFriction2 = rowFriction1;
	    }

	    final double mu2 = (contact.surface.mode & dContactMu2)!=0 ? contact.surface.mu2 : contact.surface.mu;

		// second friction direction
		if ( mu2 > 0 )
		{
			info._J[info.J1lp+s2+0] = t2.get0();
			info._J[info.J1lp+s2+1] = t2.get1();
			info._J[info.J1lp+s2+2] = t2.get2();
			dCalcVectorCross3( info._J, info.J1ap + s2, c1, t2 );
			
			if ( node[1].body != null)
			{
				info._J[info.J2lp+s2+0] = -t2.get0();
				info._J[info.J2lp+s2+1] = -t2.get1();
				info._J[info.J2lp+s2+2] = -t2.get2();
//				dReal *J2a_plus_s2 = info->J2a + s2;
//				dCalcVectorCross3( J2a_plus_s2, c2, t2 );
//				dNegateVector3( J2a_plus_s2 );
				dCalcVectorCross3( info._J, info.J2ap+s2, c2, t2 );
				dNegateVector3( info._J, info.J2ap+s2 );
			}
			
			// set right hand side
			if (( contact.surface.mode & dContactMotion2) != 0)
			{
				info.setC(rowFriction2, contact.surface.motion2);
			}
			
			// set LCP bounds and friction index. this depends on the approximation
			// mode
	        info.setLo(rowFriction2, -mu2);
	        info.setHi(rowFriction2, mu2);

			if (( contact.surface.mode & dContactApprox1_2) != 0)
				info.setFindex(rowFriction2, 0);//info.findex[2] = 0;

			// set slip (constraint force mixing)
			if (( contact.surface.mode & dContactSlip2) != 0)
				info.setCfm(rowFriction2, contact.surface.slip2);
	        rollRow = rowFriction2+1;
	    } else {
	        rollRow = rowFriction2;
	    } 

	    // Handle rolling/spinning friction
	    if ((contact.surface.mode&OdeConstants.dContactRolling)!=0) {
	        double[] rho = new double[3];
	        DVector3[] ax = new DVector3[3];
	        int[] approx = new int[3];
	  
	        // Get the coefficients
	        rho[0] = contact.surface.rho;
	        if ((contact.surface.mode&OdeConstants.dContactAxisDep)!=0) {
	            rho[1] = contact.surface.rho2;
	            rho[2] = contact.surface.rhoN;
	        } else {
	            rho[1] = rho[0];
	            rho[2] = rho[0];
	        }
	        ax[0] = t1; // Rolling around t1 creates movement parallel to t2
	        ax[1] = t2;
	        ax[2] = normal; // Spinning axis
	        // Should we use proportional force?
	        approx[0] = contact.surface.mode & dContactApprox1_1;
	        approx[1] = contact.surface.mode & dContactApprox1_2;
	        approx[2] = contact.surface.mode & dContactApprox1_N;

	        for (int ii=0;ii<3;++ii) {
	            if (rho[ii]>0) {
	              // Set the angular axis
	              OdeMath.dCopyVector3(info._J, info.J1ap+rollRow*s, ax[ii]);
	              if ( b1!=null ) {
	                OdeMath.dCopyNegatedVector3(info._J, info.J2ap+rollRow*s, ax[ii]);
	              }
	              // Set the lcp limits
	              info.setLo( rollRow, -rho[ii]);
	              info.setHi( rollRow, rho[ii]);
	              // Make limits proportional to normal force
	              if (approx[ii]!=0) info.setFindex( rollRow, 0);
	              rollRow++;
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
}

