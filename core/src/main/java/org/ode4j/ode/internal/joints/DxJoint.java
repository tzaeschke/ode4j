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
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply1_331;
import static org.ode4j.ode.OdeMath.dNegateVector3;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.ode.OdeMath.dPlaneSpace;
import static org.ode4j.ode.OdeMath.dSetCrossMatrixMinus;
import static org.ode4j.ode.OdeMath.dSetCrossMatrixPlus;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.dAtan2;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dSqrt;
import static org.ode4j.ode.internal.Common.dUASSERT;
import static org.ode4j.ode.internal.Rotation.dQMultiply1;
import static org.ode4j.ode.internal.Rotation.dQMultiply2;
import static org.ode4j.ode.internal.Rotation.dQMultiply3;

import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.DObject;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;

/**
 * design note: the general principle for giving a joint the option of connecting
 * to the static environment (i.e. the absolute frame) is to check the second
 * body (joint.node[1].body), and if it is zero then behave as if its body
 * transform is the identity.
 */
public abstract class DxJoint extends DObject implements DJoint, Cloneable {

	// joint flags
	//	enum dJOINT
	//	{
	/** if this flag is set, the joint was allocated in a joint group. */
	private static final int dJOINT_INGROUP = 1;

	/**
	 * if this flag is set, the joint was attached with arguments (0,body).
	 * our convention is to treat all attaches as (body,0), i.e. so node[0].body
	 * is always nonzero, so this flag records the fact that the arguments were
	 * swapped.
	 */
	private static final int dJOINT_REVERSE = 2;

	/** if this flag is set, the joint can not have just one body attached to it,
	 * it must have either zero or two bodies attached. */
	private static final int dJOINT_TWOBODIES = 4;

	private static final int dJOINT_DISABLED = 8;

	//	};


	//	// there are two of these nodes in the joint, one for each connection to a
	//	// body. these are node of a linked list kept by each body of it's connecting
	//	// joints. but note that the body pointer in each node points to the body that
	//	// makes use of the *other* node, not this node. this trick makes it a bit
	//	// easier to traverse the body/joint graph.
	//
	//	public class dxJointNode
	//	{
	//	    dxJoint joint;     // pointer to enclosing dxJoint object
	//	    dxBody body;       // *other* body this joint is connected to
	//	    dxJointNode next;  // next node in body's list of connected joints
	//	};


	//	struct dxJoint : public dObject
	//	{
	// naming convention: the "first" body this is connected to is node[0].body,
	// and the "second" body is node[1].body. if this joint is only connected
	// to one body then the second body is 0.

	/** 
	 * info returned by getInfo1 function. the constraint dimension is m (<=6).
	 * i.e. that is the total number of rows in the jacobian. `nub' is the
	 * number of unbounded variables (which have lo,hi = -/+ infinity).
	 */
	public static final class Info1
	{
        // Structure size should not exceed sizeof(pointer) bytes to have 
        // to have good memory pattern in dxQuickStepper()
		/** constraint dimension, e.g. total number of rows in the jacobian */
		public int m;
		/** number of unbound variables (lo, hi= -/+ infinity) */
		public int nub;
		public int getM() { return m; }
		public void incM() { m++; }
		public void setM(int m) { this.m = m; }
		public int getNub() { return nub; }
		public void setNub(int nub) { this.nub = nub; }
		
		public final void set(Info1 i) {
			this.m = i.m;
			this.nub = i.nub;
		}
	}

	/**
	 *  info returned by getInfo2 function
	 */
	public static class Info2Descr
	{
		// integrator parameters: frames per second (1/stepsize), default error
		// reduction parameter (0..1).
		public double fps;

		public double erp;

		// for the first and second body, pointers to two (linear and angular)
		// n*3 jacobian sub matrices, stored by rows. these matrices will have
		// been initialized to 0 on entry. if the second body is zero then the
		// J2xx pointers may be 0.
		//	        public double[] J1l;
		//
		//			public double[] J1a;
		//
		//			public double[] J2l, J2a;
		public int J1lp;
		public int J1ap;
		public int J2lp, J2ap;

		// elements to jump from one row to the next in J's
		private int _rowskip;
		public int rowskip() { return _rowskip; }
		public void setRowskip(int rs) {_rowskip = rs;} 
		double[] _J;
		public double[] getJ() { return _J; }
		public void setJ(double[] J) {_J = J;} 

		// right hand sides of the equation J*v = c + cfm * lambda. cfm is the
		// "constraint force mixing" vector. c is set to zero on entry, cfm is
		// set to a constant value (typically very small or zero) value on entry.
//		public dVector3 c = new dVector3();//double[] c;  //TZ: [4] ? 0,1,2,3
//		public dVector3 cfm = new dVector3();  //TZ: [3] ?
		protected double[] _cA;
		int _cP;
		protected double[] _cfmA;
		int _cfmP;
		public void setC(int i, double d) {
			_cA[_cP+i] = d;
		}
		public void setC(int i, DVector3C v) {
			_cA[_cP+i] = v.get0();
		}
		public double getC(int i) {
			return _cA[_cP+i];
		}
		public void setCfm(int i, double d) {
			_cfmA[_cfmP+i] = d;
		}

		// lo and hi limits for variables (set to -/+ infinity on entry).
//		public dVector3 lo = new dVector3();//double[] lo;  //TZ: [3] ?
//		public dVector3 hi = new dVector3();//double[] hi;  //TZ: [3] ?
		private double[] _loA;
		int _loP;
		private double[] _hiA;
		int _hiP;
		public void setLo(int i, double d) {
			_loA[_loP+i] = d;
		}
		public void setHi(int i, double d) {
			_hiA[_hiP+i] = d;
		}


		// findex vector for variables. see the LCP solver interface for a
		// description of what this does. this is set to -1 on entry.
		// note that the returned indexes are relative to the first index of
		// the constraint.
		//public int[] _findex = new int[3];  //TZ: [3] ?
		private int _findexP;
		private int[] _findexA;
		public void setFindex(int i, int val) {
			_findexA[_findexP+i] = val;
		}
		public double getFindex(int i) {
			return _findexA[_findexP+i];
		}
		/**
		 * Set POS for c, cfm, lo, hi, findex. 
		 */
		public void setAllP(int i) {
			_cP = i;
			_cfmP = i;
			_loP = i;
			_hiP = i;
			_findexP = i;
		}
		public void setArrays(double[] J, double[] c, double[] cfm,
				double[] lo, double[] hi, int[] findex) {
			_J = J;
			_cA = c;
			_cfmA = cfm;
			_loA = lo;
			_hiA = hi;
			_findexA = findex;
		}
	}

    // info returned by getSureMaxInfo function. 
    // The information is used for memory reservation in calculations.

    static class SureMaxInfo
    {
      // The value of `max_m' must ALWAYS be not less than the value of `m'
      // the getInfo1 call can generate in current joint state. Another 
      // requirement is that the value should be provided very quickly, 
      // without the excessive calculations.
      // If it is hard/impossible to quickly predict the maximal value of `m'
      // (which is the case for most joint types) the maximum for current 
      // joint type in general should be returned. If it can be known the `m'
      // will be smaller, it can save a bit of memory from being reserved 
      // for calculations if that smaller value is returned.

      //int8 max_m; // Estimate of maximal `m' in Info1
        //TODO byte?! (TZ)
        int max_m; // Estimate of maximal `m' in Info1
    };


	public int flags;                  // dJOINT_xxx flags
	public DxJointNode[] node = new DxJointNode[2];        // connections to bodies. node[1].body can be 0
	public DJoint.DJointFeedback feedback;   // optional feedback structure
	//public double[] lambda = new double[6];            // lambda generated by last step
	public final double[] lambda;            // lambda generated by last step

	
    /// Set values which are relative with respect to bodies.
    /// Each dxJoint should redefine it if needed.
    //virtual void setRelativeValues() {};

    public abstract void getInfo1( Info1 info );
    /** 
     * integrator parameters: 
     * fps=frames per second (1/stepsize), 
     * erp=default error reduction parameter (0..1).
	 * (TZ) This method is used solely to extract information required by the stepping algorithms.
	 */
	public abstract void getInfo2( double worldFPS, double worldERP, Info2Descr info );
    // This call quickly!!! estimates maximum value of "m" that could be returned by getInfo1()
    // See comments at definition of SureMaxInfo for defails.
    abstract void getSureMaxInfo( SureMaxInfo info );
//	abstract dJointType type();// const = 0;
//	public UnsupportedOperationException size() {
//		throw new UnsupportedOperationException();}// const = 0;
	//	    abstract size_t size();// const = 0;
	//	};


	//	// joint group. NOTE: any joints in the group that have their world destroyed
	//	// will have their world pointer set to 0.
	//
	//	struct dxJointGroup : public dBase
	//	{
	//	    int num;        // number of joints on the stack
	//	    dObStack stack; // a stack of (possibly differently sized) dxJoint
	//	}                  // objects.
	//

	//	// common limit and motor information for a single joint axis of movement
	//	struct dxJointLimitMotor
	//	{
	//	    dReal vel, fmax;        // powered joint: velocity, max force
	//	    dReal lostop, histop;   // joint limits, relative to initial position
	//	    dReal fudge_factor;     // when powering away from joint limits
	//	    dReal normal_cfm;       // cfm to use when not at a stop
	//	    dReal stop_erp, stop_cfm; // erp and cfm for when at joint limit
	//	    dReal bounce;           // restitution factor
	//	    // variables used between getInfo1() and getInfo2()
	//	    int limit;          // 0=free, 1=at lo limit, 2=at hi limit
	//	    dReal limit_err;    // if at limit, amount over limit
	//
	//	    void init( dxWorld * );
	//	    void set( int num, dReal value );
	//	    dReal get( int num );
	//	    int testRotationalLimit( dReal angle );
	//	    int addLimot( dxJoint *joint, dxJoint::Info2 *info, int row,
	//	                  const dVector3 ax1, int rotational );
	//	};


	protected DxJoint( DxWorld w ) 
	{
		super(w);
		//printf("constructing %p\n", this);
		dIASSERT( w!= null );
		flags = 0;
		node[0] = new DxJointNode();
		node[0].joint = this;
		node[0].body = null;
		node[0].next = null;
		node[1] = new DxJointNode();
		node[1].joint = this;
		node[1].body = null;
		node[1].next = null;
		lambda = new double[6];
//		dSetZero( lambda, 6 );

//		addObjectToList( this, ( dObject ** ) &w.firstjoint );
		addObjectToList( this, w.firstjoint );

		w.nj++;
		feedback = null;
	}

	//dxJoint::~dxJoint()
	@Override
	public void DESTRUCTOR() {
		super.DESTRUCTOR();
	}

	
	/**
	 * Test if this joint should be used in the simulation step
	 * (has the enabled flag set, and is attached to at least one dynamic body).
	 * @return <tt>true</tt> or <tt>false</tt>
	 */
	public boolean isEnabledAndDynamic()
	{
	    return ( (flags & dJOINT_DISABLED) == 0 &&
	             (node[0].body.invMass > 0 ||
	             (node[1].body != null && node[1].body.invMass > 0)) );
	}
	
	//by TZ
	boolean isFlagsReverse() {
		return (flags & dJOINT_REVERSE) != 0;
	}
	
	//by TZ
	public boolean isFlagsInGroup() {
		return (flags & dJOINT_INGROUP) != 0;
	}
	
	void setFlagsInGroup() {
		flags |= DxJoint.dJOINT_INGROUP;
	}
	
	void setFlagsTwoBodies() {
		flags |= DxJoint.dJOINT_TWOBODIES;
	}
	
	//****************************************************************************
	// externs

	// extern "C" void dBodyAddTorque (dBody, dReal fx, dReal fy, dReal fz);
	// extern "C" void dBodyAddForce (dBody, dReal fx, dReal fy, dReal fz);

	//****************************************************************************
	// utility

	// set three "ball-and-socket" rows in the constraint equation, and the
	// corresponding right hand side.

	void setBall( DxJoint joint, double fps, double erp, Info2Descr info,
			DVector3 anchor1, DVector3 anchor2 )
	{
		// anchor points in global coordinates with respect to body PORs.
		DVector3 a1 = new DVector3(), a2 = new DVector3();

		int s = info.rowskip();

		// set jacobian
		info._J[info.J1lp+0] = 1;
		info._J[info.J1lp+s+1] = 1;
		info._J[info.J1lp+2*s+2] = 1;
		dMultiply0_331( a1, joint.node[0].body.posr().R(), anchor1 );
		//    dCROSSMAT( info.J1a, a1, s, -, + );
		dSetCrossMatrixMinus( info._J, info.J1ap, a1, s );
		
		DxBody b1 = joint.node[1].body; 
		if ( b1 != null)
		{
			info._J[info.J2lp+0] = -1;
			info._J[info.J2lp+s+1] = -1;
			info._J[info.J2lp+2*s+2] = -1;
			dMultiply0_331( a2, b1.posr().R(), anchor2 );
			dSetCrossMatrixPlus( info._J, info.J2ap, a2, s );
		}

		
		// set right hand side
		double k = fps * erp;
		DxBody b0 = joint.node[0].body;
		if ( b1 != null)
		{
//			for ( int j = 0; j < 3; j++ )
//			{
//				info.setC(j, k * ( a2.v[j] + b1.posr().pos().v[j] -
//						a1.v[j] - b0._posr.pos.v[j] ));
//			}
			info.setC(0, k * ( a2.get0() + b1.posr().pos().get0() -
					a1.get0() - b0.posr().pos().get0() ));
			info.setC(1, k * ( a2.get1() + b1.posr().pos().get1() -
					a1.get1() - b0.posr().pos().get1() ));
			info.setC(2, k * ( a2.get2() + b1.posr().pos().get2() -
					a1.get2() - b0.posr().pos().get2() ));
		}
		else
		{
//			for ( int j = 0; j < 3; j++ )
//			{
//				info.setC(j, k * ( anchor2.v[j] - a1.v[j] -
//						b0._posr.pos.v[j] ));
//			}
			info.setC(0, k * ( anchor2.get0() - a1.get0() -
					b0.posr().pos().get0() ));
			info.setC(1, k * ( anchor2.get1() - a1.get1() -
					b0.posr().pos().get1() ));
			info.setC(2, k * ( anchor2.get2() - a1.get2() -
					b0.posr().pos().get2() ));
		}
	}


	// this is like setBall(), except that `axis' is a unit length vector
	// (in global coordinates) that should be used for the first jacobian
	// position row (the other two row vectors will be derived from this).
	// `erp1' is the erp value to use along the axis.

	void setBall2( DxJoint joint, double fps, double erp, Info2Descr info,
			DVector3 anchor1, DVector3 anchor2,
			DVector3 axis, double erp1 )
	{
		// anchor points in global coordinates with respect to body PORs.
		DVector3 a1 = new DVector3(), a2 = new DVector3();

		int s = info.rowskip();

		// get vectors normal to the axis. in setBall() axis,q1,q2 is [1 0 0],
		// [0 1 0] and [0 0 1], which makes everything much easier.
		DVector3 q1 = new DVector3(), q2 = new DVector3();
		dPlaneSpace( axis, q1, q2 );

		// set jacobian
//		for ( i = 0; i < 3; i++ ) info._J[info.J1lp+i] = axis.get(i);
//		for ( i = 0; i < 3; i++ ) info._J[info.J1lp+s+i] = q1.get(i);
//		for ( i = 0; i < 3; i++ ) info._J[info.J1lp+2*s+i] = q2.get(i);
		axis.wrapSet( info._J, info.J1lp );
		q1.wrapSet( info._J, info.J1lp+s );
		q2.wrapSet( info._J, info.J1lp+2*s );
		dMultiply0_331( a1, joint.node[0].body.posr().R(), anchor1 );
		dCalcVectorCross3( info._J, info.J1ap, a1, axis );
		dCalcVectorCross3( info._J, info.J1ap + s, a1, q1 );
		dCalcVectorCross3( info._J, info.J1ap + 2*s, a1, q2 );
		
		DxBody b1 = joint.node[1].body;
		if ( b1 != null)
		{
//			for ( i = 0; i < 3; i++ ) info._J[info.J2lp+i] = -axis.v[i];
//			for ( i = 0; i < 3; i++ ) info._J[info.J2lp+s+i] = -q1.v[i];
//			for ( i = 0; i < 3; i++ ) info._J[info.J2lp+2*s+i] = -q2.v[i];
			axis.wrapSub( info._J, info.J2lp );
			q1.wrapSub( info._J, info.J2lp+s );
			q2.wrapSub( info._J, info.J2lp+2*s );
//	        dMultiply0_331( a2, joint->node[1].body->posr.R, anchor2 );
//	        dReal *J2a = info->J2a;
//	        dCalcVectorCross3( J2a, a2, axis );
//	        dNegateVector3( J2a );
//	        dReal *J2a_plus_s = J2a + s;
//	        dCalcVectorCross3( J2a_plus_s, a2, q1 );
//	        dNegateVector3( J2a_plus_s );
//	        dReal *J2a_plus_2s = J2a_plus_s + s;
//	        dCalcVectorCross3( J2a_plus_2s, a2, q2 );
//	        dNegateVector3( J2a_plus_2s );
	        dMultiply0_331( a2, b1._posr.R, anchor2 );
	        double[] J = info._J;  //TZ
	        int J2ap = info.J2ap; //TZ
	        //dReal *J2a = info->J2a;
	        dCalcVectorCross3( J, J2ap, a2, axis );
	        dNegateVector3( J, J2ap );
	        //dReal *J2a_plus_s = J2a + s;
	        dCalcVectorCross3( J, J2ap+s, a2, q1 );
	        dNegateVector3( J, J2ap+s );
	        //dReal *J2a_plus_2s = J2a_plus_s + s;
	        dCalcVectorCross3( J, J2ap+2*s, a2, q2 );
	        dNegateVector3( J, J2ap+2*s );
		}

		// set right hand side - measure error along (axis,q1,q2)
		double k1 = fps * erp1;
		double k = fps * erp;

		//for ( i = 0; i < 3; i++ ) a1.v[i] += joint.node[0].body._posr.pos.v[i];
		DxBody b0 = joint.node[0].body;
		a1.add(b0.posr().pos());
		if ( b1 != null)
		{
//			for ( i = 0; i < 3; i++ ) a2.v[i] += joint.node[1].body._posr.pos.v[i];
			a2.add(b1.posr().pos());
			DVector3 a2_minus_a1 = new DVector3();
			a2_minus_a1.eqDiff(a2, a1);
			info.setC(0, k1 * ( dCalcVectorDot3( axis, a2_minus_a1 ) ));
			info.setC(1, k * ( dCalcVectorDot3( q1, a2_minus_a1 ) ));
			info.setC(2, k * ( dCalcVectorDot3( q2, a2_minus_a1 ) ));
		}
		else
		{
            DVector3 anchor2_minus_a1 = new DVector3();
            anchor2_minus_a1.eqDiff(anchor2, a1);
			info.setC(0, k1 * ( dCalcVectorDot3( axis, anchor2_minus_a1 ) ));
			info.setC(1, k * ( dCalcVectorDot3( q1, anchor2_minus_a1 ) ));
			info.setC(2, k * ( dCalcVectorDot3( q2, anchor2_minus_a1 ) ));
		}
	}


	/**
	 * set three orientation rows in the constraint equation, and the
	 * corresponding right hand side.
	 * @param joint
	 * @param info
	 * @param qrel
	 * @param start_row
	 */
	void setFixedOrientation( DxJoint joint, double fps, double erp, Info2Descr info, 
			DQuaternion qrel, int start_row )
	{
		int s = info.rowskip();
		int start_index = start_row * s;
		// 3 rows to make body rotations equal
		info._J[info.J1ap+start_index] = 1;
		info._J[info.J1ap+start_index + s + 1] = 1;
		info._J[info.J1ap+start_index + s*2+2] = 1;

		DxBody b1 = joint.node[1].body;
		if ( b1 != null)
		{
			info._J[info.J2ap+start_index] = -1;
			info._J[info.J2ap+start_index + s+1] = -1;
			info._J[info.J2ap+start_index + s*2+2] = -1;
		}

		// compute the right hand side. the first three elements will result in
		// relative angular velocity of the two bodies - this is set to bring them
		// back into alignment. the correcting angular velocity is
		//   |angular_velocity| = angle/time = erp*theta / stepsize
		//                      = (erp*fps) * theta
		//    angular_velocity  = |angular_velocity| * u
		//                      = (erp*fps) * theta * u
		// where rotation along unit length axis u by theta brings body 2's frame
		// to qrel with respect to body 1's frame. using a small angle approximation
		// for sin(), this gives
		//    angular_velocity  = (erp*fps) * 2 * v
		// where the quaternion of the relative rotation between the two bodies is
		//    q = [cos(theta/2) sin(theta/2)*u] = [s v]

		// get qerr = relative rotation (rotation error) between two bodies
		DQuaternion qerr = new DQuaternion();
		DVector3 e = new DVector3();
		DxBody b0 = joint.node[0].body;
		if ( b1 != null)
		{
			DQuaternion qq = new DQuaternion();
			dQMultiply1( qq, b0._q, b1._q );
			dQMultiply2( qerr, qq, qrel );
		}
		else
		{
			dQMultiply3( qerr, b0._q, qrel );
		}
		if ( qerr.get0() < 0)
		{
			qerr.set1( -qerr.get1() );  // adjust sign of qerr to make theta small
			qerr.set2( -qerr.get2() );
			qerr.set3( -qerr.get3() );
		}
		//TZ:
//		dMULTIPLY0_331( e, joint.node[0].body.posr.R, qerr + 1 );  // @@@ bad SIMD padding!
		DVector3 qerr2 = new DVector3();
		qerr2.set0( qerr.get1() );
		qerr2.set1( qerr.get2() );
		qerr2.set2( qerr.get3() );
		dMultiply0_331( e, b0.posr().R(), qerr2 );  // @@@ bad SIMD padding!
		double k = fps * erp;
		info.setC(start_row, 2 * k * e.get0() );
		info.setC(start_row+1, 2 * k * e.get1() );
		info.setC(start_row+2, 2 * k * e.get2() );
	}


	// compute anchor points relative to bodies

//	void setAnchors( dxJoint j, double x, double y, double z,
//	dVector3 anchor1, dVector3 anchor2 )
	final void setAnchors( DVector3C xyz,
			DVector3 anchor1, DVector3 anchor2 )
	{
		DxBody b0 = node[0].body;
		if ( b0 != null)
		{
			/// double[] q = new double[4];
			DVector3 q = new DVector3();
//			q.v[0] = x - node[0].body._posr.pos.v[0];
//			q.v[1] = y - node[0].body._posr.pos.v[1];
//			q.v[2] = z - node[0].body._posr.pos.v[2];
//			q.v[3] = 0;
			q.eqDiff(xyz, b0.posr().pos());
			dMultiply1_331( anchor1, b0.posr().R(), q );
			DxBody b1 = node[1].body;
			if ( b1 != null )
			{
//				q.v[0] = x - node[1].body._posr.pos.v[0];
//				q.v[1] = y - node[1].body._posr.pos.v[1];
//				q.v[2] = z - node[1].body._posr.pos.v[2];
//				q.v[3] = 0;
				q.eqDiff(xyz, b1.posr().pos());
				dMultiply1_331( anchor2, b1.posr().R(), q );
			}
			else
			{
//				anchor2.v[0] = x;
//				anchor2.v[1] = y;
//				anchor2.v[2] = z;
				anchor2.set(xyz);
			}
		}
//		anchor1.v[3] = 0;
//		anchor2.v[3] = 0;
	}


	/** 
	 * compute axes relative to bodies. either axis1 or axis2 can be 0.
	 */

//	void setAxes( dxJoint j, double x, double y, double z,
	void setAxes( double x, double y, double z,
			DVector3 axis1, DVector3 axis2 )
	{
		DxBody b0 = node[0].body;
		if ( b0 != null)
		{
			//        double[] q = new double[4];
			DVector3 q = new DVector3(x, y, z);
			dNormalize3( q );
			if ( axis1 != null)
			{
				dMultiply1_331( axis1, b0.posr().R(), q );
//				axis1.v[3] = 0;
			}
			if ( axis2 != null)
			{
				DxBody b1 = node[1].body;
				if ( b1 != null)
				{
					dMultiply1_331( axis2, b1.posr().R(), q );
				}
				else
				{
//					axis2.v[0] = x;
//					axis2.v[1] = y;
//					axis2.v[2] = z;
					axis2.set(x, y, z);
				}
//				axis2.v[3] = 0;
			}
		}
	}
	void setAxes( DVector3C axis, DVector3 axis1, DVector3 axis2 )
	{
		DxBody b0 = node[0].body;
		if ( b0 != null)
		{
			DVector3 q = new DVector3(axis);
			dNormalize3( q );
			if ( axis1 != null)
			{
				dMultiply1_331( axis1, b0.posr().R(), q );
			}
			if ( axis2 != null)
			{
				DxBody b1 = node[1].body;
				if ( b1 != null)
				{
					dMultiply1_331( axis2, b1.posr().R(), q );
				}
				else
				{
					axis2.set( axis );
				}
			}
		}
	}


//	void getAnchor( dxJoint j, dVector3 result, dVector3 anchor1 )
	void getAnchor( DVector3 result, DVector3 anchor1 )
	{
		DxBody b0 = node[0].body;
		if ( b0 != null)
		{
			dMultiply0_331( result, b0.posr().R(), anchor1 );
//			result.v[0] += node[0].body._posr.pos.v[0];
//			result.v[1] += node[0].body._posr.pos.v[1];
//			result.v[2] += node[0].body._posr.pos.v[2];
			result.add(b0.posr().pos());
		}
	}


//	void getAnchor2( dxJoint j, dVector3 result, dVector3 anchor2 )
	void getAnchor2( DVector3 result, DVector3 anchor2 )
	{
		DxBody b1 = node[1].body;
		if ( b1 != null)
		{
			dMultiply0_331( result, b1.posr().R(), anchor2 );
//			result.v[0] += node[1].body.posr.pos.v[0];
//			result.v[1] += node[1].body.posr.pos.v[1];
//			result.v[2] += node[1].body.posr.pos.v[2];
			result.add(b1.posr().pos());
		}
		else
		{
//			result.v[0] = anchor2.v[0];
//			result.v[1] = anchor2.v[1];
//			result.v[2] = anchor2.v[2];
			result.set(anchor2);
		}
	}


	void getAxis( DVector3 result, DVector3C axis1 )
	{
		DxBody b0 = node[0].body;
		if ( b0 != null)
		{
			dMultiply0_331( result, b0.posr().R(), axis1 );
		}
	}


	void getAxis2( DVector3 result, DVector3C axis2 )
	{
		DxBody b1 = node[1].body;
		if ( b1 != null )
		{
			dMultiply0_331( result, b1.posr().R(), axis2 );
		}
		else
		{
//			result.v[0] = axis2.v[0];
//			result.v[1] = axis2.v[1];
//			result.v[2] = axis2.v[2];
			result.set(axis2);
		}
	}


	double getHingeAngleFromRelativeQuat( DQuaternionC qrel, DVector3C axis )
	{
		// the angle between the two bodies is extracted from the quaternion that
		// represents the relative rotation between them. recall that a quaternion
		// q is:
		//    [s,v] = [ cos(theta/2) , sin(theta/2) * u ]
		// where s is a scalar and v is a 3-vector. u is a unit length axis and
		// theta is a rotation along that axis. we can get theta/2 by:
		//    theta/2 = atan2 ( sin(theta/2) , cos(theta/2) )
		// but we can't get sin(theta/2) directly, only its absolute value, i.e.:
		//    |v| = |sin(theta/2)| * |u|
		//        = |sin(theta/2)|
		// using this value will have a strange effect. recall that there are two
		// quaternion representations of a given rotation, q and -q. typically as
		// a body rotates along the axis it will go through a complete cycle using
		// one representation and then the next cycle will use the other
		// representation. this corresponds to u pointing in the direction of the
		// hinge axis and then in the opposite direction. the result is that theta
		// will appear to go "backwards" every other cycle. here is a fix: if u
		// points "away" from the direction of the hinge (motor) axis (i.e. more
		// than 90 degrees) then use -q instead of q. this represents the same
		// rotation, but results in the cos(theta/2) value being sign inverted.

		// extract the angle from the quaternion. cost2 = cos(theta/2),
		// sint2 = |sin(theta/2)|
		double cost2 = qrel.get0();
		double sint2 = dSqrt( qrel.get1() * qrel.get1() + qrel.get2() * qrel.get2() + 
				qrel.get3() * qrel.get3() );
		//double theta = ( dDOT( qrel.v, 1, axis.v, 0 ) >= 0 ) ? // @@@ padding assumptions
		double theta = ( (qrel.get1()*axis.get0() + qrel.get2()*axis.get1() + qrel.get3()*axis.get2() ) >= 0 ) ? // @@@ padding assumptions
				( 2 * dAtan2( sint2, cost2 ) ) :  // if u points in direction of axis
					( 2 * dAtan2( sint2, -cost2 ) );  // if u points in opposite direction

				// the angle we get will be between 0..2*pi, but we want to return angles
				// between -pi..pi
				if ( theta > M_PI ) theta -= 2 * M_PI;

				// the angle we've just extracted has the wrong sign
				theta = -theta;

				return theta;
	}


	// given two bodies (body1,body2), the hinge axis that they are connected by
	// w.r.t. body1 (axis), and the initial relative orientation between them
	// (q_initial), return the relative rotation angle. the initial relative
	// orientation corresponds to an angle of zero. if body2 is 0 then measure the
	// angle between body1 and the static frame.
	//
	// this will not return the correct angle if the bodies rotate along any axis
	// other than the given hinge axis.

	double getHingeAngle( DxBody body1, DxBody body2, DVector3 axis,
			DQuaternion q_initial )
	{
		// get qrel = relative rotation between the two bodies
		DQuaternion qrel = new DQuaternion();
		if ( body2 != null )
		{
			DQuaternion qq = new DQuaternion();
			dQMultiply1( qq, body1._q, body2._q );
			dQMultiply2( qrel, qq, q_initial );
		}
		else
		{
			// pretend body2.q is the identity
			dQMultiply3( qrel, body1._q, q_initial );
		}

		return getHingeAngleFromRelativeQuat( qrel, axis );
	}

	// ************** From ODE.java

//	int dJointGetNumBodies(dxJoint joint)
	int dJointGetNumBodies()
	{
		// check arguments
//		dUASSERT (joint,"bad joint argument");

		if ( node[0].body == null)
			return 0;
		else if ( node[1].body  == null)
			return 1;
		else
			return 2;
	}


	public void dJointAttach (DBody b1, DBody b2)
	{
		DxBody body1 = (DxBody) b1;
		DxBody body2 = (DxBody) b2;
		// check arguments
		//dUASSERT (joint,"bad joint argument");
		dUASSERT (body1 == null || body1 != body2,"can't have body1==body2");
		DxWorld world = this.world;
		dUASSERT ( (body1==null || body1.getWorld() == world) &&
				(body2==null || body2.getWorld() == world),
				"joint and bodies must be in same world");

		// check if the joint can not be attached to just one body
		dUASSERT (!((flags & dJOINT_TWOBODIES) != 0 &&
				((body1 != null) ^ (body2 != null))),
				"joint can not be attached to just one body");

		// remove any existing body attachments
		if (node[0].body!=null || node[1].body!=null) {
			removeJointReferencesFromAttachedBodies ();
		}

		// if a body is zero, make sure that it is body2, so 0 --> node[1].body
		if (body1==null) {
			body1 = body2;
			body2 = null;
			flags |= dJOINT_REVERSE;
		}
		else {
			flags &= (~dJOINT_REVERSE);
		}

		// attach to new bodies
		node[0].body = body1;
		node[1].body = body2;
		if (body1 != null) {
			node[1].next = body1.firstjoint.get();
			body1.firstjoint.set(node[1]);
		}
		else node[1].next = null;
		if (body2 != null) {
			node[0].next = body2.firstjoint.get();
			body2.firstjoint.set(node[0]);
		}
		else {
			node[0].next = null;
		}

		// Since the bodies are now set.
		// Calculate the values depending on the bodies.
		// Only need to calculate relative value if a body exist
		if (body1 != null || body2 != null)
			setRelativeValues();
	}

	/** Set values which are relative with respect to bodies.
	 * Each dxJoint should redefine it if needed.
	 */
	void setRelativeValues() {
		// TODO Auto-generated method stub
	}
	
	//void dJointEnable ()
	@Override
	public void enable()
	{
		flags &= ~dJOINT_DISABLED;
	}

	//void dJointDisable ()
	@Override
	public void disable()
	{
		flags |= dJOINT_DISABLED;
	}

//	boolean dJointIsEnabled ()
	@Override
	public boolean isEnabled()
	{
	  return (flags & dJOINT_DISABLED) == 0;
	}

	// remove the joint from neighbour lists of all connected bodies

//	public static void removeJointReferencesFromAttachedBodies (dxJoint j)
	public void removeJointReferencesFromAttachedBodies ()
	{
		for (int i=0; i<2; i++) {
			DxBody body = node[i].body;
			if (body != null) {
				DxJointNode n = body.firstjoint.get();
				DxJointNode last = null;
				while (n != null) {
					//TODO optimize this?? 
					if (n.joint == this) {
						if (last != null) last.next = n.next;
						else body.firstjoint.set(n.next);
						break;
					}
					last = n;
					n = n.next;
				}
			}
		}
		node[0].body = null;
		node[0].next = null;
		node[1].body = null;
		node[1].next = null;
	}

	/** deprecated */
	public void dJointSetData (Object data)
	{
		userdata = data;
	}


	/** deprecated */
	public Object dJointGetData ()
	{
		return userdata;
	}

//	dxBody dJointGetBody (dxJoint joint, int index)
	public DxBody dJointGetBody (int index)
	{
		//dAASSERT (joint);
		if (index == 0 || index == 1) {
			if ( isFlagsReverse() ) return node[1-index].body;
			else return node[index].body;
		}
		else return null;
	}


	//void dJointSetFeedback (dxJoint joint, dJointFeedback f)
	public void dJointSetFeedback (DJoint.DJointFeedback f)
	{
		//dAASSERT (joint);
		feedback = f;
	}


//	dJointFeedback dJointGetFeedback (dxJoint joint)
	DJoint.DJointFeedback dJointGetFeedback ()
	{
		//dAASSERT (joint);
		return feedback;
	}


	//by TZ
	@Override
	public Object clone() {
		try {
			return super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
	}
	
	// *************************************
	// dJoint API
	// *************************************
	
	public void destroy() {
		//Nothing to do at the moment
	}
	
	@Override
	public final int getNumBodies()
	{ return dJointGetNumBodies(); }

	@Override
	public final void attach (DBody body1, DBody body2)
	{ dJointAttach (body1, body2); }

	@Override
	public final void setData (Object data)
	{ dJointSetData (data); }
	@Override
	public Object getData()
	{ return dJointGetData (); }

	@Override
	public final DBody getBody (int index)
	{ return dJointGetBody (index); }

	@Override
	public final void setFeedback(DJoint.DJointFeedback fb)
	{ dJointSetFeedback(fb); }
	@Override
	public final DJoint.DJointFeedback getFeedback()
	{ return dJointGetFeedback(); }

	// If not implemented it will do nothing as describe in the doc
	@Override
	public abstract void setParam (PARAM_N parameter, double value);
	@Override
	public abstract double getParam (PARAM_N parameter);
}
