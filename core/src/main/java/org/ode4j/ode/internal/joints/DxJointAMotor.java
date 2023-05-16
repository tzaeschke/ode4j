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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.ou.CEnumUnsortedElementArray;

import java.util.Arrays;

import static org.ode4j.ode.DAMotorJoint.AMotorMode.dAMotorEuler;
import static org.ode4j.ode.DAMotorJoint.AMotorMode.dAMotorUser;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.CommonEnums.*;
import static org.ode4j.ode.internal.joints.JointEnums.*;


/**
 * ***************************************************************************
 * angular motor
 * 
 */
public class DxJointAMotor extends DxJoint implements DAMotorJoint
{

	//	public:
	//	dxJointAMotor(dxWorld *w);
	//	virtual ~dxJointAMotor();
	//
	//	public:
	//	virtual void getSureMaxInfo(SureMaxInfo* info);
	//	virtual void getInfo1(Info1* info);
	//	virtual void getInfo2(dReal worldFPS, dReal worldERP,
	//						  int rowskip, dReal *J1, dReal *J2,
	//						  int pairskip, dReal *pairRhsCfm, dReal *pairLoHi,
	//						  int *findex);
	//	virtual dJointType type() const;
	//	virtual size_t size() const;
	//
	//	public:
	//	void setOperationMode(int mode);
	public AMotorMode getOperationMode() { return m_mode; }

	// void setNumAxes(unsigned num);
	private int _getNumAxes() { return m_num; }

	// dJointBodyRelativity getAxisBodyRelativity(unsigned anum) const;
	//
	//	void setAxisValue(unsigned anum, dJointBodyRelativity rel, dReal x, dReal y, dReal z);
	//	void getAxisValue(dVector3 result, unsigned anum) const;
	//
	//	private:
	//	void doGetUserAxis(dVector3 result, unsigned anum) const;
	//	void doGetEulerAxis(dVector3 result, unsigned anum) const;
	//
	//	public:
	//	void setAngleValue(unsigned anum, dReal angle);
	public double getAngleValue(int anum) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		return m_angle[anum];
	}

	// dReal calculateAngleRate(unsigned anum) const;

	void setLimotParameter(int anum, PARAM limotParam, double value) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		m_limot[anum].set(limotParam, value);
	}

	double getLimotParameter(int anum, PARAM limotParam) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		return m_limot[anum].get(limotParam);
	}

	//	public:
	//	void addTorques(dReal torque1, dReal torque2, dReal torque3);
	//
	//	private:
	//	void computeGlobalAxes(dVector3 ax[dSA__MAX]) const;
	//	void doComputeGlobalUserAxes(dVector3 ax[dSA__MAX]) const;
	//	void doComputeGlobalEulerAxes(dVector3 ax[dSA__MAX]) const;
	//
	//	void computeEulerAngles(dVector3 ax[dSA__MAX]);
	//	void setEulerReferenceVectors();
	//
	//	private:
	//	inline dSpaceAxis BuildFirstBodyEulerAxis() const;
	//	inline dJointConnectedBody BuildFirstEulerAxisBody() const;
	//
	//	private:
	// friend struct dxAMotorJointPrinter;
	//
	// private:

	// a dAMotorXXX constant
	private AMotorMode m_mode;
	// number of axes (0..3)
	private int m_num;
	// what the axes are relative to (global,b1,b2)
	private final int[] m_rel = new int[dSA__MAX];
	// three axes
	private final DVector3[] m_axis = DVector3.newArray(dSA__MAX);

	// these vectors are used for calculating Euler angles
	// original axis[2], relative to body 1; original axis[0], relative to body 2
	private final DVector3[] m_references = DVector3.newArray(dJCB__MAX);
	// user-supplied angles for axes
	private final double[] m_angle = new double[dSA__MAX];
	// limit+motor info for axes
	private final DxJointLimitMotor[] m_limot = new DxJointLimitMotor[dJBR__MAX];


	/*extern */
	//void dJointSetAMotorNumAxes(dJointID j, int num)
	void dJointSetAMotorNumAxes(int num) {
		dAASSERT(dIN_RANGE(num, dSA__MIN, dSA__MAX + 1));

		num = dCLAMP(num, dSA__MIN, dSA__MAX);

		_setNumAxes(num);
	}


	/*extern */
	//	void dJointSetAMotorAxis(dJointID j, int anum, int rel/*=dJointBodyRelativity*/,
	//							 dReal x, dReal y, dReal z)
	void dJointSetAMotorAxis(int anum, int rel/*=dJointBodyRelativity*/, double x, double y, double z) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		dAASSERT(dIN_RANGE(rel, dJBR__MIN, dJBR__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		setAxisValue(anum, rel, x, y, z);
	}

	void dJointSetAMotorAxis(int anum, int rel/*=dJointBodyRelativity*/, DVector3C xyz) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		dAASSERT(dIN_RANGE(rel, dJBR__MIN, dJBR__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		setAxisValue(anum, rel, xyz);
	}

	/*extern */
	void dJointSetAMotorAngle(int anum, double angle) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		setAngleValue(anum, angle);
	}

	/*extern */
	void dJointSetAMotorParam(PARAM_N parameter, double value) {
		//int anum = parameter >> 8;
		int anum = parameter.toGROUP().getIndex();
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		//int limotParam = parameter & 0xff;
		PARAM limotParam = parameter.toSUB();
		setLimotParameter(anum, limotParam, value);
	}

	/*extern */
	void dJointSetAMotorMode(AMotorMode mode) {
		setOperationMode(mode);
	}

	/*extern */
	int dJointGetAMotorNumAxes() {
		return _getNumAxes();
	}

	/*extern */
	void dJointGetAMotorAxis(int anum, DVector3 result) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		getAxisValue(result, anum);
	}

	/*extern */
	int dJointGetAMotorAxisRel(int anum) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		return getAxisBodyRelativity(anum);
	}

	/*extern */
	double dJointGetAMotorAngle(int anum) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		return getAngleValue(anum);
	}

	/*extern */
	double dJointGetAMotorAngleRate(int anum) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		return calculateAngleRate(anum);
	}

	/*extern */
	double dJointGetAMotorParam(PARAM_N parameter) {
		int anum = parameter.toGROUP().getIndex();// >> 8;
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		anum = dCLAMP(anum, dSA__MIN, dSA__MAX - 1);

		//int limotParam = parameter & 0xff;
		PARAM limotParam = parameter.toSUB();
		return getLimotParameter(anum, limotParam);
	}

	/*extern */
	AMotorMode dJointGetAMotorMode() {
		return getOperationMode();
	}

	/*extern */
	void dJointAddAMotorTorques(double torque1, double torque2, double torque3) {
		_addTorques(torque1, torque2, torque3);
	}


	//****************************************************************************

	// BEGIN_NAMESPACE_OU();
	//EnumType=dSpaceAxis, EnumType.EnumMax = dSA__MAX, ElementType=dJointBodyRelativity, Instance=0x160703D5
		//const dJointBodyRelativity CEnumUnsortedElementArray<dSpaceAxis, dSA__MAX, dJointBodyRelativity, 0x160703D5>::m_aetElementArray[] =
	private static final CEnumUnsortedElementArray g_abrEulerAxisAllowedBodyRelativities =
			new CEnumUnsortedElementArray(new int[]{dJBR_BODY1, // dSA_X,
			dJBR_GLOBAL, // dSA_Y,
			dJBR_BODY2 // dSA_Z,
	}, dSA__MAX);
	//	dJointBodyRelativity CEnumUnsortedElementArray<dSpaceAxis, dSA__MAX, dJointBodyRelativity, 0x160703D5>::m_aetElementArray[] =
	//	{
	//		dJBR_BODY1, // dSA_X,
	//				dJBR_GLOBAL, // dSA_Y,
	//				dJBR_BODY2, // dSA_Z,
	//	};
	//END_NAMESPACE_OU();
	//  static CEnumUnsortedElementArray<dSpaceAxis, dSA__MAX, dJointBodyRelativity, 0x160703D5> g_abrEulerAxisAllowedBodyRelativities;

	static
	// dSpaceAxis EncodeJointConnectedBodyEulerAxis(dJointConnectedBody cbBodyIndex)
	int EncodeJointConnectedBodyEulerAxis(int cbBodyIndex)
	{
		dSASSERT(dJCB__MAX == 2);

		return cbBodyIndex == dJCB_FIRST_BODY ? dSA_X : dSA_Z;
	}

	static
	// dSpaceAxis EncodeOtherEulerAxis(dSpaceAxis saOneAxis)
	int EncodeOtherEulerAxis(int saOneAxis)
	{
		dIASSERT(saOneAxis == EncodeJointConnectedBodyEulerAxis(dJCB_FIRST_BODY) || saOneAxis == EncodeJointConnectedBodyEulerAxis(dJCB_SECOND_BODY));
		dSASSERT(dJCB__MAX == 2);

		return dSA_X + dSA_Z - saOneAxis;
	}


	//****************************************************************************
	// angular motor

	DxJointAMotor(DxWorld w) {
		super(w);
		m_mode = dAMotorUser;
		m_num = 0;

		//std::fill(m_rel, m_rel + dARRAY_SIZE(m_rel), dJBR__DEFAULT);
		Arrays.fill(m_rel, dJBR_GLOBAL);
		//{ for (int i = 0; i != dARRAY_SIZE(m_axis); ++i) { dZeroVector3(m_axis[i]); } }
		//for (int i = 0; i != m_axis.length; ++i) { m_axis[i].setZero(); }

		//{ for (int i = 0; i != dARRAY_SIZE(m_references); ++i) { dZeroVector3(m_references[i]); } }
		//for (int i = 0; i != m_references.length; ++i) { m_references[i].setZero(); }

		//std::fill(m_angle, m_angle + dARRAY_SIZE(m_angle), REAL(0.0));
		//Arrays.fill(m_angle, 0); This is done implicitly

		//{ for (int i = 0; i != dARRAY_SIZE(m_limot); ++i) { m_limot[i].init(w); } }
		for (int i = 0; i != m_limot.length; ++i) {
			m_limot[i] = new DxJointLimitMotor();
			m_limot[i].init(w);
		}
	}


	/*virtual */
	//dxJointAMotor::~dxJointAMotor()
	public void DESTRUCTOR() {
		// The virtual destructor
	}

	/*virtual */
	void getSureMaxInfo(SureMaxInfo info) {
		info.max_m = m_num;
	}

	/*virtual */
	@Override
	public void getInfo1(DxJoint.Info1 info) {
		info.m = 0;
		info.nub = 0;

		// compute the axes and angles, if in Euler mode
		if (m_mode == dAMotorEuler) {
			DVector3[] ax = DVector3.newArray(dSA__MAX);
			computeGlobalAxes(ax);
			computeEulerAngles(ax);
		}

		// see if we're powered or at a joint limit for each axis
		final int num = m_num;
		for (int i = 0; i != num; ++i) {
			if (m_limot[i].testRotationalLimit(m_angle[i]) || m_limot[i].fmax > 0) {
				info.m++;
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
	// compute the axes (if not global)
	DVector3[] ax = DVector3.newArray(dSA__MAX);
	computeGlobalAxes(ax);

	// in Euler angle mode we do not actually constrain the angular velocity
	// along the axes axis[0] and axis[2] (although we do use axis[1]) :
	//
	//    to get   constrain w2-w1 along  ...not
	//    ------   ---------------------  ------
	//    d(angle[0])/dt = 0 ax[1] x ax[2]   ax[0]
	//    d(angle[1])/dt = 0 ax[1]
	//    d(angle[2])/dt = 0 ax[0] x ax[1]   ax[2]
	//
	// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
	// to prove the result for angle[0], write the expression for angle[0] from
	// GetInfo1 then take the derivative. to prove this for angle[2] it is
	// easier to take the Euler rate expression for d(angle[2])/dt with respect
	// to the components of w and set that to 0.

		DVector3[] axptr = DVector3.newArray( dSA__MAX);
		for (int j = dSA__MIN; j != dSA__MAX; ++j) {
			axptr[j] = ax[j];//&ax[j]; Copy pointer instead of vector
		}

		DVector3 ax0_cross_ax1 = new DVector3();
		DVector3 ax1_cross_ax2 = new DVector3();

		if (m_mode == dAMotorEuler) {
			dCalcVectorCross3(ax0_cross_ax1, ax[dSA_X], ax[dSA_Y]);
			axptr[dSA_Z] = ax0_cross_ax1;//&ax0_cross_ax1; Copy pointer instead of vector
			dCalcVectorCross3(ax1_cross_ax2, ax[dSA_Y], ax[dSA_Z]);
			axptr[dSA_X] = ax0_cross_ax1;//&ax1_cross_ax2; Copy pointer instead of vector
		}

		int rowTotalSkip = 0, pairTotalSkip = 0;

		final int num = m_num;
		for (int i = 0; i != num; ++i) {
			if (m_limot[i].addLimot(this, worldFPS, J1A, J1Ofs + rowTotalSkip, J2A, J2Ofs + rowTotalSkip, pairRhsCfmA,
					pairRhsCfmOfs + pairTotalSkip, pairLoHiA, pairLoHiOfs + pairTotalSkip, axptr[i], true))
			{
				rowTotalSkip += rowskip;
				pairTotalSkip += pairskip;
			}
		}
	}

	//	/*virtual */
	//	dJointType type() {
	//		return dJointTypeAMotor;
	//	}

	//	/*virtual */
	//	int size() {
	//		return -1;
	//	}

	void setOperationMode(AMotorMode mode) {
		m_mode = mode;

		if (mode == dAMotorEuler) {
			m_num = dSA__MAX;
			setEulerReferenceVectors();
		}
	}


	void _setNumAxes(int num) {
		if (m_mode == dAMotorEuler) {
			m_num = dSA__MAX;
		} else {
			m_num = num;
		}
	}


	int getAxisBodyRelativity(int anum) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		int rel = m_rel[anum];
		if (dJBREncodeBodyRelativityStatus(rel) && GetIsJointReverse()) {
			rel = dJBRSwapBodyRelativity(rel); // turns 1 into 2, 2 into 1
		}

		return rel;
	}


	void setAxisValue(int anum, int rel, double x, double y, double z) {
		setAxisValue(anum, rel, new DVector3(x, y, z));
	}


	void setAxisValue(int anum, int rel, DVector3C xyz) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		dAASSERT(m_mode != dAMotorEuler || !dJBREncodeBodyRelativityStatus(rel) || rel == g_abrEulerAxisAllowedBodyRelativities.Encode(anum));

		// x,y,z is always in global coordinates regardless of rel, so we may have
		// to convert it to be relative to a body
		DVector3C r = xyz;

		// adjust rel to match the internal body order
		if (dJBREncodeBodyRelativityStatus(rel) && GetIsJointReverse()) {
			rel = dJBRSwapBodyRelativity(rel); // turns 1 into 2, 2, into 1
		}

		m_rel[anum] = rel;

		boolean assigned = false;

		if (dJBREncodeBodyRelativityStatus(rel)) {
			if (rel == dJBR_BODY1) {
				dMultiply1_331(m_axis[anum], this.node[0].body.posr().R(), r);
				assigned = true;
			}
			// rel == 2
			else if (this.node[1].body != null) {
				dIASSERT(rel == dJBR_BODY2);

				dMultiply1_331(m_axis[anum], this.node[1].body.posr().R(), r);
				assigned = true;
			}
		}

		if (!assigned) {
			m_axis[anum].set(r);
		}

		dNormalize3(m_axis[anum]);

		if (m_mode == dAMotorEuler) {
			setEulerReferenceVectors();
		}
	}

	void getAxisValue(DVector3 result, int anum) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));

		switch (m_mode) {
			case dAMotorUser: {
				doGetUserAxis(result, anum);
				break;
			}

			case dAMotorEuler: {
				doGetEulerAxis(result, anum);
				break;
			}

			default: {
				dIASSERT(false);
				break;
			}
		}
	}


	void doGetUserAxis(DVector3 result, int anum) {
		boolean retrieved = false;

		if (dJBREncodeBodyRelativityStatus(m_rel[anum])) {
			if (m_rel[anum] == dJBR_BODY1) {
				dMultiply0_331(result, this.node[0].body.posr().R(), m_axis[anum]);
				retrieved = true;
			} else if (this.node[1].body != null) {
				dMultiply0_331(result, this.node[1].body.posr().R(), m_axis[anum]);
				retrieved = true;
			}
		}

		if (!retrieved) {
			result.set(m_axis[anum]);
		}
	}

	void doGetEulerAxis(DVector3 result, int anum) {
		// If we're in Euler mode, joint->axis[1] doesn't
		// have anything sensible in it.  So don't just return
		// that, find the actual effective axis.
		// Likewise, the actual axis of rotation for the
		// the other axes is different from what's stored.
		DVector3[] axes = DVector3.newArray( dSA__MAX);
		computeGlobalAxes(axes);

		if (anum == dSA_Y) {
			//dCopyVector3(result, axes[dSA_Y]);
			result.set(axes[dSA_Y]);
		} else if (anum < dSA_Y) // Comparing against the same constant lets compiler reuse EFLAGS register for
			// another conditional jump
		{
			dSASSERT(dSA_X < dSA_Y); // Otherwise the condition above is incorrect
			dIASSERT(anum == dSA_X);

			// This won't be unit length in general,
			// but it's what's used in getInfo2
			// This may be why things freak out as
			// the body-relative axes get close to each other.
			dCalcVectorCross3(result, axes[dSA_Y], axes[dSA_Z]);
		} else {
			dSASSERT(dSA_Z > dSA_Y); // Otherwise the condition above is incorrect
			dIASSERT(anum == dSA_Z);

			// Same problem as above.
			dCalcVectorCross3(result, axes[dSA_X], axes[dSA_Y]);
		}
	}


	void setAngleValue(int anum, double angle) {
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		dAASSERT(m_mode == dAMotorUser); // This only works for the dAMotorUser

		if (m_mode == dAMotorUser) {
			m_angle[anum] = angle;
		}
	}


	double calculateAngleRate(int anum)
	{
		dAASSERT(dIN_RANGE(anum, dSA__MIN, dSA__MAX));
		dAASSERT(this.node[0].body != null); // Don't call for angle rate before the joint is set up

		DVector3 axis = new DVector3();
		getAxisValue(axis, anum);

		// NOTE!
		// For reverse joints, the rate is negated at the function exit to create swapped bodies effect
		double rate = axis.dot(this.node[0].body.avel);

		if (this.node[1].body != null)
		{
			rate -= axis.dot(this.node[1].body.avel);
		}

		// Negating the rate for reverse joints creates an effect of body swapping
		double result = !GetIsJointReverse() ? rate : -rate;
		return result;
	}


	void _addTorques(double torque1, double torque2, double torque3) {
		int num = _getNumAxes();
		dAASSERT(dIN_RANGE(num, dSA__MIN, dSA__MAX + 1));

		DVector3 sum = new DVector3();
		DVector3 torqueVector = new DVector3();
		DVector3[] axes = DVector3.newArray( dSA__MAX);

		if (num != dSA__MIN) {
			computeGlobalAxes(axes);

			if (!GetIsJointReverse()) {
				torqueVector.set(torque1, torque2, torque3);
			} else {
				// Negating torques creates an effect of swapped bodies later
				torqueVector.set(-torque1, -torque2, -torque3);
			}
		}

		switch (num) {
			case dSA_Z + 1: {
				dAddThreeScaledVectors3(sum, axes[dSA_Z], axes[dSA_Y], axes[dSA_X], torqueVector.get(dSA_Z),
						torqueVector.get(dSA_Y), torqueVector.get(dSA_X));
				break;
			}

			case dSA_Y + 1: {
				dAddScaledVectors3(sum, axes[dSA_Y], axes[dSA_X], torqueVector.get(dSA_Y), torqueVector.get(dSA_X));
				break;
			}

			case dSA_X + 1: {
				dCopyScaledVector3(sum, axes[dSA_X], torqueVector.get(dSA_X));
				break;
			}

			default: {
				dSASSERT(dSA_Z > dSA_Y); // Otherwise the addends order needs to be switched
				dSASSERT(dSA_Y > dSA_X);

				// Do nothing
				break;
			}
		}

		if (num != dSA__MIN) {
			dAASSERT(this.node[0].body != null); // Don't add torques unless you set the joint up first!

			// NOTE!
			// For reverse joints, the torqueVector negated at function entry produces the effect of swapped bodies
			//dBodyAddTorque(this.node[0].body, sum[dV3E_X], sum[dV3E_Y], sum[dV3E_Z]);
			node[0].body.dBodyAddTorque(sum.get(dV3E_X), sum.get(dV3E_Y), sum.get(dV3E_Z));

			if (this.node[1].body != null) {
				node[1].body.dBodyAddTorque(-sum.get(dV3E_X), -sum.get(dV3E_Y), -sum.get(dV3E_Z));
			}
		}
	}


	// compute the 3 axes in global coordinates
	// void computeGlobalAxes(DVector3 ax[dSA__MAX]) {
	void computeGlobalAxes(DVector3[] ax) {
		switch (m_mode) {
			case dAMotorUser: {
				doComputeGlobalUserAxes(ax);
				break;
			}

			case dAMotorEuler: {
				doComputeGlobalEulerAxes(ax);
				break;
			}

			default: {
				dIASSERT(false);
				break;
			}
		}
	}

	//void doComputeGlobalUserAxes(DVector3 ax[dSA__MAX]) {
	void doComputeGlobalUserAxes(DVector3[] ax) {
		int num = m_num;
		for (int i = 0; i != num; ++i) {
			boolean assigned = false;

			if (m_rel[i] == dJBR_BODY1) {
				// relative to b1
				dMultiply0_331(ax[i], this.node[0].body.posr().R(), m_axis[i]);
				assigned = true;
			} else if (m_rel[i] == dJBR_BODY2) {
				// relative to b2
				if (this.node[1].body != null) {
					dMultiply0_331(ax[i], this.node[1].body.posr().R(), m_axis[i]);
					assigned = true;
				}
			}

			if (!assigned) {
				// global - just copy it
				ax[i].set(m_axis[i]);
			}
		}
	}

	// void doComputeGlobalEulerAxes(DVector3 ax[dSA__MAX]) {
	void doComputeGlobalEulerAxes(DVector3[] ax) {
		// special handling for Euler mode

		// dSpaceAxis
		int firstBodyAxis = BuildFirstBodyEulerAxis();
		dMultiply0_331(ax[firstBodyAxis], this.node[0].body.posr().R(), m_axis[firstBodyAxis]);

		//dSpaceAxis
		int secondBodyAxis = EncodeOtherEulerAxis(firstBodyAxis);

		if (this.node[1].body != null) {
			dMultiply0_331(ax[secondBodyAxis], this.node[1].body.posr().R(), m_axis[secondBodyAxis]);
		} else {
			//dCopyVector3(ax[secondBodyAxis], m_axis[secondBodyAxis]);
			ax[secondBodyAxis].set(m_axis[secondBodyAxis]);
		}

		dCalcVectorCross3(ax[dSA_Y], ax[dSA_Z], ax[dSA_X]);
		dNormalize3(ax[dSA_Y]);
	}


	//	void computeEulerAngles(DVector3[] ax[dSA__MAX]) {
	void computeEulerAngles(DVector3[] ax) {
		// assumptions:
		//   global axes already calculated -. ax
		//   axis[0] is relative to body 1 --> global ax[0]
		//   axis[2] is relative to body 2 --> global ax[2]
		//   ax[1] = ax[2] x ax[0]
		//   original ax[0] and ax[2] are perpendicular
		//   reference1 is perpendicular to ax[0] (in body 1 frame)
		//   reference2 is perpendicular to ax[2] (in body 2 frame)
		//   all ax[] and reference vectors are unit length

		// calculate references in global frame
		DVector3[] refs = DVector3.newArray( dJCB__MAX);
		dMultiply0_331(refs[dJCB_FIRST_BODY], this.node[0].body.posr().R(), m_references[dJCB_FIRST_BODY]);

		if (this.node[1].body != null) {
			dMultiply0_331(refs[dJCB_SECOND_BODY], this.node[1].body.posr().R(), m_references[dJCB_SECOND_BODY]);
		} else {
			//dCopyVector3(refs[dJCB_SECOND_BODY], m_references[dJCB_SECOND_BODY]);
			refs[dJCB_SECOND_BODY].set(m_references[dJCB_SECOND_BODY]);
		}


		// get q perpendicular to both ax[0] and ref1, get first euler angle
		DVector3 q = new DVector3();
		//dJointConnectedBody
		int firstAxisBody = BuildFirstEulerAxisBody();

		dCalcVectorCross3(q, ax[dSA_X], refs[firstAxisBody]);
		m_angle[dSA_X] = -dAtan2(dCalcVectorDot3(ax[dSA_Z], q), dCalcVectorDot3(ax[dSA_Z], refs[firstAxisBody]));

		// get q perpendicular to both ax[0] and ax[1], get second euler angle
		dCalcVectorCross3(q, ax[dSA_X], ax[dSA_Y]);
		m_angle[dSA_Y] = -dAtan2(dCalcVectorDot3(ax[dSA_Z], ax[dSA_X]), dCalcVectorDot3(ax[dSA_Z], q));

		//dJointConnectedBody
		int secondAxisBody = EncodeJointOtherConnectedBody(firstAxisBody);

		// get q perpendicular to both ax[1] and ax[2], get third euler angle
		dCalcVectorCross3(q, ax[dSA_Y], ax[dSA_Z]);
		m_angle[dSA_Z] = -dAtan2(dCalcVectorDot3(refs[secondAxisBody], ax[dSA_Y]), dCalcVectorDot3(refs[secondAxisBody], q));
	}


	// set the reference vectors as follows:
	//   * reference1 = current axis[2] relative to body 1
	//   * reference2 = current axis[0] relative to body 2
	// this assumes that:
	//    * axis[0] is relative to body 1
	//    * axis[2] is relative to body 2

	void setEulerReferenceVectors() {
		if (/*this.node[0].body != NULL && */this.node[1].body != null) {
			dIASSERT(this.node[0].body != null);

			DVector3 r = new DVector3();  // axis[2] and axis[0] in global coordinates

			// dSpaceAxis
			int firstBodyAxis = BuildFirstBodyEulerAxis();
			dMultiply0_331(r, this.node[0].body.posr().R(), m_axis[firstBodyAxis]);
			dMultiply1_331(m_references[dJCB_SECOND_BODY], this.node[1].body.posr().R(), r);

			// dSpaceAxis
			int secondBodyAxis = EncodeOtherEulerAxis(firstBodyAxis);
			dMultiply0_331(r, this.node[1].body.posr().R(), m_axis[secondBodyAxis]);
			dMultiply1_331(m_references[dJCB_FIRST_BODY], this.node[0].body.posr().R(), r);
		} else {
			// We want to handle angular motors attached to passive geoms
			// Replace missing node.R with identity
			if (this.node[0].body != null) {
				// dSpaceAxis
				int firstBodyAxis = BuildFirstBodyEulerAxis();
				dMultiply0_331(m_references[dJCB_SECOND_BODY], this.node[0].body.posr().R(), m_axis[firstBodyAxis]);

				// dSpaceAxis
				int secondBodyAxis = EncodeOtherEulerAxis(firstBodyAxis);
				dMultiply1_331(m_references[dJCB_FIRST_BODY], this.node[0].body.posr().R(), m_axis[secondBodyAxis]);
			}
		}
	}

	/*inline */
	//dSpaceAxis
	int BuildFirstBodyEulerAxis() {
		return EncodeJointConnectedBodyEulerAxis(BuildFirstEulerAxisBody());
	}

	/*inline */
	//dJointConnectedBody
	int BuildFirstEulerAxisBody() {
		return !GetIsJointReverse() ? dJCB_FIRST_BODY : dJCB_SECOND_BODY;
	}

	// **********************************
	// API dAMotorJoint
	// **********************************

	@Override
	public void setMode (AMotorMode mode)
	{ dJointSetAMotorMode (mode); }
	@Override
	public AMotorMode getMode()
	{ return dJointGetAMotorMode (); }

	@Override
	public void setNumAxes (int num)
	{ dJointSetAMotorNumAxes (num); }
	@Override
	public int getNumAxes()
	{ return dJointGetAMotorNumAxes (); }

	@Override
	public void setAxis (int anum, int rel, double x, double y, double z)
	{ dJointSetAMotorAxis (anum, rel, x, y, z); }
	@Override
	public void setAxis (int anum, int rel, DVector3C a)
	{ dJointSetAMotorAxis (anum, rel, a); }
	@Override
	public void getAxis (int anum, DVector3 result)
	{ dJointGetAMotorAxis (anum, result); }
	@Override
	public int getAxisRel (int anum)
	{ return dJointGetAMotorAxisRel (anum); }

	@Override
	public void setAngle (int anum, double angle)
	{ dJointSetAMotorAngle (anum, angle); }
	@Override
	public double getAngle (int anum)
	{ return dJointGetAMotorAngle (anum); }
	@Override
	public double getAngleRate (int anum)
	{ return dJointGetAMotorAngleRate (anum); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetAMotorParam (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetAMotorParam (parameter); }

	@Override
	public void addTorques(double torque1, double torque2, double torque3)
	{ dJointAddAMotorTorques(torque1, torque2, torque3); }


	@Override
	public void setParamFMax(double d) {
		dJointSetAMotorParam(PARAM_N.dParamFMax1, d);
	}


	@Override
	public void setParamFMax2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamFMax2, d);
	}


	@Override
	public void setParamFMax3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamFMax3, d);
	}


	@Override
	public void setParamHiStop(double d) {
		dJointSetAMotorParam(PARAM_N.dParamHiStop1, d);
	}


	@Override
	public void setParamHiStop2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamHiStop2, d);
	}


	@Override
	public void setParamHiStop3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamHiStop3, d);
	}


	@Override
	public void setParamLoStop(double d) {
		dJointSetAMotorParam(PARAM_N.dParamLoStop1, d);
	}


	@Override
	public void setParamLoStop2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamLoStop2, d);
	}


	@Override
	public void setParamLoStop3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamLoStop3, d);
	}


	@Override
	public void setParamVel(double d) {
		dJointSetAMotorParam(PARAM_N.dParamVel1, d);
	}


	@Override
	public void setParamVel2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamVel2, d);
	}


	@Override
	public void setParamVel3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamVel3, d);
	}


	@Override
	public double getParamFMax() {
		return dJointGetAMotorParam(PARAM_N.dParamFMax1);
	}


	@Override
	public double getParamFMax2() {
		return dJointGetAMotorParam(PARAM_N.dParamFMax2);
	}


	@Override
	public double getParamFMax3() {
		return dJointGetAMotorParam(PARAM_N.dParamFMax3);
	}


	@Override
	public double getParamVel() {
		return dJointGetAMotorParam(PARAM_N.dParamVel1);
	}


	@Override
	public double getParamVel2() {
		return dJointGetAMotorParam(PARAM_N.dParamVel2);
	}


	@Override
	public double getParamVel3() {
		return dJointGetAMotorParam(PARAM_N.dParamVel3);
	}





//	public void dJointSetAMotorAxis( int anum, int rel, DVector3C r )
//	{
//		dAASSERT( anum >= 0 && anum <= 2 && rel >= 0 && rel <= 2 );
//
//		if ( anum < 0 ) anum = 0;
//		if ( anum > 2 ) anum = 2;
//
//		// adjust rel to match the internal body order
//		if ( (flags & dJOINT_REVERSE)!=0 && rel!=0 )
//			rel ^= 3; // turns 1 into 2, 2, into 1
//
//		_rel[anum] = rel;
//
//		// x,y,z (vector r)  is always in global coordinates regardless of rel, so we may have
//		// to convert it to be relative to a body
//		if ( rel > 0 )
//		{
//			if ( rel == 1 )
//			{
//				dMultiply1_331( axis[anum], node[0].body.posr().R(), r );
//			}
//			else // rel == 2
//			{
//				// don't assert; handle the case of attachment to a bodiless geom
//				if ( node[1].body!=null )   // jds
//				{
//					dMultiply1_331( axis[anum], node[1].body.posr().R(), r );
//				}
//				else
//				{
//					//					axis[anum].v[0] = r.v[0];
//					//					axis[anum].v[1] = r.v[1];
//					//					axis[anum].v[2] = r.v[2];
//					//					axis[anum].v[3] = r.v[3];
//					axis[anum].set(r);
//				}
//			}
//		}
//		else
//		{
//			//			axis[anum].v[0] = r.v[0];
//			//			axis[anum].v[1] = r.v[1];
//			//			axis[anum].v[2] = r.v[2];
//			axis[anum].set(r);
//		}
//		dNormalize3( axis[anum] );
//		if ( _mode == dAMotorEuler ) setEulerReferenceVectors();
//	}


}