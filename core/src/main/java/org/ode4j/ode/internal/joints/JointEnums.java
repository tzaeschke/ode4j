package org.ode4j.ode.internal.joints;

import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.CommonEnums.*;

public class JointEnums {

	public static final int GI2__J_MIN = 0;
	public static final int GI2__JL_MIN = GI2__J_MIN + dDA__L_MIN;
	public static final int GI2_JLX = GI2__J_MIN + dDA_LX;
	public static final int GI2_JLY = GI2__J_MIN + dDA_LY;
	public static final int GI2_JLZ = GI2__J_MIN + dDA_LZ;
	public static final int GI2__JL_MAX = GI2__J_MIN + dDA__L_MAX;
	public static final int GI2__JA_MIN = GI2__J_MIN + dDA__A_MIN;
	public static final int GI2_JAX = GI2__J_MIN + dDA_AX;
	public static final int GI2_JAY = GI2__J_MIN + dDA_AY;
	public static final int GI2_JAZ = GI2__J_MIN + dDA_AZ;
	public static final int GI2__JA_MAX = GI2__J_MIN + dDA__A_MAX;
	public static final int GI2__J_MAX = GI2__J_MIN + dDA__MAX;
	public static final int GI2_RHS = 0;
	public static final int GI2_CFM = GI2_RHS + 1;
	public static final int GI2__RHS_CFM_MAX = GI2_CFM + 1;
	public static final int GI2_LO = 0;
	public static final int GI2_HI = GI2_LO + 1;
	public static final int GI2__LO_HI_MAX = GI2_HI + 1;


	// dJointConnectedBody
	public static final int dJCB__MIN = 0;
	public static final int dJCB_FIRST_BODY = dJCB__MIN;
	public static final int dJCB_SECOND_BODY = dJCB_FIRST_BODY + 1;
	public static final int dJCB__MAX = dJCB_SECOND_BODY + 1;

	//static inline
	//dJointConnectedBody EncodeJointOtherConnectedBody(dJointConnectedBody cbBodyKind)
	static int EncodeJointOtherConnectedBody(int cbBodyKind) {
		dIASSERT(dIN_RANGE(cbBodyKind, dJCB__MIN, dJCB__MAX));
		dSASSERT(dJCB__MAX == 2);

		return dJCB_FIRST_BODY + dJCB_SECOND_BODY - cbBodyKind;
	}

	/* joint body relativity enumeration */
	// dJointBodyRelativity
	public static final int dJBR__MIN = 0;
	public static final int dJBR_GLOBAL = dJBR__MIN;
	public static final int dJBR__BODIES_MIN = dJBR_GLOBAL + 1;
	public static final int dJBR_BODY1 = dJBR__BODIES_MIN + dJCB_FIRST_BODY;
	public static final int dJBR_BODY2 = dJBR__BODIES_MIN + dJCB_SECOND_BODY;
	public static final int dJBR__BODIES_MAX = dJBR__BODIES_MIN + dJCB__MAX;
	public static final int dJBR__MAX = dJBR__BODIES_MAX + 1;
	public static final int dJBR__DEFAULT = dJBR_GLOBAL;
	public static final int dJBR__BODIES_COUNT = dJBR__BODIES_MAX - dJBR__BODIES_MIN;

	//	enum dJointBodyRelativity {
	//		dJBR_GLOBAL, dJBR_BODY1, dJBR_BODY2;
	//
	//		public static final int dJBR__MIN = 0;
	//		public static final int dJBR__BODIES_MIN = dJBR_GLOBAL.ordinal() + 1;
	//		public static final int dJBR__BODIES_MAX = dJBR__BODIES_MIN + dJCB__MAX;
	//		public static final int dJBR__MAX = dJBR__BODIES_MAX + 1;
	//		public static final int dJBR__DEFAULT = dJBR_GLOBAL.ordinal();
	//		public static final int dJBR__BODIES_COUNT = dJBR__BODIES_MAX - dJBR__BODIES_MIN;
	//	}


	public static boolean dJBREncodeBodyRelativityStatus(int relativity) {
		return dIN_RANGE(relativity, dJBR__BODIES_MIN, dJBR__BODIES_MAX);
	}

	// dJointBodyRelativity dJBRSwapBodyRelativity(int relativity)
	public static int dJBRSwapBodyRelativity(int relativity) {
		dIASSERT(dIN_RANGE(relativity, dJBR__BODIES_MIN, dJBR__BODIES_MAX));
		return dJBR_BODY1 + dJBR_BODY2 - relativity;
	}

	private JointEnums() {}
}
