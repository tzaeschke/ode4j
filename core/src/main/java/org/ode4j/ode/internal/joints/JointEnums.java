package org.ode4j.ode.internal.joints;

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

}
