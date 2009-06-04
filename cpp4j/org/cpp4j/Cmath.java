package org.cpp4j;

import org.cpp4j.java.RefInt;

public class Cmath extends Csetjmp {

	public static float fabs(float x) { return Math.abs(x); }
	public static double fabs(double x) { return Math.abs(x); }
	public static float cos(float x) { return (float) Math.cos(x); }
	public static double cos(double x) { return Math.cos(x); }
	public static float sin(float x) { return (float) Math.sin(x); }
	public static double sin(double x) { return Math.sin(x); }
	public static float sqrt(float x) { return (float) Math.sqrt(x); }
	public static double sqrt(double x) { return Math.sqrt(x); }
	public static float pow(float base, float exp) { 
		return (float) Math.pow(base, exp); }
	public static double pow(double base, double exp) { 
		return Math.pow(base, exp); }

	
	public static double ldexp(double num, int exp) {
		return num * Math.pow(2, exp);
	}
	
	public static final double atan2(double y, double x) {
		return Math.atan2(y, x);
	}
	
	public static final double asin(double x) {
		return Math.asin(x);
	}

	public static final float ceil(float x) {
		return (float) Math.ceil(x);
	}

	//TODO ceilf is not really part of the standard, is it?
	public static final float ceilf(float x) {
		return (float) Math.ceil(x);
	}
	public static final float ceilf(double x) {
		return (float) Math.ceil(x);
	}
	
	public static final double ceil(double x) {
		return (float) Math.ceil(x);
	}
	
	/**
	 * TODO make it faster. Use log2?
	 */
	public static double frexp(double num, RefInt exp) {
		//http://www.opengroup.org/onlinepubs/007908799/xsh/frexp.html
		if (num == 0) {
			exp.i = 0;
			return 0;
		}
		if (Double.isNaN(num)) {
			return Double.NaN;
		}
//		double log10 = Math.log10(num);
//		exp.i = (int)log10;
//		double mantissa = Math.pow(10, log10  - exp.i);
//		return mantissa;
		//http://www.math.northwestern.edu/~wphooper/code/java/
		long bits=Double.doubleToLongBits(num);
		exp.i=(int)((0x7ff0000000000000L & bits)>>52)-1022;
		return Double.longBitsToDouble((0x800fffffffffffffL & bits)| 0x3fe0000000000000L);
	}
}
