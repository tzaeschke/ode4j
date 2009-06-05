package org.ode4j.math;

import org.ode4j.math.DMatrix3.DVector3View;

/**
 * This class provides functionality for dVector3 math.
 * 
 * Performance considerations:
 * - Each class implements its own strictly typed methods. Using
 *   generic methods in super-classes showed to slow down operations
 *   like add() or set() by a factor of ~10 (SUN JDK 6 on SuSE 11.1 64bit).
 * - Using complex methods (e.g. v0.sum(v1, v2, s2)) showed to be about
 *   2 times faster than concatenating methods, e.g. v0.set(v2).scale(s2).add(v1). 
 *
 * @author Tilmann Zaeschke
 *
 */
public class DVector3 extends DVector<DVector3> implements DVector3I, DVector3C {
	private static final int LEN = 4;
	public static final DVector3 ZERO = new DVector3(0, 0, 0);
	public static final int CURRENT_LENGTH = 4;

	
	public DVector3() { 
		super(LEN); 
	}
	
	public DVector3(DVector3C v2) {
		this();
		set(v2);
	}
	
	public DVector3(double i, double j, double k) {
		this();
		set(i, j, k);
	}
	
	/** @deprecated Wrong number of arguments. */
	public DVector3(double i, double j, double k, double l) {
		this();
		set(i, j, k); 
		v[3] = l;
	}
	
	public final DVector3 set(double x, double y, double z) {
		v[0] = x; v[1] = y; v[2] = z; //v[3] = v3.v[3];
		return this;
	}
	
	public final DVector3 set(DVector3C v2) {
		v[0] = v2.get0(); v[1] = v2.get1(); v[2] = v2.get2();
		return this;
	}
	
	public final DVector3 set(DVector3View v2) {
		v[0] = v2.get0(); v[1] = v2.get1(); v[2] = v2.get2();
		return this;
	}
	
	public DVector3 clone() {
		return new DVector3(this);
	}
	
	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("dVector3[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
	}

	@Override
	public void assertLen(int n) {
		if (n!=LEN) {
			throw new IllegalStateException("LEN is " + LEN + ", not " + n);
		}		
	}
	
	public void set0(double d) {
		v[0] = d;
	}
	
	public void set1(double d) {
		v[1] = d;
	}
	
	public void set2(double d) {
		v[2] = d;
	}
	
	public double get0() {
		return v[0];
	}
	
	public double get1() {
		return v[1];
	}
	
	public double get2() {
		return v[2];
	}
	
	public DVector3 add(double a, double b, double c) {
		v[0] += a; v[1] += b; v[2] += c;
		return this;
	}
	
	public final DVector3 add(DVector3C v2) {
		v[0] += v2.get0(); v[1] += v2.get1(); v[2] += v2.get2();
		return this;
	}
	
	public final DVector3 eqSum(DVector3C v2, DVector3C v3) {
		v[0] = v2.get0() + v3.get0(); 
		v[1] = v2.get1() + v3.get1(); 
		v[2] = v2.get2() + v3.get2();
		return this;
	}
	
	public final DVector3 eqSum(DVector3 v2, double s1, DVector3 v3, double s2) {
		v[0] = v2.v[0]*s1 + v3.v[0]*s2; 
		v[1] = v2.v[1]*s1 + v3.v[1]*s2; 
		v[2] = v2.v[2]*s1 + v3.v[2]*s2;
		return this;
	}
	
	public final DVector3 eqSum(DVector3View v2, double s1, DVector3 v3, double s2) {
		v[0] = v2.get0()*s1 + v3.v[0]*s2; 
		v[1] = v2.get1()*s1 + v3.v[1]*s2; 
		v[2] = v2.get2()*s1 + v3.v[2]*s2;
		return this;
	}
	
	public final DVector3 eqSum(DVector3View v2, double s1, DVector3View v3, double s2) {
		v[0] = v2.get0()*s1 + v3.get0()*s2; 
		v[1] = v2.get1()*s1 + v3.get1()*s2; 
		v[2] = v2.get2()*s1 + v3.get2()*s2;
		return this;
	}
	
	public final DVector3 eqSum(DVector3C v2, DVector3C v3, double s2) {
		v[0] = v2.get0() + v3.get0()*s2; 
		v[1] = v2.get1() + v3.get1()*s2; 
		v[2] = v2.get2() + v3.get2()*s2;
		return this;
	}

	public final DVector3 eqSum(DVector3 v2, DVector3View v3, double s2) {
		v[0] = v2.get0() + v3.get0()*s2; 
		v[1] = v2.get1() + v3.get1()*s2; 
		v[2] = v2.get2() + v3.get2()*s2;
		return this;
	}
	
	public DVector3 sub(double a, double b, double c) {
		v[0] -= a; v[1] -= b; v[2] -= c;
		return this;
	}

	public DVector3 sub(DVector3C v2) {
		v[0] -= v2.get0(); v[1] -= v2.get1(); v[2] -= v2.get2();
		return this;
	}

	public DVector3 scale(double a, double b, double c) {
		v[0] *= a; v[1] *= b; v[2] *= c;
		return this;
	}

	public final DVector3 scale(double s) {
		v[0] *= s; v[1] *= s; v[2] *= s;
		return this;
	}

	public DVector3 scale(DVector3C v2) {
		v[0] *= v2.get0(); v[1] *= v2.get1(); v[2] *= v2.get2();
		return this;
	}

	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2;
	 * @param b 
	 * @return (this) * b
	 */
	public final double reDot(DVector3C b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2();
	}
	
	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2;
	 * @param b 
	 * @return (this) * b
	 */
	public final double reDot(DVector3View b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2();
	}
	
//	/**
//	 * Calculate the 'dot' or 'inner' product. <br>
//	 * x = b_t * c  <br>
//	 * x = b1*c1 + b2*c2 + b3*c3.
//	 * @param b
//	 * @param c
//	 * @return The dot product.
//	 */
//	public double eqDot(final dVector3C b, final dVector3C c) {
//		return b.get0()*c.get0() + b.get1()*c.get1() + b.get2()*c.get2();
//		
//	}
	
	/**
	 * Sets the current vector v0 = v2 - v3.
	 * @param v2
	 * @param v3
	 */
	public DVector3 eqDiff(DVector3C v2, DVector3C v3) {
		v[0] = v2.get0() - v3.get0(); 
		v[1] = v2.get1() - v3.get1(); 
		v[2] = v2.get2() - v3.get2();
		return this;
	}
	
	/**
	 * Return a new vector v0 = v(this) - v3.
	 * @param v2
	 * @param v3
	 */
	public DVector3 reSub(DVector3C v2) {
		return new DVector3(
				get0() - v2.get0(),
				get1() - v2.get1(),
				get2() - v2.get2());
	}
	
	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 */
	private boolean _dSafeNormalize3 ()
	{
		//TODO use double a1, a2, a3; )?)
		//TODO avoid idx by replacing it with a.scale() ?
		
		int idx;
		double[] aa = new double[3];
//		double l;

		aa[0] = Math.abs(v[0]);
		aa[1] = Math.abs(v[1]);
		aa[2] = Math.abs(v[2]);
		if (aa[1] > aa[0]) {
			if (aa[2] > aa[1]) { // aa[2] is largest
				idx = 2;
			}
			else {              // aa[1] is largest
				idx = 1;
			}
		}
		else {
			if (aa[2] > aa[0]) {// aa[2] is largest
				idx = 2;
			}
			else {              // aa[0] might be the largest
				if (aa[0] <= 0) { // aa[0] might is largest
//					a.v[0] = 1;	// if all a's are zero, this is where we'll end up.
//					a.v[1] = 0;	// return a default unit length vector.
//					a.v[2] = 0;
					this.set(1, 0, 0);
					return false;
				}
				else {
					idx = 0;
				}
			}
		}

//		a.v[0] /= aa[idx];
//		a.v[1] /= aa[idx];
//		a.v[2] /= aa[idx];
		this.scale(1./aa[idx]);
		//l = dRecipSqrt (a.v[0]*a.v[0] + a.v[1]*a.v[1] + a.v[2]*a.v[2]);
//		a.v[0] *= l;
//		a.v[1] *= l;
//		a.v[2] *= l;
		this.scale(1./this.length());
		return true;
	}
	public void normalize()
	{
		boolean bNormalizationResult = _dSafeNormalize3();
		if (!bNormalizationResult) throw new IllegalStateException();
	}

	public boolean isEq(DVector3 a) {
		return v[0]==a.v[0] && v[1]==a.v[1] && v[2]==a.v[2];
	}

	public void eqAbs() {
		for (int i = 0; i < v.length; i++) {
			v[i] = Math.abs(v[i]);
		}
	}
	
//	public void dMultiply0 (final dMatrix3C B, final dVector3C C)
//	{
//		eqMul(B, C);
//	}
//	
//	public void eqMul (final dMatrix3C B, final dVector3C c)
//	{
////		double[] B2 = ((dMatrix3) B).v;
////		double[] C2 = ((dVector3) C).v;
////		double sum;
////		int aPos = 0;
////		int bPos, bbPos =0, cPos;
////		for (int i=3; i > 0; i--) {
////			cPos = 0;
////			bPos = bbPos;
////			sum = 0;
////			for (int k=3; k > 0; k--) sum += B2[bPos++] * C2[cPos++];
////			v[aPos++] = sum;
////			bbPos += 4;
////		}
//		v[0] = B.get00()*c.get0() + B.get01()*c.get1() + B.get02()*c.get2();
//		v[1] = B.get10()*c.get0() + B.get11()*c.get1() + B.get12()*c.get2();
//		v[2] = B.get20()*c.get0() + B.get21()*c.get1() + B.get22()*c.get2();
//	}

	public final void add0(double d) {
		v[0] += d;
	}

	public final void add1(double d) {
		v[1] += d;
	}

	public final void add2(double d) {
		v[2] += d;
	}

	/**
	 * 
	 * @return '3'.
	 */
	public int dim() {
		//return LEN;
		//TODO
		return 3;
	}

	public DVector3C reAdd(DVector3C c) {
		return new DVector3(this).add(c);
	}
	
	
	/**
	 * Writes the content of this vector into 
	 * <tt>array</tt> at position <tt>pos</tt>.  
	 * @param array
	 * @param pos
	 */
	public final void wrapSet(double[] array, int pos) {
		array[pos] = get0();
		array[pos + 1] = get1();
		array[pos + 2] = get2();
	}
	
	/**
	 * Adds the content of this vector to the elements of 
	 * <tt>array</tt> at position <tt>pos</tt>.  
	 * @param array
	 * @param pos
	 */
	public final void wrapAdd(double[] array, int pos) {
		array[pos] += get0();
		array[pos + 1] += get1();
		array[pos + 2] += get2();
	}
	
	/**
	 * Subtracts the content of this vector from the elements of 
	 * <tt>array</tt> at position <tt>pos</tt>.  
	 * @param array
	 * @param pos
	 */
	public final void wrapSub(double[] array, int pos) {
		array[pos] -= get0();
		array[pos + 1] -= get1();
		array[pos + 2] -= get2();
	}

	public DVector3C reScale(int i) {
		return new DVector3(this).scale(-1);
	}
}


