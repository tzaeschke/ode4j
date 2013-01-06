/*************************************************************************
 *                                                                       *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.math;

import org.ode4j.math.DMatrix3.DVector3ColView;

/**
 * This class provides functionality for DVector3 math.
 * <p>
 * Most methods have a prefix indicating whether the (this) object
 * will be modified or not.
 * All methods starting with "eq" will write the result into (this).
 * All methods starting with "re" return a new Object containing the result.
 * For all other methods the behaviour should be obvious.
 * <p>
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
public class DVector3 implements DVector3I, DVector3C {
//public class DVector3 implements DVector3I, DVector3C {
	private double d0, d1, d2;
	private static final int LEN = 4;  //TODO 3 ?
    public static final DVector3C ZERO = new DVector3();
	public static final int CURRENT_LENGTH = 4;

	//private static int COUNT = 0;
	
	/**
	 * Creates a (0,0,0) vector.
	 */
	public DVector3() { 
		d0 = 0;
		d1 = 0;
		d2 = 0;
	}
	
	/**
	 * Creates a vector initialised as copy of v2.
	 * @param v2
	 */
	public DVector3(DVector3C v2) {
		this();
		set(v2);
	}
	
    /**
     * Creates a vector initialised with the first three values of v2.
     * @param v2
     */
	public DVector3(double[] v2) {
		this();
		set(v2);
	}
	
    /**
     * Creates a (i,j,k) vector.
     */
	public DVector3(double i, double j, double k) {
		this();
		set(i, j, k);
	}
	
	public final DVector3 set(double[] v2) {
		set(v2[0], v2[1], v2[2]);
		return this;
	}

	public void set(float[] v2) {
		set(v2[0], v2[1], v2[2]);
	}
	
	public final DVector3 set(double x, double y, double z) {
		//set0( x ); set1( y ); set2( z );
		d0 = x; d1 = y; d2 = z;
		return this;
	}
	
	public final DVector3 set(DVector3C v2) {
		set0( v2.get0() ); set1( v2.get1() ); set2( v2.get2() );
		return this;
	}
	
	public final DVector3 set(DVector3ColView v2) {
		set0( v2.get0() ); set1( v2.get1() ); set2( v2.get2() );
		return this;
	}
	
	@Override
	public DVector3 clone() {
		return new DVector3(this);
	}
	
	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("DVector3[ ");
		b.append(get0()).append(", ");
		b.append(get1()).append(", ");
		b.append(get2()).append(" ]");
//		for (int i = 0; i < v.length-1; i++) {
//			b.append(v[i]).append(", ");
//		}
//		b.append(v[v.length-1]).append("]");
		return b.toString();
	}

//	@Override
//	public void assertLen(int n) {
//		if (n!=LEN) {
//			throw new IllegalStateException("LEN is " + LEN + ", not " + n);
//		}		
//	}
	
	@Override
	public final void set0(double d) {
		d0 = d;
	}
	
	@Override
	public final void set1(double d) {
		d1 = d;
	}
	
	@Override
	public final void set2(double d) {
		d2 = d;
	}
	
	@Override
	public final double get0() {
		return d0;
	}
	
	@Override
	public final double get1() {
		return d1;
	}
	
	@Override
	public final double get2() {
		return d2;
	}
	
	public final DVector3 add(double a, double b, double c) {
		d0 += a; d1 += b; d2 += c;
		return this;
	}
	
	/**
	 * Adds v2 to the current vector.
	 * @param v2
	 * @return Current vector.
	 */
	public final DVector3 add(DVector3C v2) {
		d0 += v2.get0(); d1 += v2.get1(); d2 += v2.get2();
		return this;
	}
	
	/**
	 * Sets current vector = v2 + v3.
	 * @param v2
	 * @param v3
	 * @return Current vector.
	 */
	public final DVector3 eqSum(DVector3C v2, DVector3C v3) {
		set0( v2.get0() + v3.get0() ); 
		set1( v2.get1() + v3.get1() ); 
		set2( v2.get2() + v3.get2() );
		return this;
	}
	
	/**
	 * Convenience function that performs:
	 * this = v2*s2 + v3*s3
	 * @return this
	 */
	public final DVector3 eqSum(DVector3C v2, double s2, DVector3C v3, double s3) {
		set0( v2.get0()*s2 + v3.get0()*s3 ); 
		set1( v2.get1()*s2 + v3.get1()*s3 ); 
		set2( v2.get2()*s2 + v3.get2()*s3 );
		return this;
	}
	
	/**
	 * Convenience function that performs:
	 * this = v2*s2 + v3*s3
	 * @return this
	 */
	public final DVector3 eqSum(DVector3ColView v2, double s2, DVector3C v3, double s3) {
		set0( v2.get0()*s2 + v3.get0()*s3 ); 
		set1( v2.get1()*s2 + v3.get1()*s3 ); 
		set2( v2.get2()*s2 + v3.get2()*s3 );
		return this;
	}
	
	/**
	 * Convenience function that performs:
	 * this = v2*s2 + v3*s3
	 * @return this
	 */
	public final DVector3 eqSum(DVector3ColView v2, double s2, DVector3ColView v3, double s3) {
		set0( v2.get0()*s2 + v3.get0()*s3 ); 
		set1( v2.get1()*s2 + v3.get1()*s3 ); 
		set2( v2.get2()*s2 + v3.get2()*s3 );
		return this;
	}
	
	/**
	 * Convenience function that performs:
	 * this = v2 + v3*s3
	 * @return this
	 */
	public final DVector3 eqSum(DVector3C v2, DVector3C v3, double s3) {
		set0( v2.get0() + v3.get0()*s3 ); 
		set1( v2.get1() + v3.get1()*s3 ); 
		set2( v2.get2() + v3.get2()*s3 );
		return this;
	}

	/**
	 * Convenience function that performs:
	 * this = v2 + v3*s3
	 * @return this
	 */
	public final DVector3 eqSum(DVector3C v2, DVector3ColView v3, double s3) {
		set0( v2.get0() + v3.get0()*s3 ); 
		set1( v2.get1() + v3.get1()*s3 ); 
		set2( v2.get2() + v3.get2()*s3 );
		return this;
	}
	
	public final DVector3 sub(double a, double b, double c) {
		d0 -= a; d1 -= b; d2 -= c;
		return this;
	}

	public final DVector3 sub(DVector3C v2) {
		d0 -= v2.get0(); d1 -= v2.get1(); d2 -= v2.get2();
		return this;
	}

	public final DVector3 scale(double a, double b, double c) {
		d0 *= a; d1 *= b; d2 *= c;
		return this;
	}

	public final DVector3 scale(double s) {
		d0 *= s; d1 *= s; d2 *= s;
		return this;
	}

	public final DVector3 scale(DVector3C v2) {
		d0 *= v2.get0(); d1 *= v2.get1(); d2 *= v2.get2();
		return this;
	}

	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2;
	 * @param b 
	 * @return (this) * b
	 */
	@Override
	public final double dot(DVector3C b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2();
	}
	
	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2;
	 * @param b 
	 * @return (this) * b
	 */
	@Override
	public final double dot(DVector3View b) {
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
	 * @return (this).
	 */
	public final DVector3 eqDiff(DVector3C v2, DVector3C v3) {
		d0 = v2.get0() - v3.get0(); 
		d1 = v2.get1() - v3.get1(); 
		d2 = v2.get2() - v3.get2();
		return this;
	}
	
	/**
	 * Return a new vector v0 = v(this) - v2.
	 * @param v2
	 * @return new vector
	 */
	@Override
	public final DVector3 reSub(DVector3C v2) {
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
	 * 
	 * This method returns (1,0,0) if no normal can be determined.
	 */
	public final boolean safeNormalize ()
	{
		double s;

		double aa0 = Math.abs(get0());
		double aa1 = Math.abs(get1());
		double aa2 = Math.abs(get2());
		if (aa1 > aa0) {
			if (aa2 > aa1) { // aa[2] is largest
				s = aa2;
			}
			else {              // aa[1] is largest
				s = aa1;
			}
		}
		else {
			if (aa2 > aa0) {// aa[2] is largest
				s = aa2;
			}
			else {              // aa[0] might be the largest
				if (aa0 <= 0) { // aa[0] might is largest
//					a.d0 = 1;	// if all a's are zero, this is where we'll end up.
//					a.d1 = 0;	// return a default unit length vector.
//					a.d2 = 0;
					set(1, 0, 0);
					return false;
				}
				else {
					s = aa0;
				}
			}
		}

		scale(1./s);
		scale(1./length());
		return true;
	}
	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 * 
	 * This method throws an IllegalArgumentEception if no normal can be determined.
	 */
	public final void normalize()
	{
		if (!safeNormalize()) throw new IllegalStateException(
				"Normalization failed: " + this);
	}

	/**
	 * Distance between this vector and a.
	 * @param a
	 * @return distance
	 */
	@Override
	public final double distance(DVector3C a) {
	    double r1 = get0()-a.get0();
	    double r2 = get1()-a.get1();
	    double r3 = get2()-a.get2();
	    return Math.sqrt(r1*r1 + r2*r2 + r3*r3);
	}
	
	/**
	 * Check whether two vectors contains the same values.
	 * Due to Java's polymorphism handling, this method can be much faster than
	 * v.equals(a).
	 * @param a
	 * @return quality
	 */
	public final boolean isEq(DVector3 a) {
		return get0()==a.get0() && get1()==a.get1() && get2()==a.get2();
	}

	/**
	 * Set this vector equal to abs(this).
	 */
	public final void eqAbs() {
		set0( Math.abs(get0()));
		set1( Math.abs(get1()));
		set2( Math.abs(get2()));
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
//		d0 = B.get00()*c.get0() + B.get01()*c.get1() + B.get02()*c.get2();
//		d1 = B.get10()*c.get0() + B.get11()*c.get1() + B.get12()*c.get2();
//		d2 = B.get20()*c.get0() + B.get21()*c.get1() + B.get22()*c.get2();
//	}

	public final void add0(double d) {
		d0 += d;
	}

	public final void add1(double d) {
		d1 += d;
	}

	public final void add2(double d) {
		d2 += d;
	}

	/**
	 * 
	 * @return '3'.
	 */
	public final int dim() {
		//return LEN;
		//TODO
		return 3;
	}

	/**
	 * Returns a new vector with the sum of (this)+c.
	 * @param c
	 * @return new vector
	 */
	public final DVector3C reAdd(DVector3C c) {
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

    /**
     * Returns a new vector which equals (this)*d.
     * @param d
     * @return new vector
     */
	@Override
	public final DVector3 reScale(double d) {
		return new DVector3(this).scale(d);
	}

	public final void eqZero() {
		set(0, 0, 0);
	}

	public final void setZero() {
		eqZero();
	}
	
	public final void eqIdentity() {
		set(1, 0, 0);
	}

	public final void setIdentity() {
		eqIdentity();
	}

	@Override
	public final float[] toFloatArray4() {
		return new float[]{ (float) get0(), (float) get1(), (float) get2(), 0.0f };
	}
	
	/**
	 * @return The geometric length of this vector.
	 */
	@Override
	public final double length() {
		return Math.sqrt( get0()*get0() + get1()*get1() + get2()*get2() ) ;
	}

	@Override
	public final double lengthSquared() {
		return get0()*get0() + get1()*get1() + get2()*get2();
	}
	
	@Override
	public final double get(int i) {
	    switch (i) {
        case 0: return d0;
        case 1: return d1;
        case 2: return d2;
        default: throw new IllegalArgumentException("i=" + i);
	    }
	}

	@Override
	public final float[] toFloatArray() {
		return new float[]{(float) get0(), (float) get1(), (float) get2()};
	}
	
	public final void set(int i, double d) {
        switch (i) {
        case 0: d0 = d; break;
        case 1: d1 = d; break;
        case 2: d2 = d; break;
        default: throw new IllegalArgumentException("i=" + i);
        }
	}

	public final void scale(int i, double d) {
        switch (i) {
        case 0: d0 *= d; break;
        case 1: d1 *= d; break;
        case 2: d2 *= d; break;
        default: throw new IllegalArgumentException("i=" + i);
        }
	}

	public final void add(int i, double d) {
        switch (i) {
        case 0: d0 += d; break;
        case 1: d1 += d; break;
        case 2: d2 += d; break;
        default: throw new IllegalArgumentException("i=" + i);
        }
	}
	
	/**
	 * Calculates the dot product of this vector with the specified column 
	 * of the given Matrix.
	 * @param m
	 * @param col
	 */
	@Override
	public final double dotCol(DMatrix3C m, int col) {
		if (col == 0) {
			return get0()*m.get00() + get1()*m.get10() + get2()*m.get20();
		} else if (col == 1) {
			return get0()*m.get01() + get1()*m.get11() + get2()*m.get21();
		} else if (col == 2) {
			return get0()*m.get02() + get1()*m.get12() + get2()*m.get22();
		} else {
			throw new IllegalArgumentException("col="+col);
		}
	}
	
	
	/**
	 * Any implementation of DVector3I will return true if get0(), get1()
	 * and get2() return the same values.
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (!(obj instanceof DVector3I)) return false;
		DVector3I v = (DVector3I) obj;
		return get0()==v.get0() && get1()==v.get1() && get2()==v.get2();
	}
	
	@Override
	public int hashCode() {
		return (int) (Double.doubleToRawLongBits(get0())  * 
		Double.doubleToRawLongBits(get1()) * 
		Double.doubleToRawLongBits(get2()));
	}

	/** 
	 * Scales the first parameter with the second and then adds the 
	 * result to the current vector.
	 */
	public final DVector3 addScaled(DVector3C v2, double d) {
		d0 += v2.get0()*d; d1 += v2.get1()*d; d2 += v2.get2()*d;
		return this;
	}

	/**
	 * Set this vector = b x c.
	 * @param b
	 * @param c
	 */
	public final void eqCross(DVector3C b, DVector3C c) {
		set0( b.get1()*c.get2() - b.get2()*c.get1() ); 
		set1( b.get2()*c.get0() - b.get0()*c.get2() ); 
		set2( b.get0()*c.get1() - b.get1()*c.get0() );
	}

	/** 
	 * Calculates the ordinary matrix product for a 3x3 Matrix and a 3-Vector.
	 * <pre>
	 * a0 = m00*v0 + m01*v1 + m02*v2 
	 * a1 = m10*v0 + m11*v1 + m12*v2 
	 * a2 = m20*v0 + m21*v1 + m22*v2
	 * </pre> 
	 */
	public final void eqProd(DMatrix3C m, DVector3C v2) {
	    set0( m.get00()*v2.get0()+  m.get01()*v2.get1()+  m.get02()*v2.get2() );
	    set1( m.get10()*v2.get0()+  m.get11()*v2.get1()+  m.get12()*v2.get2() );
	    set2( m.get20()*v2.get0()+  m.get21()*v2.get1()+  m.get22()*v2.get2() );
	}
}


