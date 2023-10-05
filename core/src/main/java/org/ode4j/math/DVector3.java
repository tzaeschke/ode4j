/*************************************************************************
 *                                                                       *
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
import org.ode4j.ode.OdeMath;

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
public class DVector3 implements DVector3C {

	private double d0, d1, d2;
    public static final DVector3C ZERO = new DVector3();

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
	 * @param v2 v2
	 */
	public DVector3(DVector3C v2) {
		this();
		set(v2);
	}
	
    /**
     * Creates a vector initialised with the first three values of v2.
     * @param v2 v2
     */
	public DVector3(double[] v2) {
		this();
		set(v2);
	}
	
    /**
     * Creates a (i,j,k) vector.
     * @param i value  at position 0
     * @param j value  at position 1
     * @param k value  at position 2
     */
	public DVector3(double i, double j, double k) {
		this();
		set(i, j, k);
	}

	public static DVector3 fromDoubleArray(double[] a) {
		return new DVector3(a);
	}

	public static DVector3 fromFloatArray(float[] a) {
		return new DVector3(a[0], a[1], a[2]);
	}

	public final DVector3 set(double[] v2) {
		set(v2[0], v2[1], v2[2]);
		return this;
	}

	public DVector3 set(float[] v2) {
		set(v2[0], v2[1], v2[2]);
		return this;
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
	@Deprecated // See interface
	public DVector3 clone() {
		return new DVector3(this);
	}

	@Override
	public DVector3 copy() {
		return new DVector3(this);
	}

	@Override
	public String toString() {
		return "DVector3[ " +
				get0() + ", " +
				get1() + ", " +
				get2() + " ]";
	}

	public final DVector3 set0(double d) {
		d0 = d;
		return this;
	}
	
	public final DVector3 set1(double d) {
		d1 = d;
		return this;
	}
	
	public final DVector3 set2(double d) {
		d2 = d;
		return this;
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
	 * @param v2 v2
	 * @return Current vector.
	 */
	public final DVector3 add(DVector3C v2) {
		d0 += v2.get0(); d1 += v2.get1(); d2 += v2.get2();
		return this;
	}
	
	/**
	 * Sets current vector = v2 + v3.
	 * @param v2 v2
	 * @param v3 v3 
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
	 * @param v2 v2
	 * @param s2 s2
	 * @param v3 v3
	 * @param s3 s3
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
	 * @param v2 v2
	 * @param s2 s2
	 * @param v3 v3
	 * @param s3 s3
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
	 * @param v2 v2
	 * @param s2 s2
	 * @param v3 v3
	 * @param s3 s3
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
	 * @param v2 v2
	 * @param v3 v3
	 * @param s3 s3
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
	 * @param v2 v2
	 * @param v3 v3
	 * @param s3 s3
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
	 * @param b b
	 * @return (this) * b
	 */
	@Override
	public final double dot(DVector3C b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2();
	}
	
	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2;
	 * @param b b
	 * @return (this) * b
	 */
	@Override
	public final double dot(DVector3View b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2();
	}
	
	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2;
	 * @param da vector a
	 * @param pos offset in a
	 * @return (this) * b
	 */
	public final double dot(double[] da, int pos) {
		return get0()*da[pos] + get1()*da[pos+1] + get2()*da[pos+2];
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
	 * @param v2 v2
	 * @param v3 v3 
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
	 * @param v2 v2
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
	 * <p>
	 * This method does not modify the vector if no normal can be determined.
	 * @return 'false' if no normal could be determined
	 */
	public final boolean safeNormalize () {
		return OdeMath.dxSafeNormalize3(this);
	}

	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 * <p>
	 * This method throws an IllegalArgumentException if no normal can be determined.
	 * @return This vector.
	 */
	public final DVector3 normalize() {
		OdeMath.dNormalize3(this);
		return this;
	}

	/**
	 * Distance between this vector and a.
	 * @param a a
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
	 * @param a other vector
	 * @param eps maximum allowed difference per value
	 * @return equality
	 */
	@Override
	public final boolean isEq(DVector3C a, double eps) {
		return Math.abs(get0() - a.get0()) <= eps
				&& Math.abs(get1() - a.get1()) <= eps
				&& Math.abs(get2() - a.get2()) <= eps;
	}

	/**
	 * Check whether two vectors contains the same values.
	 * Due to Java's polymorphism handling, this method can be much faster than
	 * v.equals(a).
	 * @param x other vector x
	 * @param y other vector y
	 * @param z other vector z
	 * @param eps maximum allowed difference per value
	 * @return quality
	 */
	@Override
	public final boolean isEq(double x, double y, double z, double eps) {
		return Math.abs(get0() - x) <= eps && Math.abs(get1() - y) <= eps && Math.abs(get2() - z) <= eps;
	}

	/**
	 * Check whether two vectors contains the same values.
	 * Due to Java's polymorphism handling, this method can be much faster than
	 * v.equals(a).
	 * @param a a
	 * @return quality
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public final boolean isEq(DVector3C a) {
		return get0()==a.get0() && get1()==a.get1() && get2()==a.get2();
	}

	/**
	 * Set this vector equal to abs(this).
	 * @return This vector.
	 */
	public final DVector3 eqAbs() {
		set0( Math.abs(get0()));
		set1( Math.abs(get1()));
		set2( Math.abs(get2()));
		return this;
	}

	public final DVector3 add0(double d) {
		d0 += d;
		return this;
	}

	public final DVector3 add1(double d) {
		d1 += d;
		return this;
	}

	public final DVector3 add2(double d) {
		d2 += d;
		return this;
	}

	/**
	 * 
	 * @return '3'.
	 */
	public final int dim() {
		return 3;
	}

	/**
	 * Returns a new vector with the sum of (this)+c.
	 * @param c c
	 * @return new vector
	 */
	public final DVector3 reAdd(DVector3C c) {
		return new DVector3(this).add(c);
	}
	public final DVector3 reAdd(double x, double y, double z) {
		return new DVector3(this).add(x, y, z);
	}

    /**
     * Returns a new vector which equals (this)*d.
     * @param d d
     * @return new vector
     */
	@Override
	public final DVector3 reScale(double d) {
		return new DVector3(this).scale(d);
	}

	public final DVector3 eqZero() {
		set(0, 0, 0);
		return this;
	}

	public final DVector3 setZero() {
		eqZero();
		return this;
	}
	
	public final DVector3 eqIdentity() {
		set(1, 0, 0);
		return this;
	}

	public final DVector3 setIdentity() {
		eqIdentity();
		return this;
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
	public final double[] toDoubleArray() {
		return new double[]{get0(), get1(), get2()};
	}

	@Override
	public final float[] toFloatArray() {
		return new float[]{(float) get0(), (float) get1(), (float) get2()};
	}

	public final DVector3 set(int i, double d) {
        switch (i) {
        case 0: d0 = d; break;
        case 1: d1 = d; break;
        case 2: d2 = d; break;
        default: throw new IllegalArgumentException("i=" + i);
        }
		return this;
	}

	public final DVector3 scale(int i, double d) {
        switch (i) {
        case 0: d0 *= d; break;
        case 1: d1 *= d; break;
        case 2: d2 *= d; break;
        default: throw new IllegalArgumentException("i=" + i);
        }
		return this;
	}

	public final DVector3 add(int i, double d) {
        switch (i) {
        case 0: d0 += d; break;
        case 1: d1 += d; break;
        case 2: d2 += d; break;
        default: throw new IllegalArgumentException("i=" + i);
        }
		return this;
	}
	
	/**
	 * Calculates the dot product of this vector with the specified column 
	 * of the given Matrix.
	 * @param m m
	 * @param col col
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
	 * Do not use. This can be slow, use isEq() instead.
	 * <p>
	 * Any implementation of DVector3I will return true if get0(), get1()
	 * and get2() return the same values.
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (!(obj instanceof DVector3I)) return false;
		DVector3I v = (DVector3I) obj;
		return get0()==v.get0() && get1()==v.get1() && get2()==v.get2();
	}
	
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public int hashCode() {
		return (int) (Double.doubleToRawLongBits(get0())  * 
		Double.doubleToRawLongBits(get1()) * 
		Double.doubleToRawLongBits(get2()));
	}

	/** 
	 * Scales the first parameter with the second and then adds the 
	 * result to the current vector.
	 * @param v2 Vector
	 * @param d scale
	 * @return this + v2*d
	 */
	public final DVector3 addScaled(DVector3C v2, double d) {
		d0 += v2.get0()*d; d1 += v2.get1()*d; d2 += v2.get2()*d;
		return this;
	}

	/**
	 * Set this vector = b x c.
	 * @param b b
	 * @param c c
	 * @return This vector.
	 */
	public final DVector3 eqCross(DVector3C b, DVector3C c) {
		set0( b.get1()*c.get2() - b.get2()*c.get1() );
		set1( b.get2()*c.get0() - b.get0()*c.get2() );
		set2( b.get0()*c.get1() - b.get1()*c.get0() );
		return this;
	}

	/**
	 * Create new vector v = (this) x b.
	 * @param b Other vector
	 * @return cross product of (this) and b
	 * @see DVector3#cross(DVector3C)
	 */
	@Override
	public final DVector3 cross(DVector3C b) {
		DVector3 a = new DVector3();
		a.set0( get1()*b.get2() - get2()*b.get1() );
		a.set1( get2()*b.get0() - get0()*b.get2() );
		a.set2( get0()*b.get1() - get1()*b.get0() );
		return a;
	}

	/** 
	 * Calculates the ordinary matrix product for a 3x3 Matrix and a 3-Vector.
	 * <pre>
	 * a0 = m00*v0 + m01*v1 + m02*v2 
	 * a1 = m10*v0 + m11*v1 + m12*v2 
	 * a2 = m20*v0 + m21*v1 + m22*v2
	 * </pre> 
	 * @param m matrix m
	 * @param v2 vector v
	 * @return This vector.
	 */
	public final DVector3 eqProd(DMatrix3C m, DVector3C v2) {
	    set0( m.get00()*v2.get0()+  m.get01()*v2.get1()+  m.get02()*v2.get2() );
	    set1( m.get10()*v2.get0()+  m.get11()*v2.get1()+  m.get12()*v2.get2() );
	    set2( m.get20()*v2.get0()+  m.get21()*v2.get1()+  m.get22()*v2.get2() );
		return this;
	}

	/**
	 * Convert radians to degrees.
	 * @return (this) vector
	 */
	public DVector3 eqToDegrees() {
		set0( Math.toDegrees(get0()));
		set1( Math.toDegrees(get1()));
		set2( Math.toDegrees(get2()));
		return this;
	}

	/**
	 * Convert degrees to radians.
	 * @return (this) vector
	 */
	public DVector3 eqToRadians() {
		set0( Math.toRadians( get0() ) );
		set1( Math.toRadians( get1() ) );
		set2( Math.toRadians( get2() ) );
		return this;
	}

	/**
	 * Create an array of DVector instances.
	 * @param size size of new array
	 * @return AN array of DVector
	 */
	public static DVector3[] newArray(int size) {
		DVector3[] a = new DVector3[size];
		for (int i = 0; i < size; i++) {
			a[i] = new DVector3();
		}
		return a;
	}
}


