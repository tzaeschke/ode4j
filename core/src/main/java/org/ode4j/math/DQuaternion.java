/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2023 Tilmann Zaeschke     *
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


import static org.ode4j.ode.internal.Common.M_PI;

/**
 * A quaternion consists of four numbers, [w, x, y, z].
 * They are used top represent rigid body orientations.
 *
 * @author Tilmann Zaeschke
 */
public class DQuaternion implements DQuaternionC {

	private double w;
	private double x;
	private double y;
	private double z;
	public static final int LEN = 4;

	public static final DQuaternionC ZERO = new DQuaternion();
	public static final DQuaternionC IDENTITY = new DQuaternion(1, 0, 0, 0);

	public DQuaternion() {
		// Nothing
	}

	public DQuaternion(double w, double x, double y, double z) {
		this();
		set(w, x, y, z);
	}

	public DQuaternion(DQuaternionC x) {
		this();
		set(x);
	}

	@Override
	public String toString() {
		return "DQuaternion[ " +
				get0() + ", " +
				get1() + ", " +
				get2() + ", " +
				get3() + " ]";
	}

	public DQuaternion set(double w, double x, double y, double z) {
		this.w = w;
		this.x = x;
		this.y = y;
		this.z = z;
		return this;
	}

	public DQuaternion set(DQuaternionC q) {
		w = q.get0();
		x = q.get1();
		y = q.get2();
		z = q.get3();
		return this;
	}

	public DQuaternion scale(double d) {
		w *= d;
		x *= d;
		y *= d;
		z *= d;
		return this;
	}

	public DQuaternion add(DQuaternion q) {
		w += q.get0();
		x += q.get1();
		y += q.get2();
		z += q.get3();
		return this;
	}

	@Override
	public double get0() {
		return w;
	}

	@Override
	public double get1() {
		return x;
	}

	@Override
	public double get2() {
		return y;
	}

	@Override
	public double get3() {
		return z;
	}

	public int dim() {
		return LEN;
	}

	/**
	 * Sets w of [w, x, y, z].
	 * @param w w
	 * @return This quaternion.
	 */
	public DQuaternion set0(double w) {
		this.w = w;
		return this;
	}

	/**
	 * Sets x of [w, x, y, z].
	 * @param x x
	 * @return This quaternion.
	 */
	public DQuaternion set1(double x) {
		this.x = x;
		return this;
	}

	/**
	 * Sets y of [w, x, y, z].
	 * @param y y
	 * @return This quaternion.
	 */
	public DQuaternion set2(double y) {
		this.y = y;
		return this;
	}

	/**
	 * Sets z of [w, x, y, z].
	 * @param z z
	 * @return This quaternion.
	 */
	public DQuaternion set3(double z) {
		this.z = z;
		return this;
	}

	@Override
	public boolean isEq(DQuaternion q, double epsilon) {
		return Math.abs(w - q.w) <= epsilon
				&& Math.abs(x - q.x) <= epsilon
				&& Math.abs(y - q.y) <= epsilon
				&& Math.abs(z - q.z) <= epsilon;
	}

	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean isEq(DQuaternion q) {
		return w == q.w && x == q.x && y == q.y && z == q.z;
	}

	/**
	 * Do not use. This can be slow, use isEq() instead.
	 * @param obj object
	 * @return true if equal
	 * @deprecated
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (!(obj instanceof DQuaternion)) return false;
		return isEq((DQuaternion)obj);
	}

	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public int hashCode() {
		int h = 0;
		h |= Double.doubleToRawLongBits(w);
		h <<= 6;
		h |= Double.doubleToRawLongBits(x);
		h <<= 6;
		h |= Double.doubleToRawLongBits(y);
		h <<= 6;
		h |= Double.doubleToRawLongBits(z);
		h <<= 6;
		return h;
	}

	@Override
	public DQuaternion copy() {
		return new DQuaternion(this);
	}

	public DQuaternion add(double dw, double dx, double dy, double dz) {
		w += dw;
		x += dx;
		y += dy;
		z += dz;
		return this;
	}

	public final DQuaternion sum(DQuaternion q1, DQuaternion q2, double d2) {
		w = q1.get0() + q2.get0() * d2;
		x = q1.get1() + q2.get1() * d2;
		y = q1.get2() + q2.get2() * d2;
		z = q1.get3() + q2.get3() * d2;
		return this;
	}

	/**
	 * Set a vector/matrix at position i to a specific value.
	 * @param i position
	 * @param d value
	 * @return This quaternion.
	 */
	public final DQuaternion set(int i, double d) {
		switch (i) {
			case 0: w = d; break;
			case 1: x = d; break;
			case 2: y = d; break;
			case 3: z = d; break;
			default:
				throw new ArrayIndexOutOfBoundsException("i=" + i);
		}
		return this;
	}

	public final DQuaternion setZero() {
		set(0, 0, 0, 0);
		return this;
	}

	public final DQuaternion scale(int i, double l) {
		switch (i) {
			case 0: w *= l; break;
			case 1: x *= l; break;
			case 2: y *= l; break;
			case 3: z *= l; break;
			default:
				throw new ArrayIndexOutOfBoundsException("i=" + i);
		}
		return this;
	}

	public final double lengthSquared() {
		return w * w + x * x + y * y + z * z;
	}

	@Override
	public final double get(int i) {
		switch (i) {
			case 0: return w;
			case 1: return x;
			case 2: return y;
			case 3: return z;
			default:
				throw new ArrayIndexOutOfBoundsException("i=" + i);
		}
	}

	public final double length() {
		return Math.sqrt(lengthSquared());
	}

	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 * @return 'false' if normnalization failed (returns unit vector)
	 */
	public final boolean safeNormalize4 () {
		double d = Math.abs(get0());
//		for (int i = 1; i < v.length; i++) {
//			if (Math.abs(v[i]) > d) {
//				d = Math.abs(v[i]);
//			}
//		}
		if (Math.abs(get1()) > d) d = Math.abs(get1());
		if (Math.abs(get2()) > d) d = Math.abs(get2());
		if (Math.abs(get3()) > d) d = Math.abs(get3());

		if (d <= Double.MIN_NORMAL) {
			set(1, 0, 0, 0);
			return false;
		}

		scale(1/d);
//		for (int i = 0; i < v.length; i++) {
//			v[i] /= d;
//		}

//		double sum = 0;
//		for (double d2: v) {
//			sum += d2*d2;
//		}

		double l = 1./length();//Math.sqrt(sum);
//		for (int i = 0; i < v.length; i++) {
//			v[i] *= l;
//		}
		scale(l);
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
	 * @return This quaternion.
	 */
	public DQuaternion normalize() {
		if (!safeNormalize4()) throw new IllegalStateException(
				"Normalization failed: " + this);
		return this;
	}

	public DQuaternion setIdentity() {
		set(1, 0, 0, 0);
		return this;
	}

	public boolean isZero() {
		return w == 0 && x == 0 && y == 0 && z == 0;
	}

	/**
	 * @param roll  radians roll
	 * @param pitch radians pitch
	 * @param yaw   radians yaw
	 * @return quaternion
	 */
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	public static DQuaternion fromEuler(double roll, double pitch, double yaw) {
		double cr = Math.cos(roll * 0.5);
		double sr = Math.sin(roll * 0.5);
		double cp = Math.cos(pitch * 0.5);
		double sp = Math.sin(pitch * 0.5);
		double cy = Math.cos(yaw * 0.5);
		double sy = Math.sin(yaw * 0.5);

		DQuaternion q = new DQuaternion();
		q.w = cr * cp * cy + sr * sp * sy;
		q.x = sr * cp * cy - cr * sp * sy;
		q.y = cr * sp * cy + sr * cp * sy;
		q.z = cr * cp * sy - sr * sp * cy;
		return q;
	}

	/**
	 * @param angles roll, pitch and yaw as radians.
	 * @return Quaternion
	 */
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	public static DQuaternion fromEuler(DVector3C angles) {
		return fromEuler(angles.get0(), angles.get1(), angles.get2());
	}

	/**
	 * @param roll  roll degrees
	 * @param pitch pitch degrees
	 * @param yaw   yaw degrees
	 * @return quaternion
	 */
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	public static DQuaternion fromEulerDegrees(double roll, double pitch, double yaw) {
		return fromEuler(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw));
	}

	/**
	 * @param angles roll, pitch and yaw as degrees, i.e. 0..360.
	 * @return Quaternion
	 */
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	public static DQuaternion fromEulerDegrees(DVector3C angles) {
		return fromEulerDegrees(angles.get0(), angles.get1(), angles.get2());
	}

	@Override
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	public DVector3 toEuler() {
		DVector3 angles = new DVector3();
		DQuaternion q = new DQuaternion(this);
		q.safeNormalize4();

		// roll (x-axis rotation)
		double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
		double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
		angles.set0(Math.atan2(sinr_cosp, cosr_cosp));

		// pitch (y-axis rotation)
		double sinp = Math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
		double cosp = Math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
		angles.set1(2 * Math.atan2(sinp, cosp) - M_PI / 2);

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
		angles.set2(Math.atan2(siny_cosp, cosy_cosp));

		return angles;
	}

	@Override
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	public DVector3 toEulerDegrees() {
		return toEuler().eqToDegrees();
	}

	/**
	 * Calculates the inverse of the quaternion.
	 * @return the inverted quaternion
	 */
	public DQuaternion eqInverse() {
		double norm = lengthSquared();
		if (norm > 0.0) {
			double invNorm = 1.0 / norm;
			w *= invNorm;
			x *= -invNorm;
			y *= -invNorm;
			z *= -invNorm;
		}
		return this;
	}

	/**
	 * Calculates the inverse of the quaternion and returns it as a new quaternion.
	 * @return an inverted quaternion
	 */
	@Override
	public DQuaternion reInverse() {
		DQuaternion ret = new DQuaternion(this);
		ret.eqInverse();
		return ret;
	}

	/**
	 * Return the 'dot' product of two quaternions.
	 * r = a0*b0 + a1*b1 + a2*b2 + a3*b3;
	 * @param b b
	 * @return (this) * b
	 */
	@Override
	public double dot(DQuaternionC b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2() + get3()*b.get3();
	}
}
