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


import java.util.Arrays;

/**
 * 3x3 matrix class.
 * Internally this uses a 4x3 matrix for compatibility.
 *
 * @author Tilmann Zaeschke
 */
public final class DMatrix3 implements DMatrix3C {
	
	private final double[] v;
	public static final int MAX_I = 3;
	public static final int MAX_J = 4;
	public static final int LEN = MAX_I*MAX_J;
	public static final DMatrix3C ZERO = new DMatrix3();

	/** 
	 * 
	 * @param d row 0 column 0 
	 * @param e row 0 column 1 
	 * @param f row 0 column 2
	 * @param g row 0 column 3 (must be 0) 
	 * @param h row 1 column 0
	 * @param i row 1 column 1
	 * @param j row 1 column 2
	 * @param k row 1 column 3 (must be 0)
	 * @param l row 2 column 0
	 * @param m row 2 column 1
	 * @param n row 2 column 2
	 * @param o row 2 column 3
	 * @deprecated 
	 */
	@Deprecated
    public DMatrix3(double d, double e, double f,
                    double g, double h, double i, double j, double k, double l,
                    double m, double n, double o) {
		this();
		v[0] = d; v[1] = e; v[2] = f; v[3] = g;
		v[4] = h; v[5] = i; v[6] = j; v[7] = k;
		v[8] = l; v[9] = m; v[10] = n; v[11] = o;
		if (g!=0 || k!=0 || o!=0) {
			System.err.println("Warning: 4th column in dMatrix3 != 0 !");
		}
	}
	
	
	/**
	 * Create new Matrix of the form:
	 * <pre>
	 * d, e, f
	 * g, h, i
	 * j, k, l
	 * </pre>
	 * @param d 0,0
	 * @param e 0,1
	 * @param f 0,2
	 * @param g 1,0
	 * @param h 1,1
	 * @param i 1,2
	 * @param j 2,0
	 * @param k 2,1
	 * @param l 2,2
	 */
	public DMatrix3(double d, double e, double f, double g, double h, double i,
			double j, double k, double l) {
		this();
		v[0] = d; v[1] = e; v[2] = f; 
		v[4] = g; v[5] = h; v[6] = i; 
		v[8] = j; v[9] = k; v[10] = l; 
	}


	public DMatrix3(DMatrix3C matrix3) {
		this();
		set(matrix3);
	}
	
	
	public DMatrix3() {
		v = new double[MAX_I * MAX_J];
	}
	

	/**
	 * Private to enforce usage of wrap().
	 * @param a other array
	 */
	private DMatrix3(double[] a) {
		v = a;
	}
	
	
	public static DMatrix3 wrap(double[] a) {
		return new DMatrix3(a);
	}
	
	
	public DMatrix3 set(DMatrix3C m) {
//		System.arraycopy(((DMatrix3)m3).v, 0, v, 0, v.length);
//		//v[0] = v3.v[0]; v[1] = v3.v[1]; v[2] = v3.v[2]; v[3] = v3.v[3];
		set00( m.get00() ); set01( m.get01() ); set02( m.get02() );
		set10( m.get10() ); set11( m.get11() ); set12( m.get12() );
		set20( m.get20() ); set21( m.get21() ); set22( m.get22() );
		return this;
	}


	/**
	 * Returns a clone of this Matrix.
	 */
	@Override
	@Deprecated
	public DMatrix3 clone() {
		return new DMatrix3(this);
	}

	/**
	 * Returns a copy of this Matrix.
	 */
	@Override
	public DMatrix3 copy() {
		return new DMatrix3(this);
	}

	
	@Override
	public String toString() {
//		StringBuffer b = new StringBuffer();
//		b.append("DMatrix3[");
//		for (int i = 0; i < v.length-1; i++) {
//			b.append(v[i]).append(", ");
//		}
//		b.append(v[v.length-1]).append("]");
//		return b.toString();
		StringBuilder b = new StringBuilder();
		b.append("DMatrix3[[");
		b.append(get00()).append(", ");
		b.append(get01()).append(", ");
		b.append(get02()).append("], [");
		b.append(get10()).append(", ");
		b.append(get11()).append(", ");
		b.append(get12()).append("], [");
		b.append(get20()).append(", ");
		b.append(get21()).append(", ");
		b.append(get22()).append("]]");
		return b.toString();
	}

	public DMatrix3 setOfs(int ofs, DVector3 v3) {
		v[ofs] = v3.get0(); v[ofs+1] = v3.get1(); v[ofs+2] = v3.get2(); //v[ofs+3] = 0;//v3.v[3];
		return this;
	}

	public DMatrix3 setCol(int i, DVector3 v3) {
		int ofs = i*4;
		v[ofs] = v3.get0(); v[ofs+1] = v3.get1(); v[ofs+2] = v3.get2(); //v[ofs+3] = 0;//v3.v[3];
		return this;
	}

	public DMatrix3 set(double i, double j, double k, double l, double m,
			double n, double o, double p, double q) {
//		v[0] = i; v[1] = j; v[2] = k;
//		v[4] = l; v[5] = m; v[6] = n;
//		v[8] = o; v[9] = p; v[10] = q;
		set00( i ); set01( j ); set02( k );
		set10( l ); set11( m ); set12( n );
		set20( o ); set21( p ); set22( q );
		return this;
	}

	/**
	 * Initialises this matrix from a 3*4 double [] with 12 fields, ignoring 
	 * the 4th, 8th and 12th field. This is useful when using padded arrays.
	 * @param da Initialisztion matrix
	 * @param da_ofs Reading offset
	 * @return this.
	 */
	public DMatrix3 set12(double[] da, int da_ofs) {
		System.arraycopy(da, da_ofs, v, 0, da.length);
		return this;
	}
	
	public double get(int i) {
		return v[i];
	}
	
	/**
	 * Return a new dVector containing the specified column.
	 * For padding=4 this uses the elements c, 4+c, 8+c;
	 * @param c Column (0, 1, 2)
	 * @return A new dVector.
	 */
	@Override
	public DVector3 columnAsNewVector(int c) {
//		return new dVector3(get(0, c), get(1, c), get(2, c));
		return new DVector3(v[c], v[c + MAX_J], v[c + 2*MAX_J]);
	}


	public DMatrix3 add(DMatrix3C m) {
		set00( get00() + m.get00() ); set01( get01() + m.get01() ); set02( get02() + m.get02() );
		set10( get10() + m.get10() ); set11( get11() + m.get11() ); set12( get12() + m.get12() );
		set20( get20() + m.get20() ); set21( get21() + m.get21() ); set22( get22() + m.get22() );
//		DMatrix3 M = (DMatrix3) M2; 
//		for (int i = 0; i < v.length; i++) {
//			v[i] += M.v[i];
//		}
		return this;
	}


	public DMatrix3 scale(double scale) {
		for (int i = 0; i < v.length; i++) {
			v[i] *= scale;
		}
		return this;
	}
	
	/** 
	 * Matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed:
	 *   0:   A = B  * C   (sizes: A:p*r B:p*q C:q*r)
	 *   1:   A = B' * C   (sizes: A:p*r B:q*p C:q*r)
	 *   2:   A = B  * C'  (sizes: A:p*r B:p*q C:r*q)
	 * case 1,2 are equivalent to saying that the operation is A=B*C but
	 * B or C are stored in standard column format.
	 * @param B source B
	 * @param C source C
	 * @return this.
	 */
	public DMatrix3 dMultiply0 (final DMatrix3C B,
			final DMatrix3C C) {
		eqMul(B, C);
		return this;
	}

	public DMatrix3 eqMul (final DMatrix3C B,
			final DMatrix3C C)
	{
//		dMatrix3 B2 = (dMatrix3) B;
//		dMatrix3 C2 = (dMatrix3) C;
//		double sum;
//		int aPos = 0;
//		int bPos, bbPos =0, cPos;
//		for (int i=3; i > 0; i--) {
//			for (int j=0 ; j<3; j++) {
//				cPos = j;
//				bPos = bbPos;
//				sum = 0;
//				for (int k=3; k > 0; k--, cPos+=MAX_J) sum += B2.v[bPos++] * C2.v[cPos];
//				v[aPos++] = sum;
//			}
//			aPos++;
//			bbPos += MAX_J;
//		}
//		set( B.get00()*C.get00() + B.get01()*C.get10() + B.get02()*C.get20(),
//				B.get00()*C.get01() + B.get01()*C.get11() + B.get02()*C.get21(),
//				B.get00()*C.get02() + B.get01()*C.get12() + B.get02()*C.get22(),
//				B.get10()*C.get00() + B.get11()*C.get10() + B.get12()*C.get20(),
//				B.get10()*C.get01() + B.get11()*C.get11() + B.get12()*C.get21(),
//				B.get10()*C.get02() + B.get11()*C.get12() + B.get12()*C.get22(),
//				B.get20()*C.get00() + B.get21()*C.get10() + B.get22()*C.get20(),
//				B.get20()*C.get01() + B.get21()*C.get11() + B.get22()*C.get21(),
//				B.get20()*C.get02() + B.get21()*C.get12() + B.get22()*C.get22());
		set00( B.get00()*C.get00() + B.get01()*C.get10() + B.get02()*C.get20() );
		set01( B.get00()*C.get01() + B.get01()*C.get11() + B.get02()*C.get21() );
		set02( B.get00()*C.get02() + B.get01()*C.get12() + B.get02()*C.get22() );
		set10( B.get10()*C.get00() + B.get11()*C.get10() + B.get12()*C.get20() );
		set11( B.get10()*C.get01() + B.get11()*C.get11() + B.get12()*C.get21() );
		set12( B.get10()*C.get02() + B.get11()*C.get12() + B.get12()*C.get22() );
		set20( B.get20()*C.get00() + B.get21()*C.get10() + B.get22()*C.get20() );
		set21( B.get20()*C.get01() + B.get21()*C.get11() + B.get22()*C.get21() );
		set22( B.get20()*C.get02() + B.get21()*C.get12() + B.get22()*C.get22() );
		return this;
	}

	
	
//	public void dMultiply0 (final dMatrix B, final dMatrix C)
//	{
//		int i,j,k,qskip,rskip,rpad;
//		//COM.dAASSERT (B, C);
//		//COM.dAASSERT(p>0 && q>0 && r>0);
//		qskip = COM.dPAD(q);
//		rskip = COM.dPAD(r);
//		rpad = rskip - r;
//		double sum;
//		int aPos = 0;
//		//final double[] b,c,bb;
//		int bPos, bbPos =0, cPos;
//		//TZ? final double bb;
//		//TZ? bb = B;
//		for (i=p; i > 0; i--) {
//			for (j=0 ; j<r; j++) {
//				//c = C + j;
//				cPos = j;
//				//b = bb;
//				bPos = bbPos;
//				sum = 0;
//				//for (k=q; k > 0; k--, cPos+=rskip) sum += (*(b++))*(*c);
//				for (k=q; k > 0; k--, cPos+=rskip) sum += B.v[bPos++] * C.v[cPos];
//				//*(A++) = sum; 
//				v[aPos++] = sum;
//			}
//			//			A += rpad;
//			//			bb += qskip;
//			aPos += rpad;
//			//bb += qskip;
//			bbPos += qskip;
//		}
//	}
//
//
//	/** 
//	 * Matrix multiplication. all matrices are stored in standard row format.
//	 * the digit refers to the argument that is transposed:
//	 *   0:   A = B  * C   (sizes: A:p*r B:p*q C:q*r)
//	 *   1:   A = B' * C   (sizes: A:p*r B:q*p C:q*r)
//	 *   2:   A = B  * C'  (sizes: A:p*r B:p*q C:r*q)
//	 * case 1,2 are equivalent to saying that the operation is A=B*C but
//	 * B or C are stored in standard column format.
//	 */
//	public void dMultiply1 (final dMatrix B, final dMatrix C)
//	{
//		int i,j,k,pskip,rskip;
//		double sum;
////		COM.dAASSERT (A , B, C);
////		COM.dAASSERT(p>0 && q>0 && r>0);
//		pskip = COM.dPAD(p);
//		rskip = COM.dPAD(r);
//		for (i=0; i<p; i++) {
//			for (j=0; j<r; j++) {
//				sum = 0;
//				for (k=0; k<q; k++) sum += B.v[i+k*pskip] * C.v[j+k*rskip];
//				v[i*rskip+j] = sum;
//			}
//		}
//	}
//
//
//	/** 
//	 * Matrix multiplication. all matrices are stored in standard row format.
//	 * the digit refers to the argument that is transposed:
//	 *   0:   A = B  * C   (sizes: A:p*r B:p*q C:q*r)
//	 *   1:   A = B' * C   (sizes: A:p*r B:q*p C:q*r)
//	 *   2:   A = B  * C'  (sizes: A:p*r B:p*q C:r*q)
//	 * case 1,2 are equivalent to saying that the operation is A=B*C but
//	 * B or C are stored in standard column format.
//	 */
//	public void dMultiply2 (final dMatrix B, final dMatrix C)
//	{
//		int i,j,k,z,rpad,qskip;
//		double sum;
//		//final double[] bb,cc;
//		//TZ:
//		int aPos = 0, bPos, cPos;
////		COM.dAASSERT (A, B , C);
////		COM.dAASSERT(p>0 && q>0 && r>0);
//		rpad = COM.dPAD(r) - r;
//		qskip = COM.dPAD(q);
//		//bb = B;
//		bPos = 0;
//		for (i=p; i>0; i--) {
//			//cc = C;
//			cPos = 0;
//			for (j=r; j>0; j--) {
//				z = 0;
//				sum = 0;
//				//for (k=q; k>0; k--,z++) sum += bb[z] * cc[z];
//				for (k=q; k>0; k--,z++) sum += B.v[bPos + z] * C.v[cPos + z];
//				//*(A++) = sum; 
//				v[aPos++] = sum; 
//				//cc += qskip;
//				cPos += qskip;
//			}
//			//A += rpad;
//			aPos += rpad;
//			//bb += qskip;
//			bPos += qskip;
//		}
//	}
	
	
	/**
	 * View a particular column as dVector3. For example: <br><tt>
	 * //  dir[0] = R[0*4+2];  <br>
	 * //  dir[1] = R[1*4+2];  <br>
	 * //  dir[2] = R[2*4+2];  <br>
  	 * dir.set(R.getColumnView(2));
	 * </tt>
	 * @param column The column to return [0, 1, 2].
	 */
	@Override
	public DVector3ColView viewCol(int column) {
		return new DVector3ColView(column);
	}
	

	public DVector3RowTView viewRowT(int row) {
		return new DVector3RowTView(row);
	}

	
	public class DVector3ColView extends DVector3View {
		private final int _column;
		
		public DVector3ColView(int c) {
			_column = c;
		}
		
		@Override
		public double get(int i) {
			return v[i * MAX_J + _column];
		}
		
		@Override
		public double get0() {
			return v[_column];
		}
		
		@Override
		public double get1() {
			return v[1 * MAX_J + _column];
		}
		
		@Override
		public double get2() {
			return v[2 * MAX_J + _column];
		}

		@Override
		public void set0(double d) {
			v[_column] = d;
		}

		@Override
		public void set1(double d) {
			v[1 * MAX_J + _column] = d;
		}

		@Override
		public void set2(double d) {
			v[2 * MAX_J + _column] = d;
		}

		@Override
		public String toString() {
			return "DVector3ColView" + super.toString();
		}
	}

	public class DVector3RowTView extends DVector3View {
		private final int _ofs;
		
		public DVector3RowTView(int row) {
			_ofs = row * MAX_J;
		}
		
		@Override
		public double get(int i) {
			return v[_ofs + i];
		}
		
		@Override
		public double get0() {
			return v[_ofs];
		}
		
		@Override
		public double get1() {
			return v[1 + _ofs];
		}
		
		@Override
		public double get2() {
			return v[2 + _ofs];
		}

		@Override
		public void set0(double d) {
			v[_ofs] = d;
		}

		@Override
		public void set1(double d) {
			v[_ofs + 1] = d;
		}

		@Override
		public void set2(double d) {
			v[_ofs + 2] = d;
		}

		@Override
		public String toString() {
			return "DVector3RowTView" + super.toString();
		}
	}

	/**
	 * @return value at row 0, column 0 ([0]).
	 */
	@Override
	public final double get00() {
		return v[0];
	}


	/**
	 * @return value at row 0, column 1 ([1]).
	 */
	@Override
	public final double get01() {
		return v[1];
	}


	/**
	 * @return value at row 0, column 2 ([2]).
	 */
	@Override
	public final double get02() {
		return v[2];
	}


	/**
	 * @return value at row 1, column 0 ([4]).
	 */
	@Override
	public final double get10() {
		return v[1*MAX_J + 0];
	}


	/**
	 * @return value at row 1, column 1 ([5]).
	 */
	@Override
	public final double get11() {
		return v[1*MAX_J + 1];
	}


	/**
	 * @return value at row 1, column 2 ([6]).
	 */
	@Override
	public final double get12() {
		return v[1*MAX_J + 2];
	}


	/**
	 * @return value at row 2, column 0 ([8]).
	 */
	@Override
	public final double get20() {
		return v[2*MAX_J + 0];
	}


	/**
	 * @return value at row 2, column 1 ([9]).
	 */
	@Override
	public final double get21() {
		return v[2*MAX_J + 1];
	}


	/**
	 * @return value at row 2, column 2 ([10]).
	 */
	@Override
	public final double get22() {
		return v[2*MAX_J + 2];
	}

	
	public DMatrix3 set00(double d) {
		v[0] = d;
		return this;
	}


	public DMatrix3 set01(double d) {
		v[1] = d;
		return this;
	}


	public DMatrix3 set02(double d) {
		v[2] = d;
		return this;
	}


	public DMatrix3 set10(double d) {
		v[1*MAX_J + 0] = d;
		return this;
	}


	public DMatrix3 set11(double d) {
		v[1*MAX_J + 1] = d;
		return this;
	}


	public DMatrix3 set12(double d) {
		v[1*MAX_J + 2] = d;
		return this;
	}


	public DMatrix3 set20(double d) {
		v[2*MAX_J + 0] = d;
		return this;
	}


	public DMatrix3 set21(double d) {
		v[2*MAX_J + 1] = d;
		return this;
	}


	public DMatrix3 set22(double d) {
		v[2*MAX_J + 2] = d;
		return this;
	}


	public int dimI() {
		return MAX_I;
	}


	public int dimJ() {
		//TODO MAX_J, once MAX_J==3
		return 3;
	}

	/**
	 * Compares two matrices for equality.
	 * This is marginally faster than <tt>equals(Object o)</tt>.
	 */
	@Override
	public boolean isEq(DMatrix3C m, double epsilon) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (Math.abs(get(i, j) - m.get(i, j)) > epsilon) return false;
			}
		}
		return true;
	}

	/**
	 * Compares two matrices for equality.
	 * This is marginally faster than <tt>equals(Object o)</tt>.
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public final boolean isEq(DMatrix3C m) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (get(i, j) != m.get(i, j)) return false;
			}
		}
		return true;
	}

	/**
	 * Please use isEq() instead.
	 * @param m other matrix
	 * @return 'true' if both matrices are equal
	 */
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public final boolean isEqual(DMatrix3C m) {
		return isEq(m);
	}

	/**
	 * Do not use. This can be slow, use isEquals() instead.
	 * Compares two objects for equality.
	 * This is marginally slower than <tt>isEquals(DMatrix3C m)</tt>.
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public final boolean equals(Object o) {
		if (o == null) {
			return false;
		}
		if (! (o instanceof DMatrix3C)) {
			return false;
		}
		return isEq((DMatrix3C) o);
	}

	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public int hashCode() {
		int h = 0;
		for (double d: v) {
			h |= Double.doubleToRawLongBits(d);
			h <<= 2;
		}
		return h;
	}

	/**
	 * Make the matrix an identity matrix.
	 * Same as setIdenity().
	 * @return This matrix.
	 */
	public DMatrix3 eqIdentity() {
		eqZero();
		set00(1);
		set11(1);
		set22(1);
		return this;
	}


	/**
	 * Make the matrix an identity matrix.
	 * Same as eqIdenity().
	 * @return This matrix.
	 */
	public DMatrix3 setIdentity() {
		eqIdentity();
		return this;
	}

	
	/**
	 * Set the matrix to zero.
	 * Same as setZero().
	 * @return This matrix.
	 */
	public DMatrix3 eqZero() {
		Arrays.fill(v, 0);
		return this;
	}


	/**
	 * Set the matrix to zero.
	 * Same as eqZero().
	 * @return This matrix.
	 */
	public DMatrix3 setZero() {
		eqZero();
		return this;
	}


	@Override
	public final float[] toFloatArray12() {
		return new float[]{ 
				(float)get00(), (float)get01(), (float)get02(), 0.0f,
				(float)get10(), (float)get11(), (float)get12(), 0.0f,
				(float)get20(), (float)get21(), (float)get22(), 0.0f };
	}


	@Override
	public final float[] toFloatArray() {
		return new float[]{ 
				(float)get00(), (float)get01(), (float)get02(),
				(float)get10(), (float)get11(), (float)get12(),
				(float)get20(), (float)get21(), (float)get22() };
	}
	
	
	/**
	 * Transpose this matrix.
	 * @return This matrix.
	 */
	public final DMatrix3 eqTranspose() {
		double t;
		t = get01(); set01( get10() ); set10( t );
		t = get02(); set02( get20() ); set20( t );
		t = get21(); set21( get12() ); set12( t );
		return this;
	}
	
	
	/**
	 * Create a new transposed version of this matrix.
	 * @return The transposed copy of this matrix.
	 */
	@Override
	public final DMatrix3 reTranspose() {
		return new DMatrix3(
				get00(), get10(), get20(),
				get01(), get11(), get21(),
				get02(), get12(), get22());
	}
	
	
	/**
	 * Calculates the dot product of the the specified column of this matrix
	 * with the given vector.
	 * @param col column pos
	 * @param v3 vector
	 */
	@Override
	public final double dotCol(int col, DVector3C v3) {
		if (col == 0) {
			return get00()*v3.get0() + get10()*v3.get1() + get20()*v3.get2();
		} else if (col == 1) {
			return get01()*v3.get0() + get11()*v3.get1() + get21()*v3.get2();
		} else if (col == 2) {
			return get02()*v3.get0() + get12()*v3.get1() + get22()*v3.get2();
		} else {
			throw new IllegalArgumentException("col="+col);
		}
	}

	/**
	 * Calculates the dot product of the the specified row of this matrix
	 * with the given vector.
	 * @param row row pos
	 * @param v3 vector
	 */
	@Override
	public final double dotRow(int row, DVector3C v3) {
		if (row == 0) {
			return get00()*v3.get0() + get01()*v3.get1() + get02()*v3.get2();
		} else if (row == 1) {
			return get10()*v3.get0() + get11()*v3.get1() + get12()*v3.get2();
		} else if (row == 2) {
			return get20()*v3.get0() + get21()*v3.get1() + get22()*v3.get2();
		} else {
			throw new IllegalArgumentException("row="+row);
		}
	}

	/**
	 * Calculates the dot product of the the specified column <tt>col</tt> 
	 * of this matrix with the specified column <tt>col2</tt> of the second 
	 * matrix <tt>m2</tt>.
	 * @param col column pos
	 * @param m2 matrix M2
	 * @param col2 column pos in matrix M2
	 */
	@Override
	public final double dotColCol(int col, DMatrix3C m2, int col2) {
		if (col == 0) {
			if (col2 == 0) {
				return get00()*m2.get00() + get10()*m2.get10() + get20()*m2.get20();
			} else if (col2 == 1) {
				return get00()*m2.get01() + get10()*m2.get11() + get20()*m2.get21();
			} else if (col2 == 2) {
				return get00()*m2.get02() + get10()*m2.get12() + get20()*m2.get22();
			}
		} else if (col == 1) {
			if (col2 == 0) {
				return get01()*m2.get00() + get11()*m2.get10() + get21()*m2.get20();
			} else if (col2 == 1) {
				return get01()*m2.get01() + get11()*m2.get11() + get21()*m2.get21();
			} else if (col2 == 2) {
				return get01()*m2.get02() + get11()*m2.get12() + get21()*m2.get22();
			}
		} else if (col == 2) {
			if (col2 == 0) {
				return get02()*m2.get00() + get12()*m2.get10() + get22()*m2.get20();
			} else if (col2 == 1) {
				return get02()*m2.get01() + get12()*m2.get11() + get22()*m2.get21();
			} else if (col2 == 2) {
				return get02()*m2.get02() + get12()*m2.get12() + get22()*m2.get22();
			}
		} 
		throw new IllegalArgumentException("col="+col+" col2="+col2);
	}


	/**
	 * Calculates the dot product of the the specified row <tt>row</tt> 
	 * of this matrix with the specified column <tt>col2</tt> of the second 
	 * matrix <tt>m2</tt>.
	 * @param row Row pos
	 * @param m2 Matrix M2
	 * @param col2 Col pos in Matrix m2
	 */
	@Override
	public double dotRowCol(int row, DMatrix3C m2, int col2) {
		if (row == 0) {
			if (col2 == 0) {
				return get00()*m2.get00() + get01()*m2.get10() + get02()*m2.get20();
			} else if (col2 == 1) {
				return get00()*m2.get01() + get01()*m2.get11() + get02()*m2.get21();
			} else if (col2 == 2) {
				return get00()*m2.get02() + get01()*m2.get12() + get02()*m2.get22();
			}
		} else if (row == 1) {
			if (col2 == 0) {
				return get10()*m2.get00() + get11()*m2.get10() + get12()*m2.get20();
			} else if (col2 == 1) {
				return get10()*m2.get01() + get11()*m2.get11() + get12()*m2.get21();
			} else if (col2 == 2) {
				return get10()*m2.get02() + get11()*m2.get12() + get12()*m2.get22();
			}
		} else if (row == 2) {
			if (col2 == 0) {
				return get20()*m2.get00() + get21()*m2.get10() + get22()*m2.get20();
			} else if (col2 == 1) {
				return get20()*m2.get01() + get21()*m2.get11() + get22()*m2.get21();
			} else if (col2 == 2) {
				return get20()*m2.get02() + get21()*m2.get12() + get22()*m2.get22();
			}
		} 
		throw new IllegalArgumentException("row="+row+" col2="+col2);
	}


	/**
	 * Calculates the dot product of the the specified row of this matrix
	 * with the given array at the given offset.
	 * @param row 	The row.
	 * @param c 	The array.
	 * @param cOfs The offset in c.
	 */
	@Override
	public final double dotRow(int row, double[] c, int cOfs) {
		if (row == 0) {
			return get00()*c[cOfs+0] + get01()*c[cOfs+1] + get02()*c[cOfs+2];
		} else if (row == 1) {
			return get10()*c[cOfs+0] + get11()*c[cOfs+1] + get12()*c[cOfs+2];
		} else if (row == 2) {
			return get20()*c[cOfs+0] + get21()*c[cOfs+1] + get22()*c[cOfs+2];
		} else {
			throw new IllegalArgumentException("row="+row);
		}
	}


	/**
	 * Calculates the dot product of the the specified row <tt>row</tt> 
	 * of this matrix with the specified row <tt>row2</tt> of the second 
	 * matrix <tt>m2</tt>.
	 * @param row row pos
	 * @param m2 Matrix M2
	 * @param row2 row pos in M2
	 */
	@Override
	public double dotRowRow(int row, DMatrix3C m2, int row2) {
		if (row == 0) {
			if (row2 == 0) {
				return get00()*m2.get00() + get01()*m2.get01() + get02()*m2.get02();
			} else if (row2 == 1) {
				return get00()*m2.get10() + get01()*m2.get11() + get02()*m2.get12();
			} else if (row2 == 2) {
				return get00()*m2.get20() + get01()*m2.get21() + get02()*m2.get22();
			}
		} else if (row == 1) {
			if (row2 == 0) {
				return get10()*m2.get00() + get11()*m2.get01() + get12()*m2.get02();
			} else if (row2 == 1) {
				return get10()*m2.get10() + get11()*m2.get11() + get12()*m2.get12();
			} else if (row2 == 2) {
				return get10()*m2.get20() + get11()*m2.get21() + get12()*m2.get22();
			}
		} else if (row == 2) {
			if (row2 == 0) {
				return get20()*m2.get00() + get21()*m2.get01() + get22()*m2.get02();
			} else if (row2 == 1) {
				return get20()*m2.get10() + get21()*m2.get11() + get22()*m2.get12();
			} else if (row2 == 2) {
				return get20()*m2.get20() + get21()*m2.get21() + get22()*m2.get22();
			}
		} 
		throw new IllegalArgumentException("row="+row+" row2="+row2);
	}


	/**
	 * @param i row 
	 * @param j column
	 * @return Value at (i,j).
	 */
	@Override
	public double get(int i, int j) {
		return v[i*MAX_J + j];
	}

	/**
	 * @param i row
	 * @param j column
	 * @param a value at (i,j)
	 * @return This matrix.
	 */
	public DMatrix3 set(int i, int j, double a) {
		v[i*MAX_J + j] = a;
		return this;
	}


	public DMatrix3 add(int i, int j, double d) {
		v[i*MAX_J + j] += d;
		return this;
	}


	public DMatrix3 sub(int i, int j, double d) {
		v[i*MAX_J + j] -= d;
		return this;
	}


	@Override
	public void getColumn0(DVector3 result) {
		result.set(get00(), get10(), get20());
	}

	@Override
	public void getColumn1(DVector3 result) {
		result.set(get01(), get11(), get21());
	}

	@Override
	public void getColumn2(DVector3 result) {
		result.set(get02(), get12(), get22());
	}
	
	/**
	 * Create an array of DVector instances.
	 * @param size Size of array
	 * @return An array of DVector
	 */
	public static DMatrix3[] newArray(int size) {
		DMatrix3[] a = new DMatrix3[size];
		for (int i = 0; i < size; i++) {
			a[i] = new DMatrix3();
		}
		return a;
	}
}
