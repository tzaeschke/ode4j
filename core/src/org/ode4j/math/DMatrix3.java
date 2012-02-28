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

/**
 * The {@link DMatrix3C} implementation class.
 * 
 * @author Andrew Wagner
 */
public final class DMatrix3 implements DMatrix3C {
	/**
	 * A DVector3-like "view" of a column of a {@link DMatrix3C}. The view can
	 * be changed to another column index of the matrix in order to see that
	 * column's vector data.
	 * 
	 * @author Andrew Wagner
	 */
	public class DVector3ColView extends DVector3View {
		private final int _column;

		/**
		 * Creates a new DVector3ColView with the specified view index.
		 * 
		 * @param column
		 *            - the view index.
		 */
		public DVector3ColView(int column) {
			_column = column;
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

	/**
	 * A DVector3-like "view" of a row of a {@link DMatrix3C}. The view can be
	 * changed to another row index of the matrix in order to see that row's
	 * vector data.
	 * 
	 * @author Andrew Wagner
	 */
	public class DVector3RowTView extends DVector3View {
		private final int _ofs;

		/**
		 * Creates a new DVector3ColView with the specified view index.
		 * 
		 * @param row
		 *            - the view index.
		 */
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
	 * Creates a new DMatrix3 object with the specified double array.
	 * 
	 * @param a
	 *            - the double array to wrap.
	 * @return the resulting DMatrix3.
	 */
	public static DMatrix3 wrap(double[] a) {
		return new DMatrix3(a);
	}

	private final double[] v;

	/**
	 * The maximum number of rows in a DMatrix3.
	 */
	public static final int MAX_I = 3;

	/**
	 * The maximum number of columns in a DMatrix3.
	 */
	public static final int MAX_J = 4;

	/**
	 * The total number of elements within a DMatrix3.
	 */
	public static final int LEN = MAX_I * MAX_J;

	/**
	 * 
	 */
	public static final DMatrix3 ZERO = new DMatrix3();

	/**
	 * Creates a new empty DMatrix3.
	 */
	public DMatrix3() {
		v = new double[MAX_I * MAX_J];
	}

	/**
	 * Creates a new DMatrix3 from the contents of the specified
	 * {@link DMatrix3C}.
	 * 
	 * @param matrix3
	 *            - the DMatrix3C to copy.
	 */
	public DMatrix3(DMatrix3C matrix3) {
		this();
		set(matrix3);
	}

	/**
	 * Create new DMatrix3 of the form:
	 * 
	 * <pre>
	 * d, e, f
	 * g, h, i
	 * j, k, l
	 * </pre>
	 * 
	 * @param d
	 *            - element at (0, 0).
	 * @param e
	 *            - element at (0, 1).
	 * @param f
	 *            - element at (0, 2).
	 * @param g
	 *            - element at (1, 0).
	 * @param h
	 *            - element at (1, 1).
	 * @param i
	 *            - element at (1, 2).
	 * @param j
	 *            - element at (2, 0).
	 * @param k
	 *            - element at (2, 1).
	 * @param l
	 *            - element at (2, 2).
	 */
	public DMatrix3(double d, double e, double f, double g, double h, double i,
			double j, double k, double l) {
		this();
		v[0] = d;
		v[1] = e;
		v[2] = f;
		v[4] = g;
		v[5] = h;
		v[6] = i;
		v[8] = j;
		v[9] = k;
		v[10] = l;
	}

	/**
	 * Creates a new DMatrix3 of the form:
	 * 
	 * <pre>
	 * d, e, f, g
	 * h, i, j, k
	 * l, m, n, o
	 * </pre>
	 * 
	 * @param d
	 *            - element at (0, 0).
	 * @param e
	 *            - element at (0, 1).
	 * @param f
	 *            - element at (0, 2).
	 * @param g
	 *            - element at (0, 3).
	 * @param h
	 *            - element at (1, 0).
	 * @param i
	 *            - element at (1, 1).
	 * @param j
	 *            - element at (1, 2).
	 * @param k
	 *            - element at (1, 3).
	 * @param l
	 *            - element at (2, 0).
	 * @param m
	 *            - element at (2, 1).
	 * @param n
	 *            - element at (2, 2).
	 * @param o
	 *            - element at (2, 3).
	 */
	@Deprecated
	public DMatrix3(double d, double e, double f, double g, double h, double i,
			double j, double k, double l, double m, double n, double o) {
		this();
		v[0] = d;
		v[1] = e;
		v[2] = f;
		v[3] = g;
		v[4] = h;
		v[5] = i;
		v[6] = j;
		v[7] = k;
		v[8] = l;
		v[9] = m;
		v[10] = n;
		v[11] = o;
		if (g != 0 || k != 0 || o != 0) {
			System.err.println("Warning: 4th column in dMatrix3 != 0 !");
		}
	}

	/**
	 * Private to enforce usage of wrap().
	 * 
	 * @param a
	 */
	private DMatrix3(double[] a) {
		v = a;
	}

	/**
	 * Adds the elements of the specified {@link DMatrix3C} to this DMatrix3.
	 * 
	 * @param m
	 *            - the DMatrix3C to add.
	 * @return the reference to this DMatrix3 object.
	 */
	public DMatrix3 add(DMatrix3C m) {
		set00(get00() + m.get00());
		set01(get01() + m.get01());
		set02(get02() + m.get02());
		set10(get10() + m.get10());
		set11(get11() + m.get11());
		set12(get12() + m.get12());
		set20(get20() + m.get20());
		set21(get21() + m.get21());
		set22(get22() + m.get22());
		// DMatrix3 M = (DMatrix3) M2;
		// for (int i = 0; i < v.length; i++) {
		// v[i] += M.v[i];
		// }
		return this;
	}

	/**
	 * Adds the specified value to the element at the given row and column.
	 * 
	 * @param i
	 *            - the row index.
	 * @param j
	 *            - the column index.
	 * @param d
	 *            - the value to add.
	 */
	public void add(int i, int j, double d) {
		v[i * MAX_J + j] += d;
	}

	/**
	 * Returns a clone of this DMatrix3.
	 */
	@Override
	public DMatrix3 clone() {
		return new DMatrix3(this);
	}

	/**
	 * Return a new dVector containing the specified column. For padding=4 this
	 * uses the elements c, 4+c, 8+c;
	 * 
	 * @param c
	 *            Column (0, 1, 2)
	 * @return A new dVector.
	 */
	@Override
	public DVector3 columnAsNewVector(int c) {
		// return new dVector3(get(0, c), get(1, c), get(2, c));
		return new DVector3(v[c], v[c + MAX_J], v[c + 2 * MAX_J]);
	}

	/**
	 * Gets the row dimension.
	 * 
	 * @return the row dimension.
	 */
	public final int dimI() {
		return MAX_I;
	}

	/**
	 * Gets the column dimension.
	 * 
	 * @return the column dimension.
	 */
	public final int dimJ() {
		// TODO MAX_J, once MAX_J==3
		return 3;
	}

	/**
	 * Multiplies the specified {@link DMatrix3C} objects and sets this DMatrix3
	 * to the result. All matrices are stored in standard row format. the digit
	 * refers to the argument that is transposed: 0: A = B * C (sizes: A:p*r
	 * B:p*q C:q*r) 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) 2: A = B * C'
	 * (sizes: A:p*r B:p*q C:r*q) case 1,2 are equivalent to saying that the
	 * operation is A=B*C but B or C are stored in standard column format.
	 * 
	 * @param B
	 *            - the left-hand side of multiplication.
	 * @param C
	 *            - the right-hand side of multiplication.
	 */
	public void dMultiply0(final DMatrix3C B, final DMatrix3C C) {
		eqMul(B, C);
	}

	/**
	 * Calculates the dot product of the the specified column of this DMatrix3
	 * with the given {@link DVector3C}.
	 * 
	 * @param col
	 *            - the column index.
	 * @param v3
	 *            - the DVector3C to dot.
	 */
	@Override
	public final double dotCol(int col, DVector3C v3) {
		if (col == 0) {
			return get00() * v3.get0() + get10() * v3.get1() + get20()
					* v3.get2();
		} else if (col == 1) {
			return get01() * v3.get0() + get11() * v3.get1() + get21()
					* v3.get2();
		} else if (col == 2) {
			return get02() * v3.get0() + get12() * v3.get1() + get22()
					* v3.get2();
		} else {
			throw new IllegalArgumentException("col=" + col);
		}
	}

	/**
	 * Calculates the dot product of the the specified column of this DMatrix3
	 * with the specified column of another DMatrix3C.
	 * 
	 * @param col
	 *            - the column index of this DMatrix3.
	 * @param m2
	 *            - the other DMatrix3C.
	 * @param col2
	 *            - the column index of the other DMatrix3C.
	 */
	@Override
	public final double dotColCol(int col, DMatrix3C m2, int col2) {
		if (col == 0) {
			if (col2 == 0) {
				return get00() * m2.get00() + get10() * m2.get10() + get20()
						* m2.get20();
			} else if (col2 == 1) {
				return get00() * m2.get01() + get10() * m2.get11() + get20()
						* m2.get21();
			} else if (col2 == 2) {
				return get00() * m2.get02() + get10() * m2.get12() + get20()
						* m2.get22();
			}
		} else if (col == 1) {
			if (col2 == 0) {
				return get01() * m2.get00() + get11() * m2.get10() + get21()
						* m2.get20();
			} else if (col2 == 1) {
				return get01() * m2.get01() + get11() * m2.get11() + get21()
						* m2.get21();
			} else if (col2 == 2) {
				return get01() * m2.get02() + get11() * m2.get12() + get21()
						* m2.get22();
			}
		} else if (col == 2) {
			if (col2 == 0) {
				return get02() * m2.get00() + get12() * m2.get10() + get22()
						* m2.get20();
			} else if (col2 == 1) {
				return get02() * m2.get01() + get12() * m2.get11() + get22()
						* m2.get21();
			} else if (col2 == 2) {
				return get02() * m2.get02() + get12() * m2.get12() + get22()
						* m2.get22();
			}
		}
		throw new IllegalArgumentException("col=" + col + " col2=" + col2);
	}

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the given double array at the given offset.
	 * 
	 * @param row
	 *            - the row of this DMatrix3.
	 * @param c
	 *            - the double array.
	 * @param arrayOffset
	 *            - the offset of the double array.
	 */
	@Override
	public final double dotRow(int row, double[] c, int arrayOffset) {
		if (row == 0) {
			return get00() * c[arrayOffset + 0] + get01() * c[arrayOffset + 1]
					+ get02() * c[arrayOffset + 2];
		} else if (row == 1) {
			return get10() * c[arrayOffset + 0] + get11() * c[arrayOffset + 1]
					+ get12() * c[arrayOffset + 2];
		} else if (row == 2) {
			return get20() * c[arrayOffset + 0] + get21() * c[arrayOffset + 1]
					+ get22() * c[arrayOffset + 2];
		} else {
			throw new IllegalArgumentException("row=" + row);
		}
	}

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the given {@link DVector3C}.
	 * 
	 * @param row
	 *            - the row index.
	 * @param v3
	 *            - the DVector3C.
	 */
	@Override
	public final double dotRow(int row, DVector3C v3) {
		if (row == 0) {
			return get00() * v3.get0() + get01() * v3.get1() + get02()
					* v3.get2();
		} else if (row == 1) {
			return get10() * v3.get0() + get11() * v3.get1() + get12()
					* v3.get2();
		} else if (row == 2) {
			return get20() * v3.get0() + get21() * v3.get1() + get22()
					* v3.get2();
		} else {
			throw new IllegalArgumentException("row=" + row);
		}
	}

	// public void dMultiply0 (final dMatrix B, final dMatrix C)
	// {
	// int i,j,k,qskip,rskip,rpad;
	// //COM.dAASSERT (B, C);
	// //COM.dAASSERT(p>0 && q>0 && r>0);
	// qskip = COM.dPAD(q);
	// rskip = COM.dPAD(r);
	// rpad = rskip - r;
	// double sum;
	// int aPos = 0;
	// //final double[] b,c,bb;
	// int bPos, bbPos =0, cPos;
	// //TZ? final double bb;
	// //TZ? bb = B;
	// for (i=p; i > 0; i--) {
	// for (j=0 ; j<r; j++) {
	// //c = C + j;
	// cPos = j;
	// //b = bb;
	// bPos = bbPos;
	// sum = 0;
	// //for (k=q; k > 0; k--, cPos+=rskip) sum += (*(b++))*(*c);
	// for (k=q; k > 0; k--, cPos+=rskip) sum += B.v[bPos++] * C.v[cPos];
	// //*(A++) = sum;
	// v[aPos++] = sum;
	// }
	// // A += rpad;
	// // bb += qskip;
	// aPos += rpad;
	// //bb += qskip;
	// bbPos += qskip;
	// }
	// }
	//
	//
	// /**
	// * Matrix multiplication. all matrices are stored in standard row format.
	// * the digit refers to the argument that is transposed:
	// * 0: A = B * C (sizes: A:p*r B:p*q C:q*r)
	// * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r)
	// * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q)
	// * case 1,2 are equivalent to saying that the operation is A=B*C but
	// * B or C are stored in standard column format.
	// */
	// public void dMultiply1 (final dMatrix B, final dMatrix C)
	// {
	// int i,j,k,pskip,rskip;
	// double sum;
	// // COM.dAASSERT (A , B, C);
	// // COM.dAASSERT(p>0 && q>0 && r>0);
	// pskip = COM.dPAD(p);
	// rskip = COM.dPAD(r);
	// for (i=0; i<p; i++) {
	// for (j=0; j<r; j++) {
	// sum = 0;
	// for (k=0; k<q; k++) sum += B.v[i+k*pskip] * C.v[j+k*rskip];
	// v[i*rskip+j] = sum;
	// }
	// }
	// }
	//
	//
	// /**
	// * Matrix multiplication. all matrices are stored in standard row format.
	// * the digit refers to the argument that is transposed:
	// * 0: A = B * C (sizes: A:p*r B:p*q C:q*r)
	// * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r)
	// * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q)
	// * case 1,2 are equivalent to saying that the operation is A=B*C but
	// * B or C are stored in standard column format.
	// */
	// public void dMultiply2 (final dMatrix B, final dMatrix C)
	// {
	// int i,j,k,z,rpad,qskip;
	// double sum;
	// //final double[] bb,cc;
	// //TZ:
	// int aPos = 0, bPos, cPos;
	// // COM.dAASSERT (A, B , C);
	// // COM.dAASSERT(p>0 && q>0 && r>0);
	// rpad = COM.dPAD(r) - r;
	// qskip = COM.dPAD(q);
	// //bb = B;
	// bPos = 0;
	// for (i=p; i>0; i--) {
	// //cc = C;
	// cPos = 0;
	// for (j=r; j>0; j--) {
	// z = 0;
	// sum = 0;
	// //for (k=q; k>0; k--,z++) sum += bb[z] * cc[z];
	// for (k=q; k>0; k--,z++) sum += B.v[bPos + z] * C.v[cPos + z];
	// //*(A++) = sum;
	// v[aPos++] = sum;
	// //cc += qskip;
	// cPos += qskip;
	// }
	// //A += rpad;
	// aPos += rpad;
	// //bb += qskip;
	// bPos += qskip;
	// }
	// }

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the specified column of another {@link DMatrix3C}.
	 * 
	 * @param row
	 *            - the row index of this DMatrix3.
	 * @param m2
	 *            - the other DMatrix3C.
	 * @param col
	 *            - the column index of the other DMatrix3C.
	 */
	@Override
	public double dotRowCol(int row, DMatrix3C m2, int col) {
		if (row == 0) {
			if (col == 0) {
				return get00() * m2.get00() + get01() * m2.get10() + get02()
						* m2.get20();
			} else if (col == 1) {
				return get00() * m2.get01() + get01() * m2.get11() + get02()
						* m2.get21();
			} else if (col == 2) {
				return get00() * m2.get02() + get01() * m2.get12() + get02()
						* m2.get22();
			}
		} else if (row == 1) {
			if (col == 0) {
				return get10() * m2.get00() + get11() * m2.get10() + get12()
						* m2.get20();
			} else if (col == 1) {
				return get10() * m2.get01() + get11() * m2.get11() + get12()
						* m2.get21();
			} else if (col == 2) {
				return get10() * m2.get02() + get11() * m2.get12() + get12()
						* m2.get22();
			}
		} else if (row == 2) {
			if (col == 0) {
				return get20() * m2.get00() + get21() * m2.get10() + get22()
						* m2.get20();
			} else if (col == 1) {
				return get20() * m2.get01() + get21() * m2.get11() + get22()
						* m2.get21();
			} else if (col == 2) {
				return get20() * m2.get02() + get21() * m2.get12() + get22()
						* m2.get22();
			}
		}
		throw new IllegalArgumentException("row=" + row + " col2=" + col);
	}

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the specified row of another {@link DMatrix3C}.
	 * 
	 * @param row
	 *            - the row index of this DMatrix3.
	 * @param m2
	 *            - the other DMatrix3C.
	 * @param row2
	 *            - the row index of the other DMatrix3C.
	 */
	@Override
	public double dotRowRow(int row, DMatrix3C m2, int row2) {
		if (row == 0) {
			if (row2 == 0) {
				return get00() * m2.get00() + get01() * m2.get01() + get02()
						* m2.get02();
			} else if (row2 == 1) {
				return get00() * m2.get10() + get01() * m2.get11() + get02()
						* m2.get12();
			} else if (row2 == 2) {
				return get00() * m2.get20() + get01() * m2.get21() + get02()
						* m2.get22();
			}
		} else if (row == 1) {
			if (row2 == 0) {
				return get10() * m2.get00() + get11() * m2.get01() + get12()
						* m2.get02();
			} else if (row2 == 1) {
				return get10() * m2.get10() + get11() * m2.get11() + get12()
						* m2.get12();
			} else if (row2 == 2) {
				return get10() * m2.get20() + get11() * m2.get21() + get12()
						* m2.get22();
			}
		} else if (row == 2) {
			if (row2 == 0) {
				return get20() * m2.get00() + get21() * m2.get01() + get22()
						* m2.get02();
			} else if (row2 == 1) {
				return get20() * m2.get10() + get21() * m2.get11() + get22()
						* m2.get12();
			} else if (row2 == 2) {
				return get20() * m2.get20() + get21() * m2.get21() + get22()
						* m2.get22();
			}
		}
		throw new IllegalArgumentException("row=" + row + " row2=" + row2);
	}

	/**
	 * Sets this DMatrix3 contents to the identity matrix.
	 */
	public final void eqIdentity() {
		eqZero();
		set00(1);
		set11(1);
		set22(1);
	}

	/**
	 * Multiplies the specified {@link DMatrix3C} objects and sets this DMatrix3
	 * to the result. All matrices are stored in standard row format. the digit
	 * refers to the argument that is transposed: 0: A = B * C (sizes: A:p*r
	 * B:p*q C:q*r) 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) 2: A = B * C'
	 * (sizes: A:p*r B:p*q C:r*q) case 1,2 are equivalent to saying that the
	 * operation is A=B*C but B or C are stored in standard column format.
	 * 
	 * @param B
	 *            - the left-hand side of multiplication.
	 * @param C
	 *            - the right-hand side of multiplication.
	 */
	public void eqMul(final DMatrix3C B, final DMatrix3C C) {
		// dMatrix3 B2 = (dMatrix3) B;
		// dMatrix3 C2 = (dMatrix3) C;
		// double sum;
		// int aPos = 0;
		// int bPos, bbPos =0, cPos;
		// for (int i=3; i > 0; i--) {
		// for (int j=0 ; j<3; j++) {
		// cPos = j;
		// bPos = bbPos;
		// sum = 0;
		// for (int k=3; k > 0; k--, cPos+=MAX_J) sum += B2.v[bPos++] *
		// C2.v[cPos];
		// v[aPos++] = sum;
		// }
		// aPos++;
		// bbPos += MAX_J;
		// }
		// set( B.get00()*C.get00() + B.get01()*C.get10() + B.get02()*C.get20(),
		// B.get00()*C.get01() + B.get01()*C.get11() + B.get02()*C.get21(),
		// B.get00()*C.get02() + B.get01()*C.get12() + B.get02()*C.get22(),
		// B.get10()*C.get00() + B.get11()*C.get10() + B.get12()*C.get20(),
		// B.get10()*C.get01() + B.get11()*C.get11() + B.get12()*C.get21(),
		// B.get10()*C.get02() + B.get11()*C.get12() + B.get12()*C.get22(),
		// B.get20()*C.get00() + B.get21()*C.get10() + B.get22()*C.get20(),
		// B.get20()*C.get01() + B.get21()*C.get11() + B.get22()*C.get21(),
		// B.get20()*C.get02() + B.get21()*C.get12() + B.get22()*C.get22());
		set00(B.get00() * C.get00() + B.get01() * C.get10() + B.get02()
				* C.get20());
		set01(B.get00() * C.get01() + B.get01() * C.get11() + B.get02()
				* C.get21());
		set02(B.get00() * C.get02() + B.get01() * C.get12() + B.get02()
				* C.get22());
		set10(B.get10() * C.get00() + B.get11() * C.get10() + B.get12()
				* C.get20());
		set11(B.get10() * C.get01() + B.get11() * C.get11() + B.get12()
				* C.get21());
		set12(B.get10() * C.get02() + B.get11() * C.get12() + B.get12()
				* C.get22());
		set20(B.get20() * C.get00() + B.get21() * C.get10() + B.get22()
				* C.get20());
		set21(B.get20() * C.get01() + B.get21() * C.get11() + B.get22()
				* C.get21());
		set22(B.get20() * C.get02() + B.get21() * C.get12() + B.get22()
				* C.get22());
	}

	/**
	 * Transposes this DMatrix3.
	 * 
	 * @return the reference to this DMatrix3 object.
	 */
	public final DMatrix3 eqTranspose() {
		double t;
		t = get01();
		set01(get10());
		set10(t);
		t = get02();
		set02(get20());
		set20(t);
		t = get21();
		set21(get12());
		set12(t);
		return this;
	}

	/**
	 * Compares two objects for equality. This is marginally slower than
	 * {@code isEquals(DMatrix3C m)}.
	 * 
	 * @param o
	 *            - the other object.
	 */
	@Override
	public final boolean equals(Object o) {
		if (!(o instanceof DMatrix3C)) {
			return false;
		}
		DMatrix3C m = (DMatrix3C) o;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (get(i, j) != m.get(i, j))
					return false;
			}
		}
		return true;
	}

	/**
	 * Sets all values in this DMatrix3 to zero.
	 */
	public final void eqZero() {
		for (int i = 0; i < v.length; i++) {
			v[i] = 0;
		}
	}

	/**
	 * Gets the value at the specified index in the array backing this DMatrix3
	 * object.
	 * 
	 * @param i
	 *            - the index.
	 * @return the value at the specified index.
	 */
	public double get(int i) {
		return v[i];
	}

	@Override
	public double get(int i, int j) {
		return v[i * MAX_J + j];
	}

	@Override
	public final double get00() {
		return v[0];
	}

	@Override
	public final double get01() {
		return v[1];
	}

	@Override
	public final double get02() {
		return v[2];
	}

	@Override
	public final double get10() {
		return v[1 * MAX_J + 0];
	}

	@Override
	public final double get11() {
		return v[1 * MAX_J + 1];
	}

	@Override
	public final double get12() {
		return v[1 * MAX_J + 2];
	}

	@Override
	public final double get20() {
		return v[2 * MAX_J + 0];
	}

	@Override
	public final double get21() {
		return v[2 * MAX_J + 1];
	}

	@Override
	public final double get22() {
		return v[2 * MAX_J + 2];
	}

	/**
	 * Compares this DMatrix3 and the specified {@link DMatrix3C} for equality.
	 * This is marginally faster than {@code equals(Object o)}.
	 * 
	 * @param m
	 *            - the other DMatrix3C.
	 * @return - {@code true} if equal, {@code false} otherwise.
	 */
	public final boolean isEqual(DMatrix3C m) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (get(i, j) != m.get(i, j))
					return false;
			}
		}
		return true;
	}

	/**
	 * Create a new transposed version of this matrix.
	 * 
	 * @return The transposed copy of this matrix.
	 */
	public final DMatrix3 reTranspose() {
		return new DMatrix3(get00(), get10(), get20(), get01(), get11(),
				get21(), get02(), get12(), get22());
	}

	/**
	 * Scales the DMatrix3 by the specified scaling factor.
	 * 
	 * @param scale
	 *            - the scaling factor.
	 */
	public void scale(double scale) {
		for (int i = 0; i < v.length; i++) {
			v[i] *= scale;
		}
	}

	/**
	 * Sets the contents of this DMatrix3 to the contents of the specified
	 * {@link DMatrix3C} object.
	 * 
	 * @param m
	 *            - the DMatrix3C object.
	 * @return the reference to this DMatrix3 object.
	 */
	public DMatrix3 set(DMatrix3C m) {
		// System.arraycopy(((DMatrix3)m3).v, 0, v, 0, v.length);
		// //v[0] = v3.v[0]; v[1] = v3.v[1]; v[2] = v3.v[2]; v[3] = v3.v[3];
		set00(m.get00());
		set01(m.get01());
		set02(m.get02());
		set10(m.get10());
		set11(m.get11());
		set12(m.get12());
		set20(m.get20());
		set21(m.get21());
		set22(m.get22());
		return this;
	}

	/**
	 * Set the data of the DMatrix3 to the specified values. The matrix is in
	 * the form:
	 * 
	 * <pre>
	 * d, e, f
	 * g, h, i
	 * j, k, l
	 * </pre>
	 * 
	 * @param d
	 * @param e
	 * @param f
	 * @param g
	 * @param h
	 * @param i
	 * @param j
	 * @param k
	 * @param l
	 */
	public void set(double d, double e, double f, double g, double h, double i,
			double j, double k, double l) {
		// v[0] = i; v[1] = j; v[2] = k;
		// v[4] = l; v[5] = m; v[6] = n;
		// v[8] = o; v[9] = p; v[10] = q;
		set00(d);
		set01(e);
		set02(f);
		set10(g);
		set11(h);
		set12(i);
		set20(j);
		set21(k);
		set22(l);
	}

	/**
	 * Sets the contents of this DMatrix3 from a 3*4 double [] and the specified
	 * offset, ignoring the 4th, 8th and 12th field. This is useful when using
	 * padded arrays.
	 * 
	 * @param da
	 *            - the double array to copy.
	 * @param da_ofs
	 *            - the offset from which to start copying.
	 */
	public void set(double[] da, int da_ofs) {
		System.arraycopy(da, da_ofs, v, 0, da.length);
	}

	/**
	 * Sets the specified row and column to the given value.
	 * 
	 * @param i
	 *            - the row index.
	 * @param j
	 *            - the column index.
	 * @param a
	 *            - the value to set.
	 */
	public void set(int i, int j, double a) {
		v[i * MAX_J + j] = a;
	}

	/**
	 * Sets the value at index (0, 0).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set00(double d) {
		v[0] = d;
	}

	/**
	 * Sets the value at index (0, 1).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set01(double d) {
		v[1] = d;
	}

	/**
	 * Sets the value at index (0, 2).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set02(double d) {
		v[2] = d;
	}

	/**
	 * Sets the value at index (1, 0).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set10(double d) {
		v[1 * MAX_J + 0] = d;
	}

	/**
	 * Sets the value at index (1, 1).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set11(double d) {
		v[1 * MAX_J + 1] = d;
	}

	/**
	 * Sets the value at index (1, 2).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set12(double d) {
		v[1 * MAX_J + 2] = d;
	}

	/**
	 * Sets the value at index (2, 0).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set20(double d) {
		v[2 * MAX_J + 0] = d;
	}

	/**
	 * Sets the value at index (2, 1).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set21(double d) {
		v[2 * MAX_J + 1] = d;
	}

	/**
	 * Sets the value at index (2, 2).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	public final void set22(double d) {
		v[2 * MAX_J + 2] = d;
	}

	/**
	 * Sets the specified column with the values of the given {@link DVector3C}
	 * object.
	 * 
	 * @param i
	 *            - the column index.
	 * @param v3
	 *            - the DVector3C object.
	 */
	public void setCol(int i, DVector3C v3) {
		int ofs = i * 4;
		v[ofs] = v3.get0();
		v[ofs + 1] = v3.get1();
		v[ofs + 2] = v3.get2(); // v[ofs+3] = 0;//v3.v[3];
	}

	/**
	 * Sets this DMatrix3 contents to the identity matrix.
	 * 
	 * @return the reference to this DMatrix3 object.
	 */
	public final DMatrix3 setIdentity() {
		eqIdentity();
		return this;
	}

	/**
	 * Sets the contents of the double array of this DMatrix3 to the contents of
	 * the specified {@link DVector3C} object starting at the given array
	 * offset.
	 * 
	 * @param ofs
	 *            - the offset index.
	 * @param v3
	 *            - the DVector3C object.
	 */
	public void setOfs(int ofs, DVector3C v3) {
		v[ofs] = v3.get0();
		v[ofs + 1] = v3.get1();
		v[ofs + 2] = v3.get2(); // v[ofs+3] = 0;//v3.v[3];
	}

	/**
	 * Sets all values in this DMatrix3 to zero.
	 */
	public final void setZero() {
		eqZero();
	}

	/**
	 * Subtracts the specified value from the value at the given row and column.
	 * 
	 * @param i
	 *            - the row index.
	 * @param j
	 *            - the column index.
	 * @param d
	 *            - the value to subtract.
	 */
	public void sub(int i, int j, double d) {
		v[i * MAX_J + j] -= d;
	}

	@Override
	public final float[] toFloatArray() {
		return new float[] { (float) get00(), (float) get01(), (float) get02(),
				(float) get10(), (float) get11(), (float) get12(),
				(float) get20(), (float) get21(), (float) get22() };
	}

	@Override
	public final float[] toFloatArray12() {
		return new float[] { (float) get00(), (float) get01(), (float) get02(),
				0.0f, (float) get10(), (float) get11(), (float) get12(), 0.0f,
				(float) get20(), (float) get21(), (float) get22(), 0.0f };
	}

	@Override
	public String toString() {
		// StringBuffer b = new StringBuffer();
		// b.append("DMatrix3[");
		// for (int i = 0; i < v.length-1; i++) {
		// b.append(v[i]).append(", ");
		// }
		// b.append(v[v.length-1]).append("]");
		// return b.toString();
		StringBuffer b = new StringBuffer();
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

	@Override
	public DVector3ColView viewCol(int column) {
		return new DVector3ColView(column);
	}

	@Override
	public DVector3RowTView viewRowT(int row) {
		return new DVector3RowTView(row);
	}
}
