package org.ode4j.math;



public class DMatrix3 extends DMatrix<DVector3, DVector3> implements DMatrix3C {
	
	public static final int MAX_I = 3;
	public static final int MAX_J = 4;
	public static final int LEN = MAX_I*MAX_J;
	public static final DMatrix3 ZERO = new DMatrix3();

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
		super(MAX_I, MAX_J);
	}
	

	/**
	 * Private to enforce usage of wrap().
	 * @param a
	 */
	private DMatrix3(double[] a) {
		super(a, MAX_I, MAX_J);
	}
	
	
	public static DMatrix3 wrap(double[] a) {
		return new DMatrix3(a);
	}
	
	
	public void set(DMatrix3C m3) {
		System.arraycopy(((DMatrix3)m3).v, 0, v, 0, v.length);
		//v[0] = v3.v[0]; v[1] = v3.v[1]; v[2] = v3.v[2]; v[3] = v3.v[3];
	}
	
	
	/**
	 * @deprecated Use set(..) instead
	 */
	public DMatrix3 clone() {
		return new DMatrix3(this);
	}
	
	
	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("DMatrix3[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
	}
	public void setOfs(int ofs, DVector3 v3) {
		v[ofs] = v3.v[0]; v[ofs+1] = v3.v[1]; v[ofs+2] = v3.v[2]; v[ofs+3] = v3.v[3];
	}
	public void setCol(int i, DVector3 v3) {
		int ofs = i*4;
		v[ofs] = v3.v[0]; v[ofs+1] = v3.v[1]; v[ofs+2] = v3.v[2]; v[ofs+3] = v3.v[3];
	}
	public void set(double i, double j, double k, double l, double m,
			double n, double o, double p, double q) {
		v[0] = i; v[1] = j; v[2] = k;
		v[4] = l; v[5] = m; v[6] = n;
		v[8] = o; v[9] = p; v[10] = q;
	}
	public void set(double[] da, int da_ofs) {
		System.arraycopy(da, da_ofs, v, 0, da.length);
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
	public DVector3 columnAsNewVector(int c) {
//		return new dVector3(get(0, c), get(1, c), get(2, c));
		return new DVector3(v[c], v[c + MAX_J], v[c + 2*MAX_J]);
	}


	public DMatrix3 add(DMatrix3C M2) {
		DMatrix3 M = (DMatrix3) M2; 
		for (int i = 0; i < v.length; i++) {
			v[i] += M.v[i];
		}
		return this;
	}


	public void scale(double scale) {
		for (int i = 0; i < v.length; i++) {
			v[i] *= scale;
		}
	}
	
	/** 
	 * Matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed:
	 *   0:   A = B  * C   (sizes: A:p*r B:p*q C:q*r)
	 *   1:   A = B' * C   (sizes: A:p*r B:q*p C:q*r)
	 *   2:   A = B  * C'  (sizes: A:p*r B:p*q C:r*q)
	 * case 1,2 are equivalent to saying that the operation is A=B*C but
	 * B or C are stored in standard column format.
	 */
	public void dMultiply0 (final DMatrix3C B, 
			final DMatrix3C C) {
		eqMul(B, C);
	}	
	public void eqMul (final DMatrix3C B, 
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
		set( B.get00()*C.get00() + B.get01()*C.get10() + B.get02()*C.get20(),
				B.get00()*C.get01() + B.get01()*C.get11() + B.get02()*C.get21(),
				B.get00()*C.get02() + B.get01()*C.get12() + B.get02()*C.get22(),
				B.get10()*C.get00() + B.get11()*C.get10() + B.get12()*C.get20(),
				B.get10()*C.get01() + B.get11()*C.get11() + B.get12()*C.get21(),
				B.get10()*C.get02() + B.get11()*C.get12() + B.get12()*C.get22(),
				B.get20()*C.get00() + B.get21()*C.get10() + B.get22()*C.get20(),
				B.get20()*C.get01() + B.get21()*C.get11() + B.get22()*C.get21(),
				B.get20()*C.get02() + B.get21()*C.get12() + B.get22()*C.get22());
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
	 * @param c The column to return [0, 1, 2].
	 */
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

	@Override
	public double get00() {
		return v[0];
	}


	@Override
	public double get01() {
		return v[1];
	}


	@Override
	public double get02() {
		return v[2];
	}


	@Override
	public double get10() {
		return v[1*MAX_J + 0];
	}


	@Override
	public double get11() {
		return v[1*MAX_J + 1];
	}


	@Override
	public double get12() {
		return v[1*MAX_J + 2];
	}


	@Override
	public double get20() {
		return v[2*MAX_J + 0];
	}


	@Override
	public double get21() {
		return v[2*MAX_J + 1];
	}


	@Override
	public double get22() {
		return v[2*MAX_J + 2];
	}

	
	public void set00(double d) {
		v[0] = d;
	}


	public void set01(double d) {
		v[1] = d;
	}


	public void set02(double d) {
		v[2] = d;
	}


	public void set10(double d) {
		v[1*MAX_J + 0] = d;
	}


	public void set11(double d) {
		v[1*MAX_J + 1] = d;
	}


	public void set12(double d) {
		v[1*MAX_J + 2] = d;
	}


	public void set20(double d) {
		v[2*MAX_J + 0] = d;
	}


	public void set21(double d) {
		v[2*MAX_J + 1] = d;
	}


	public void set22(double d) {
		v[2*MAX_J + 2] = d;
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
	public boolean isEqual(DMatrix3C m) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (get(i, j) != m.get(i, j)) return false;
			}
		}
		return true;
	}

	/**
	 * Compares two objects for equality.
	 * This is marginally slower than <tt>isEquals(DMatrix3C m)</tt>.
	 */
	@Override
	public boolean equals(Object o) {
		if (! (o instanceof DMatrix3C)) {
			return false;
		}
		DMatrix3C m = (DMatrix3C) o;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				if (get(i, j) != m.get(i, j)) return false;
			}
		}
		return true;
	}


	public void eqIdentity() {
		eqZero();
		set00(1);
		set11(1);
		set22(1);
	}


	public void setIdentity() {
		eqIdentity();
	}

	
	public void eqZero() {
		for (int i = 0; i < v.length; i++) {
			v[i] = 0;
		}
	}


	public void setZero() {
		eqZero();
	}
}
