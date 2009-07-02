package org.ode4j.math;

public abstract class DVector3View implements DVector3I {
	public abstract double get0();
	public abstract double get1();
	public abstract double get2();
	public abstract double get(int i);
	public abstract void set0(double d);
	public abstract void set1(double d);
	public abstract void set2(double d);

	public final double length() {
		return Math.sqrt(get0()*get0() + get1()*get1() + get2()*get2());
	}

	public double dot(DVector3View v2) {
		return get0()*v2.get0() + get1()*v2.get1() + get2()*v2.get2();
	}

	public final void set(DVector3 v2) {
		set0( v2.get0() );
		set1( v2.get1() );
		set2( v2.get2() );
	}

	public final void set(double x, double y, double z) {
		set0( x );
		set1( y );
		set2( z );
	}

	public final void scale(double s) {
		set0( get0() * s );
		set1( get1() * s );
		set2( get2() * s );
	}

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("[").append( get0() ).append(", ");
		b.append( get1() ).append(", ");
		b.append( get2() ).append("]");
		return b.toString();
	}
}
