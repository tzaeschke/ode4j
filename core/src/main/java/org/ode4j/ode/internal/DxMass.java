/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
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
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import static org.ode4j.ode.OdeMath.dCalcVectorCross3;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply0_333;
import static org.ode4j.ode.OdeMath.dMultiply2_333;
import static org.ode4j.ode.OdeMath.dSetCrossMatrixPlus;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.dDEBUGMSG;
import static org.ode4j.ode.internal.Common.dNODEBUG;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.Common.dUASSERT;
import static org.ode4j.ode.internal.Matrix.dIsPositiveDefinite;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DMassC;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.internal.cpp4j.java.FormattedStringBuilder;
import org.ode4j.ode.internal.trimesh.DxTriMesh;

/**
 * DxMass.
 *
 * @author Tilmann Zaeschke
 */
public class DxMass implements DMass {

	double _mass;
	DVector3 _c;
	DMatrix3 _I;

	

	//#define	SQR(x)			((x)*(x))						//!< Returns x square
	private double SQR(double x) {return x * x;}
	//#define	CUBE(x)			((x)*(x)*(x))					//!< Returns x cube
	private double CUBE(double x) {return x * x * x;}
//
//	//#define _I(i,j) I[(i)*4+(j)]
//	private int _I(int i, int j) {return i * 4 + j;}


	public DxMass() {
		super();
		_c = new DVector3();
		_I = new DMatrix3();
		dMassSetZero (); 
	}
	
	// return 1 if ok, 0 if bad
	public boolean dMassCheck ()
	{
		if (_mass <= 0) {
			dDEBUGMSG ("mass must be > 0 but is " + _mass);
			return false;
		}
		if (!dIsPositiveDefinite (_I)) {
			dDEBUGMSG ("inertia must be positive definite");
			return false;
		}

		// verify that the center of mass position is consistent with the mass
		// and inertia matrix. this is done by checking that the inertia around
		// the center of mass is also positive definite. from the comment in
		// dMassTranslate(), if the body is translated so that its center of mass
		// is at the point of reference, then the new inertia is:
		//   I + mass*crossmat(c)^2
		// note that requiring this to be positive definite is exactly equivalent
		// to requiring that the spatial inertia matrix
		//   [ mass*eye(3,3)   M*crossmat(c)^T ]
		//   [ M*crossmat(c)   I               ]
		// is positive definite, given that I is PD and mass>0. see the theorem
		// about partitioned PD matrices for proof.

		DMatrix3 I2 = new DMatrix3(),chat = new DMatrix3();
		//chat.setZero();//dSetZero (chat,12);
		//dCROSSMAT (chat,m.c,4,+,-);
		dSetCrossMatrixPlus (chat,_c);
		dMultiply0_333 (I2,chat,chat);
//		for (i=0; i<3; i++)  I2.v[i] = _I.v[i] + _mass*I2.v[i];
//		for (i=4; i<7; i++)  I2.v[i] = _I.v[i] + _mass*I2.v[i];
//		for (i=8; i<11; i++) I2.v[i] = _I.v[i] + _mass*I2.v[i];
		I2.scale(_mass);
		I2.add(_I);
		
		if (!dIsPositiveDefinite (I2)) {
			dDEBUGMSG ("center of mass inconsistent with mass parameters");
			return false;
		}
		return true;
	}

	public void dMassSetZero ()
	{
		//dAASSERT (m);
		_mass = 0.0;
		_c.setZero();//dSetZero (_c.v);//,sizeof(m.c) / sizeof(double));
		_I.setZero();//dSetZero (_I.v);//,sizeof(m.I) / sizeof(double));
	}


//	public void dMassSetParameters (dxMass m, double themass,
//			double cgx, double cgy, double cgz,
//			double I11, double I22, double I33,
//			double I12, double I13, double I23)
	public void dMassSetParameters (double themass,
			double cgx, double cgy, double cgz,
			double I11, double I22, double I33,
			double I12, double I13, double I23)
	{
		//dAASSERT (m);
		dMassSetZero ();
		_mass = themass;
//		_c.v[0] = cgx;
//		_c.v[1] = cgy;
//		_c.v[2] = cgz;
		_c.set(cgx, cgy, cgz);
//		_I.v[_I(0,0)] = I11;
//		_I.v[_I(1,1)] = I22;
//		_I.v[_I(2,2)] = I33;
//		_I.v[_I(0,1)] = I12;
//		_I.v[_I(0,2)] = I13;
//		_I.v[_I(1,2)] = I23;
//		_I.v[_I(1,0)] = I12;
//		_I.v[_I(2,0)] = I13;
//		_I.v[_I(2,1)] = I23;
		_I.set(I11, I12, I13,     I12, I22, I23,     I13, I23, I33);
		dMassCheck ();
	}


//	public void dMassSetSphere (dxMass m, double density, double radius)
	public void dMassSetSphere (double density, double radius)
	{
		dMassSetSphereTotal ( ((4.0/3.0) * M_PI *
				radius*radius*radius * density), radius);
	}


//	public void dMassSetSphereTotal (dxMass m, double total_mass, double radius)
	public void dMassSetSphereTotal (double total_mass, double radius)
	{
		//dAASSERT (m);
		dMassSetZero ();
		_mass = total_mass;
		double II = 0.4 * total_mass * radius*radius;
		_I.set00( II );
		_I.set11( II );
		_I.set22( II );

		if (!dNODEBUG) {//# ifndef dNODEBUG
			dMassCheck ();
		}//# endif
	}


//	public void dMassSetCapsule (dMass m, double density, int direction,
//			double radius, double length)
	public void dMassSetCapsule (double density, int direction,
			double radius, double length)
	{
		double M1,M2,Ia,Ib;
//		dAASSERT (m);
		dUASSERT (direction >= 1 && direction <= 3,"bad direction number");
		dMassSetZero ();
		M1 = M_PI*radius*radius*length*density;			  // cylinder mass
		M2 = (4.0/3.0)*M_PI*radius*radius*radius*density; // total cap mass
		_mass = M1+M2;
		Ia = M1*(0.25*radius*radius + (1.0/12.0)*length*length) +
		M2*(0.4*radius*radius + 0.375*radius*length + 0.25*length*length);
		Ib = (M1*0.5 + M2*0.4)*radius*radius;
		_I.set00( Ia );
		_I.set11( Ia );
		_I.set22( Ia );
		_I.set( direction-1, direction-1, Ib );

		if (!dNODEBUG) {//# ifndef dNODEBUG
			dMassCheck ();
		}//# endif
	}


	public void dMassSetCapsuleTotal (double total_mass, int direction,
			double radius, double length)
	{
		dMassSetCapsule (1.0, direction, radius, length);
		dMassAdjust (total_mass);
	}


	public void dMassSetCylinder (double density, int direction,
			double radius, double length)
	{
		dMassSetCylinderTotal (M_PI*radius*radius*length*density,
				direction, radius, length);
	}

	
	public void dMassSetCylinderTotal (double total_mass, int direction,
			double radius, double length)
	{
		double r2,I;
		//dAASSERT (m);
		dUASSERT (direction >= 1 && direction <= 3,"bad direction number");
		dMassSetZero ();
		r2 = radius*radius;
		_mass = total_mass;
		I = total_mass*(0.25*r2 + (1.0/12.0)*length*length);
		_I.set00( I );
		_I.set11( I );
		_I.set22( I );
		_I.set( direction-1, direction-1, total_mass*0.5*r2 );

		if (!dNODEBUG) {//# ifndef dNODEBUG
			dMassCheck ();
		}//# endif
	}


	public void dMassSetBox (double density,
			double lx, double ly, double lz)
	{
		dMassSetBoxTotal (lx*ly*lz*density, lx, ly, lz);
	}


	public void dMassSetBoxTotal (double total_mass,
			double lx, double ly, double lz)
	{
//		dAASSERT (m);
		dMassSetZero ();
		_mass = total_mass;
		_I.set00( total_mass/12.0 * (ly*ly + lz*lz) );
		_I.set11( total_mass/12.0 * (lx*lx + lz*lz) );
		_I.set22( total_mass/12.0 * (lx*lx + ly*ly) );

		if (!dNODEBUG) {//# ifndef dNODEBUG
			dMassCheck ();
		}//# endif
	}



	/**
	 * dMassSetTrimesh, implementation by Gero Mueller.
	 * Based on Brian Mirtich, "Fast and Accurate Computation of
	 * Polyhedral Mass Properties," journal of graphics tools, volume 1,
	 * number 2, 1996.
	 * @param density density
	 * @param g g
	 */
//	public void dMassSetTrimesh( dxMass m, double density, dxGeom g )
	public void dMassSetTrimesh( double density, DTriMesh g )
	{
		dUASSERT(g != null, "argument not a trimesh");// && g.type == dTriMeshClass, "argument not a trimesh");

		dMassSetZero ();

		DxTriMesh TriMesh = (DxTriMesh )g;
		//unsigned 
		int triangles = TriMesh.getMeshTriangleCount();

		double nx, ny, nz;
		//unsigned 
		int i, A, B, C;
		// face integrals
		double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

		// projection integrals
		double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

		double T0 = 0;
		double[] T1 = {0., 0., 0.};
		double[] T2 = {0., 0., 0.};
		double[] TP = {0., 0., 0.};

		for( i = 0; i < triangles; i++ )	 	
		{
			//DVector3[] v = {new DVector3(), new DVector3(), new DVector3()};//[3];
			DVector3 v0 = new DVector3(), v1 = new DVector3(), v2 = new DVector3();//[3];
			TriMesh.fetchMeshTransformedTriangle(v0, v1, v2, i);

			DVector3 n = new DVector3(), a = new DVector3(), b = new DVector3();
//			dOP( a.v, OP.SUB, v[1].v, v[0].v ); 
//			dOP( b.v, OP.SUB, v[2].v, v[0].v ); 
			a.eqDiff(v1, v0);
			b.eqDiff(v2, v0);
			dCalcVectorCross3( n, b, a );
			nx = Math.abs(n.get0());
			ny = Math.abs(n.get1());
			nz = Math.abs(n.get2());

			if( nx > ny && nx > nz )
				C = 0;
			else
				C = (ny > nz) ? 1 : 2;

			// Even though all triangles might be initially valid, 
			// a triangle may degenerate into a segment after applying 
			// space transformation.
			if (n.get(C) != 0.0)
			{
				A = (C + 1) % 3;
				B = (A + 1) % 3;

				// calculate face integrals
				{
					double w;
					double k1, k2, k3, k4;

					//compProjectionIntegrals(f);
					{
						double a0=0, a1=0, da;
						double b0=0, b1=0, db;
						double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
						double a1_2, a1_3, b1_2, b1_3;
						double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
						double Cab, Kab, Caab, Kaab, Cabb, Kabb;

						P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

						for( int j = 0; j < 3; j++)
						{
							switch(j)
							{
							case 0:
								a0 = v0.get(A);
								b0 = v0.get(B);
								a1 = v1.get(A);
								b1 = v1.get(B);
								break;
							case 1:
								a0 = v1.get(A);
								b0 = v1.get(B);
								a1 = v2.get(A);
								b1 = v2.get(B);
								break;
							case 2:
								a0 = v2.get(A);
								b0 = v2.get(B);
								a1 = v0.get(A);
								b1 = v0.get(B);
								break;
							}
							da = a1 - a0;
							db = b1 - b0;
							a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
							b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
							a1_2 = a1 * a1; a1_3 = a1_2 * a1; 
							b1_2 = b1 * b1; b1_3 = b1_2 * b1;

							C1 = a1 + a0;
							Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
							Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
							Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
							Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
							Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
							Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

							P1 += db*C1;
							Pa += db*Ca;
							Paa += db*Caa;
							Paaa += db*Caaa;
							Pb += da*Cb;
							Pbb += da*Cbb;
							Pbbb += da*Cbbb;
							Pab += db*(b1*Cab + b0*Kab);
							Paab += db*(b1*Caab + b0*Kaab);
							Pabb += da*(a1*Cabb + a0*Kabb);
						}

						P1 /= 2.0;
						Pa /= 6.0;
						Paa /= 12.0;
						Paaa /= 20.0;
						Pb /= -6.0;
						Pbb /= -12.0;
						Pbbb /= -20.0;
						Pab /= 24.0;
						Paab /= 60.0;
						Pabb /= -60.0;
					}			

					w = - n.dot(v0);

					k1 = 1 / n.get(C); k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

					Fa = k1 * Pa;
					Fb = k1 * Pb;
					Fc = -k2 * (n.get(A)*Pa + n.get(B)*Pb + w*P1);

					Faa = k1 * Paa;
					Fbb = k1 * Pbb;
					Fcc = k3 * (SQR(n.get(A))*Paa + 2*n.get(A)*n.get(B)*Pab + SQR(n.get(B))*Pbb +
							w*(2*(n.get(A)*Pa + n.get(B)*Pb) + w*P1));

					Faaa = k1 * Paaa;
					Fbbb = k1 * Pbbb;
					Fccc = -k4 * (CUBE(n.get(A))*Paaa + 3*SQR(n.get(A))*n.get(B)*Paab 
							+ 3*n.get(A)*SQR(n.get(B))*Pabb + CUBE(n.get(B))*Pbbb
							+ 3*w*(SQR(n.get(A))*Paa + 2*n.get(A)*n.get(B)*Pab + SQR(n.get(B))*Pbb)
							+ w*w*(3*(n.get(A)*Pa + n.get(B)*Pb) + w*P1));

					Faab = k1 * Paab;
					Fbbc = -k2 * (n.get(A)*Pabb + n.get(B)*Pbbb + w*Pbb);
					Fcca = k3 * (SQR(n.get(A))*Paaa + 2*n.get(A)*n.get(B)*Paab + SQR(n.get(B))*Pabb
							+ w*(2*(n.get(A)*Paa + n.get(B)*Pab) + w*Pa));
				}


				T0 += n.get0() * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

				T1[A] += n.get(A) * Faa;
				T1[B] += n.get(B) * Fbb;
				T1[C] += n.get(C) * Fcc;
				T2[A] += n.get(A) * Faaa;
				T2[B] += n.get(B) * Fbbb;
				T2[C] += n.get(C) * Fccc;
				TP[A] += n.get(A) * Faab;
				TP[B] += n.get(B) * Fbbc;
				TP[C] += n.get(C) * Fcca;
			}
		}

		T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
		T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
		TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;

		_mass = density * T0;
		_I.set00( density * (T2[1] + T2[2]) );
		_I.set11( density * (T2[2] + T2[0]) );
		_I.set22( density * (T2[0] + T2[1]) );
		_I.set01( - density * TP[0] );
		_I.set10( - density * TP[0] );
		_I.set21( - density * TP[1] );
		_I.set12( - density * TP[1] );
		_I.set20( - density * TP[2] );
		_I.set02( - density * TP[2] );

		// Added to address SF bug 1729095
		//dMassTranslate( T1[0] / T0,  T1[1] / T0,  T1[2] / T0 );
		DVector3 vT = new DVector3( T1[0] / T0,  T1[1] / T0,  T1[2] / T0 ); 
		dMassTranslate( vT );

		if (!dNODEBUG) { //# ifndef dNODEBUG
			dMassCheck ();
		} //# endif
	}


//	public void dMassSetTrimeshTotal( dxMass m, double total_mass, dxGeom g)
	public void dMassSetTrimeshTotal(double total_mass, DTriMesh g)
	{
		dUASSERT( g != null, "argument not a trimesh");// && g.type == dTriMeshClass, "argument not a trimesh" );
		dMassSetTrimesh( 1.0, g );
		dMassAdjust( total_mass );
	}


	public void dMassAdjust (double newmass)
	{
		double scale = newmass / _mass;
		_mass = newmass;
		//for (int i=0; i<3; i++) for (int j=0; j<3; j++) _I.v[_I(i,j)] *= scale;
		_I.scale(scale);

		if (!dNODEBUG) {//# ifndef dNODEBUG
			dMassCheck ();
		}	//# endif
	}


//	public void dMassTranslate (dxMass m, double x, double y, double z)
	public void dMassTranslate (DVector3C xyz)
	{
		// if the body is translated by `a' relative to its point of reference,
		// the new inertia about the point of reference is:
		//
		//   I + mass*(crossmat(c)^2 - crossmat(c+a)^2)
		//
		// where c is the existing center of mass and I is the old inertia.

		int i,j;
		DMatrix3 ahat = new DMatrix3(),chat = new DMatrix3();
		DMatrix3 t1 = new DMatrix3(),t2 = new DMatrix3();

		// adjust inertia matrix
		//chat.dSetZero();//dSetZero (chat,12);
		dSetCrossMatrixPlus (chat,_c);
		//double a[3];
		DVector3 a = new DVector3(xyz);
		a.add(_c);
//		a.v[0] = x + _c.v[0];
//		a.v[1] = y + _c.v[1];
//		a.v[2] = z + _c.v[2];
		//ahat.dSetZero();//dSetZero (ahat,12);
		dSetCrossMatrixPlus (ahat,a);
		dMultiply0_333 (t1,ahat,ahat);
		dMultiply0_333 (t2,chat,chat);
		for (i=0; i<3; i++) for (j=0; j<3; j++)
			_I.add(i, j, _mass * (t2.get(i, j)-t1.get(i, j)) );

		// ensure perfect symmetry
		_I.set10( _I.get01() );//v[_I(1,0)] = _I.v[_I(0,1)];
		_I.set20( _I.get02() );//v[_I(2,0)] = _I.v[_I(0,2)];
		_I.set21( _I.get12() );//v[_I(2,1)] = _I.v[_I(1,2)];

		// adjust center of mass
//		_c.v[0] += x;
//		_c.v[1] += y;
//		_c.v[2] += z;
		_c.add(xyz);

		if (!dNODEBUG) {//# ifndef dNODEBUG
			dMassCheck ();
		}//# endif
	}

	
	public void dMassRotate (final DMatrix3C aR)
	{
		// if the body is rotated by `R' relative to its point of reference,
		// the new inertia about the point of reference is:
		//
		//   R * I * R'
		//
		// where I is the old inertia.

		DMatrix3 t1 = new DMatrix3();
		//double t2[3];
		DVector3 t2 = new DVector3();

		// rotate inertia matrix
		dMultiply2_333 (t1,_I,aR);
		dMultiply0_333 (_I,aR,t1);

		// ensure perfect symmetry
		_I.set10( _I.get01() );//v[_I(1,0)] = _I.v[_I(0,1)];
		_I.set20( _I.get02() );//v[_I(2,0)] = _I.v[_I(0,2)];
		_I.set21( _I.get12() );//v[_I(2,1)] = _I.v[_I(1,2)];

		// rotate center of mass
		dMultiply0_331 (t2,aR,_c);
//		_c.v[0] = t2.v[0];
//		_c.v[1] = t2.v[1];
//		_c.v[2] = t2.v[2];
		_c.set(t2);

		if (!dNODEBUG) { //# ifndef dNODEBUG
			dMassCheck ();
		} //# endif
	}


//	public void dMassAdd (dxMass a, final dxMass b)
	public void dMassAdd (DMassC b)
	{
		//dAASSERT (b);
		double denom = dRecip (_mass + b.getMass());
		//for (i=0; i<3; i++) a._c.v[i] = (a._c.v[i]*a._mass + b._c.v[i]*b._mass)*denom;
		_c.eqSum( _c, _mass, b.getC(), b.getMass() ).scale( denom );
		
		_mass += b.getMass();
		//for (i=0; i<12; i++) a._I.v[i] += b._I.v[i];
		_I.add(b.getI());
	}
	

//	// Backwards compatible API
//	void dMassSetCappedCylinder(dMass *a, dReal b, int c, dReal d, dReal e)
//	{
//	  return dMassSetCapsule(a,b,c,d,e);
//	}
//
//	void dMassSetCappedCylinderTotal(dMass *a, dReal b, int c, dReal d, dReal e)
//	{
//	  return dMassSetCapsuleTotal(a,b,c,d,e);
//	}

	void set(DMassC m) {
		_mass = m.getMass();
		_c = new DVector3(m.getC());
		_I = new DMatrix3(m.getI());
	}
	
	@Override
	public String toString() {
		FormattedStringBuilder b = new FormattedStringBuilder();
		b.appendln("Mass = " + _mass);
		b.appendln("c = ", _c.toString());
		b.appendln("I = ", _I.toString());
		return b.toString();
	}
	
	// *****************************************8
	// dMass API
	// *****************************************8
	
	
	@Override
	public void setZero()
	{ dMassSetZero (); }
	
	@Override
	public void setParameters (double themass, double cgx, double cgy, double cgz,
			double I11, double I22, double I33,
			double I12, double I13, double I23)
	{ dMassSetParameters (themass,cgx,cgy,cgz,I11,I22,I33,I12,I13,I23); }
	
	@Override
	public void setSphere (double density, double radius)
	{ dMassSetSphere (density,radius); }
	
	@Override
	public void setSphereTotal(double total, double radius) 
	{ dMassSetSphereTotal(total, radius);	}

	@Override
	public void setCapsule (double density, int direction, double radius, double length)
	{ dMassSetCapsule (density,direction,radius,length); }
	
	@Override
	public void setCapsuleTotal (double total, int direction, double radius, double length)
	{ dMassSetCapsuleTotal (total,direction,radius,length); }
	
	@Override
	public void setCylinder (double density, int direction, double radius, double length)
	{ dMassSetCylinder(density, direction, radius, length); }
	
	@Override
	public void setCylinderTotal (double total, int direction, double radius, double length)
	{ dMassSetCylinderTotal(total, direction, radius, length); }
	
	@Override
	public void setBox (double density, double lx, double ly, double lz)
	{ dMassSetBox (density,lx,ly,lz); }
	
	@Override
	public void setBox (double density, DVector3C lxyz)
	{ dMassSetBox (density, lxyz.get0(), lxyz.get1(), lxyz.get2()); }
	
	@Override
	public void setBoxTotal (double total, double lx, double ly, double lz)
	{ dMassSetBoxTotal (total,lx,ly,lz); }
	
	@Override
	public void setTrimesh(double density, DTriMesh geom) {
		dMassSetTrimesh(density, geom);
	}

	@Override
	public void setTrimeshTotal(double total, DTriMesh geom) {
		dMassSetTrimeshTotal(total, geom);
	}

	@Override
	public void adjust (double newmass)
	{ dMassAdjust (newmass); }
	
	@Override
	public void translate (double x, double y, double z)
	{ dMassTranslate (new DVector3(x,y,z)); }
	
	@Override
	public void translate (DVector3C xyz)
	{ dMassTranslate (xyz); }
	
	@Override
	public void rotate (DMatrix3C R)
	{ dMassRotate (R); }
	
	@Override
	public void add (DMassC b)
	{ dMassAdd (b); }

	
	
	/**
	 * TZ
	 */
	@Override
	public DVector3C getC() {
		return _c;
	}

	/**
	 * TZ
	 */
	@Override
	public double getMass() {
		return _mass;
	}

	/**
	 * TZ
	 */
	@Override
	public void setMass(double d) {
		_mass = d;
	}
	
	/**
	 * TZ
	 */
	@Override
	public DMatrix3C getI() {
		return _I;
	}
	
	/**
	 * TZ
	 */
	@Override
	public void setC(DVector3C c) {
		_c.set(c);
	}

	/**
	 * TZ
	 */
	@Override
	public void setI(DMatrix3C I) {
		_I.set(I);
	}

	@Override
	public boolean check() {
		return dMassCheck();
	}
}
