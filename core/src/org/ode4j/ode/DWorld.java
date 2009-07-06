/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

/**
 * dynamics world.
 * 
 * From odecpp.h.
 */
public interface DWorld {

	//~dWorld()
	void DESTRUCTOR();

	//		  public dWorld id() 
	//		    { return _id; }
	////TZ		  operator dWorld() const
	////		    { return _id; }

	void setGravity (double x, double y, double z);
	void setGravity (DVector3C g);
	void getGravity (DVector3 g) ;

	void setERP (double erp);
	double getERP() ;

	void setCFM (double cfm);
	double getCFM() ;

	void step (double stepsize);

	void stepFast1 (double stepsize, int maxiterations);
	void setAutoEnableDepthSF1(int depth); 
	int getAutoEnableDepthSF1();

	void quickStep(double stepsize);
	void setQuickStepNumIterations(int num);
	int getQuickStepNumIterations() ;
	void setQuickStepW(double over_relaxation);
	double getQuickStepW();

	void  setAutoDisableLinearThreshold (double threshold);
	double getAutoDisableLinearThreshold();
	void setAutoDisableAngularThreshold (double threshold);
	double getAutoDisableAngularThreshold();
	void setAutoDisableSteps (int steps);
	int getAutoDisableSteps();
	void setAutoDisableTime (double time);
	double getAutoDisableTime();
	void setAutoDisableFlag (boolean do_auto_disable);
	int getAutoDisableFlag();

	double getLinearDampingThreshold();
	void setLinearDampingThreshold(double threshold);
	double getAngularDampingThreshold();
	void setAngularDampingThreshold(double threshold);
	double getLinearDamping();
	void setLinearDamping(double scale);
	void setAngularDamping(double scale);
	void setDamping(double linear_scale, double angular_scale);

	double getMaxAngularSpeed();
	void setMaxAngularSpeed(double max_speed);

	void setContactSurfaceLayer(double depth);
	double getContactSurfaceLayer();

	void impulseToForce (double stepsize, 
			double ix, double iy, double iz, DVector3 force);

	//TZ
	void setAutoDisableAverageSamplesCount (int average_samples_count);
	void setContactMaxCorrectingVel (double vel);
	void destroy();
	
	//	//private dWorld _id;
	//	private dxWorld _id;
	//
	//		  // intentionally undefined, don't use these
	////		  dWorld (const dWorld &);
	////		  void operator= (const dWorld &);
	//
	//		  public dWorld()
	//		    { _id = dxWorld.dWorldCreate(); }
	//		  //~dWorld()
	//		  @Override
	//		  protected void DESTRUCTOR()
	//		    { dWorldDestroy (_id); super.DESTRUCTOR(); }
	//
	//		  public dWorld id() 
	//		    { return _id; }
	////TZ TODO		  operator dWorld() const
	////		    { return _id; }
	//
	//		  public void setGravity (double x, double y, double z)
	//		    { _id.dWorldSetGravity (x,y,z); }
	//		  void setGravity (final dVector3 g)
	//		    { setGravity (g.v[0], g.v[1], g.v[2]); }
	//		  void getGravity (dVector3 g) 
	//		    { _id.dWorldGetGravity (_id,g); }
	//
	//		  void setERP (double erp)
	//		    { dWorldSetERP(_id, erp); }
	//		  double getERP() 
	//		    { return dWorldGetERP(_id); }
	//
	//		  void setCFM (double cfm)
	//		    { dWorldSetCFM(_id, cfm); }
	//		  double getCFM() 
	//		    { return dWorldGetCFM(_id); }
	//
	//		  public void step (double stepsize)
	//		    { dWorldStep (_id,stepsize); }
	//
	//		  void stepFast1 (double stepsize, int maxiterations)
	//		    { dWorldStepFast1 (_id,stepsize,maxiterations); }
	//		//TZ TODO report:		  void setAutoEnableDepthSF1(dWorld, int depth)
	//		  void setAutoEnableDepthSF1(int depth)
	//		    { dWorldSetAutoEnableDepthSF1 (_id, depth); }
	////TZ TODO report:		  int getAutoEnableDepthSF1(dWorld) 
	//		  int getAutoEnableDepthSF1() 
	//		    { return dWorldGetAutoEnableDepthSF1 (_id); }
	//
	//		  void quickStep(double stepsize)
	//		    { dWorldQuickStep (_id, stepsize); }
	//		  void setQuickStepNumIterations(int num)
	//		    { dWorldSetQuickStepNumIterations (_id, num); }
	//		  int getQuickStepNumIterations() 
	//		    { return dWorldGetQuickStepNumIterations (_id); }
	//		  void setQuickStepW(double over_relaxation)
	//		    { dWorldSetQuickStepW (_id, over_relaxation); }
	//		  double getQuickStepW() 
	//		    { return dWorldGetQuickStepW (_id); }
	//
	//		  void  setAutoDisableLinearThreshold (double threshold) 
	//		    { dWorldSetAutoDisableLinearThreshold (_id,threshold); }
	//		  double getAutoDisableLinearThreshold()
	//		    { return dWorldGetAutoDisableLinearThreshold (_id); }
	//		  void setAutoDisableAngularThreshold (double threshold)
	//		    { dWorldSetAutoDisableAngularThreshold (_id,threshold); }
	//		  double getAutoDisableAngularThreshold()
	//		    { return dWorldGetAutoDisableAngularThreshold (_id); }
	//		  void setAutoDisableSteps (int steps)
	//		    { dWorldSetAutoDisableSteps (_id,steps); }
	//		  int getAutoDisableSteps()
	//		    { return dWorldGetAutoDisableSteps (_id); }
	//		  void setAutoDisableTime (double time)
	//		    { dWorldSetAutoDisableTime (_id,time); }
	//		  double getAutoDisableTime() 
	//		    { return dWorldGetAutoDisableTime (_id); }
	//		  void setAutoDisableFlag (int do_auto_disable)
	//		    { dWorldSetAutoDisableFlag (_id,do_auto_disable); }
	//		  int getAutoDisableFlag() 
	//		    { return dWorldGetAutoDisableFlag (_id); }
	//
	//		  double getLinearDampingThreshold() 
	//		    { return dWorldGetLinearDampingThreshold(_id); }
	//		  void setLinearDampingThreshold(double threshold)
	//		    { dWorldSetLinearDampingThreshold(_id, threshold); }
	//		  double getAngularDampingThreshold() 
	//		    { return dWorldGetAngularDampingThreshold(_id); }
	//		  void setAngularDampingThreshold(double threshold)
	//		    { dWorldSetAngularDampingThreshold(_id, threshold); }
	//		  double getLinearDamping() 
	//		    { return dWorldGetLinearDamping(_id); }
	//		  void setLinearDamping(double scale)
	//		    { dWorldSetLinearDamping(_id, scale); }
	//		  double getAngularDamping() 
	//		    { return dWorldGetAngularDamping(_id); }
	//		  void setAngularDamping(double scale)
	//		    { dWorldSetAngularDamping(_id, scale); }
	//		  void setDamping(double linear_scale, double angular_scale)
	//		    { dWorldSetDamping(_id, linear_scale, angular_scale); }
	//
	//		  double getMaxAngularSpeed() 
	//		    { return dWorldGetMaxAngularSpeed(_id); }
	//		  void setMaxAngularSpeed(double max_speed)
	//		    { dWorldSetMaxAngularSpeed(_id, max_speed); }
	//
	//		  void setContactSurfaceLayer(double depth)
	//		    { dWorldSetContactSurfaceLayer (_id, depth); }
	//		  double getContactSurfaceLayer() 
	//		    { return dWorldGetContactSurfaceLayer (_id); }
	//
	//		  void impulseToForce (double stepsize, double ix, double iy, double iz,
	//				       dVector3 force)
	//		    { dWorldImpulseToForce (_id,stepsize,ix,iy,iz,force); }
	//
}
