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
package org.ode4j.drawstuff;


import static org.ode4j.drawstuff.DrawStuff.*;

public class DsTest extends dsFunctions {

	@Override
	public void start()
	{
		// adjust the starting viewpoint a bit
		float[] xyz = new float[3], hpr = new float[3];
		dsGetViewpoint (xyz,hpr);
		hpr[0] += 7;
		dsSetViewpoint (xyz,hpr);
	}

	private float a = 0;

	private void simLoop (boolean pause)
	{
		float[] pos = new float[3];
		float[] R = new float[12];

		if (!pause) a += 0.02f;
		if (a > (2*Math.PI)) a -= (float) (2*Math.PI);
		float ca = (float) Math.cos(a);
		float sa = (float) Math.sin(a);

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

		float b = (a > Math.PI) ? (2*(a-(float)Math.PI)) : a*2;
		pos[0] = -0.3f;
		pos[1] = 0;
		pos[2] = (float) (0.1f*(2*Math.PI*b - b*b) + 0.65f);
		R[0] = ca; R[1] = 0; R[2] = -sa;
		R[4] = 0;  R[5] = 1; R[6] = 0;
		R[8] = sa; R[9] = 0; R[10] = ca;
		dsSetColor (1,0.8f,0.6f);
		dsDrawSphere (pos,R,0.3f);

		dsSetTexture (DS_TEXTURE_NUMBER.DS_NONE);

		pos[0] = -0.2f;
		pos[1] = 0.8f;
		pos[2] = 0.4f;
		R[0] = ca; R[1] = -sa; R[2] = 0;
		R[4] = sa; R[5] = ca;  R[6] = 0;
		R[8] = 0;  R[9] = 0;	 R[10] = 1;
		float[] sides = {0.1f,0.4f,0.8f};
		dsSetColor (0.6f,0.6f,1);
		dsDrawBox (pos,R,sides);

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

		float r = 0.3f;		      // cylinder radius
		float d = (float)Math.cos(a*2) * 0.4f;
		float cd = (float)Math.cos(-d/r);
		float sd = (float)Math.sin(-d/r);
		pos[0] = -0.2f;
		pos[1] = -1 + d;
		pos[2] = 0.3f;
		R[0] = 0;   R[1] = 0;  R[2] = -1;
		R[4] = -sd; R[5] = cd; R[6] =  0;
		R[8] =  cd; R[9] = sd; R[10] = 0;
		dsSetColor (0.4f,1,1);
		dsDrawCylinder (pos,R,0.8f,r);

		pos[0] = 0;
		pos[1] = 0;
		pos[2] = 0.2f;
		R[0] = 0; R[1] = sa; R[2] = -ca;
		R[4] = 0; R[5] = ca; R[6] = sa;
		R[8] = 1; R[9] = 0;  R[10] = 0;
		dsSetColor (1,0.9f,0.2f);
		dsDrawCapsule (pos,R,0.8f,0.2f);
	}


	@Override
	public void command (char cmd)
	{
		dsPrint ("received command %d (`%c')\n",cmd,cmd);
	}


	public static void main (String[] args)
	{
		new DsTest().demo(args);
	}
	
	private void demo(String[] args) {
		// run simulation
		dsSimulationLoop (args,400,400,this);
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
		//Nothing
	}
}