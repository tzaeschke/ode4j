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
package org.ode4j.drawstuff.internal;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;


/**
 * functions supplied and used by the platform specific code.
 */
abstract class Internal {

	// supplied by platform specific code

	abstract void dsPlatformSimLoop (int window_width, int window_height,
			dsFunctions fn, boolean initial_pause);


	// used by platform specific code

	abstract void dsStartGraphics (int width, int height, dsFunctions fn);
	abstract void dsDrawFrame (int width, int height, dsFunctions fn, boolean pause);
	abstract void dsStopGraphics();
	abstract void dsMotion (int mode, int deltax, int deltay);

	abstract boolean dsGetShadows();
	abstract void dsSetShadows (boolean a);

	abstract boolean dsGetTextures();
	abstract void dsSetTextures (boolean a);
}
