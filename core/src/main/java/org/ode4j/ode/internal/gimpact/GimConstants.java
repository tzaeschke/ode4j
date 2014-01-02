/**
 * ----------------------------------------------------------------------------
 * This source file is part of the ODE4J library (ported to
 * Java from the GIMPACT Library).
 * 
 * For the latest info on ODE4J, see http://www.ode4j.org/
 * For the latest info on GIMPACT, see http://gimpact.sourceforge.net/
 * 
 * Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
 * email: projectileman@yahoo.com
 * Copyright of ODE4J (c) 2009-2014 Tilmann Zäschke.
 * email: ode4j.gmx.de
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file GIMPACT-LICENSE-LGPL.TXT and LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file GIMPACT-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-BSD.TXT, LICENSE.TXT and 
 * ODE4J-LICENSE-BSD.TXT for more details.
 * 
 * ----------------------------------------------------------------------------
 */
package org.ode4j.ode.internal.gimpact;

/**
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public interface GimConstants {
	static final int GUINT_BIT_COUNT = 32;
	static final int GUINT_EXPONENT = 5;

	/**
	 * 	Memory Access constants.
	 */
//	#define G_MA_READ_ONLY 1
//	#define G_MA_WRITE_ONLY 2
//	#define G_MA_READ_WRITE 3
	static final int G_MA_READ_ONLY = 1;
	static final int G_MA_WRITE_ONLY = 2;
	static final int G_MA_READ_WRITE = 3;

	/**
	 * Memory usage constants.
	 */
	/// Don't care how memory is used
	static final int G_MU_EITHER = 0;
	/// specified once, doesn't allow read information
	static final int G_MU_STATIC_WRITE = 1;
	/// specified once, allows to read information from a shadow buffer
	static final int G_MU_STATIC_READ = 2;
	/// write directly on buffer, allows to read information from a shadow buffer
	static final int G_MU_STATIC_READ_DYNAMIC_WRITE = 3;
	/// upload data to buffer from the shadow buffer, allows to read information from a shadow buffer
	static final int G_MU_STATIC_READ_DYNAMIC_WRITE_COPY = 4;
	/// specified once, allows to read information directly from memory
	static final int G_MU_STATIC_WRITE_DYNAMIC_READ = 5;
	/// write directly on buffer, allows to read information directly from memory
	static final int G_MU_DYNAMIC_READ_WRITE = 6;

	/**
	 * Buffer operation errors
	 */
//	#define G_BUFFER_OP_SUCCESS 0
//	#define G_BUFFER_OP_INVALID 1
//	#define G_BUFFER_OP_STILLREFCOUNTED 2
	static final int G_BUFFER_OP_SUCCESS = 0;
	static final int G_BUFFER_OP_INVALID = 1;
	static final int G_BUFFER_OP_STILLREFCOUNTED = 2;

	/**
	 * Buffer manager identifiers
	 */
	enum G_BUFFER_MANAGER
	{
		SYSTEM, //G_BUFFER_MANAGER_SYSTEM,
		SHARED; //G_BUFFER_MANAGER_SHARED,

		//MAX;//G_BUFFER_MANAGER__MAX
	};
}
