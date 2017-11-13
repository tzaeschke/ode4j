/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J                                               *
 * Copyright (C) 2017 Piotr Piastucki, Tilmann Zaeschke                  *
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
package org.ode4j.ode.internal.aabbtree;

public class AABBTreeNode<T> {

	int escapeNodeOffset;
	T externalObject;
	boolean isStatic;
	double minX, minY, minZ, maxX, maxY, maxZ;

	AABBTreeNode(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, int escapeNodeOffset,
			T externalNode, boolean isStatic) {
	    reset(minX, minY, minZ, maxX, maxY, maxZ, escapeNodeOffset, externalNode, isStatic);
	}

	void reset(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, int escapeNodeOffset,
			T externalNode, boolean isStatic) {
		this.escapeNodeOffset = escapeNodeOffset;
		this.externalObject = externalNode;
        this.isStatic = isStatic;
        bounds(minX, minY, minZ, maxX, maxY, maxZ);
	}

    void bounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        this.minX = minX;
        this.minY = minY;
        this.minZ = minZ;
        this.maxX = maxX;
        this.maxY = maxY;
        this.maxZ = maxZ;
    }

	boolean isLeaf() {
        return this.externalObject != null;
    }

	void clear() {
		escapeNodeOffset = 1;
		externalObject = null;
		isStatic = false;
		minX = minY = minZ = Double.MAX_VALUE;
		maxX = maxY = maxZ =-Double.MAX_VALUE; 
	}

}
