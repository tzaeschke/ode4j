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

/*
 * Based on https://github.com/turbulenz/turbulenz_engine/blob/master/tslib/aabbtree.ts
 */
public class HQSort extends Sort {

	final static HQSort INSTANCE = new HQSort();
	
	void sortNodes(AABBTreeNode<?>[] nodes, int numNodes, int numNodesLeaf) {
		sortNodesRecursive(nodes, 0, numNodes, false, numNodesLeaf);
	}

	void sortNodesRecursive(AABBTreeNode<?>[] nodes, int startIndex, int endIndex, boolean reverse,
			int numNodesLeaf) {
		int splitNodeIndex = ((startIndex + endIndex) >> 1);

		nthElement(nodes, startIndex, splitNodeIndex, endIndex, 0, false);
		double sahX = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

		nthElement(nodes, startIndex, splitNodeIndex, endIndex, 1, false);
		double sahY = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

		nthElement(nodes, startIndex, splitNodeIndex, endIndex, 2, false);
		double sahZ = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

		nthElement(nodes, startIndex, splitNodeIndex, endIndex, 3, false);
		double sahXZ = (calculateSAH(nodes, startIndex, splitNodeIndex)
				+ calculateSAH(nodes, splitNodeIndex, endIndex));

		nthElement(nodes, startIndex, splitNodeIndex, endIndex, 4, false);
		double sahZX = (calculateSAH(nodes, startIndex, splitNodeIndex)
				+ calculateSAH(nodes, splitNodeIndex, endIndex));

		int axis;
		if (sahX <= sahY && sahX <= sahZ && sahX <= sahXZ && sahX <= sahZX) {
			axis = 0;
		} else if (sahZ <= sahY && sahZ <= sahXZ && sahZ <= sahZX) {
			axis = 2;
		} else if (sahY <= sahXZ && sahY <= sahZX) {
			axis = 1;
		} else if (sahXZ <= sahZX) {
			axis = 3;
		} else {
			axis = 4;
		}

		nthElement(nodes, startIndex, splitNodeIndex, endIndex, axis, reverse);
		reverse = !reverse;
		if ((startIndex + numNodesLeaf) < splitNodeIndex) {
			sortNodesRecursive(nodes, startIndex, splitNodeIndex, reverse, numNodesLeaf);
		}
		if ((splitNodeIndex + numNodesLeaf) < endIndex) {
			sortNodesRecursive(nodes, splitNodeIndex, endIndex, reverse, numNodesLeaf);
		}
	}

	protected double getkey(AABBTreeNode<?> node, int axis, boolean reverse) {
		double v;
		switch (axis) {
		case 0:
            v = node.minX + node.maxX;
		    break;
		case 1:
            v = node.minY + node.maxY;
            break;
        case 2:
            v = node.minZ + node.maxZ;
            break;
        case 3:
            v = node.minX + node.maxX + node.minZ + node.maxZ;
            break;
        default:
            v = node.minX - node.maxX + node.minZ - node.maxZ;
            break;
		}
		return reverse ? -v : v;
	}

}
