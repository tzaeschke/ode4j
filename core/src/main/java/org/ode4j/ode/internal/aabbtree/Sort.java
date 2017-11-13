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
public abstract class Sort {

	protected void nthElement(AABBTreeNode<?>[] nodes, int first, int nth, int last, int axis, boolean reverse) {
		while ((last - first) > 3) {
			double midValue = medianFn(getkey(nodes[first], axis, reverse),
					getkey(nodes[first + ((last - first) >> 1)], axis, reverse),
					getkey(nodes[last - 1], axis, reverse));

			int firstPos = first;
			int lastPos = last;
			for (;;) {
				while (getkey(nodes[firstPos], axis, reverse) < midValue) {
					firstPos++;
				}
				do {
					lastPos--;
				} while (midValue < getkey(nodes[lastPos], axis, reverse));

				if (firstPos >= lastPos) {
					break;
				}
				AABBTreeNode<?> temp = nodes[firstPos];
				nodes[firstPos] = nodes[lastPos];
				nodes[lastPos] = temp;
				firstPos++;
			}
			if (firstPos <= nth) {
				first = firstPos;
			} else {
				last = firstPos;
			}
		}
		insertionSort(nodes, first, last, axis, reverse);
	}

	void insertionSort(AABBTreeNode<?>[] nodes, int first, int last, int axis, boolean reverse) {
		int sorted = first + 1;
		while (sorted != last) {
			AABBTreeNode<?> tempNode = nodes[sorted];
			double tempKey = getkey(tempNode, axis, reverse);

			int next = sorted;
			int current = sorted - 1;

			while (next != first && tempKey < getkey(nodes[current], axis, reverse)) {
				nodes[next] = nodes[current];
				next--;
				current--;
			}

			if (next != sorted) {
				nodes[next] = tempNode;
			}

			sorted++;
		}
	}

	protected abstract double getkey(AABBTreeNode<?> tempNode, int axis, boolean reverse);

	static double medianFn(double a, double b, double c) {
		if (a < b) {
			if (b < c) {
				return b;
			} else if (a < c) {
				return c;
			} else {
				return a;
			}
		} else if (a < c) {
			return a;
		} else if (b < c) {
			return c;
		}
		return b;
	}

	static double calculateSAH(AABBTreeNode<?>[] buildNodes, int startIndex, int endIndex) {
		AABBTreeNode<?> buildNode = buildNodes[startIndex];
        double minX = buildNode.minX;
        double minY = buildNode.minY;
        double minZ = buildNode.minZ;
        double maxX = buildNode.maxX;
        double maxY = buildNode.maxY;
        double maxZ = buildNode.maxZ;

		for (int n = startIndex + 1; n < endIndex; n++) {
			buildNode = buildNodes[n];
			if (minX > buildNode.minX) {
				minX = buildNode.minX;
			}
			if (minY > buildNode.minY) {
				minY = buildNode.minY;
			}
			if (minZ > buildNode.minZ) {
				minZ = buildNode.minZ;
			}
			if (maxX < buildNode.maxX) {
				maxX = buildNode.maxX;
			}
			if (maxY < buildNode.maxY) {
				maxY = buildNode.maxY;
			}
			if (maxZ < buildNode.maxZ) {
				maxZ = buildNode.maxZ;
			}
		}
		double x = maxX - minX;
		double y = maxY - minY;
		double z = maxZ - minZ;
		// return x * y * z;
		return x * y + x * z + y * z;
	}

}
