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

import java.util.Arrays;

/*
 * Based on https://github.com/turbulenz/turbulenz_engine/blob/master/tslib/aabbtree.ts
 */
public class AABBTree<T> {

    public static final int UNDEFINED_INDEX = -1;
    private static final int FREE_NODES_POOL_SIZE = 100;

    private final ExternalObjectHandler<T> externalObjectHandler;
    private final int numNodesLeaf;
    private final double fatAabbMargin;

    private AABBTreeNode<T>[] nodes;
    private int endNode;
    private boolean needsRebuild;
    private boolean needsRebound;
    private int numAdds;
    private int numUpdates;
    private int numRemoves;
    private int numExternalNodes;
    private int startUpdate;
    private int endUpdate;
    private boolean highQuality;
    private int nodesStack[];

    @SuppressWarnings("unchecked")
    public AABBTree(ExternalObjectHandler<T> externalObjectHandler, int numNodesLeaf, boolean highQuality, double fatAabbMargin) {
        this.externalObjectHandler = externalObjectHandler;
        this.numNodesLeaf = numNodesLeaf;
        this.highQuality = highQuality;
        this.fatAabbMargin = fatAabbMargin;
        startUpdate = 0x7FFFFFFF;
        endUpdate = -0x7FFFFFFF;
        nodesStack = new int[32];
        nodes = (AABBTreeNode<T>[]) new AABBTreeNode<?>[FREE_NODES_POOL_SIZE];
    }

    private void allocateNodes(int size) {
        if (nodes.length < size + FREE_NODES_POOL_SIZE) {
            nodes = Arrays.copyOf(nodes, size + FREE_NODES_POOL_SIZE);
        }
    }

    private void ensureNodes(int size) {
        if (nodes.length <= size) {
            allocateNodes(size);
        }
    }

    AABBTreeNode<T> allocateNode(double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
            int escapeNodeOffset, T externalNode, boolean isStatic) {
        return new AABBTreeNode<T>(minX, minY, minZ, maxX, maxY, maxZ, escapeNodeOffset, externalNode, isStatic);
    }

    public void add(T object, double[] extents) {
        AABBTreeNode<T> node = allocateNode(extents[0], extents[1], extents[2], extents[3], extents[4], extents[5], 1,
                object, externalObjectHandler.isStatic(object));
        externalObjectHandler.setSpatialIndex(object, endNode);
        ensureNodes(endNode);
        nodes[endNode] = node;
        endNode++;
        needsRebuild = true;
        numAdds++;
        numExternalNodes++;
    }

    public void remove(T object) {
        int index = externalObjectHandler.getSpatialIndex(object);
        if (index != UNDEFINED_INDEX) {
            numRemoves++;
            if (numExternalNodes > 1) {
                // assert(nodes.get(index).externalNode == externalNode);
                nodes[index].clear();
                if ((index + 1) >= endNode) {
                    while (!nodes[endNode - 1].isLeaf()) {
                        endNode--;
                    }
                } else {
                    needsRebuild = true;
                }
                numExternalNodes--;
            } else {
                clear();
            }
            externalObjectHandler.setSpatialIndex(object, UNDEFINED_INDEX);
        }
    }

    private AABBTreeNode<T> findParent(int nodeIndex) {
        int parentIndex = nodeIndex;
        int nodeDist = 0;
        AABBTreeNode<T> parent;
        do {
            parentIndex -= 1;
            nodeDist += 1;
            parent = nodes[parentIndex];
        } while (parent.escapeNodeOffset <= nodeDist);
        return parent;
    }

    public void update(T object, double[] extents) {

        int index = externalObjectHandler.getSpatialIndex(object);
        if (index != UNDEFINED_INDEX) {
            double min0 = extents[0];
            double min1 = extents[1];
            double min2 = extents[2];
            double max0 = extents[3];
            double max1 = extents[4];
            double max2 = extents[5];

            AABBTreeNode<T> node = nodes[index];

            boolean doUpdate = (needsRebuild || needsRebound || node.minX > min0 || node.minY > min1 || node.minZ > min2
                    || node.maxX < max0 || node.maxY < max1 || node.maxZ < max2);

            if (doUpdate) {
                node.bounds(min0, min1, min2, max0, max1, max2);
                if (!needsRebuild && endNode > 1) {
                    numUpdates++;
                    if (!needsRebound) {
                        // force a rebound when things change too much
                        if ((2 * numUpdates) > numExternalNodes) {
                            needsRebound = true;
                        } else {
                            AABBTreeNode<T> parent = findParent(index);
                            if (parent.minX > min0 || parent.minY > min1 || parent.minZ > min2 || parent.maxX < max0
                                    || parent.maxY < max1 || parent.maxZ < max2) {
                                needsRebound = true;
                            }
                        }
                    } else {
                        // force a rebuild when things change too much
                        if (numUpdates > (3 * numExternalNodes)) {
                            needsRebuild = true;
                            numAdds = numUpdates;
                        }
                    }
                    if (needsRebound) {
                        if (startUpdate > index) {
                            startUpdate = index;
                        }
                        if (endUpdate < index) {
                            endUpdate = index;
                        }
                        // force a rebuild when things change too much
                        if (2 * (endUpdate - startUpdate) > endNode) {
                            needsRebuild = true;
                        }
                    }
                }
            }
        } else {
            add(object, extents);
        }
    }

    boolean needsFinalize() {
        return needsRebuild || needsRebound;
    }

    public boolean finalizeUpdate() {
        boolean b = needsRebuild;
        if (needsRebuild) {
            rebuild();
        } else if (needsRebound) {
            rebound();
        }
        return b;
    }

    private void rebound() {
        if (endNode > 1) {
            int startUpdateNodeIndex = startUpdate;
            int endUpdateNodeIndex = endUpdate;

            int numNodesStack = 0;
            int topNodeIndex = 0;
            for (;;) {
                AABBTreeNode<T> topNode = nodes[topNodeIndex];
                int currentNodeIndex = topNodeIndex;
                int currentEscapeNodeIndex = (topNodeIndex + topNode.escapeNodeOffset);
                int nodeIndex = (topNodeIndex + 1); // First child
                do {
                    AABBTreeNode<T> node = nodes[nodeIndex];
                    int escapeNodeIndex = (nodeIndex + node.escapeNodeOffset);
                    if (nodeIndex < endUpdateNodeIndex) {
                        if (!node.isLeaf()) {
                            if (escapeNodeIndex > startUpdateNodeIndex) {
                                nodesStack[numNodesStack] = topNodeIndex;
                                numNodesStack++;
                                topNodeIndex = nodeIndex;
                            }
                        }
                    } else {
                        break;
                    }
                    nodeIndex = escapeNodeIndex;
                } while (nodeIndex < currentEscapeNodeIndex);

                if (topNodeIndex == currentNodeIndex) {
                    nodeIndex = (topNodeIndex + 1); // First child
                    AABBTreeNode<T> node = nodes[nodeIndex];

                    double minX = node.minX;
                    double minY = node.minY;
                    double minZ = node.minZ;
                    double maxX = node.maxX;
                    double maxY = node.maxY;
                    double maxZ = node.maxZ;

                    nodeIndex = (nodeIndex + node.escapeNodeOffset);
                    while (nodeIndex < currentEscapeNodeIndex) {
                        node = nodes[nodeIndex];
                        if (minX > node.minX) {
                            minX = node.minX;
                        }
                        if (minY > node.minY) {
                            minY = node.minY;
                        }
                        if (minZ > node.minZ) {
                            minZ = node.minZ;
                        }
                        if (maxX < node.maxX) {
                            maxX = node.maxX;
                        }
                        if (maxY < node.maxY) {
                            maxY = node.maxY;
                        }
                        if (maxZ < node.maxZ) {
                            maxZ = node.maxZ;
                        }
                        nodeIndex = (nodeIndex + node.escapeNodeOffset);
                    }

                    topNode.bounds(minX, minY, minZ, maxX, maxY, maxZ);

                    endUpdateNodeIndex = topNodeIndex;

                    if (numNodesStack > 0) {
                        numNodesStack--;
                        topNodeIndex = nodesStack[numNodesStack];
                    } else {
                        break;
                    }
                }
            }
        }

        needsRebuild = false;
        needsRebound = false;
        numAdds = 0;
        // numUpdates = 0;
        startUpdate = 0x7FFFFFFF;
        endUpdate = -0x7FFFFFFF;
    }

    @SuppressWarnings("unchecked")
    void rebuild() {
        if (numExternalNodes > 0) {
            int numBuildNodes = 0;
            AABBTreeNode<T>[] buildNodes;
            if (numExternalNodes == endNode) {
                buildNodes = nodes;
                numBuildNodes = endNode;
                nodes = (AABBTreeNode<T>[]) new AABBTreeNode<?>[FREE_NODES_POOL_SIZE];
            } else {
                buildNodes = (AABBTreeNode<T>[]) new AABBTreeNode<?>[numExternalNodes];
                int endNodeIndex = endNode;
                for (int n = 0; n < endNodeIndex; n++) {
                    AABBTreeNode<T> currentNode = nodes[n];
                    if (currentNode.isLeaf()) {
                        nodes[n] = null;
                        buildNodes[numBuildNodes++] = currentNode;
                    }
                }
            }

            if (numBuildNodes > 1) {
                if (numBuildNodes > numNodesLeaf && numAdds > 0) {
                    if (highQuality) {
                        HQSort.INSTANCE.sortNodes(buildNodes, numBuildNodes, numNodesLeaf);
                    } else {
                        LQSort.INSTANCE.sortNodes(buildNodes, numBuildNodes, numNodesLeaf);
                    }
                }
                int predictedNumNodes = predictNumNodes(0, numBuildNodes, 0);
                allocateNodes(predictedNumNodes);
                recursiveBuild(buildNodes, 0, numBuildNodes, 0);
                endNode = nodes[0].escapeNodeOffset;
            } else {
                AABBTreeNode<T> rootNode = buildNodes[0];
                externalObjectHandler.setSpatialIndex(rootNode.externalObject, 0);
                nodes[0] = rootNode;
                endNode = 1;
            }
        }

        needsRebuild = false;
        needsRebound = false;
        numAdds = 0;
        numUpdates = 0;
        numRemoves = 0;
        startUpdate = 0x7FFFFFFF;
        endUpdate = -0x7FFFFFFF;
    }

    void recursiveBuild(AABBTreeNode<T>[] buildNodes, int startIndex, int endIndex, int lastNodeIndex) {

        int nodeIndex = lastNodeIndex;
        lastNodeIndex++;

        boolean isStatic;
        double minX, minY, minZ, maxX, maxY, maxZ;

        AABBTreeNode<T> lastNode;
        if ((startIndex + numNodesLeaf) >= endIndex) {
            AABBTreeNode<T> buildNode = buildNodes[startIndex];
            minX = buildNode.minX;
            minY = buildNode.minY;
            minZ = buildNode.minZ;
            maxX = buildNode.maxX;
            maxY = buildNode.maxY;
            maxZ = buildNode.maxZ;
            isStatic = buildNode.isStatic;

            externalObjectHandler.setSpatialIndex(buildNode.externalObject, lastNodeIndex);

            nodes[lastNodeIndex] = buildNode; // replaceNode

            for (int n = (startIndex + 1); n < endIndex; n += 1) {
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
                isStatic = isStatic && buildNode.isStatic;
                lastNodeIndex++;
                externalObjectHandler.setSpatialIndex(buildNode.externalObject, lastNodeIndex);
                nodes[lastNodeIndex] = buildNode; // replaceNode
            }
            minX -= fatAabbMargin;
            minY -= fatAabbMargin;
            minZ -= fatAabbMargin;
            maxX += fatAabbMargin;
            maxY += fatAabbMargin;
            maxZ += fatAabbMargin;
            lastNode = nodes[lastNodeIndex];
        } else {
            int splitPosIndex = ((startIndex + endIndex) >> 1);
            if ((startIndex + 1) >= splitPosIndex) {
                AABBTreeNode<T> buildNode = buildNodes[startIndex];
                externalObjectHandler.setSpatialIndex(buildNode.externalObject, lastNodeIndex);
                nodes[lastNodeIndex] = buildNode; // replaceNode
            } else {
                recursiveBuild(buildNodes, startIndex, splitPosIndex, lastNodeIndex);
            }
            lastNode = nodes[lastNodeIndex];
            minX = lastNode.minX;
            minY = lastNode.minY;
            minZ = lastNode.minZ;
            maxX = lastNode.maxX;
            maxY = lastNode.maxY;
            maxZ = lastNode.maxZ;
            isStatic = lastNode.isStatic;

            lastNodeIndex = (lastNodeIndex + lastNode.escapeNodeOffset);

            if ((splitPosIndex + 1) >= endIndex) {
                AABBTreeNode<T> buildNode = buildNodes[splitPosIndex];
                externalObjectHandler.setSpatialIndex(buildNode.externalObject, lastNodeIndex);
                nodes[lastNodeIndex] = buildNode; // replaceNode
            } else {
                recursiveBuild(buildNodes, splitPosIndex, endIndex, lastNodeIndex);
            }

            lastNode = nodes[lastNodeIndex];
            if (minX > lastNode.minX) {
                minX = lastNode.minX;
            }
            if (minY > lastNode.minY) {
                minY = lastNode.minY;
            }
            if (minZ > lastNode.minZ) {
                minZ = lastNode.minZ;
            }
            if (maxX < lastNode.maxX) {
                maxX = lastNode.maxX;
            }
            if (maxY < lastNode.maxY) {
                maxY = lastNode.maxY;
            }
            if (maxZ < lastNode.maxZ) {
                maxZ = lastNode.maxZ;
            }
            isStatic = isStatic && lastNode.isStatic;
        }
        if (nodes[nodeIndex] == null) {
            nodes[nodeIndex] = allocateNode(minX, minY, minZ, maxX, maxY, maxZ,
                    (lastNodeIndex + lastNode.escapeNodeOffset - nodeIndex), null, isStatic);
        } else {
            nodes[nodeIndex].reset(minX, minY, minZ, maxX, maxY, maxZ,
                    (lastNodeIndex + lastNode.escapeNodeOffset - nodeIndex), null, isStatic);
        }
    }

    private int predictNumNodes(int startIndex, int endIndex, int lastNodeIndex) {
        lastNodeIndex++;
        if ((startIndex + numNodesLeaf) >= endIndex) {
            lastNodeIndex += (endIndex - startIndex);
        } else {
            int splitPosIndex = ((startIndex + endIndex) >> 1);
            if ((startIndex + 1) >= splitPosIndex) {
                lastNodeIndex++;
            } else {
                lastNodeIndex = predictNumNodes(startIndex, splitPosIndex, lastNodeIndex);
            }
            if ((splitPosIndex + 1) >= endIndex) {
                lastNodeIndex++;
            } else {
                lastNodeIndex = predictNumNodes(splitPosIndex, endIndex, lastNodeIndex);
            }
        }
        return lastNodeIndex;
    }

    @SuppressWarnings("unchecked")
    void clear() {
        nodes = (AABBTreeNode<T>[]) new AABBTreeNode<?>[0];
        needsRebuild = false;
        needsRebound = false;
        numAdds = 0;
        numUpdates = 0;
        numExternalNodes = 0;
        endNode = 0;
        startUpdate = 0x7FFFFFFF;
        endUpdate = -0x7FFFFFFF;
    }

    public void getOverlappingPairs(AABBTreePairCallback<T> callback) {
        if (numExternalNodes > 1) {
            for (int currentNodeIndex = 0; currentNodeIndex < endNode; currentNodeIndex++) {
                while (!nodes[currentNodeIndex].isLeaf()) {
                    currentNodeIndex++;
                }
                AABBTreeNode<T> currentNode = nodes[currentNodeIndex];
                if (!externalObjectHandler.isEnabled(currentNode.externalObject)) {
                    continue;
                }
                double minX = currentNode.minX;
                double minY = currentNode.minY;
                double minZ = currentNode.minZ;
                double maxX = currentNode.maxX;
                double maxY = currentNode.maxY;
                double maxZ = currentNode.maxZ;

                for (int nodeIndex = currentNodeIndex + 1; nodeIndex < endNode;) {
                    AABBTreeNode<T> node = nodes[nodeIndex];
                    if ((!currentNode.isStatic || !node.isStatic) && minX <= node.maxX && minY <= node.maxY
                            && minZ <= node.maxZ && maxX >= node.minX && maxY >= node.minY && maxZ >= node.minZ) {
                        nodeIndex++;
                        if (node.isLeaf()) {
                            if (externalObjectHandler.isEnabled(node.externalObject)) {
                                callback.overlap(currentNode.externalObject, node.externalObject);
                            }
                        }
                    } else {
                        nodeIndex += node.escapeNodeOffset;
                    }
                }
            }
        }
    }

    public void getOverlappingNodes(double[] queryExtents, AABBTreeNodeCallback<T> callback) {
        if (numExternalNodes > 0) {
            double queryMinX = queryExtents[0];
            double queryMinY = queryExtents[1];
            double queryMinZ = queryExtents[2];
            double queryMaxX = queryExtents[3];
            double queryMaxY = queryExtents[4];
            double queryMaxZ = queryExtents[5];
            int endNodeIndex = endNode;
            int nodeIndex = 0;
            for (;;) {
                AABBTreeNode<T> node = nodes[nodeIndex];
                double minX = node.minX;
                double minY = node.minY;
                double minZ = node.minZ;
                double maxX = node.maxX;
                double maxY = node.maxY;
                double maxZ = node.maxZ;
                if (queryMinX <= maxX && queryMinY <= maxY && queryMinZ <= maxZ && queryMaxX >= minX
                        && queryMaxY >= minY && queryMaxZ >= minZ) {
                    if (node.isLeaf()) {
                        if (externalObjectHandler.isEnabled(node.externalObject)) {
                            callback.overlap(node.externalObject);
                        }
                        nodeIndex++;
                        if (nodeIndex >= endNodeIndex) {
                            break;
                        }
                    } else {
                        if (queryMaxX >= maxX && queryMaxY >= maxY && queryMaxZ >= maxZ && queryMinX <= minX
                                && queryMinY <= minY && queryMinZ <= minZ) {
                            int endChildren = (nodeIndex + node.escapeNodeOffset);
                            nodeIndex++;
                            do {
                                node = nodes[nodeIndex];
                                if (node.isLeaf() && externalObjectHandler.isEnabled(node.externalObject)) {
                                    callback.overlap(node.externalObject);
                                }
                                nodeIndex++;
                            } while (nodeIndex < endChildren);
                            if (nodeIndex >= endNodeIndex) {
                                break;
                            }
                        } else {
                            nodeIndex++;
                        }
                    }
                } else {
                    nodeIndex += node.escapeNodeOffset;
                    if (nodeIndex >= endNodeIndex) {
                        break;
                    }
                }
            }
        }
    }

}
