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
package org.ode4j.ode.threading.task;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;

public class Task implements Runnable {

    public final TaskExecutor executor;
    public final String name;
    public final TaskGroup parent;
    public final Runnable runnable;
    protected final CountDownLatch completed;
    protected final AtomicInteger subtaskCount;

    protected Task(TaskExecutor executor, String name, TaskGroup parent, Runnable runnable) {
        this.executor = executor;
        this.name = name;
        this.parent = parent;
        this.runnable = runnable;
        subtaskCount = new AtomicInteger(0);
        completed = new CountDownLatch(1);
    }

    public void awaitCompletion() {
        try {
            executor.flush();
            completed.await();
        } catch (InterruptedException e) {
        }
    }

    public void submit() {
        executor.submit(this);
    }

    @Override
    public void run() {
        try {
//            System.out.println("Running " + name);
            runnable.run();
//            System.out.println("Completed " + name);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            completed.countDown();
            if (parent != null) {
                parent.subtaskCompleted();
            }
        }
    }

}
