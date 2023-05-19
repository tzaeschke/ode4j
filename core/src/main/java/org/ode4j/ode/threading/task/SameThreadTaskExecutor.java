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

import java.util.LinkedList;
import java.util.Queue;

public class SameThreadTaskExecutor extends AbstractTaskExecutor {

    private final Queue<Task> queue = new LinkedList<>();

	public SameThreadTaskExecutor() {}

    @Override
	public void submit(Task task) {
        queue.add(task);
	}

	@Override
	public int getThreadCount() {
		return 1;
	}

	public void flush() {
	    for (Task task = queue.poll(); task != null; task = queue.poll()) {
            if (task.parent != null) {
                task.parent.subtaskCompleted();
            }
	        task.runnable.run();
	        task.completed.countDown();
	    }
	}
}
