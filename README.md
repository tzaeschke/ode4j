ODE4J
=====

ode4j is a Java port of [ODE](http://www.ode.org/).

ODE is an open source, high performance library for simulating rigid body dynamics. It is fully featured, stable, mature and platform independent with an easy to use C/C++ API. It has advanced joint types and integrated collision detection with friction. ODE is useful for simulating vehicles, objects in virtual reality environments and virtual creatures.
It is currently used in many computer games, 3D authoring tools and simulation tools.

The latest released version of ode4j is 0.3.0, but the master branch contains numerous fixes and improvements.
Version 0.2.4 up to 0.2.9 are ports of ODE 0.12.1, Version 0.3.0 is a port of ODE 0.13.1

ode4j contains also some features that are not present in ODE, such as a ragdoll and heightfields with holes. See [Wiki](https://github.com/tzaeschke/ode4j/wiki/Functionality-beyond-ODE).

The [ODE forum](https://groups.google.com/forum/#!forum/ode-users) is useful for questions around physics and general API usage: 

The [ode4j forum](https://groups.google.com/forum/?hl=en#!forum/ode4j) is for problems and functionality specific to ode4j/Java. 

There is also the [old website](http://www.zaeschke.com/ode4j/), including some [screenshots](http://www.zaeschke.com/ode4j/ode4j-features.html).


News
====

2017-11-16: Snapshot release 0.3.2
 * Java 7 and updated dependencies
 * Implemented/migrated multi-threading for the stepper (Pjotr)
 * SAP-Space optimization: Avoid collision detection for immobile bodies (Pjotr)
 

2017-10-06: Release of ode4j 0.3.1

 * Numerous bugfixes and improvement, see CHANGELOG
 * This is the last release built with Java 6.


Legal
=====

ode4j:
Copyright (c) 2009-2017 Tilmann Zäschke <ode4j(AT)gmx.de>.
All rights reserved.




Like the original ODE, ode4j is licensed under GPL v2.1 and BSD 3-clause. Choose whichever license suits your needs. 


### ode4j contains Java ports of the following software

ODE/OpenDE:
Copyright  (c) 2001,2002 Russell L. Smith
All rights reserved.

GIMPACT:
Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
email: projectileman(AT)yahoo.com

LIBCCD:
Copyright (c) 2010 Daniel Fiser <danfis(AT)danfis.cz>


### ode4j uses the following libraries

JUnit: 
Copyright © 2002-2014 JUnit. All Rights Reserved. 

slf4j: 
Copyright © 2004-2015 QOS.ch


Contact
=======

Tilmann Zaeschke
ode4j (AT) gmx.de

