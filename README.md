ODE4J
=====

![Java 8](https://github.com/tzaeschke/ode4j/actions/workflows/build-java-8.yml/badge.svg)
![Java 9+](https://github.com/tzaeschke/ode4j/actions/workflows/build-java-9-plus.yml/badge.svg)

ode4j is a Java port of [ODE](http://www.ode.org/).

ODE is an open source, high performance library for simulating rigid body dynamics. It is fully featured, stable, mature and platform independent with an easy to use C/C++ API. It has advanced joint types and integrated collision detection with friction. ODE is useful for simulating vehicles, objects in virtual reality environments and virtual creatures.
It is currently used in many computer games, 3D authoring tools and simulation tools.

The latest released version of ode4j is 0.4.2, but the master branch may contains fixes and improvements.
Version 0.2.4 up to 0.2.9 are ports of ODE 0.12.1, Version 0.3.1 is a port of ODE 0.13.1. Release 0.4.0 contains most changes between 0.13.1 and 0.16.0.  

ode4j contains some custom features that are not present in ODE (see also [Wiki](https://github.com/tzaeschke/ode4j/wiki/Functionality-beyond-ODE)): 
 * `DRagdoll` & `DConstrainedBallJoint`, see `DemoRagdoll` and `DemoJointConstrainedBall`.
 * `DTrimeshHeightfield` with support for holes. See `DemoTrimeshHeightfield`.
 * Improved SAP space (`SapSpace2`) implementation that allows labelling bodies as "immobile", see `SpacePerformanceTest`.
 * BVH space based on a bounding volume hierarchy index, see `SpacePerformanceTest`.
 * Java multi-threading support, see `DemoMultiThreading`.

The [ODE forum](https://groups.google.com/forum/#!forum/ode-users) is useful for questions around physics and general API usage: 

There is a new [Discord channel](https://discord.gg/UFXJcXv2P8) around ode4j/Java.

The [ode4j forum](https://groups.google.com/forum/?hl=en#!forum/ode4j) is for problems and functionality specific to ode4j/Java. 

There is also the [old website](https://tzaeschke.github.io/ode4j-old/), including some [screenshots](https://tzaeschke.github.io/ode4j-old/ode4j-features.html).


The following artifact contains the complete physics engine (examples etc are [not included](https://github.com/tzaeschke/ode4j/wiki/Maven-HOWTO)):

``` 
<dependency>
    <groupId>org.ode4j</groupId>
    <artifactId>core</artifactId>
    <version>0.4.2</version>
</dependency>
```

News
====

2023-31-03: Release 0.4.1 & 0.4.2. Mostly a bugfix release + some API helper methods: 
 * Fix OSGI bundle info to require Java 1.7 instead of 7.0
 * New helper methods: `
   * `DBody` : `addLinearVelocity()`
   * `DVector3`: `reAdd()`, `eqToRadians()`, `eqToDegrees()` (convert angles in a DVector3, `eq` prefix means that the object is set equal to the result)
   * `DQuaternion`: `ZERO`, `IDENTITY`, `isEq()`, `length()`, `lengthSquared()`, `toEuler()`, `fromEuler()`, `toEulerDegrees()`, `fromEulerDegrees()`, `eqInverse()`, `reInverse()`.
 * 0.4.2 fixes some small regressions with 0.4.1   

2019-01-03: Release 0.4.0. This release contains most of the changes that happened between ODE 0.13.1 and ODE 0.16.0, plus some original features: 
 * Java 9 / modularization (generated jar files are Java 7) (io7m)
 * Implemented/migrated multi-threading for the stepper (Pjotr)
 * SAP-Space optimization: Avoid collision detection for immobile bodies (Pjotr)
 * New BVH tree for [better scalability with 10'000 bodies or more](https://github.com/tzaeschke/ode4j/pull/58), ported from the [Turbulenz Engine](https://github.com/turbulenz/turbulenz_engine) (Pjotr)
 * Fixed javadoc to compile without warnings
 

2018-03-26: Snapshot release 0.4.0
 * Java 9 / modularization (generated jar files are Java 7) (io7m)
 
2017-11-16: Snapshot release 0.4.0
 * Java 7 and updated dependencies
 * Implemented/migrated multi-threading for the stepper (Pjotr)
 * SAP-Space optimization: Avoid collision detection for immobile bodies (Pjotr)
 * New BVH tree for [better scalability with 10'000 bodies or more](https://github.com/tzaeschke/ode4j/pull/58), ported from the [Turbulenz Engine](https://github.com/turbulenz/turbulenz_engine) (Pjotr)
 

2017-10-06: Release of ode4j 0.3.1

 * Numerous bugfixes and improvement, see CHANGELOG
 * This is the last release built with Java 6.


Basic Usage Tips
================
 * Use `World.quickStep(...)` instead of `World.step()`. The latter is slower and appears to be less stable.
 * Make sure to set `Common.dNODEBUG = true` for best performance.
 * Avoid using `core-cpp` and ignore demos in `demo-cpp`. 


Legal
=====

ode4j:
Copyright (c) 2009-2023 Tilmann Zäschke <ode4j(AT)gmx.de>.
All rights reserved.

Like the original ODE, ode4j is licensed under LGPL v2.1 and BSD 3-clause. Choose whichever license suits your needs. 


### ode4j contains Java ports of the following software

[ODE/OpenDE](http://www.ode.org/):
Copyright  (c) 2001,2002 Russell L. Smith
All rights reserved.

GIMPACT (part of ODE/OpenDE):
Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
email: projectileman(AT)yahoo.com

[LIBCCD](https://github.com/danfis/libccd):
Copyright (c) 2010 Daniel Fiser <danfis(AT)danfis.cz>;
3-clause BSD License

[Turbulenz Engine](https://github.com/turbulenz/turbulenz_engine):
Copyright (c) 2009-2014 Turbulenz Limited; MIT License

### ode4j uses the following libraries

JUnit: 
Copyright © 2002-2014 JUnit. All Rights Reserved. 

slf4j: 
Copyright © 2004-2015 QOS.ch


Contact
=======

Tilmann Zaeschke
ode4j (AT) gmx.de

