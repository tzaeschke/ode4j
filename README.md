# ode4j

![Java 8](https://github.com/tzaeschke/ode4j/actions/workflows/build-java-8.yml/badge.svg)
![Java 9+](https://github.com/tzaeschke/ode4j/actions/workflows/build-java-9-plus.yml/badge.svg)

ode4j is a Java port of [ODE](http://www.ode.org/).

ODE is an open source, high performance library for simulating rigid body dynamics. It is fully featured, stable, mature
and platform independent with an easy to use C/C++ API. It has advanced joint types and integrated collision detection
with friction. ODE is useful for simulating vehicles, objects in virtual reality environments and virtual creatures.

The latest released version of ode4j is 0.5.2, but the master branch may contain fixes and improvements. Release 0.5.0
contains all changes up to ODE 0.16.3.

Resources
 
* ode4j contains numerous [custom features](#additional-features-in-ode4j) that are not present in ODE (see
also [Wiki](https://github.com/tzaeschke/ode4j/wiki/Functionality-beyond-ODE)).

* The [ODE forum](https://groups.google.com/forum/#!forum/ode-users) is useful for questions around physics and general
API usage:

* There is a new [Discord channel](https://discord.gg/UFXJcXv2P8) around ode4j/Java.

* The [ode4j forum](https://groups.google.com/forum/?hl=en#!forum/ode4j) is for problems and functionality specific to
ode4j/Java.

* There is also the [old website](https://tzaeschke.github.io/ode4j-old/), including
some [screenshots](https://tzaeschke.github.io/ode4j-old/ode4j-features.html).

There is also a [GWT compatible fork](https://github.com/antzGames/gdx-ode4j) of ode4j.

The following artifact contains the complete physics engine (examples etc.
are [not included](https://github.com/tzaeschke/ode4j/wiki/Maven-HOWTO)):

``` 
<dependency>
    <groupId>org.ode4j</groupId>
    <artifactId>core</artifactId>
    <version>0.5.2</version>
</dependency>
```

## News

2023-10-07: Release 0.5.2:

* Bug fix for DVector3.cross() + some minor improvements

2023-09-17: Release 0.5.1:

* Bug fix for demos running on Apple Silicon/Retina

2023-05-27: Release 0.5.0. Full update to ODE 0.16.3 + Java 8 as baseline:

* Port of all changes up to ODE 0.16.3
* Java 8 required
* LWJGL 3.x for demos
* Tested Android compatibility with API level 24 (Android 7 / Nougat)
* CI for Java 8 + 9
* Improved API (mostly in `math` package)
* Cleanup (e.g. full JUnit 4 style test, fixed most lint warnings up to Java 17)
* Bug fixes
* **BREAKING CHANGE**: `DSpace.getGeoms()` now returns `DGeom` instead of `DxGeom`.

2023-03-31: Release 0.4.1 & 0.4.2. Mostly a bugfix release + some API helper methods:

* Fix OSGI bundle info to require Java 1.7 instead of 7.0
* New helper methods: `
    * `DBody` : `addLinearVelocity()`
    * `DVector3`: `reAdd()`, `eqToRadians()`, `eqToDegrees()` (convert angles in a DVector3, `eq` prefix means that the
      object is set equal to the result)
    * `DQuaternion`: `ZERO`, `IDENTITY`, `isEq()`, `length()`, `lengthSquared()`, `toEuler()`, `fromEuler()`, `toEulerDegrees()`, `fromEulerDegrees()`, `eqInverse()`, `reInverse()`.
* 0.4.2 fixes some small regressions with 0.4.1

2019-01-03: Release 0.4.0. This release contains most of the changes that happened between ODE 0.13.1 and ODE 0.16.0,
plus some original features:

* Java 9 / modularization (generated jar files are Java 7) (io7m)
* Implemented/migrated multi-threading for the stepper (Pjotr)
* SAP-Space optimization: Avoid collision detection for immobile bodies (Pjotr)
* New BVH tree for [better scalability with 10'000 bodies or more](https://github.com/tzaeschke/ode4j/pull/58), ported
  from the [Turbulenz Engine](https://github.com/turbulenz/turbulenz_engine) (Pjotr)
* Fixed javadoc to compile without warnings

2018-03-26: Snapshot release 0.4.0

* Java 9 / modularization (generated jar files are Java 7) (io7m)

2017-11-16: Snapshot release 0.4.0

* Java 7 and updated dependencies
* Implemented/migrated multi-threading for the stepper (Pjotr)
* SAP-Space optimization: Avoid collision detection for immobile bodies (Pjotr)
* New BVH tree for [better scalability with 10'000 bodies or more](https://github.com/tzaeschke/ode4j/pull/58), ported
  from the [Turbulenz Engine](https://github.com/turbulenz/turbulenz_engine) (Pjotr)

2017-10-06: Release of ode4j 0.3.1

* Numerous bugfixes and improvement, see CHANGELOG
* This is the last release built with Java 6.

## Project overview

* The "core" package contains the main library code. The public API is in:
    * `org.ode4j.math`
    * `org.ode4j.ode`
    * `org.ode4j.ode.ragdoll`

  All other packages are "internal" and should normally not be used.

* The "demo" package contains demo application for various simulation problems.
  The package contains the "drawstuff" library which is handy for visualizing simple
  physics simulation but should not be used for real applications (it is slow and clunky). Please
  use proper library such as [lwjgl](https://www.lwjgl.org/) for your own games or simulations.

* The "core-cpp" and "demo-cpp" packages are legacy packages and should be ignored.

## General recommendations

* Please use `DWorld.quickStep(...)` instead of `DWorld.step()`. The latter is slower and appears to be less stable.
* Consider disabling geometries that do not move. For an example, refer to [`DemoCrash`](https://github.com/tzaeschke/ode4j/blob/28a8338abccaccf13042e825ef615e0037aa91d3/demo/src/main/java/org/ode4j/demo/DemoCrash.java#L590).
The demo disables boxes that have not moved for a few steps. If a whole "island" of bodies is disabled, it is automatically excluded from simulation (bodies are automatically re-enabled when necessary), see also [ODE wiki](http://ode.org/wiki/index.php?title=Manual#Islands_and_Disabled_Bodies).
* Set `OdeConfig.dDEBUG = true` for debugging and `= false` for best performance.
* When compiling with JDK 9 or later, `mvn` will automatically use the`on-jdk-9-plus` profile and create modules.
* In case of GC problems (e.g. on Android):
  There may be excessive garbage collection due to frequent allocation of `DContact` objects.
  To avoid this, these objects can be nullified and reused. See `PerfCards.nearCallback()` for an example.

## ODE vs od4j

While ode4j is a pretty literal port from ODE, there are some considerable differences
(besides one being Java and the other being C++). Some differences concern the API, others
are mostly internal.

### ode4j supported ODE features

* ode4j ist mostly similar to ODE when compiling ODE with
  `./configure --enable-double-precision --with-trimesh=gimpact --enable-libccd`. That means:
    * ode4j uses `double` precision throughout
    * ode4j supports only `GIMPACT` trimeshes, `OPCODE` is not supported
    * ode4j always uses `libccd` for collisions
* ode4j has multithreading support (ODE's "island" solver), however, ode4j does not support
  ODE's cooperative solver.
* ode4j has no special memory management (ODE has a "memory arena" concept).

### Additional features in ode4j

ode4j contains some custom features that are not present in ODE (see
also [Wiki](https://github.com/tzaeschke/ode4j/wiki/Functionality-beyond-ODE)):

* `DRagdoll` & `DConstrainedBallJoint`, see `DemoRagdoll` and `DemoJointConstrainedBall`.
* `DTrimeshHeightfield` with support for holes. See `DemoTrimeshHeightfield`.
* Improved SAP space (`SapSpace2`) implementation that allows labelling bodies as "immobile",
  see `SpacePerformanceTest`.
* BVH space based on a bounding volume hierarchy index, see `SpacePerformanceTest`.
* Java multi-threading support, see `DemoMultiThreading`.

### ode4j API vs ODE API

ode4j's API is very similar to ODE's API, so most tutorials / demos should be easily translatable.
However, there are some notable differences:

* Almost all objects in ode4j are created via the `OdeHelper` class.
* Math functions are located in `ode4j.math` and `OdeMath`. ode4j has different versions for some math objects,
  e.g. `DVector3` and `DVector3C` where the trailing `C` indicates an immutable or **C**onst version of the object.
* Class names in ode4j start with `D` instead of `d` in ODE. `d...ID` classes have been removed,
  use `d...` instead, e.g. `dBodyID` becomes `DBody`.
* ODE uses almost exclusively static methods. ode4j uses instead methods on objects where possible and
  methods have been renamed accordingly and their signature changed. Some examples:
    * `dGeomSetRotation(geom, ...)` becomes `geom.setRotation(...)`
    * `dJointSetHingeAxis(hingeJoint, ...)` becomes `hingeJoint.setAxis(...)`

  Many ODE methods have a `skip` parameter, this has been removed in ode4j.
* `dCollide` becomes `OdeHelper.collide()` and takes as argument a `DContactGeomBuffer`.
  This can be created via:
   ```
   DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS); 
   OdeHelper.collide (o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());
   ```
* Many classes have a "DESTRUCTOR" that replicates some C++ features, however this can usually
  be ignored.

### ode4j internal differences

* ode4j's DLCP uses `ROWPTRS = false`
* SAP-Space uses different merge-sort instead of radix-sort
* ...

### Version overview

ode4j versions and corresponding ODE versions:

* ode4j 0.5.0 contains all changes up to ODE 0.16.3.
* ode4j 0.4.0 contains most changes between 0.13.1 and 0.16.0.
* ode4j 0.3.1 is a port of ODE 0.13.1.
* ode4j 0.2.4 up to 0.2.9 are ports of ODE 0.12.1

## Legal

ode4j:
Copyright (c) 2009-2023 Tilmann Zäschke <ode4j(AT)gmx.de>.
All rights reserved.

Like the original ODE, ode4j is licensed under LGPL v2.1 and BSD 3-clause. Choose whichever license suits your needs.

### ode4j contains Java ports of the following software

* [ODE/OpenDE](http://www.ode.org/):
  Copyright  (c) 2001,2002 Russell L. Smith
  All rights reserved.

* GIMPACT (part of ODE/OpenDE):
  Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
  email: projectileman(AT)yahoo.com

* [LIBCCD](https://github.com/danfis/libccd):
  Copyright (c) 2010 Daniel Fiser <danfis(AT)danfis.cz>;
  3-clause BSD License

* [Turbulenz Engine](https://github.com/turbulenz/turbulenz_engine):
  Copyright (c) 2009-2014 Turbulenz Limited; MIT License

### ode4j uses the following libraries

* [lwjgl](https://www.lwjgl.org/)maven

* [JUnit 4](https://junit.org/junit4/)

* [slf4j](https://www.slf4j.org/)

## Contact

Tilmann Zaeschke
ode4j (AT) gmx.de

## Special thanks to contributors

*Please let me know if I missed anyone!*

* [valb3r](https://github.com/valb3r)
* [io7m](https://github.com/io7m)
* [ppiastucki(piotr)](https://github.com/ppiastucki) <-- Shoutout for major contributions!
* [fommil](https://github.com/fommil)
* [peterkir](https://github.com/peterkir)
* [neduard](https://github.com/neduard)
* [Namek](https://github.com/Namek)
