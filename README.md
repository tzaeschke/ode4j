[![](https://jitpack.io/v/antzGames/ode4j.svg)](https://jitpack.io/#antzGames/ode4j)

# libGDX Open Dynamics Engine (ODE4J) - physics engine LIBRARY

![image](https://github.com/antzGames/gdx-ode4j/assets/10563814/c59c282d-9198-4066-81b7-0de4e5808f64)

This is a ODE4J library compatible with all libGDX backends, including GWT.  It is based on
version **0.5.1** of [Open Dynamics Engine for Java](https://github.com/tzaeschke/ode4j).

If you want to use ODE4J only on libGDX Desktop/Android/iOS backends then I recommend you use [odej4](https://github.com/tzaeschke/ode4j) directly.  
However if you want cross platform support (i.e include GWT support) then you need to use this library.

Currently this is the only 3D physics engine option for GWT on libGDX.

## How to use in your project

Add the dependency in your core project:

```gradle
project(":core") {
    ...

    dependencies {
        ...
          api "com.github.antzGames:ode4j:0.5.1"
    }
}
```

If you are targeting HTML (GWT) you will also need the following:

```gradle
project(":html") {
    ...
    dependencies {
        ...
        implementation "com.github.antzGames:ode4j:0.5.1:sources"
        implementation "com.github.tommyettinger:formic:0.1.4:sources"
    }
}
```

and lastly add this to your `GdxDefinition.gwt.xml` file:

```xml
<module>
    ...
    <inherits name="gdx_ode4j" />
    <inherits name="formic" />
    ...
</module>
```

## Runtime examples

Some examples can be found in my [https://github.com/antzGames/gdx-ode4j-examples](https://github.com/antzGames/gdx-ode4j-examples/tree/0.5.1_Demos) repository.

Or play with them on [itch.io](https://antzgames.itch.io/physics-in-a-browser).

https://github.com/antzGames/gdx-ode4j/assets/10563814/bc46a9a9-f2e8-414b-bfde-8be6ea54e46b

## Math classes

Ode4j has its own math classes similar to libGDX's Vector3, Matrix3, Matrix4, and Quaternion.

I added a math utility class called [Ode2GDXMathUtils](https://github.com/antzGames/gdx-ode4j/blob/master/src/main/java/com/github/antzGames/gdx/ode4j/Ode2GdxMathUtils.java).  Use the following methods to create the libGDX Quaternion from ode4j's QuanternionC or DMatrix3C:

```java
    Quaternion q1 = Ode2GdxMathUtils.getGdxQuaternion(odeQuaternion);  
    Quaternion q2 = Ode2GdxMathUtils.getGdxQuaternion(odeMat3);
```

In addition ode4j uses double and not float like most of libGDX's math classes.

## Limitations

### Fixed timesteps

ODE was made to work with fixed timesteps.  Do not pass `Gdx.graphics.getDeltaTime()` to `world.quickStep()`.

The following online [ODE HOWTO entry](https://ode.org/wiki/index.php/HOWTO_fixed_vs_variable_timestep) discusses how to incorporate this limitation into a game.

Using `vsync=true` in your game launch configuration helps, but some people might run at 60hz, while others might run at 50Hz, 75Hz, 144Hz.
This means that objects will fall/interact at different speeds on these different refresh rates.

Below is example code that you can use to force physics to only update at a fixed interval (timestep).

```java
public static final float MIN_FRAME_LENGTH = 1f/60f;
        ...

@Override
public void render(float deltaTime){
        timeSinceLastRender += deltaTime;

        // Only compute 60Hz for physics
        if (timeSinceLastRender >= MIN_FRAME_LENGTH) {
            odePhysicsSystem.update(MIN_FRAME_LENGTH);   // My custom class, which eventaully calls world.quickStep(MIN_FRAME_LENGTH)
            timeSinceLastRender -= MIN_FRAME_LENGTH;

            if(timeSinceLastRender > MIN_FRAME_LENGTH)
                timeSinceLastRender = 0;
        }

        ...
}
```

### Performance

I have tried jBullet, PhysX and ODE physics engines with libGDX.  ODE is the slowest, and the reason being is that in ODE everything is using double precision.

## Where to get ODE/ode4j documentation and help

ODE official manual: http://ode.org/wiki/index.php/Manual

By far the most useful part is the [HOWTO](http://ode.org/wiki/index.php/HOWTO) section

ode4j discord channel : https://discord.gg/UFXJcXv2P8 ode4j/Java

ode4j contains also some features that are not present in ODE, such as a ragdoll and heightfields with holes. See ode4j's [Wiki](https://github.com/tzaeschke/ode4j/wiki/Functionality-beyond-ODE).

The [ODE forum](https://groups.google.com/forum/#!forum/ode-users) is useful for questions around physics and general API usage.

The [ode4j forum](https://groups.google.com/forum/?hl=en#!forum/ode4j) is for problems and functionality specific to ode4j/Java.

There is also the [old website](https://tzaeschke.github.io/ode4j-old/), including some [screenshots](https://tzaeschke.github.io/ode4j-old/ode4j-features.html).

Here is a [Youtube video](https://www.youtube.com/watch?v=ENlpu_Jjp3Q) of a list of games that used ODE from 2002-2015.  You will be suprised how many of your favorite games used this physcis libary.

## Versions

Both ODE4J `v0.4.1` and `v0.5.1` have been ported.

Version v0.4.1 GitHub repo: [https://github.com/antzGames/gdx-ode4j](https://github.com/antzGames/gdx-ode4j)

Version v0.5.1 GitHub repo: [https://github.com/antzGames/ode4j](https://github.com/antzGames/ode4j)

## Where has ODE4J libGDX version been used?

Here are some games and projects and tutorials that have used v0.4.1 of ODE4J for libGDX:

[https://github.com/MonstrousSoftware/VehicleTest](https://github.com/MonstrousSoftware/VehicleTest)

[https://monstrous-software.itch.io/base-invaders](https://monstrous-software.itch.io/base-invaders)

[https://monstrous-software.itch.io/fps-demo](https://monstrous-software.itch.io/fps-demo)

[https://antzgames.itch.io/physics-in-a-browser](https://antzgames.itch.io/physics-in-a-browser)

[https://antzgames.itch.io/shotgun-wedding](https://antzgames.itch.io/shotgun-wedding)

[https://antzgames.itch.io/tank-vs-dinos](https://antzgames.itch.io/tank-vs-dinos)

## 3D FPS Tutorial using ODE4J

A full blown tutorial on making a libGDX FPS game using ODE4J can be found here:

[Tutorial on creating a 3D game with LibGDX](https://monstroussoftware.github.io/2023/11/01/Tutorial-3D-step1.html)

## ODE, ode4j and other Licenses

### Licensing & Copyright for ODE and ode4j

This library is under copyright by Tilmann Zäschke (Java port), Russell L. Smith (copyright holder of the original ODE code), Francisco Leon (copyright holder of the original GIMPACT code) and Daniel Fiser (copyright holder of the original libccd).

This library is free software; you can redistribute it and/or modify it under the terms of EITHER:
(1) The GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version. The text of the GNU Lesser General Public License is included with this library in the file LICENSE.TXT.
(2) The BSD-style license that is included with this library in the files ODE-LICENSE-BSD.TXT and ODE4j-LICENSE-BSD.TXT.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files LICENSE.TXT, ODE-LICENSE-BSD.TXT, GIMPACT-LICENSE-BSD.TXT, GIMPACT-LICENSE-LGPL.TXT, ODE4J-LICENSE-BSD.TXT and LIBCCD_BSD-LICENSE for more details.

The LICENSE.TXT, ODE-LICENSE-BSD.TXT, GIMPACT-LICENSE-BSD.TXT, GIMPACT-LICENSE-LGPL.TXT, LIBCCD_BSD-LICENSE and ODE4J-LICENSE-BSD.TXT files are available in the source code.

## Legal

ode4j:
Copyright (c) 2009-2017 Tilmann Zäschke ode4j@gmx.de.
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
