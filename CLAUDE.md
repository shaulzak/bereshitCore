# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

bereshitCore is a C++20 3D rigid-body physics engine (Unity-style GameObject/Component model) that is exposed to Python via pybind11. All sources live flat in the repo root; there is no `src/` directory.

## Build

There are two separate CMake configurations:

- **`CMakeLists.txt`**: native executable (`bereshitCore`) used for debugging/testing in CLion. Entry point is `main.cpp`, which times one scenario function.
- **`cbuild.txt`**: pybind11 module build (`bindings.cpp` + engine sources, without `main.cpp` and `test/`). To use it, copy it over `CMakeLists.txt` (or swap them), then run the commands in `build.txt`:
  ```
  cmake -S . -B build -Dpybind11_DIR="<python>/Lib/site-packages/pybind11/share/cmake/pybind11"
  cmake --build build --config Release
  ```
  `__init__.py` loads the compiled module from `build/Release/` and re-exports the Python-facing classes.

Native build:
```
cmake -S . -B cmake-build-debug
cmake --build cmake-build-debug
./cmake-build-debug/bereshitCore
```

When adding a new source file, add it to **both** `CMakeLists.txt` and `cbuild.txt` (unless it is test-only). If it should be scriptable from Python, also bind it in `bindings.cpp` and export it in `__init__.py`.

Note: the CMake files reference `quaternion.cpp/.h` in lowercase while the files are `Quaternion.*` — this only works on case-insensitive filesystems (Windows).

## Tests

There is no test framework. Files under `test/` are scenario programs, each in its own namespace exposing `void Main()` (e.g. `TestFixedJoint::Main`, `DiffrentMasses::Main`, `Flages::Main`). They build a scene, step `World::Update()` in a loop, and print state to stdout. To run a different scenario, change the forward declaration and call in `main.cpp`, rebuild, and inspect the output. New scenarios must also be added to `add_executable` in `CMakeLists.txt`.

## Architecture

- **GameObject / Component**: `GameObject` owns a `Transform`, a `Cache` (dirty flags + cached rotation matrices), child GameObjects, and a list of `Component*`. Components are held as raw, non-owning pointers (tests stack-allocate them). `GetComponent<T>()` finds components via `dynamic_cast`. A component's `attach()` is called on `AddComponent` and is where it caches pointers to its parent's transform/rigidbody.
- **Component lifecycle hooks**: `Start`, `Update`, `PhysicsUpdateFirstIteration`, `PhysicsUpdate`, the `OnCollision*/OnTrigger*` callbacks, `Copy`, `RemapReferences` (fixes GameObject pointers after `GameObject::DeepCopy`), and `ResetToDefault`.
- **Physics objects**: only GameObjects that have **both** a `Rigidbody` and a `Collider` participate in physics (`isPhysicsObject`). `World` caches flattened lists of all children, physics children, colliders, rigidbodies, and joints; `AddChild` appends to these caches incrementally rather than rebuilding them.
- **Simulation step** (`World::Update`):
  1. optionally call `Component::Update` on all objects
  2. apply gravity to all rigidbodies
  3. broad phase `Collider::SweepAndPrune` on AABBs, then narrow phase `CheckCollision` (SAT in `Collider`/`BoxCollider`), producing a `std::vector<Contact>` (skipped if both bodies are kinematic)
  4. iterate `physics_epochs + 1` times: sequential impulses (`Rigidbody::SolveImpulse` with friction/restitution), then `Joint::Solve`, then `Component::PhysicsUpdate`
  5. integrate every rigidbody
- **Joints**: `Joint` is the base class (anchors, effective-mass matrix `K`, 3x3/2x2 solvers, Baumgarte `beta`). Subclasses (`FixedJoint`, `HingeJoint`) override `SolveLinear` / `SolveAngular`. A joint is attached to body A and references body B through its constructor argument.
- **Physics** holds a static `World*` for global queries such as `Physics::RayCast`.
- **Python bindings** (`bindings.cpp`): `PyComponent` is a pybind11 trampoline that lets Python subclasses of `Component` override `Start`, `Update`, `PhysicsUpdate`, `Copy`, `ResetToDefault`, and `OnCollisionEnter`. If you expose another virtual hook to Python, add an override to the trampoline.
