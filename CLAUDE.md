# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Run Commands

**Build**: `cd src && make`
**Run**: `cd src && ./rl_test`
**Clean**: `cd src && make clean`

The project uses a standard Raylib Makefile that supports multiple platforms (Desktop, Web, DRM). The main executable is built from the source files listed in `PROJECT_SOURCE_FILES`.

## Project Architecture

This is a 3D rigid body physics simulation built with Raylib. The system demonstrates real-time physics with collision detection, response, and visualization.

### Core Components

**RigidBody System** (`rigidbody.c/.h`):
- Central physics object with position, rotation, velocity, and angular momentum
- Semi-implicit Euler integration for stability
- Momentum-based angular dynamics for better numerical behavior
- Mass properties computed from mesh geometry using tetrahedral decomposition

**Collision System** (`collision.c/.h`):
- OBB vs OBB collision detection using Separating Axis Theorem (SAT)
- Two-phase collision response: positional correction + impulse resolution
- Coulomb friction model with configurable coefficients
- Debug visualization system for contact points and normals

**Specialized Bodies**:
- `cuboid_rb.c/.h`: Simplified cuboid creation with analytical inertia tensors
- `staticbody.c/.h`: Immovable objects (ground, walls) with zero inverse mass

### Physics Loop Structure

The simulation uses fixed timestep (0.001s) with accumulator pattern:
1. Update rigid body positions/rotations via integration
2. Detect collisions between all body pairs
3. Resolve collisions (separate overlapping objects, apply impulses)
4. Apply external forces (gravity, damping)

### Key Data Flow

Forces → Momentum → Integration → Collision Detection → Response → Updated State

### Graphics Integration

- Uses Raylib for 3D rendering, input, and math utilities
- Custom shaders for lighting (`assets/lighting.vs`, `assets/lighting_new.fs`)
- Debug visualization with colored vectors and contact spheres
- Orbital camera system with spherical coordinates

## Development Notes

- Physics state is momentum-based rather than velocity-based for numerical stability
- Collision response separates positional correction from velocity resolution
- The system supports both mesh-based and analytical (cuboid) rigid bodies
- Debug contacts are cleared each frame and rebuilt during collision detection