![Boulette Physiques](https://dl.dropboxusercontent.com/u/76675545/boulette.png)

Basic physics using fixed-point or integers.
For determinism, stability and consistency across all setups (useful for networked 
simulations using the lockstep model).  

## Evolution and current state -- Please read before reviewing --

I was initially planning to implement this physics engine the way I'm used to : have collision primitives, and write functions to detect collisions between 
them.  
The following (legacy) files show my initial efforts :
- `aabb.hpp`
- `box.hpp`
- `disk.hpp`
  
But soon I needed to implement velocity and acceleration, so I figured I'd do
it the old way :
```C
void Physics::update() {
    //For each object...
    obj.position += obj.velocity;
    obj.velocity += obj.acceleration;
}
```
I saw no problems with it, until I saw [this article from Gaffer on Games](http://gafferongames.com/game-physics/integration-basics/) and basically
learned that what I was doing sucked pretty bad because of the accuracy loss
over time (the example shown by the author is really convincing).  
I then moved all my efforts into implementing a Verlet Integration-based physics system, 
which is, right now, this project's main selling point.  
It appears quite good for cloth simulation, and it was also used in "Hitman : Codename 47" for ragdolls and other stuff.  
  
The file is `include/boulette/verlet.hpp`. One can test it using various types, by changing
the `unit` and `real` `typedefs` in `include/TestVerlet.hpp` (this is documented in that file).  
  
There's no render-time simulation-space to screen-space transformation, but it would be a nice feature
for `TestVerlet` to implement (and perhaps it would allow integer-based simulations, since the simulation
wouldn't be limited by the window's size).  
   
I also took it as an opportunity to put what I've learned about
Data-Oriented Design (DOD) into practice. However, since it looks like OOP's 
nemesis (while still valuing understandable code), the Verlet System's code is a 
bit unusual, so I took extra time to document it.  

DOD values the understanding of the target hardware and assembly, which is why I
concern myself with the layout of the data, the access patterns, and the assembly
output (even though it's pointless because SDL2's renderer is one big overhead).  
The files in `sse2_tests` show my experiments with SIMD instructions, and finally helped me
understand why the assembly output of my program was not what I expected :
- First, methods seem to be inlined, which is why GDB's `disas` won't allow me
  to inspect most of them. This can be "solved" using GCC's `__attribute__((noinline))`.
- The compiler cannot prove that my arrays are aligned on a 16-bytes boundary, so it has
  to generate some extra instructions to handle potentially misaligned data.
- The compiler cannot prove that my `float`/`int` arrays have an element count that is a multiple of 4 
  (the SIMD loop could cause a segmentation fault because of this, so extra checks are performed).

