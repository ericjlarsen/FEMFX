# AMD FEMFX

FEMFX is a physics system for deformable objects released on GPUOpen
(https://github.com/GPUOpen-Effects/FEMFX). This fork has some refactoring of the library and a few tweaks which are WIP (see CHANGELOG for current changes).

The FEMFXViewer sample is pretty basic and only renders tetrahedral meshes to visualize simulation. There's also an Unreal Engine plugin with a small demo:
https://github.com/ericjlarsen/FEMFX-UEPlugin/tree/FEMFX-AlienPods-5.7

(You need Unreal source code access to see the plugin project: https://dev.epicgames.com/documentation/en-us/unreal-engine/downloading-source-code-in-unreal-engine)

## Dependencies
The FEMFXViewer sample relies on:

- JSON for Modern C++ (https://github.com/nlohmann/json) which is included as a submodule
- GLFW (https://www.glfw.org/download.html) - 64-bit Windows binaries package must be downloaded and copied to external\glfw

Optionally for profiling compatible with chrome://tracing
- Minitrace (https://github.com/hrydgard/minitrace) which is included as a submodule

## Building

VS2022 project files are included. Can open samples\FEMFXViewer\FEMFXViewer_vs2022.sln to build the sample and libraries.

Other project files can be created using premake5 (https://premake.github.io/). To create all project files for the sample and libraries, from the samples\FEMFXViewer\ directory run: premake5 [action]

## Files:
1. Documentation
   * amd_femfx\docs\
2. Library
   * amd_femfx\inc\ : Public API (Look first at AMD_FEMFX.h)
   * amd_femfx\src\ : Implementation files
   * amd_femfx\TL\inc\ : Threading library headers
   * amd_femfx\TL\src\ : Threading library source files
3. Sample code
   * samples\FEMFXViewer\ : Basic sample; displays tetrahedral meshes and debug 
     information
   * samples\common\TestScenes.*: Setup for tech demo scenes used by FEMFXViewer
   * samples\common\FemResource.* : Container for the data loaded from a .FEM 
     file
   * samples\common\LoadFem.*: Code for parsing a .FEM and loading into an 
     FEMResource
   * samples\common\RenderTetAssignment.* : Assigns rendering mesh vertices to 
     tetrahedra for skinning
   * samples\ExampleRigidBodies\ : Basic demonstration of interfacing an 
     external rigid body system with FEMFX library
4. Houdini plugin for content creation
   * houdini16.5\AMD_FEM_Assets.otl
5. External dependencies
   * external\glfw\ : Path to install GLFW for viewer; can download Windows
     pre-compiled binaries from https://www.glfw.org/download.html
   * external\json\ : Path of json parser submodule used in LoadFemFile.*
   * external\minitrace\ : Path of trace profiling submodule
