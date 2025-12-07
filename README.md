## Title
first-person real‑time ray-traced room scene with interactive controls.

<video src="piece.mp4" width="320" height="240" controls></video>

## Instructions: 
```
cmake -B build-parallel/ -DCMAKE_BUILD_TYPE=Release
cd build-parallel
./raytracing
```

## Description
Made a real‑time ray-traced room scene with interactive controls.

Key features:
  - use CPU to render, openMP paralleled 

  - Room + table + metal cube + mirror scene:
    Constructed in main.cpp via build_scene(). 
    - Room and table meshes come from build_room_mesh() / build_table_mesh()
    - the metal cube is catmull subdivided once
    - the mirror is a quad on the back wall. 

  All are converted to triangle soups with materials in build_scene().

  - Materials: 
    - Walls use a pale near-white color
    - table is wood-like
    - cube is metal (high ks/km), 
    - mirror is highly reflective (high ks/km). 

  Defined and assigned in build_scene() in main.cpp.

  - Dynamic lights: 
    Two point lights in build_scene():
      1. a dim overhead fill (I ≈ [0.2, 0.2, 0.2]) 
      2. a flashlight attached to the camera (flashlight->p updated each frame; I ≈ [0.9, 0.8, 0.7]). 
    The flashlight follows the camera in the main loop’s update_flashlight.

  - Camera controls: 
    Keyboard-only. First-person view of the scene.
    - WASD moves the target
    - Arrow keys rotate the view
    - Space/Ctrl move up/down

    (Because CPU rendering is slow, and the user would be confused by how fast/far should they drag.)
    SDL_KEYDOWN handling and camera_eye usage in main.cpp. 

  - Ray-traced rendering loop:
    Windowed SDL viewer in main.cpp starts ray-trace jobs (render_frame) and updates an SDL texture when a frame finishes.
    Resolution matches the window size.

---

  Primary code locations:

  - Scene build, materials, lights, cube height: main.cpp (build_scene section).

  - Camera basis and rotation: fill_camera and arrow-key handling in main.cpp.

  - Flashlight update per frame: update_flashlight lambda in main.cpp.

  - Mesh builders: src/mesh/mesh_builders.cpp and types in include/mesh_types.h.


