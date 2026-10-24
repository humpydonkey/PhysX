# Media for NVIDIA-Omniverse/PhysX issue #584 and PR #585

This branch only holds files that the issue and the pull request link to. It has no PhysX code and it is not part of the fix.

- `pr585-before-after.mp4`: 18 s video. Stock `main` (left) and the fix of PR #585 (right), the same scene: a 100 x 15.7 x 117.7 mm triangle mesh block (with SDF) on a plane, GPU dynamics, TGS, 240 Hz, start tilt 1.5 deg. Playback is 0.2x for 0 to 2 s and 0.5x for 2 to 6 s of the simulation.
- `pr585-before-after-still.png`: frame at t = 4.50 s. Stock still rocks. The fix is at rest.
- `pr585-before-after-still-settled.png`: frame at t = 3.00 s.

The captions say that the stock run is still rocking at 10 s and that 0 of 5 start tilts settled within 10 s. A note at the bottom of each frame points to the "Longer runs" section of issue #584 (600 s runs: stock settles in 2 of 5 tilts, the fix in 5 of 5).

The scene is the program `rocking_mesh_plane.cpp` of issue #584. Poses and contact points in the video are real program output. Colours, markers and the slow motion are added.
