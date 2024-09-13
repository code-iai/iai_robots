
This repo is a mix of

https://git.h2t.iar.kit.edu/sw/armarx-integration/robots/armar6/models

and

https://github.com/translearn/armar-urdf

The first repo has wrong joint limits and superfluous links, probably as a result of a URDF auto-generation script.
The second repo has a much simpler URDF with way fewer links, but the neck is wielded onto the torso and the grippers are one solid mesh.

In this repo, I took the second simpler URDF, extended it with pan and tilt joints using the information from the first URDF, and copied the hands from the first URDF but removed all the superfluous links and came up with plausible joint limits.

The first repo has a GPS license, the second is Creative Commons. Using the stricter of the two for this repo.