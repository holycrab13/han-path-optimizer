# Han Path Optimizer

Implementation of the algorithm suggested in https://aaai.org/ocs/index.php/SOCS/SOCS20/paper/view/18524 by Han et al. as a mod for the Unity A* Pathfinding Project by Aron Granberg. The script is implemented as a MonoMod. Can be combined with other MonoMods.
Takes a 4-directional path as input and tries to find as many non-blocked any-angle diagonal paths as possible.

I did some optimizations by pre-calculating the cells visited by diagonal lines into several n x m matrices.
The A* pathfinder returning an approximate path when the actual path from A to B is blocked will lead to faulty results and a potential infinite loop, which is prevented by a maximum amount of iterations.

All feedback greatly appreciated!
Excuse the ugly developer art

![alt text](/mod-example.png)


Seeker Setup

![alt text](/setup-example.png)
