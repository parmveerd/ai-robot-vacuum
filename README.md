# ai vacuum robot
 
The goal in this project is to practice design and implementation of the common search algorithms for intelligent 
agents in typical environments. This project is provided in the form of a shell module and you are going to add 
or modify it to fullfill the following requirements. Some of the functions int he module have printout comments 
saying "students to implement" or such. Once you have done that part, remove the print function please.

1-The setting is as in the last project, meaning a 2D grid of rooms among which a random number of them are 
'dirty' and there are random number of blocked rooms as well. The goal is to use various search algorithms to help 
the vacuuming robot to go around and clean rooms, as in the last project.
2-You are going to implement 4 searching algorithms: BF Graph, DF Graph, UC, and A*. Currently BF Graph Search is 
partially implemented. The tasks are basically to complete and implement all the algorithms. 
3-The way implementation works is that the path is computed when you select the search from the menu and the path is 
returned. Then using 'next' or 'run' you can step through the path, one step at a time. For 'run' the steps follow 
each other with a delay of 1 seconds automatically.
4-The agent starts from the middle of the grid, and finds the 'closest' dirty room, based on the chosen search, and 
find the path to it. Once that goal is achieved. The next search is done for the next dirty room and so on, till all 
grid is clean.
5-Notice that path_cost() function and heuristic function h() have been modified from what is int he base class 
Problem. Their updated definition is in the main script xy_vacuum_environment.py in VacuumPlanning class. The change 
is described in the comment section for each function. Read that carefully. Remind you, that path_cost() function is 
used for UCS search and A*. Heuristic h() function is used in A* for computing the total cost f(x) = path_cost(s) + h(s).

-Each algorithms is worth 20% except the A* which is 25%. There is also 15% mark set aside for the analysis section 
below.

For each search the following visual components should be present:
1- Explored area being colored in pink. This is currently done for BFS. This should be done for all searches. 
2- Number-of-steps and Performance labels at the top should be updated as stepping through the path is performed. 
Number-of-steps should be measured from the start of the cleaning till the current location. This means this is a 
an accumulative cost as we go through all the dirty rooms. Similarly for the Performance label. 
3- The chosen path should be rendered with a different color than yellow, maybe orange or light blue. Everytime a 
new search is performed (for finding a path to the next dirty room) the previous explored and path coloring should 
be cleared.