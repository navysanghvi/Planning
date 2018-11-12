Domain-independent symbolic planner for three environments:
(1) Blocks environment: environments/example.txt
(2) Blocks & Triangles environment: environments/BlocksTriangle.txt
(3) Fire Extinguisher environment: environments/FireExtinguisher.txt

Default is no heurstic (Dijkstra).
Uncomment line 318 in planner.cpp for empty-delete-list heuristic. This is admissible, and A* is optimal and complete.
Uncomment line 319 in planner.cpp for goal-conditions-unsatisfied heuristic. This is inadmissible, and A* is not optimal, but is complete.


To run the planner, execute

g++ -o symbolplan Compare.cpp GraphVertex.cpp planner_classes.cpp planner.cpp PlanSym.cpp




followed by 

./symbolplan environments/example.txt

or 

./symbolplan environments/BlocksTriangle.txt

or

./symbolplan environments/FireExtinguisher.txt
