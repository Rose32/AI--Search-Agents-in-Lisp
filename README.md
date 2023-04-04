# AI--Search-Agents-in-Lisp
Implementations of Reflex Agent, Model-based Agent, Hill Climbing Agent, Uniform Cost Search Agent and A* Search Agent in CLISP. This was done as an assignment to my AI course. The basic problem is for the agents to reach the goal in a simulated robot world with several obstacles.

Folder and files diescription
- Source Folder: Has all the code files. search-assignment.lisp has  the algorithms for all the search agents. run-assignment.lisp has the code to run all the agents in various worlds with varying environments. The methodologies and description of the implementation of the algorithm in the robot world as well as in another domain (other-domain.lisp) is in the pdf file 'AI- Search Robots' uploaded in the repo.
- Runs folder: Has a simple visualization of the robots running in the simulated environments as configured in run-assignment.lisp. The text files show a rectangular world with ">" as the robot and the direction of hte arrow as the direction the robot is facing. The obstacles are denoted by @ symbol. '...' are the emply slots and '+++' are the walls.
