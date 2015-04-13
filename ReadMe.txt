Distributed Round-Robin Q-Learning (D-RR-QL) is a Reinforcement Learning algorithm that allows to approximate the optimal joint-policy of a multi-agent system in a two-step fashion. First, each agent learns in its own local state-action following a round-robin schedule, thus avoiding non-stationarity due to the rest of agents learning their own policies. Then a coordination procedure approximates the optimal joint-policy by a greedy selection procedure using message passing.

The main advantage of D-RR-QL is that it allows each agent to use Modular State-Action Vetoes, which is a technique that allows RL agents to boost their exploration efficiency when approaching over-constrained systems, such as Linked Multicomponent Robotic Systems. The following source-code was used in the experiments of the following paper:

"Learning Multirobot Hose Transportation and Deployment by Round-Robin Distributed Q-Learning"
    Borja Fernandez-Gauna, Ismael Etxeberria-Agiriano and Manuel Graña 
    Plos-One 

These 4 C/C++ projects were used to simulate and compare 4 different Multi-agent Reinforcement Learning algorithms on a multi-robot hose transportation task:

-Distributed Round-Robin Q-Learning ("Hose-D-RR-QL-ETraces")
-Coordinated RL ("Hose-CoordinatedRL")
-Distributed Q-Learning ("Hose-DistributedQL")
-Team Q-Learning ("Hose-TeamQ-ETraces")

======================================================================================================

INPUT:

The parameters of the system are set within each project in the file "\experiments\parameters.txt"


OUTPUT:

The logs of the experiments are generated in the "\experiments" folder: the results of the training episodes are saved in one file, and those of the evaluation episodes are saved in another one.