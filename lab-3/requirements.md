题目描述
Try to solve Grid Word Problem based on MDP. 
The grid word is 5*5 size. The cells of the grid correspond to the states of the environment. At each cell, four actions are possible: north, south, east, and west which deterministically cause the agent to move one cell in the respective direction on the grid. Actions that would take the agent off the grid leave its location unchanged, but also result in reward of -1. Other actions result in a reward of 0, except those that move the agent out of the special states A and B.
From state A, all four actions yield a reward of A_reward and take the agent to A_to. From state B, all four actions yield a reward of B_reward and take the agent to B_to. The discount factor gamma=0.9.  
Try to use value iteration and policy iteration. 
输入
There are two lines in the input, as the form of: 
A A_to A_reward 
B B_to B_reward 
where A, B are the two special states.  
输出
样例输入 Copy
[0,1] [4,1] 10.0
[0,3] [2,3] 5.0
样例输出	Copy
Value Iteration
[22.0, 24.4, 22.0, 19.4, 17.5]
[19.8, 22.0, 19.8, 17.8, 16.0]
[17.8, 19.8, 17.8, 16.0, 14.4]
[16.0, 17.8, 16.0, 14.4, 13.0]
[14.4, 16.0, 14.4, 13.0, 11.7]
Policy Iteration
[22.0, 24.4, 22.0, 19.4, 17.5]
[19.8, 22.0, 19.8, 17.8, 16.0]
[17.8, 19.8, 17.8, 16.0, 14.4]
[16.0, 17.8, 16.0, 14.4, 13.0]
[14.4, 16.0, 14.4, 13.0, 11.7]