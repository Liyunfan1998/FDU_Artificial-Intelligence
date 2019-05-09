# -*- coding: utf-8 -*-
# __author__ = 'siyuan'
import random
WORLD_SIZE = 5
discount = 0.9
# left, up, right, down
actions = ['L', 'U', 'R', 'D']


def construct_MDP(A_POS, A_TO_POS, A_REWARD, B_POS, B_TO_POS, B_REWARD):
    nextState = []
    actionReward = []
    for i in range(0, WORLD_SIZE):
        nextState.append([])
        actionReward.append([])
        for j in range(0, WORLD_SIZE):
            next = dict()
            reward = dict()
            if i == 0:
                next['U'] = [i, j]
                reward['U'] = -1.0
            else:
                next['U'] = [i - 1, j]
                reward['U'] = 0.0

            if i == WORLD_SIZE - 1:
                next['D'] = [i, j]
                reward['D'] = -1.0
            else:
                next['D'] = [i + 1, j]
                reward['D'] = 0.0

            if j == 0:
                next['L'] = [i, j]
                reward['L'] = -1.0
            else:
                next['L'] = [i, j - 1]
                reward['L'] = 0.0

            if j == WORLD_SIZE - 1:
                next['R'] = [i, j]
                reward['R'] = -1.0
            else:
                next['R'] = [i, j + 1]
                reward['R'] = 0.0

            if [i, j] == A_POS:
                next['L'] = next['R'] = next['D'] = next['U'] = A_TO_POS
                reward['L'] = reward['R'] = reward['D'] = reward['U'] = A_REWARD

            if [i, j] == B_POS:
                next['L'] = next['R'] = next['D'] = next['U'] = B_TO_POS
                reward['L'] = reward['R'] = reward['D'] = reward['U'] = B_REWARD

            nextState[i].append(next)
            actionReward[i].append(reward)

    return nextState, actionReward


# value iteration
def value_iteration(nextState, actionReward):# 伪代码里，参数是MDP(或者说States)，Action，Transition Model，Rewards和Discount
    #我们现在有的参数是 nextState， actionReward（包括了所有采取action的后继state）,（rewards行动代价为0）,Transition Model被固定为1，采取行动必然成功
    world = [[0 for _ in range(WORLD_SIZE)] for _ in range(WORLD_SIZE)]
    history_U_s = [] # MY CODE TO KEEP TRACK OF ITERATION
    while True:
        # keep iteration until convergence
        difference = 0
        ## Begin your code
        history_U_s.append(world.copy())
        newWorld = world.copy()
        # U_ = []
        policy # TODO: need to setup policy # LATER: (NO!)
        # initialize U' and U
        # ---- it seems that U and U_ is not calculated by using policy_evaluation
        #for i in len(nextState):
        #    U_.append(policy_evaluation(world,policy,nextState[i],actionReward[i]))

        # TODO: HAVE TO THINK HARDER WHAT THIS MEANS!!!
        # U = U_.copy()

        for i in range(5):
            for j in range(5):
                state_reward # TODO: need to get state reward, I think it's somehow different from action reward
                #idx = U.index(max(U)) # find the s'(also indicates the action taken) which is argmax to U
                #U_.append(state_reward + discount * world[i][j])
                for action in actionReward[i][j]:
                    state_reward.append(actionReward[i][j][action])
                state_reward.index(max(state_reward))
                newWorld[i][j] = 0 + discount * max(state_reward)
                abs_diff = abs(newWorld[i][j] - world[i][j])
                if abs_diff > difference: # ABS
                    difference = abs_diff
        world = newWorld
        ## End your code

		# keep iteration until convergence
        if difference < 1e-4:
            print('Value Iteration')
            for j in range(WORLD_SIZE):
                print([round(each_v, 1)for each_v in world[j]])
            break


def policy_evaluation(world, policy, nextState, actionReward):
    while True:
        difference = 0
        ## Begin your code

		## End your code

        if difference < 1e-4:
            break
    return world


# policy iteration
def policy_iteration(nextState, actionReward):
    # random initialize state value and policy
    world = [[0 for _ in range(WORLD_SIZE)] for _ in range(WORLD_SIZE)]
    history_U_s = [] # MY CODE TO KEEP TRACK OF ITERATION
    policy = [[random.randint(0, len(actions)-1) for _ in range(WORLD_SIZE)] for _ in range(WORLD_SIZE)]

    while True:
        ## Begin your code
        history_U_s.append(world.copy())
        U = []
        # initialize U
        for i in len(nextState):
            U.append(policy_evaluation(world, policy, nextState[i], actionReward[i]))
        unchanged = True
        for i in range(5):
            for j in range(5):
                state_reward # TODO: need to get state reward, I think it's somehow different from action reward
                idx = U.index(max(U)) # find the s'(also indicates the action taken) which is argmax to U
                U_.append(state_reward + discount * U[idx])
                if U_[idx]-U[idx] > difference: # ABS or not
                    difference = U_[idx]-U[idx]

        for nstate in nextState:
            idx = nextState.index(nstate)
            currentStrategy = []
            state_reward # TODO: need to get state reward, I think it's somehow different from action reward
            U_.append(state_reward + discount * U[idx])
            if U_[idx]-U[idx] > difference: # ABS or not
                difference = U_[idx]-U[idx]

		## End your code

    print('Policy Iteration')
    for j in range(WORLD_SIZE):
        print([round(each_v, 1) for each_v in world[j]])


def process_read(x):
    from_state = [int(x[0][1]), int(x[0][-2])]
    to_state = [int(x[1][1]), int(x[1][-2])]
    reward = float(x[-1])
    return from_state, to_state, reward


while True:
    try:
        A_list = input().strip().split()
        B_list = input().strip().split()
        A_POS, A_TO_POS, A_REWARD = process_read(A_list)
        B_POS, B_TO_POS, B_REWARD = process_read(B_list)
        nextState, actionReward = construct_MDP(A_POS, A_TO_POS, A_REWARD, B_POS, B_TO_POS, B_REWARD)
        value_iteration(nextState, actionReward)
        policy_iteration(nextState, actionReward)
    except EOFError:
        break
