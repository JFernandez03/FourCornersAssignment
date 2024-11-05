from fourCornerProblem import Problem
from tkinter import *
import time
from itertools import combinations
from problemGraphics import pacmanGraphic
import heapq
import time
from queue import PriorityQueue

def construct_path(node, visited):
    path = []
    while node:
        food = tuple(node[1])
        node, action = visited[(node[0], food)]
        if action != None:
            if type(action) == list: 
                path = action + path
            else: 
                path = [action] + path
    return path

def bfs(p):
    start = p.startState()
    frontier = [(start, None, None)]
    visited = {}
    count = 0
    while frontier:
        node, parent, action = frontier.pop(0)
        food = tuple(node[1])
        if (node[0], food) in visited: 
            continue
        visited[(node[0], food)] = (parent, action)
        if p.isGoal(node):
            print('BFS number of nodes explored: ', count)
            return construct_path(node, visited)
        count += 1
        neighbors = p.transition(node)
        for n, a, c in neighbors:
            food = tuple(n[1])
            if (n, node, a) not in frontier and (n[0], food) not in visited:
                frontier.append((n, node, a))
    return []

def ucs(p):
    pq = [(0, p.startState(), None, None)]
    visited = {}
    count = 0
    while pq:
        gCost, node, parent, action = heapq.heappop(pq)

        food = tuple(node[1])
        if (node[0], food) in visited: 
            continue
        visited[(node[0], food)] = (parent, action)

        if p.isGoal(node):
            print('UCS number of nodes explored: ', count)
            return gCost, construct_path(node, visited)

        count += 1
        neighbors = p.nextStates(node)
        for stepCost, n, a in neighbors:
            food = tuple(n[1])
            newState = (stepCost + gCost, n, node, a)
            heapq.heappush(pq, newState)
    return None

def AStar(self):
    start_state = self.startState()
    frontier = PriorityQueue()
    frontier.put((0, start_state))
    came_from = {}
    cost_so_far = {start_state: 0}

    while not frontier.empty():
        current = frontier.get()[1]

        if self.isGoal(current):
            break

        for next_state in self.transition(current):
            new_cost = cost_so_far[current] + 1  # Assume cost is uniform (1 step)
            if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                cost_so_far[next_state] = new_cost
                priority = new_cost + self.h(next_state)
                frontier.put((priority, next_state))
                came_from[next_state] = current

    # Reconstruct path
    current = next_state
    plan = []
    while current != start_state:
        plan.append(current)
        current = came_from[current]
    plan.reverse()

    return cost_so_far[next_state], plan


filename = 'tinyCorners.txt'  # The maze file to use

# Get an instance of the problem
p = Problem(filename)

# -------------------------------------------------------
# BFS:
# -------------------------------------------------------
startTime = time.time()
plan = bfs(p)  # Assuming the BFS method is part of the FourCornerProblem class
endTime = time.time()
print("BFS Plan:", plan)
print('BFS Plan length:', len(plan))
print('BFS Time: ', (endTime - startTime) * 10**3, "ms")
print('------------------------')

pac = pacmanGraphic(1300, 700)
pac.setup(p)
pac.runPlan(p, plan)

# -------------------------------------------------------
# UCS:
# -------------------------------------------------------
p = Problem(filename)  # Create a new instance for UCS
startTime = time.time()
p.compute_distances()  # Precompute distances if necessary
cost, plan = ucs(p)  # Assuming the UCS method is part of the FourCornerProblem class
endTime = time.time()
print('UCS Cost: ', cost)
print("UCS Plan:", plan)
print('UCS Plan length=', len(plan))
print('UCS Time: ', (endTime - startTime) * 10**3, "ms")

pac = pacmanGraphic(1300, 700)
pac.setup(p)
pac.runPlan(p, plan)

# -------------------------------------------------------
# A*
# -------------------------------------------------------
p = Problem(filename)  # Create another instance for A*
startTime = time.time()
p.compute_distances()  # Precompute distances if necessary
cost, plan = AStar(p)  # Assuming the AStar method is part of the FourCornerProblem class
endTime = time.time()

print('A* Time: ', (endTime - startTime) * 10**3, "ms")
print('A* Cost=', cost)
print("A* Plan:", plan)
print('A* Plan length=', len(plan))

# Leave this code for plan execution and moves Pacman to collect the dots
pac = pacmanGraphic(1300, 700)
pac.setup(p)
pac.runPlan(p, plan)

