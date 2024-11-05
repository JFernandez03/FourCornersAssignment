class Problem:
    def __init__(self, mazeFile):
        self.xStep = 40
        self.yStep = 40
        self.readMaze(mazeFile)
        self.width = self.xMax  # Set width based on xMax
        self.height = self.yMax  # Set height based on yMax

    def readMaze(self, mazeFile):
        self.walls = []
        self.pacman = 0
        self.food = []
        self.xMax = 0
        self.yMax = 0
        
        with open(mazeFile, 'r') as f:
            y = 0
            while True:
                s = list(f.readline())
                if s == []: 
                    break
                if s[-1] == '\n': 
                    s.pop()
                x = 0
                for k in s:
                    if k == '*': 
                        self.walls.append((x, y))
                    if k == 'P': 
                        self.pacman = (x, y)
                    if k == '.': 
                        self.food.append((x, y))
                    x += 1
                y += 1
                self.xMax = max(self.xMax, x)
            self.yMax = y

    def is_valid_position(self, position):
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def startState (self):
        return (self.pacman, self.food)

    def isGoal (self, node):
        return node[1] == []

    def transition (self, node):
        x, y = node[0]
        newState = []

        potential_moves = [((x+1, y), 'R'),
                           ((x-1, y), 'L'),
                           ((x, y+1), 'D'),
                           ((x, y-1), 'U')]

        moves = [(move, a) for move, a in potential_moves if 0 <= move[0] < self.xMax and
                 0 <= move[1] < self.yMax and move not in self.walls]
        newStates = []
        
        for k, action in moves:
            remainingDots = node[1].copy()
            if k in remainingDots:
                remainingDots.remove(k)
            newState.append (((k, remainingDots), action, 1))

        return newState

    def compute_distances (self):
        self.dist = {}
        for f in self.food:
            self.dist[(self.pacman, f)] = self.BFS(self.pacman, f)
            for d in self.food:
                if d == f: continue
                plan1 = self.BFS(f, d)
                self.dist[(f, d)] = plan1.copy()
                # reverse plan1:
                plan2=[]
                for k in plan1:
                    if k == 'U': plan2.append('D')
                    if k == 'D': plan2.append('U')
                    if k == 'L': plan2.append('R')
                    if k == 'R': plan2.append('L')
                plan2.reverse()
                self.dist[(d, f)] = plan2

    def BFS(self, start_pos, target_pos):
        def construct_path(node, visited):
            path = []
            while node:
                node, a = visited[node]
                if node != None: path = [a] + path
            return path
        def transition(node):
            x, y = node

            potential_moves = [((x+1, y), 'R'),
                               ((x-1, y), 'L'),
                               ((x, y+1), 'D'),
                               ((x, y-1), 'U')]

            moves = [(move, a) for move, a in potential_moves if 0 <= move[0] < self.xMax and
                     0 <= move[1] < self.yMax and move not in self.walls]
            return moves
        
        frontier = [(start_pos, None, None)]
        visited = {}
        while frontier:
            node, parent, action = frontier.pop(0)
            if node in visited: continue
            visited[node] = (parent, action)
            if node == target_pos: return construct_path(node, visited)
            neighbors = transition(node)
            for n, a in neighbors:
                frontier.append ((n, node, a))
        return None
                

    def h(self, state):
        pacman_pos, food_list = state
        if not food_list:
            return 0

        # Create a lookup table for distances
        distance_lookup = {}
        all_positions = [pacman_pos] + food_list
        for i in range(len(all_positions)):
            for j in range(i + 1, len(all_positions)):
                pos1 = all_positions[i]
                pos2 = all_positions[j]
                # Compute the distance using BFS
                distance = self.BFS(pos1, pos2)[0]
                distance_lookup[(pos1, pos2)] = distance
                distance_lookup[(pos2, pos1)] = distance

        sum_steps = 0
        mst = {food: False for food in food_list}

        while not all(mst.values()):
            # Find the food with the minimum steps from the current position
            min_steps = float('inf')
            next_food = None

            for food in food_list:
                if not mst[food]:
                    steps = distance_lookup[(pacman_pos, food)]
                    if steps < min_steps:
                        min_steps = steps
                        next_food = food
        
            # Update state
            sum_steps += min_steps
            mst[next_food] = True
            pacman_pos = next_food

        return sum_steps
        

    def load_from_file(self, filename):
        # Example: Load the maze from the file
        # This should set up the maze structure and return the initial state
        pass

    def get_dimensions(self):
        # Assuming that self.maze is a list of lists representing the maze
        # For example:
        # self.maze = [['.', '#', '.'], ['.', '.', '#'], ['#', '.', '.']]
        if hasattr(self, 'maze') and self.maze:
            height = len(self.maze)
            width = len(self.maze[0]) if height > 0 else 0
            return width, height
        else:
            return 0, 0  # Or some other default value if the maze is not loaded

  
    
    def nextStates(self, node):
        pacman_pos, food_list = node  # Assuming node is a tuple: (pacman_pos, food_list)
        next_states = []

        # Possible moves: up, down, left, right
        possible_moves = [(0, 1, 'D'), (1, 0, 'R'), (0, -1, 'U'), (-1, 0, 'L')]  # (dx, dy, action)

        for dx, dy, action in possible_moves:
            new_pos = (pacman_pos[0] + dx, pacman_pos[1] + dy)

            # Check if the new position is valid (within bounds and not a wall)
            if self.is_valid_position(new_pos):
                # Create a new food list based on the move
                new_food_list = food_list.copy()  # Copy the current food list
                if new_pos in new_food_list:  # If Pacman eats food
                    new_food_list.remove(new_pos)

                # Append a tuple of (cost, new_state, action)
                next_states.append((1, (new_pos, new_food_list), action))  # Assuming cost is 1 for each move

        return next_states

        

    