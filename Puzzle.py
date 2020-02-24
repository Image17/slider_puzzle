import copy
import math
import time
import heapq
import numpy as np

class Puzzle():
    """ a sliding-block puzzle"""

    def __init__(self, grid, path=[]):
        """instances a differ by their number of configurations"""
        self.grid = copy.deepcopy(grid)  # no aliasing?
        # extract into constructor, do not allow uneven grids
        loc = (0, 0)
        dim = len(self.grid) - 1
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                number = self.grid[i][j]
                if number is ' ':
                    loc = (i, j)

        self.loc = loc
        self.dim = dim
        self.path = path
        hval = 0

    def __lt__(self, other):
        return self.hval < other.hval

    def __eq__(self, other):
        return self.grid == other.grid

    def __hash__(self) -> int:
        return hash(frozenset([item for sublist in self.grid for item in sublist]))

    def display(self):
        """print the puzzle"""
        for row in self.grid:
            for number in row:
                print(number, end=' ')
            print()
        print()

    def moves(self):
        """return a list of possible moves given the current configuration"""
        # now that we have the location.. lets see what directions we can go
        moves = []
        # can we move it north?
        if self.loc[0] != 0:
            moves.append('N')

        # can we go south?
        if self.loc[0] != self.dim:
            moves.append('S')

        # can we go east?
        if self.loc[1] != self.dim:
            moves.append(('E'))

        # can we go west?
        if self.loc[1] != 0:
            moves.append('W')

        # print('ok here are our moves given location ', self.loc)
        # print(moves)

        return moves

    def neighbor(self, move):
        """ return a puzzle instance like this one but with one move made"""
        new_grid = []
        new_path = []
        if move is 'N':
            destination = (self.loc[0] - 1, self.loc[1])
            swap = self.grid[destination[0]][destination[1]]
            new_grid = copy.deepcopy(self.grid)
            new_grid[self.loc[0]][self.loc[1]] = swap
            new_grid[destination[0]][destination[1]] = ' '
            new_path = copy.deepcopy(self.path)
            new_path.append(move)
        elif move is 'S':
            destination = (self.loc[0] + 1, self.loc[1])
            swap = self.grid[destination[0]][destination[1]]
            new_grid = copy.deepcopy(self.grid)
            new_grid[self.loc[0]][self.loc[1]] = swap
            new_grid[destination[0]][destination[1]] = ' '
            new_path = copy.deepcopy(self.path)
            new_path.append(move)
        elif move is 'W':
            destination = (self.loc[0], self.loc[1] - 1)
            swap = self.grid[destination[0]][destination[1]]
            new_grid = copy.deepcopy(self.grid)
            new_grid[self.loc[0]][self.loc[1]] = swap
            new_grid[destination[0]][destination[1]] = ' '
            new_path = copy.deepcopy(self.path)
            new_path.append(move)
        elif move is 'E':
            destination = (self.loc[0], self.loc[1] + 1)
            swap = self.grid[destination[0]][destination[1]]
            new_grid = copy.deepcopy(self.grid)
            new_grid[self.loc[0]][self.loc[1]] = swap
            new_grid[destination[0]][destination[1]] = ' '
            new_path = copy.deepcopy(self.path)
            new_path.append(move)
        else:
            print('What on earth is ', move, 'supposed to mean?')
        newpyhundo = Puzzle(new_grid, new_path)
        #newpyhundo.display()
        return newpyhundo

    def h(self, goal):
        """
        compute distance heuristic from this instance to the goal
        """
        # fill this in, flat listing not a good idea i guess..
        flat_grid = [item for sublist in self.grid for item in sublist]
        flat_goal = [item for sublist in goal.grid for item in sublist]
        h = 0
        #print(flat_grid)
        #print(flat_goal)
        for e in flat_grid:
            fi = flat_grid.index(e)
            fj = flat_goal.index(e)
            dist = (abs(fi - fj))
            #print('for',e,'there is a distance of',dist,'between where we are and where we should be..')
            h += dist
        #print(h)
        return h

    def h2(self, goal):
        distance = 0
        xdiff = 0
        ydiff = 0

        flat_grid = [item for sublist in self.grid for item in sublist]
        flat_goal = [item for sublist in goal.grid for item in sublist]

        for i in flat_grid:
            diff = abs(flat_goal.index(i) - flat_grid.index(i))
            if i is not 0:
                xdiff = diff % (self.dim + 1)
                ydiff = diff / (self.dim + 1)
                distance += xdiff + int(math.floor(ydiff))
                if abs(flat_goal.index(i) % (self.dim + 1) - flat_grid.index(i) % (self.dim + 1)) == 2 and diff % (self.dim + 1) == 1:
                    distance += 2

        return distance + len(self.path)

class Agent():
    """knows how to solve a sliding-block puzzle to match the goal"""

    def astar(self, puzzle, goal):
        """return a  list of moves to get the puzzle to match the goal"""
        visited = set()
        frontier = []
        heapq.heappush(frontier, puzzle)
        while frontier:
            p = heapq.heappop(frontier)
            if p.__eq__(goal):
                return p.path
            moves = p.moves()
            for m in moves:
                neighbor = p.neighbor(m)
                if neighbor not in visited and neighbor not in frontier:
                    neighbor.hval = neighbor.h2(goal)
                    heapq.heappush(frontier, neighbor)
                    visited.add(neighbor)

def main():
    """create  apuzzle, solve it with A*, and console animate"""
    puzzle = Puzzle([[1, 2, 5], [4, 8, 7], [3, 6, ' ']])
    puzzle.display()

    agent = Agent()
    goal = Puzzle([[' ', 1, 2], [3, 4, 5], [6, 7, 8]])
    path = agent.astar(puzzle, goal)
    print(path)
    while path:
        move = path.pop(0)
        puzzle = puzzle.neighbor(move)
        time.sleep(1)
        puzzle.display()


if __name__ == '__main__':
    main()
