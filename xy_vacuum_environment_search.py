import os.path
from tkinter import *
from tkinter import messagebox
from agents import *
from search import *
import sys
import copy
from utils import PriorityQueue

"""
1- BFS: Breadth first search. Using tree or graph version, whichever makes more sense for the problem
2- DFS: Depth-First search. Again using tree or graph version.
3- UCS: Uniform-Cost-Search. Using the following cost function to optimise the path, from initial to current state.
4- A*:  Using A star search.
"""
searchTypes = ['None', 'BFS', 'DFS', 'UCS', 'A*']


class VacuumPlanning(Problem):
    """ The problem of find the next room to clean in a grid of m x n rooms.
    A state is represented by state of the grid. Each room is specified by index set
    (i, j), i in range(m) and j in range (n). Final goal is to find all dirty rooms. But
     we go by sub-goal, meaning finding next dirty room to clean, at a time."""

    def __init__(self, env, searchtype):
        """ Define goal state and initialize a problem
            initial is a pair (i, j) of where the agent is
            goal is next pair(k, l) where map[k][l] is dirty
        """
        self.solution = None
        self.env = env
        self.state = env.agent.location
        super().__init__(self.state)
        self.map = env.things
        self.searchType = searchtype
        self.agent = env.agent



    def generateSolution(self):
        """ generate search engien based on type of the search chosen by user"""
        self.env.read_env()
        self.state = env.agent.location
        xi, yi = self.state
        global finished
        super().__init__(self.state)
        if self.searchType == 'BFS':
            results = breadth_first_graph_search(self)
            # If the dirt is stuck
            if results == None:
                self.done = True
                print("Dirt is stuck and cannot be accessed.")
                messagebox.showinfo("Error", "Dirt is stuck and cannot be accessed.")
                finished = True
                return
            path, explored = results
            sol = path.solution()
            self.env.set_solution(sol)
            self.env.display_explored(explored)
            # Function to colour in the path the agent will take
            self.env.path_env(sol, xi, yi)
        elif self.searchType == 'DFS':
            results = depth_first_graph_search(self)
            if results == None:
                self.done = True
                print("Dirt is stuck and cannot be accessed.")
                messagebox.showinfo("Error", "Dirt is stuck and cannot be accessed.")
                finished = True
                return
            path, explored = results
            sol = path.solution()
            self.env.set_solution(sol)
            self.env.display_explored(explored)
            self.env.path_env(sol, xi, yi)
        elif self.searchType == 'UCS':
            results = best_first_graph_search(self, lambda node: node.path_cost)
            if results == None:
                self.done = True
                print("Dirt is stuck and cannot be accessed.")
                messagebox.showinfo("Error", "Dirt is stuck and cannot be accessed.")
                finished = True
                return
            path, explored = results
            sol = path.solution()
            self.env.set_solution(sol)
            self.env.display_explored(explored)
            self.env.path_env(sol, xi, yi)
        elif self.searchType == 'A*':
            results = astar_search(self)
            if results == None:
                self.done = True
                print("Dirt is stuck and cannot be accessed.")
                messagebox.showinfo("Error", "Dirt is stuck and cannot be accessed.")
                finished = True
                return
            path, explored = results
            sol = path.solution()
            self.env.set_solution(sol)
            self.env.display_explored(list(explored))
            self.env.path_env(sol, xi, yi)
        else:
            raise 'NameError'
            # return


    def generateNextSolution(self):
        self.generateSolution()


    def actions(self, state):
        """ Return the actions that can be executed in the given state.
        The result would be a list, since there are only four possible actions
        in any given state of the environment """
        xi, yi = state

        possible_neighbors = self.env.things_near(state)
        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']

        # Remove any direction where a wall is present
        if env.some_things_at((xi + 1, yi), Wall):
            possible_actions.remove("RIGHT")
        if env.some_things_at((xi - 1, yi), Wall):
            possible_actions.remove("LEFT")
        if env.some_things_at((xi, yi + 1), Wall):
            possible_actions.remove("UP")
        if env.some_things_at((xi, yi - 1), Wall):
            possible_actions.remove("DOWN")

        return possible_actions

    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """
        # print(action)

        xi, yi = state

        # Update the state based on the action and return it
        if action == "RIGHT":
            temp = xi+1, yi
        if action == "LEFT":
            temp = xi-1, yi
        if action == "UP":
            temp = xi, yi+1
        if action == "DOWN":
            temp = xi, yi-1

        new_state = list(temp)
        return tuple(new_state)

    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """
        return self.env.some_things_at(state, Dirt)


    def path_cost(self, c, state1, action, state2):
        """To be used for UCS and A* search. Returns the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. For our problem
        state is (x, y) coordinate pair. To make our problem more interesting we are going to associate
        a height to each state as z = sqrt(x*x + y*y). This effectively means our grid is a bowl shape and
        the center of the grid is the center of the bowl. So now the distance between 2 states become the
        square of Euclidean distance as distance = (x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2"""

        # If the search type is not UCS or A*
        if self.searchType == 'BFS' or self.searchType == 'DFS' or self.searchType == 'None':
            return c+1

        # Calculate and return the path cost from state 1 to 2
        x1, y1 = state1
        x2, y2 = state2
        z1 = (x1*x1 + y1*y1) ** 0.5
        z2 = (x2*x2 + y2*y2) ** 0.5

        distance = (x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2

        distance += c
        return distance


    def h(self, node):
        """ to be used for A* search. Return the heuristic value for a given state. For this problem use minimum Manhattan
        distance to all the dirty rooms + absolute value of height distance as described above in path_cost() function. .
    """
        xi, yi = node.state

        # Initialize the minimum Manhattan distance to a large value
        min_manhattan_distance = float('inf')

        # Iterate over the entire environment
        for i in range(env.width):
            for j in range(env.height):
                if env.some_things_at((i,j), Dirt):
                    # Calculate the Manhattan distance from the current state to the dirty location
                    manhattan_distance = abs(xi - i) + abs(yi - j)
                    # Update the minimum distance if necessary
                    min_manhattan_distance = min(min_manhattan_distance, manhattan_distance)

        # Calculate the height distance
        height_distance = (xi*xi + yi*yi) ** 0.5

        # Calculate the heuristic value and return it
        heuristic_value = min_manhattan_distance + height_distance
        return heuristic_value

# ______________________________________________________________________________


def agent_label(agt):
    """creates a label based on direction"""
    dir = agt.direction
    lbl = '^'
    if dir.direction == Direction.D:
        lbl = 'v'
    elif dir.direction == Direction.L:
        lbl = '<'
    elif dir.direction == Direction.R:
        lbl = '>'

    return lbl


def is_agent_label(lbl):
    """determines if the label is one of the labels tht agents have: ^ v < or >"""
    return lbl == '^' or lbl == 'v' or lbl == '<' or lbl == '>'


class Gui(VacuumEnvironment):
    """This is a two-dimensional GUI environment. Each location may be
    dirty, clean or can have a wall. The user can change these at each step.
    """
    xi, yi = (0, 0)

    perceptible_distance = 1

    def __init__(self, root, width, height):
        self.searchAgent = None
        print("creating xv with width ={} and height={}".format(width, height))
        super().__init__(width, height)

        self.agent = None
        self.root = root
        self.create_frames(height)
        self.create_buttons(width)
        self.create_walls()
        self.setupTestEnvironment()

    def setupTestEnvironment(self):
        """ first reset the agent"""

        # Just moved this out of the first conditional, so can use it for the loop as well
        x = self.width // 2
        y = self.height // 2

        if self.agent is not None:
            xi, yi = self.agent.location
            self.buttons[yi][xi].config(bg='white', text='', state='normal')
            self.agent.location = (x, y)
            self.buttons[y][x].config(bg='white', text=agent_label(self.agent), state='normal')
            self.searchType = searchTypes[0]
            self.agent.performance = 0

        """next create a random number of block walls inside the grid as well"""
        roomCount = (self.width - 1) * (self.height - 1)
        blockCount = random.choice(range(roomCount//10, roomCount//5))
        for _ in range(blockCount):
            rownum = random.choice(range(1, self.height - 1))
            colnum = random.choice(range(1, self.width - 1))
            if (rownum, colnum) != (x, y): # Added this to make sure it doesn't place a wall on vacuum after reset
                self.buttons[rownum][colnum].config(bg='red', text='W', disabledforeground='black')

        self.create_dirts()
        self.stepCount = 0
        self.searchType = None
        self.solution = []
        self.explored = set()
        self.read_env()

    def setupTestEnvironment2(self, search):
        """ first reset the agent"""

        # Just moved this out of the first conditional, so can use it for the loop as well
        x = self.width // 2
        y = self.height // 2

        if self.agent is not None:
            xi, yi = self.agent.location
            self.buttons[yi][xi].config(bg='white', text='', state='normal')
            self.agent.location = (x, y)
            self.buttons[y][x].config(bg='white', text=agent_label(self.agent), state='normal')
            self.searchType = searchTypes[0]
            self.agent.performance = 0

        """next create a random number of block walls inside the grid as well"""
        roomCount = (self.width - 1) * (self.height - 1)
        blockCount = random.choice(range(roomCount//10, roomCount//5))
        for _ in range(blockCount):
            rownum = random.choice(range(1, self.height - 1))
            colnum = random.choice(range(1, self.width - 1))
            if (rownum, colnum) != (x, y): # Added this to make sure it doesn't place a wall on vacuum after reset
                self.buttons[rownum][colnum].config(bg='red', text='W', disabledforeground='black')

        self.create_dirts()
        self.stepCount = 0
        if search == None:
            self.searchType = None
        global finished
        finished = False
        self.solution = []
        self.explored = set()
        self.read_env()

    def create_frames(self, h):
        """Adds h row frames to the GUI environment."""
        self.frames = []
        for _ in range(h):
            frame = Frame(self.root, bg='blue')
            frame.pack(side='bottom')
            self.frames.append(frame)

    def create_buttons(self, w):
        """Adds w buttons to the respective row frames in the GUI."""
        self.buttons = []
        for frame in self.frames:
            button_row = []
            for _ in range(w):
                button = Button(frame, bg='white', state='normal', height=1, width=1, padx=1, pady=1)
                button.config(command=lambda btn=button: self.toggle_element(btn))
                button.pack(side='left')
                button_row.append(button)
            self.buttons.append(button_row)

    def create_walls(self):
        """Creates the outer boundary walls which do not move. Also create a random number of
        internal blocks of walls."""
        for row, button_row in enumerate(self.buttons):
            if row == 0 or row == len(self.buttons) - 1:
                for button in button_row:
                    button.config(bg='red', text='W', state='disabled', disabledforeground='black')
            else:
                button_row[0].config(bg='red', text='W', state='disabled', disabledforeground='black')
                button_row[len(button_row) - 1].config(bg='red', text='W', state='disabled', disabledforeground='black')

    def create_dirts(self):
        """ set a small random number of rooms to be dirty at random location on the grid
        This function should be called after create_walls()"""
        self.read_env()   # this is needed to make sure wall objects are created
        roomCount = (self.width-1) * (self.height -1)
        self.dirtCount = random.choice(range(5, 15))
        dirtCreated = 0
        while dirtCreated != self.dirtCount:
            rownum = random.choice(range(1, self.height-1))
            colnum = random.choice(range(1, self.width-1))
            if self.some_things_at((colnum, rownum)):
                continue
            self.buttons[rownum][colnum].config(bg='grey')
            dirtCreated += 1

    def setSearchEngine(self, choice):
        """sets the chosen search engine for solving this problem"""
        self.searchType = choice
        self.searchAgent = VacuumPlanning(self, self.searchType)
        self.searchAgent.generateSolution()
        self.done = False

    def set_solution(self, sol):
        self.solution = list(reversed(sol))

    def display_explored(self, explored):
        """display explored slots in a light pink color"""
        if len(self.explored) > 0:     # means we have explored list from previous search. So need to clear their visual fist
            for (x, y) in self.explored:
                self.buttons[y][x].config(bg='white')

        self.explored = explored
        for (x, y) in explored:
            self.buttons[y][x].config(bg='pink')

    def add_agent(self, agt, loc):
        """add an agent to the GUI"""
        self.add_thing(agt, loc)
        # Place the agent at the provided location.
        lbl = agent_label(agt)
        self.buttons[loc[1]][loc[0]].config(bg='white', text=lbl, state='normal')
        self.agent = agt

    def toggle_element(self, button):
        """toggle the element type on the GUI."""
        bgcolor = button['bg']
        txt = button['text']
        if is_agent_label(txt):
            if bgcolor == 'grey':
                button.config(bg='white', state='normal')
            else:
                button.config(bg='grey')
        else:
            if bgcolor == 'red':
                button.config(bg='grey', text='')
            elif bgcolor == 'grey':
                button.config(bg='white', text='', state='normal')
            elif bgcolor == 'white':
                button.config(bg='red', text='W')

    def execute_action(self, agent, action):
        """Determines the action the agent performs.""" 
        # If search type is none, then just return
        if self.searchType == None:
            return

        xi, yi = agent.location

        # If action is suck, then clean up the dirt
        if action == 'Suck':
            dirt_list = self.list_things_at(agent.location, Dirt)
            if dirt_list:
                dirt = dirt_list[0]
                agent.performance += 100
                
                # Sometimes a dirt is added twice, so this is to make sure no double suck occurs
                if len(dirt_list) == 2:
                    self.delete_thing(dirt_list[1])
                
                self.delete_thing(dirt)
                self.buttons[yi][xi].config(bg='white')   

        # Else move in the desired direction
        else:
            if action == "RIGHT":
                self.buttons[yi][xi].config(text='')
                self.buttons[yi][xi+1].config(text='^')
                agent.location = xi+1, yi
            elif action == "LEFT":
                self.buttons[yi][xi].config(text='')
                self.buttons[yi][xi-1].config(text='^')
                agent.location = xi-1, yi
            elif action == "UP":
                self.buttons[yi][xi].config(text='')
                self.buttons[yi+1][xi].config(text='^')
                agent.location = xi, yi+1
            elif action == "DOWN":
                self.buttons[yi][xi].config(text='')
                self.buttons[yi-1][xi].config(text='^')
                agent.location = xi, yi-1
        
        # Update the performance for every action
        if action != 'NoOp':
            agent.performance -= 1
        
        # Update the number of steps and performance labels
        NumSteps_label.config(text=str(self.stepCount))
        TotalCost_label.config(text=str(self.agent.performance))

    def read_env(self):
        """read_env: This sets proper wall or Dirt status based on bg color"""
        """Reads the current state of the GUI environment."""
        self.dirtCount = 0
        for j, btn_row in enumerate(self.buttons):
            for i, btn in enumerate(btn_row):
                if (j != 0 and j != len(self.buttons) - 1) and (i != 0 and i != len(btn_row) - 1):
                    if self.some_things_at((i, j)):  # and (i, j) != agt_loc:
                        for thing in self.list_things_at((i, j)):
                            if not isinstance(thing, Agent):
                                self.delete_thing(thing)
                    if btn['bg'] == 'grey':  # adding dirt
                        self.add_thing(Dirt(), (i, j))
                        self.dirtCount += 1
                    elif btn['bg'] == 'red':  # adding wall
                        self.add_thing(Wall(), (i, j))

    def update_env(self):
        """Updates the GUI environment according to the current state."""
        self.read_env()
        self.step()
        self.stepCount += 1

    def step(self):
        """updates the environment one step. Currently it is associated with one click of 'Step' button.
        """
        if env.dirtCount == 0:
            print("Everything is clean. DONE!")
            self.done = True
            return

        if len(self.solution) == 0:
            self.execute_action(self.agent, 'Suck')
            self.read_env()
            if env.dirtCount > 0 and self.searchAgent is not None:
                self.searchAgent.generateNextSolution()
                self.running = False
        else:
            move = self.solution.pop()
            self.execute_action(self.agent, move)


    def run(self, delay=2):
        """Run the Environment for given number of time steps,"""
        # print("run: to be implemented by students")

        # Nothing to run if the search type is none
        if self.searchType == None:
            print("Nothing to run when search type is None.")
            return
        
        count = 1
        self.update_env()

        global finished

        # Break if no dirt found or more than 1000 steps have been reached or dirt is stuck
        while env.dirtCount > 0 and count < 1000 and finished == False:
            self.update_env()
            count += 1
            sleep(1)
            Tk.update(self.root)
        
        # Dirt was stuck so let the user know
        if finished == True:
            print("Dirt piece is stuck. Run has completed.")
            messagebox.showinfo("Error", "Dirt piece is stuck. The run is complete.")
        
        finished = False

    def reset_env(self):
        """Resets the GUI and agents environment to the initial clear state."""
        self.running = False
        NumSteps_label.config(text=str(0))
        TotalCost_label.config(text=str(0))


        for j, btn_row in enumerate(self.buttons):
            for i, btn in enumerate(btn_row):
                if (j != 0 and j != len(self.buttons) - 1) and (i != 0 and i != len(btn_row) - 1):
                    if self.some_things_at((i, j)):
                        for thing in self.list_things_at((i, j)):
                            self.delete_thing(thing)
                    btn.config(bg='white', text='', state='normal')

        self.setupTestEnvironment2(self.searchType)      

    # Helper function to show the agent's path in orange colour
    def path_env(self, sol, xi, yi):
        x = xi
        y = yi
        count = len(sol)
        i = 0
        self.buttons[y][x].config(bg='orange')
        for s in sol:
            i += 1
            if i == count:
                break
            
            if s == 'UP':
                self.buttons[y+1][x].config(bg='orange', text='')
                y += 1
            elif s == 'DOWN':
                self.buttons[y-1][x].config(bg='orange', text='')
                y -= 1
            elif s == 'RIGHT':
                self.buttons[y][x+1].config(bg='orange', text='')
                x += 1
            elif s == 'LEFT':
                self.buttons[y][x-1].config(bg='orange', text='')
                x -= 1


"""
Our search Agents ignore ignore environment percepts for planning. The planning is done based ons static
 data from environment at the beginning. The environment if fully observable
 """
def XYSearchAgentProgram(percept):
    pass


class XYSearchAgent(Agent):
    """The modified SimpleRuleAgent for the GUI environment."""

    def __init__(self, program, loc):
        super().__init__(program)
        self.location = loc
        self.direction = Direction("up")
        self.searchType = searchTypes[0]
        self.stepCount = 0


if __name__ == "__main__":
    win = Tk()
    win.title("Searching Cleaning Robot")
    win.geometry("800x750+50+50")
    win.resizable(True, True)
    frame = Frame(win, bg='black')
    frame.pack(side='bottom')
    topframe = Frame(win, bg='black')
    topframe.pack(side='top')

    global finished
    finished = False

    wid = 10
    if len(sys.argv) > 1:
        wid = int(sys.argv[1])

    hig = 10
    if len(sys.argv) > 2:
        hig = int(sys.argv[2])

    env = Gui(win, wid, hig)

    start = hig//2
    end = wid//2

    theAgent = XYSearchAgent(program=XYSearchAgentProgram, loc=(hig//2, wid//2))
    x, y = theAgent.location
    env.add_agent(theAgent, (y, x))

    NumSteps_label = Label(topframe, text='NumSteps: 0', bg='green', fg='white', bd=2, padx=2, pady=2)
    NumSteps_label.pack(side='left')
    TotalCost_label = Label(topframe, text='TotalCost: 0', bg='blue', fg='white', padx=2, pady=2)
    TotalCost_label.pack(side='right')
    reset_button = Button(frame, text='Reset', height=2, width=5, padx=2, pady=2)
    reset_button.pack(side='left')
    next_button = Button(frame, text='Next', height=2, width=5, padx=2, pady=2)
    next_button.pack(side='left')
    run_button = Button(frame, text='Run', height=2, width=5, padx=2, pady=2)
    run_button.pack(side='left')

    next_button.config(command=env.update_env)
    reset_button.config(command=env.reset_env)
    run_button.config(command=env.run)

    searchTypeStr = StringVar(win)
    searchTypeStr.set(searchTypes[0])
    searchTypeStr_dropdown = OptionMenu(frame, searchTypeStr, *searchTypes, command=env.setSearchEngine)
    searchTypeStr_dropdown.pack(side='left')

    win.mainloop()
