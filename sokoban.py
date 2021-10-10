from queue import Queue
import sys, os
import keyboard
import time
this_dir = sys.path[0]

MAP = []
ALL_STATES = []
dx = [-1, 1, 0, 0] 
dy = [0, 0, -1, 1] # move: up, down, left, right

N, M = 0, 0 # size of map
pos = (0, 0) # player position
boxs, docks, goals = [], [], {} #list of box, dock, box on dock

def printInfo():
    print("@ - palyer")
    print("$ - box")
    print(". - dock")
    print("+ - player on dock")
    print("* - box on dock (goal)")
    print("# - wall \n")

    print("\n <Beginning state> \n")
    for m in MAP:
        print(*m)

    print("\n <Searching> \n")

def readMap(inPut):
    # load map data to MAP
    f = open(inPut, "r")
    for line in f:
        if len(line) > 1:
            MAP.append(list(line.strip()))


def init():
    # extract all data to variable
    global pos, boxs, docks, goals
    for i in range(N):
        for j in range(M):
            if MAP[i][j] == "@":
                pos = (i, j)
            if MAP[i][j] == "$":
                boxs.append((i, j))
            if MAP[i][j] == ".":
                docks.append((i, j))
                goals[(i, j)] = False
            if MAP[i][j] == "*":
                boxs.append((i, j))
                docks.append((i, j))
                goals[(i, j)] = True
            if MAP[i][j] != "#":
                MAP[i][j] = " "


def direction(x):
    return ["up", "down", "left", "right"][x] # move

def hashState(x, y, state):
    # hash all box position
    boxs = tuple(state[1])
    return ((x, y), boxs)

def hashAll(state):
    #hash all objects (player position, box list, goal list) position
    pos = state[0]
    boxs = tuple(state[1])
    goal = tuple(state[2].items())
    return (pos, boxs, goal)

def isBox(x, y, boxs):
    #check if this position is a box
    for box in boxs:
        if box == (x, y):
            return True
    return False

def blocked(x, y):
    # check if this object is blocked or not
    count = 0
    for i in range(4):
        if MAP[x + dx[i]][y + dy[i]] == "#":
            count += 1

    return count >= 3 # 3-side is wall

def validMove(x, y, state, direction):
    # check if next move is valid
    if not(0 <= x < N) or not(0 <= y < M) or MAP[x][y] == "#":
        return False

    hs = hashState(x, y, state)
    boxs = state[1]

    for box in boxs:
        # check if this box is blocked
        if MAP[box[0]][box[1]] == "#" or blocked(box[0], box[1]): 
            return False

    if isBox(x, y, boxs):
        # check if position is out of map
        if direction == "up" and x - 1 >= 0 and (isBox(x - 1, y, boxs) or MAP[x - 1][y] == "#"):
            return False

        if direction == "down" and x + 1 < N and (isBox(x + 1, y, boxs) or MAP[x + 1][y] == "#"):
            return False

        if direction == "left" and y - 1 >= 0 and (isBox(x, y - 1, boxs) or MAP[x][y - 1] == "#"):
            return False

        if direction == "right" and y + 1 < M and (isBox(x, y + 1, boxs) or MAP[x][y + 1] == "#"):
            return False

    elif hs in ALL_STATES:
            return False

    ALL_STATES.append(hs)
    return True

def moveBox(x, y, direction, boxs, goals):
    vector = (dx[direction], dy[direction])

    for i in range(len(boxs)):
        b_x = boxs[i][0]
        b_y = boxs[i][1]
        if b_x == x and b_y == y: #if next move is free, move box
            if boxs[i] in docks:
                goals[boxs[i]] = False
            boxs[i] = (b_x + vector[0], b_y + vector[1])
            if boxs[i] in docks: #if next move is dock, make it goal
                goals[boxs[i]] = True

def win(goals):
    for item in goals.items():
        if not item[1]: #all of goal are true
            return False
    return True

def printMap(state):
    #print all object of map, position with both object box ($) and dock (.) is goal (*)
    MAP2 = [[] for _ in range(N)]
    for i in range(N):
        MAP2[i] = MAP[i].copy()

    pos = state[0]
    boxs = state[1]
    docks = state[2]

    MAP2[pos[0]][pos[1]] = "@"
    for b in boxs:
        MAP2[b[0]][b[1]] = "$"
    for d in docks:
        if MAP2[d[0]][d[1]] == "$":
            MAP2[d[0]][d[1]] = "*"
        elif MAP2[d[0]][d[1]] == "@":
            MAP2[d[0]][d[1]] = "+"
        else: MAP2[d[0]][d[1]] = "."
    
    for m in MAP2:
        print(*m)

    print("\n")

def printAns(state):
    
    # count step moved
    print("Number of steps needed: ", len(state[3]))
    print("\n")

    #get all step moved
    steps = [direction(i) for i in state[3]]
    print(steps)
    
    # write to output file
    f = open("result.txt", "w")
    for step in steps:
        f.write(step + " ")
    f.close()

def bfs(inPut):
    global N, M, pos, boxs, goals

    #init
    readMap(inPut)
    printInfo()
    N, M = len(MAP), len(MAP[0])
    init()

    Q = Queue() # create a queue for bfs
    state = (pos, boxs, goals, [])
    Q.put(state) # mark the first state as visited and enqueue it
    
    ALL_STATES.append(hashState(pos[0], pos[1], state)) # next state

    count = 0 # count step 
    while not Q.empty():
        # dequeue a state from queue and print it
        actState = Q.get()
        count += 1
        print("States visited: ", count)
        printMap(actState)

        # check end game
        if win(actState[2]):
            print("\n <SOLUTION FOUND!> \n")
            printMap(actState)
            printAns(actState)
            break
        
        s_pos = actState[0]
        # get all adjacent states of current state
        for i in range(4):
            # next state (move up/down/left/right)
            xNew = s_pos[0] + dx[i] # next position
            yNew = s_pos[1] + dy[i]
            boxs, goals = actState[1].copy(), actState[2].copy() # get object of state
            moves = actState[3].copy()
            

            # if this state has not been visited, mark it visited and enqueue it
            if validMove(xNew, yNew, actState, direction(i)):
                if isBox(xNew, yNew, actState[1]):
                    moveBox(xNew, yNew, i, boxs, goals)
                moves.append(i)
                newState = ((xNew, yNew), boxs, goals, moves)
                Q.put(newState)



def gameAction():
    steps = [] # list of move
    f = open("result.txt", "r")
    s = f.readlines()
    for step in s[0].split():
        steps.append(step)

    print("\nOpen game from the link above and press ENTER to start auto solve!")
    keyboard.wait("enter")

    # press key from list of move
    print("\nGame solving...")
    for move in steps:
        if move == "up":
            keyboard.press("up")

        if move == "down":
            keyboard.press("down")

        if move == "left":
            keyboard.press("left")

        if move == "right":
            keyboard.press("right")
        
        time.sleep(0.3)

    print("\nGame solved!")

if __name__ == "__main__":
    print("Input list:\n")
    f = open("input/listInput.txt", "r")
    print(f.read())
    f.close()
    level = input("Choose input: ")
    if level == "7":
        print ("Mini Cosmos Level 7") 
        inPut = "input/mini_7.txt"
#   elif:

    print("\nPress ENTER to start!")
    keyboard.wait("enter")

    bfs(inPut)
    gameAction()