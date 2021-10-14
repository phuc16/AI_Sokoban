from queue import Queue
import sys, os
import keyboard
import heapq
import time
import datetime

MAP = []
ALL_STATES = set()

dx = [-1, 1, 0, 0] 
dy = [0, 0, -1, 1] # move: up, down, left, right

N, M = 0, 0 # size of map (height: N, width: M)
pos = (0, 0) # player position
boxs, docks, goals = [], [], {} #list of box, dock, box on dock

def printInfo():
    print("@ - player")
    print("$ - box")
    print(". - dock")
    print("+ - player on dock")
    print("* - box on dock (goal)")
    print("# - wall \n")

    print("\n <INIT STATE> \n")
    for m in MAP:
        print(*m)

    print("\nPress ENTER to start finding solution!")
    keyboard.wait("enter")

    print("\n <FINDING SOLUTION...> \n")

def readMap(inPut):
    # load map data to MAP
    f = open(inPut, "r")
    for line in f:
        if len(line) > 1:
            MAP.append(list(line.strip()))
    f.close()

def init():
    # extract all data to variable
    global pos, boxs, docks, goals
    for i in range(N):
        for j in range(M):
            if MAP[i][j] == "@":
                pos = (i, j)

            if MAP[i][j] == "+":
                pos = (i, j)
                docks.append((i, j))
                goals[(i, j)] = False

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
    # hash box position
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
        # check if next position is out of map
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

    ALL_STATES.add(hs)
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
    for box in boxs:
        MAP2[box[0]][box[1]] = "$"
    for dock in docks:
        if MAP2[dock[0]][dock[1]] == "$":
            MAP2[dock[0]][dock[1]] = "*"
        elif MAP2[dock[0]][dock[1]] == "@":
            MAP2[dock[0]][dock[1]] = "+"
        else: MAP2[dock[0]][dock[1]] = "."
    
    for m in MAP2:
        print(*m)

    print("\n")

def printAns(state):
    
    # count step moved
    print("Number of steps needed: ", len(state[3]), "\n")

    #get all step moved
    steps = [direction(i) for i in state[3]]
    print(steps)
    
    # write to output file
    f = open("result.txt", "w")
    for step in steps:
        f.write(step + " ")

def bfs(inPut):
    global N, M, pos, boxs, goals

    #init
    readMap(inPut)
    printInfo()
    N, M = len(MAP), len(MAP[0])
    print(M, N)
    init()

    Q = Queue() # create a queue for bfs
    state = (pos, boxs, goals, [])
    Q.put(state) # mark the first state as visited and enqueue it
    
    ALL_STATES.add(hashState(pos[0], pos[1], state)) # next state

    count = 0 # count step 
    while not Q.empty():
        # dequeue a state from queue and print it
        actState = Q.get()
        count += 1
        
        if win(actState[2]): # check complete
            print("\n <SOLUTION FOUND!> \n")
            print("States visited: ", count)
            print("\n <GOAL STATE> \n")
            printMap(actState)
            printAns(actState)
            break
        else:
            print("States visited: ", count, "\n")
            printMap(actState)
        
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

def heuristic(state):
    min_dist = 1e9

    #position of player
    player_x, player_y = state[0][0], state[0][1]
    
    #boxs
    chests = state[1]
    for c in chests:
        for g in goals:
            #calculate distance = (player->box) + (box->goals) 
            act_dist = abs(player_x - c[0]) + abs(player_y - c[1])
            act_dist += abs(c[0] - g[0]) + abs(c[1] - g[1])
            min_dist = min(min_dist, act_dist)

    return min_dist + len(state[3])



def aStar(inPut):
    global N, M, pos, boxs, goals

    #init map
    readMap(inPut)
    printInfo()
    N, M = len(MAP), len(MAP[0])
    print(M, N)
    init()
    Q = []  # priority queue
    #init state
    state = (pos, boxs, goals, [])
    heapq.heappush(Q, (heuristic(state), state))


    ALL_STATES.add(hashState(pos[0], pos[1], state))

    while len(Q) > 0:
      
        act_state = heapq.heappop(Q)  
        act_state = act_state[1]
        # print_map(act_state)


        #check complete game
        if win(act_state[2]):
            print('\nSOLUTION FOUND!!\n')
            printMap(act_state)
            printAns(act_state)
            break
        

        s_pos = act_state[0]
        for i in range(4):

            #update new position 
            new_x = s_pos[0] + dx[i]
            new_y = s_pos[1] + dy[i]

            #get objects
            boxs, goals = act_state[1].copy(), act_state[2].copy()
            moves = act_state[3].copy()
            

            if validMove(new_x, new_y, act_state, direction(i)):
                
                #check if new position is box => move box
                if isBox(new_x, new_y, act_state[1]):
                    moveBox(new_x, new_y, i, boxs, goals)

                moves.append(i)

                #update new state
                new_state = ((new_x, new_y), boxs, goals, moves)
                #push new state to Q
                heapq.heappush(Q, (heuristic(new_state), new_state))


def gameAction():
    steps = [] # list of move
    f = open("result.txt", "r")
    s = f.readlines()
    for step in s[0].split():
        steps.append(step)

    print("\nOpen game from the above link and press SPACE to run auto solver!")
    keyboard.wait("space")
    
    print("\nGAME SOLVING...")

    # press key from list of move
    for move in steps:
        if move == "up":
            keyboard.press("up")

        if move == "down":
            keyboard.press("down")

        if move == "left":
            keyboard.press("left")

        if move == "right":
            keyboard.press("right")
        
        time.sleep(0.25)

    print("\nGAME SOLVED!")
    f.close()

if __name__ == "__main__":
    print("Input list:\n")
    f = open("input/listInput.txt", "r")
    print(f.read())
    f.close()
    level = input("\nChoose input: ")
    if level == "1":
        link = "https://ksokoban.online/Mini%20Cosmos/1"
        print ("\nMini Cosmos Level 1\n") 
        inPut = "input/mini_1.txt" 
    elif level == "2":
        link = "https://ksokoban.online/Mini%20Cosmos/2"
        print ("\nMini Cosmos Level 2\n") 
        inPut = "input/mini_2.txt" 
    elif level == "3":
        link = "https://ksokoban.online/Mini%20Cosmos/3"
        print ("\nMini Cosmos Level 3\n") 
        inPut = "input/mini_3.txt" 
    elif level == "4":
        link = "https://ksokoban.online/Mini%20Cosmos/4"
        print ("\nMini Cosmos Level 4\n") 
        inPut = "input/mini_4.txt" 
    elif level == "5":
        link = "https://ksokoban.online/Mini%20Cosmos/5"
        print ("\nMini Cosmos Level 5\n") 
        inPut = "input/mini_5.txt"    
    elif level == "6":
        link = "https://ksokoban.online/Mini%20Cosmos/6"
        print ("\nMini Cosmos Level 6\n") 
        inPut = "input/mini_6.txt"
    elif level == "7":
        link = "https://ksokoban.online/Mini%20Cosmos/10"
        print ("\nMini Cosmos Level 10\n") 
        inPut = "input/mini_10.txt"
    elif level == "8":
        link = "https://ksokoban.online/Mini%20Cosmos/20"
        print ("\nMini Cosmos Level 20\n") 
        inPut = "input/mini_20.txt"
    elif level == "9":
        link = "https://ksokoban.online/Mini%20Cosmos/30"
        print ("\nMini Cosmos Level 30\n") 
        inPut = "input/mini_30.txt"
    elif level == "10":
        link = "https://ksokoban.online/Mini%20Cosmos/40"
        print ("\nMini Cosmos Level 40\n") 
        inPut = "input/mini_40.txt"
    elif level == "11":
        link = "https://ksokoban.online/Micro%20Cosmos/1"
        print ("\nMicro Cosmos Level 1\n") 
        inPut = "input/micro_1.txt"
    elif level == "12":
        link = "https://ksokoban.online/Micro%20Cosmos/2"
        print ("\nMicro Cosmos Level 2\n") 
        inPut = "input/micro_2.txt"
    elif level == "13":
        link = "https://ksokoban.online/Micro%20Cosmos/3"
        print ("\nMicro Cosmos Level 3\n") 
        inPut = "input/micro_3.txt"
    elif level == "14":
        link = "https://ksokoban.online/Micro%20Cosmos/4"
        print ("\nMicro Cosmos Level 4\n") 
        inPut = "input/micro_4.txt"
    elif level == "15":
        link = "https://ksokoban.online/Micro%20Cosmos/5"
        print ("\nMicro Cosmos Level 5\n") 
        inPut = "input/micro_5.txt"
    elif level == "16":
        link = "https://ksokoban.online/Micro%20Cosmos/6"
        print ("\nMicro Cosmos Level 6\n") 
        inPut = "input/micro_6.txt"
    elif level == "17":
        link = "https://ksokoban.online/Micro%20Cosmos/7"
        print ("\nMicro Cosmos Level 7\n") 
        inPut = "input/micro_7.txt"
    else:
        print("\nPLEASE CHOOSE A VALID INPUT!")
        sys.exit()
#   elif:

    print("Algorithm list:")
    print ("1. Breadth-first search")
    print ("2. A star")

    algorithm = input("\nChoose algorithm: ")

    startTime = time.time()
    if algorithm == "1":
        bfs(inPut)
    elif algorithm == "2":
        aStar(inPut)
    else:
        print("\nPLEASE CHOOSE A VALID ALGORITHM!")
        sys.exit()
    print("\nTime cost:", datetime.timedelta(seconds = time.time() - startTime), "\n")

    print(link)
    gameAction()