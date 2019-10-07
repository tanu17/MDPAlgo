import socket
import json
import numpy as np
import os
import time
import tornado.web as web
import tornado.websocket as websocket
import tornado.ioloop as ioloop
import threading
from threading import Thread

from tornado.options import define, options
from Algo.Exploration import Exploration
from Algo.FastestPath import FastestPath
from Algo.Constants import START, GOAL, NORTH, SOUTH, WEST, SENSOR, LEFT, RIGHT, FORWARD, FORWARDFAST, BACKWARDS, BACKWARDSFAST, ALIGNRIGHT, ALIGNFRONT, MAX_ROWS, MAX_COLS

# Global Variables
define("port", default=8888, help="run on the given port 8888", type=int)
clients = dict()
currentMap = np.zeros([MAX_ROWS, MAX_COLS])
######################################
#################Update this accordingly
######################################
mdfCounter = 3 # For map descriptor

log_file = open('log.txt', 'w')

area = 0 # % of area that has been explored
exp = '' # Global variable to store Exploration class
fsp = '' # Global variable to store Fastest Path class
visited = dict() # Stores how many times each position on the arena has been visited
waypoint = None # Stores the coordinates of the way point
steps = 0 # Records down the number of steps which the robot has taken
numCycle = 1 # Records down the number of times which the robot has ran one round around the arena
t_s = 0 # Records down timestamps
direction = 1

map_name = 'map.txt'

step = 0.1


class FuncThread(threading.Thread):

    """Class to create and run functions on different threads
    """

    def __init__(self, target, *args):
        """Construction to initialize the thread

        Args:
            target (function): Function to be run on new threads
            *args: arguments to be passed to the function
        """
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        """Overrides run function to run the function with given arguments
        """
        self._target(*self._args)


class IndexHandler(web.RequestHandler):

    """To display the front-end interface
    """

    @web.asynchronous
    def get(self):
        self.render("index.html")


class WebSocketHandler(websocket.WebSocketHandler):

    """Handles web-socket requests from the front-end to receive/send messages

    Attributes:
        id (string): id string from GET request
    """

    def open(self):
        """Open a web socket for communication
        """
        self.id = self.get_argument("Id")
        self.stream.set_nodelay(True)
        clients[self.id] = {"id": self.id, "object": self}
        print("WebSocket opened")

    def on_message(self, message):
        """Displays any message received

        Args:
            message (string): Message received from front-end
        """
        print("Client " + str(self.id) + " received a message : " + str(message))

    def on_close(self):
        """Run when the web socket is closed
        """
        print("WebSocket closed")
        if self.id in clients:
            del clients[self.id]


class StartHandler(web.RequestHandler):

    """Handles the start of exploration for the maze
    """

    @web.asynchronous
    def get(self):
        self.write("Starting...")
        self.step = self.get_argument("step")
        self.limit = self.get_argument("limit")
        self.coverage = self.get_argument("coverage")
        global step
        step = float(self.step)
        startExploration(self.limit, self.coverage)
        self.flush()


class ResetHandler(web.RequestHandler):

    """Handles the reset of the current map
    """

    @web.asynchronous
    def get(self):
        self.write("Reset...")
        global exp
        exp = Exploration(map_name, 5)
        update(np.zeros([MAX_ROWS, MAX_COLS]), exp.exploredArea, exp.robot.center, exp.robot.head,
               START, GOAL, 0)


class FSPHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        self.x = self.get_argument("x")
        self.y = self.get_argument("y")
        self.write("Starting...")
        startFastestPath([self.x, self.y])
        self.flush()


class LoadMapHandler(web.RequestHandler):

    """Handles the start of fastest path for the maze
    """

    @web.asynchronous
    def get(self):
        global map_name
        self.name = self.get_argument("name")
        map_name = self.name


def startExploration(limit, coverage):
    """To start the exploration of the maze
    """
    global exp, t_s
    exp = Exploration(map_name, 5)
    t_s = time.time()
    t2 = FuncThread(exploration, exp, limit, coverage)
    t2.start()
    # t2.join()


def exploration(exp, limit, coverage):
    """To explore the map and update the front-end after each move

    Args:
        exp (Exploration): New instance of the exploration class
    """
    global currentMap, area
    time_limit = float(limit)
    elapsedTime = 0
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL, 0)
    logger('Exploration Started !')
    current = exp.moveStep()
    currentMap = exp.currentMap
    area = exp.exploredArea
    visited = dict()
    steps = 0
    numCycle = 1
    while (not current[1] and elapsedTime <= time_limit and exp.exploredArea < int(coverage)):
        elapsedTime = round(time.time()-t_s, 2)
        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
               elapsedTime)
        current = exp.moveStep()
        currentMap = exp.currentMap
        area = exp.exploredArea
        steps += 1
        currentPos = tuple(exp.robot.center)
        if (currentPos in visited):
            visited[currentPos] += 1
            if (area > 15 and visited[currentPos] > 1) or (visited[currentPos] > 2):
                neighbour = exp.getExploredNeighbour()
                if (neighbour):
                    neighbour = np.asarray(neighbour)
                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                      exp.robot.direction, None)
                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                    exp.robot.center = neighbour
                    exp.robot.head = fsp.robot.head
                    exp.robot.direction = fsp.robot.direction
                    if (exp.robot.direction == NORTH):
                        if(0<=exp.robot.center[1]+2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 ):
                            exp.robot.moveBot(RIGHT)
                            exp.robot.getSensors()
                        elif(0<=exp.robot.center[1]-2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 1][exp.robot.center[1] -2] == 0)):
                            exp.robot.moveBot(LEFT)
                            exp.robot.getSensors()
                    elif (exp.robot.direction == SOUTH):
                        if(0<= exp.robot.center[1] +2<MAX_COLS and (exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] +2] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 1][exp.robot.center[1] + 2] == 0)):
                            exp.robot.moveBot(LEFT)
                            exp.robot.getSensors()
                        elif(0<=exp.robot.center[1]-2<MAX_COLS and exp.robot.exploredMap[exp.robot.center[0]][exp.robot.center[1] -2] == 0):
                            exp.robot.moveBot(RIGHT)
                            exp.robot.getSensors()
                    elif (exp.robot.direction == EAST):
                        if(0<= exp.robot.center[0]+2 <MAX_ROWS and exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0):
                            exp.robot.moveBot(RIGHT)
                            exp.robot.getSensors()
                        elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] - 2][exp.robot.center[1] - 1] == 0)):
                            exp.robot.moveBot(LEFT)
                            exp.robot.getSensors()
                    else:
                        if(0<= exp.robot.center[0]+2 < MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]+2][exp.robot.center[1]] == 0 or exp.robot.exploredMap[exp.robot.center[0] + 2][exp.robot.center[1] + 1] == 0)):
                            exp.robot.moveBot(LEFT)
                            exp.robot.getSensors()
                        elif(0<= exp.robot.center[0]-2 <MAX_ROWS and (exp.robot.exploredMap[exp.robot.center[0]-2][exp.robot.center[1]] == 0)):
                            exp.robot.moveBot(RIGHT)
                            exp.robot.getSensors()
                else:
                    break
        else:
            visited[currentPos] = 1
        if (np.array_equal(exp.robot.center, START)):
            numCycle += 1
            # If robot has came back to start position more than once and it took more than 4 steps
            if (numCycle > 2 and steps > 4):
                # Try to get any unexplored neighbours
                neighbour = exp.getExploredNeighbour()
                # If there are unexplored neighbours, go and explore it
                if (neighbour):
                    neighbour = np.asarray(neighbour)
                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                      exp.robot.direction, None)
                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                    exp.robot.center = neighbour
                # If there are no unexplored neighbours, do nothing
                else:
                    break
        time.sleep(float(step))
    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head, START, GOAL,
           elapsedTime)
    logger('Exploration Done !')
    logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
    logger("Map Descriptor 2  -->  "+str(exp.robot.descriptor_2()))
    print currentMap
    fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction, None)
    logger('Fastest Path Started !')
    fastestPath(fsp, START, exp.exploredArea, None)


def startFastestPath(waypoint):
    """To start the fastest path of the maze
    """
    global fsp
    global t_s
    waypoint = map(int, waypoint)
    fsp = FastestPath(currentMap, START, GOAL, NORTH, waypoint)
    t_s = time.time()
    logger('Fastest Path Started !')
    t3 = FuncThread(fastestPath, fsp, GOAL, area, waypoint)
    t3.start()
    # t3.join() this causes the thread to close after exploration and websocket closes


def markMap(curMap, waypoint):
    if waypoint:
        curMap[tuple(waypoint)] = 7
    return curMap


# Function to shorten muliple forward commands into a single command
# def combineMovement(movement):
#     counter = 0
#     shortMove = []
#     while (counter < len(movement)):
#         if (counter < len(movement)-7) and all(x == 'W' for x in movement[counter:counter+7]):
#             shortMove.append('Q')
#             counter += 7
#         elif (counter < len(movement)-5) and all(x == 'W' for x in movement[counter:counter+5]):
#             shortMove.append('K')
#             counter += 5
#         elif (counter < len(movement)-3) and all(x == 'W' for x in movement[counter:counter+3]):
#             shortMove.append('X')
#             counter += 3
#         elif (counter < len(movement)-2) and all(x == 'W' for x in movement[counter:counter+2]):
#             shortMove.append('P')
#             counter += 2
#         else:
#             shortMove.append(movement[counter])
#             counter += 1
#     shortMove += movement[counter:]
#     return shortMove


def fastestPath(fsp, goal, area, waypoint, backwards=False):
    if(backwards == False):
        fsp.getFastestPath()
    else:
        fsp.getFastestPath(backwards=True)
    logger(json.dumps(fsp.path))
    while (fsp.robot.center.tolist() != goal.tolist()):
        if(backwards == False):
            fsp.moveStep()
        else:
            fsp.moveStep(backwards=True)
        update(markMap(np.copy(fsp.exploredMap), waypoint), area, fsp.robot.center, fsp.robot.head,
               START, GOAL, 0)
    logger('Fastest Path Done !')


def update(current_map, exploredArea, center, head, start, goal, elapsedTime):
    """To send messages to update the front-end

    Args:
        current_map (Numpy array): Current state of the exploration map
        exploredArea (int): Number of cells that have been explored
        center (list): Location of center of the robot
        head (list): Location of head of the robot
        start (list): Location of the starting point for the robot
        goal (list): Location of the finishing point for the robot
        elapsedTime (float): The time that has elapsed since exploration started
    """
    for key in clients:
        message = dict()
        message['area'] = '%.2f' % (exploredArea)
        tempMap = current_map.copy()
        tempMap[start[0]-1: start[0]+2, start[1]-1: start[1]+2] = 3
        tempMap[goal[0]-1: goal[0]+2, goal[1]-1: goal[1]+2] = 4
        message['map'] = json.dumps(tempMap.astype(int).tolist())
        message['center'] = json.dumps(center.astype(int).tolist())
        message['head'] = json.dumps(head.astype(int).tolist())
        message['time'] = '%.2f' % (elapsedTime)
        clients[key]['object'].write_message(json.dumps(message))


def logger(message):
    for key in clients:
        log = {'log': message}
        clients[key]['object'].write_message(json.dumps(log))


def output_formatter(msg, movement):
    if not isinstance(movement, list):
        movement = movement.tolist()
    movement = map(str, movement)
    return msg+'|'+'|'.join(movement)

def android_message_formatter(msg, array):
    if not isinstance(array, list):
        array = array.tolist()
    return "B" + msg + '|' + '|'.join(map(str, array))

def arduino_message_formatter(movement, getSensor=True):
    if not isinstance(movement, list):
        movement = movement.tolist()
    string = "".join(map(str, movement))
    res = ""

    count = 1

    #Add in first character
    res += string[0]

    #Iterate through loop, skipping last one
    for i in range(len(string)-1):
        if(string[i] == string[i+1]):
            count+=1
            if count == 9:
                res += str(count)
                res += string[i+1]
                count = 1
        else:
            res += str(count)
            res += string[i+1]
            count = 1
    #print last one
    res += str(count)
    if getSensor == True:
        return "A" + res + SENSOR + "1"
    else:
        return "A" + res


class RPi(threading.Thread):
    def __init__(self):
        print "starting rpi communication"
        threading.Thread.__init__(self)

        self.ip = "192.168.6.6"  # Connecting to IP address of MDPGrp26
        self.port = 1273

        # Create a TCP/IP socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        print "sent connection request"

        # Receive and send data to RPi data
    def receive_send(self):
        time_t = time.time()
        while True:
            current_pos = None
            data = self.client_socket.recv(2048) # Parameter is the maximum size of data to be received from client
            log_file.write(data+'\n')
            log_file.flush()
            if (data):
                print ('Received %s from RPi' % (data))
                data = str(data)
                data = data.rstrip()
                split_data = data.split("|") # To be updated
                global exp, t_s, area, steps, numCycle, currentMap, exp, fsp
                # Initialise exploration
                if (split_data[0] == 'EXPLORE'):
                    t_s = time.time()
                    exp = Exploration(sim=False)
                    current_pos = exp.robot.center
                    visited[tuple(current_pos)] = 1
                    update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                           START, GOAL, 0)
                    arduino_msg = arduino_message_formatter(["L"], getSensor=False)
                    self.client_socket.send(arduino_msg)
                    print ('Sent %s to RPi' % (arduino_msg))
                # Set waypoint
                elif (split_data[0] == 'WAYPOINT'):
                    global waypoint
                    waypoint = map(int, split_data[1:])
                    waypoint[0] = 19 - waypoint[0]
                elif (split_data[0] == 'COMPUTE'):
                    print(split_data)
                    print 'Time 0: %s s' % (time.time() - time_t)
                    # Get sensor values
                    sensors = map(int, split_data[1:])
                    current_pos = exp.robot.center
                    current = exp.moveStep(sensors) # Get next movements and whether or not 100% covergae is reached
                    currentMap = exp.currentMap
                    # If not 100% coverage
                    if (not current[1]):
                        time_t = time.time()
                        move = current[0]
                        elapsedTime = round(time.time()-t_s, 2)
                        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                               START, GOAL, elapsedTime)
                        steps += 1
                        current_pos = tuple(exp.robot.center)
                        # If the current position has been visited
                        if (current_pos in visited):
                            # Increase the visited count of the current position by one
                            visited[current_pos] += 1
                            # If the current position has been visited for more than three times
                            if (visited[current_pos] >= 1):
                                # Get a valid unexplored neighbour of explored spaces
                                neighbour = exp.getExploredNeighbour()
                                # If there is such a neighbour
                                if (neighbour):
                                    neighbour = np.asarray(neighbour)
                                    # Get the shortest path to go to that neighbour
                                    fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                      exp.robot.direction, None, sim=False)
                                    # Move and update robot virtually
                                    fastestPath(fsp, neighbour, exp.exploredArea, None)
                                    move.extend(fsp.movement)
                                    exp.robot.phase = 2
                                    exp.robot.center = neighbour
                                    exp.robot.head = fsp.robot.head
                                    exp.robot.direction = fsp.robot.direction
                                    currentMap = exp.currentMap
                            # If the robot goes back to the start after explorin more than 50% of the arena
                            if (np.array_equal(exp.robot.center, START) and exp.exploredArea > 50):
                                # Increase cycle count by 1
                                numCycle += 1
                                # If the number of cycles is greater than one and robot has taken more than 4 steps
                                if (numCycle > 1 and steps > 4):
                                    # Get valid neighbours of unexplored spaces that is the closest to the robot's current position
                                    neighbour = exp.getExploredNeighbour()
                                    if (neighbour):
                                        neighbour = np.asarray(neighbour)
                                        # If the neighbour exists, get the fastest path from robot's current position to that neighbour
                                        fsp = FastestPath(currentMap, exp.robot.center, neighbour,
                                                          exp.robot.direction, None, sim=False)
                                        # Move the robot vrtually
                                        fastestPath(fsp, neighbour, exp.exploredArea, None)
                                        # Shorten multiple forward commands and append to move
                                        move.extend(fsp.movement)
                                        exp.robot.phase = 2
                                        exp.robot.center = neighbour
                                        exp.robot.head = fsp.robot.head
                                        exp.robot.direction = fsp.robot.direction
                                        currentMap = exp.currentMap
                        print 'Time 1: %s s' % (time.time() - time_t)
                        time_t = time.time()
                        ######################################
                        ###############Update this accordingly
                        ######################################
                        # arduino_msg and android_msg is the message to be sent to Rpi
                        arduino_msg = arduino_message_formatter(move)
                        android_msg = android_message_formatter('EXPLORE',[str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction])
                        print 'Time 2: %s s' % (time.time() - time_t)
                    # If 100% coverage
                    else:
                        # Send movement to Rpi
                        move = current[0]
                        arduino_msg = arduino_message_formatter(move)
                        android_msg = android_message_formatter('DONE', [str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction])
                        self.client_socket.send(android_msg)
                        print ('Sent %s to RPi' % (android_msg))
                        self.client_socket.send(arduino_msg)
                        print ('Sent %s to RPi' % (arduino_msg))
                        log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                        log_file.write('Sent %s to RPi\n\n' % (android_msg))
                        log_file.write('Sent %s to RPi\n\n' % (arduino_msg))
                        log_file.flush()
                        time.sleep(2)
                        # Inform front end that exploration is complete
                        update(exp.currentMap, exp.exploredArea, exp.robot.center, exp.robot.head,
                               START, GOAL, elapsedTime)
                        logger('Exploration Done !')
                        logger("Map Descriptor 1  -->  "+str(exp.robot.descriptor_1()))
                        logger("Map Descriptor 2  -->  "+str(exp.robot.descriptor_2()))
                        # Initiate fastest path without any waypoint to get from robot's current position to the start point
                        fsp = FastestPath(currentMap, exp.robot.center, START, exp.robot.direction,
                                          None, sim=False)
                        logger('Fastest Path Started !')
                        fastestPath(fsp, START, exp.exploredArea, None)
                        move = fsp.movement
                        currentMap = exp.currentMap
                        global direction
                        ######################################
                        ###############Update this accordingly
                        ######################################
                        if (fsp.robot.direction == WEST):
                            calibrate_move = [LEFT, ALIGNRIGHT, ALIGNFRONT]
                        else:
                            calibrate_move = [ALIGNRIGHT, ALIGNFRONT]
                        # After calibrating robot such that it faces North, set direction as North
                        direction = SOUTH
                        ######################################
                        ###############Update this accordingly
                        ######################################
                        arduino_msg = arduino_message_formatter(move + calibrate_move, getSensor=False)
                        android_msg = android_message_formatter('ALIGN', [str(exp.robot.descriptor_1()), str(exp.robot.descriptor_2()), str(exp.robot.descriptor_2()), "[" + str(19 - exp.robot.center[0]) + "," + str(exp.robot.center[1]) + "]", exp.robot.direction])
                        time.sleep(1)
                    self.client_socket.send(android_msg)
                    print ('Sent %s to RPi' % (android_msg))
                    self.client_socket.send(arduino_msg)
                    print ('Sent %s to RPi' % (arduino_msg))
                    log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                    log_file.write('Sent %s to RPi\n\n' % (android_msg))
                    log_file.write('Sent %s to RPi\n\n' % (arduino_msg))
                    log_file.flush()
                # Start fastest path
                elif (split_data[0] == 'FASTEST'):
                    fsp = FastestPath(currentMap, START, GOAL, direction, waypoint, sim=False)
                    #file1= np.ones([20, 15])
                    #sim=True
                    #fsp = FastestPath(file1, START, GOAL, direction, waypoint, sim)

                    current_pos = fsp.robot.center
                    fastestPath(fsp, GOAL, 300, waypoint, backwards=False)
                    # move = fsp.movement
                    move = fsp.movement
                    path = fsp.path
                    arduino_msg = arduino_message_formatter(move, getSensor=False)
                    android_msg = android_message_formatter('FASTEST', path)
                    self.client_socket.send(android_msg)
                    print ('Sent %s to RPi' % (android_msg))
                    self.client_socket.send(arduino_msg) 
                    print ('Sent %s to RPi' % (arduino_msg))
                    log_file.write('Robot Center: %s\n' % (str(exp.robot.center)))
                    log_file.write('Sent %s to RPi\n\n' % (android_msg))
                    log_file.write('Sent %s to RPi\n\n' % (arduino_msg))
                    log_file.flush()
                # To move the robot manually
                elif (split_data[0] == 'MANUAL'):
                    manual_movement = split_data[1:]
                    for move in manual_movement:
                        exp.robot.moveBot(move)

    def keep_main(self):
        while True:
            time.sleep(0.5)
            # edit out sleep to check if we can optimize run time
            # edit out fastestPath during exploration and check if it optimizes run time

settings = dict(
    template_path=os.path.join(os.path.dirname(__file__), "GUI", "templates"),
    debug=True
)

app = web.Application([
    (r'/', IndexHandler),
    (r'/websocket', WebSocketHandler),
    (r'/start', StartHandler),
    (r'/reset', ResetHandler),
    (r'/fsp', FSPHandler),
    (r'/lm', LoadMapHandler),
    (r'/(.*)', web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)


def func1():
    print "Starting communication with RPi"
    client_rpi = RPi()
    rt = threading.Thread(target=client_rpi.receive_send)
    rt.daemon = True
    rt.start()
    client_rpi.keep_main()


def func2():
    print "Starting communication with front-end"
    app.listen(options.port)
    t1 = FuncThread(ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()

if __name__ == '__main__':
    Thread(target=func1).start()
    Thread(target=func2).start()
