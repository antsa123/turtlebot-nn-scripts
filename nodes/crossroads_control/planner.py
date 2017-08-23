#! /usr/bin/env python

# This is a ROS node for calculating paths for crossroads_control and publishing them to the robot.
# It is supposed to be run on a separate computer from the robot.

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from time import sleep
import os

class Node:
    def __init__(self, name, type, neighbors):
        self.name = name
        self.type = type # C = corridor, T = T-junction, I = intersection, D = dead end
        self.neighbors = neighbors # ordered as: up,down,right,left

        # For Dijkstra
        self.previous = 0
        self.dist = float('inf')

    def tellName(self):
        return self.name

    def tellType(self):
        return self.type

    def getNeighbors(self):
        return self.neighbors

    def initializeDijkstra(self, isSource):
        self.previous = 0
        if isSource:
            self.dist = 0
        else:
            self.dist = float('inf')

    def setPrev(self, nodename):
        self.previous = nodename

    def setDist(self, dist):
        self.dist = dist

    def getPrev(self):
        return self.previous

    def getDist(self):
        return self.dist

class Planner:
    def __init__(self):
        self.currentNode = 1
        self.currentTask = 1
        self.path = ""
        self.direction = 0 # options: 0 = up, down = 1, right = 2, left = 3
        self.nodes = {}
        self.finished = True

        self.planPub = rospy.Publisher("/path", String, queue_size=10)
        self.doneSub = rospy.Subscriber("plan_done", Bool, self.setTaskFinished)

    def updateCurrentNode(self):
        # when the goal is reached currenNode is updated
        self.currentNode = self.currentTask
        print(self.currentNode)

    def addNewNode(self,n):
        if n.name not in self.nodes:
            self.nodes[int(n.tellName())] = n
            print "Node: ", n.tellName(), " added."
        else:
            print "Node: ", n.tellName()," already exists"

    def setNewTask(self, task):
        if task in self.nodes:
            self.currentTask = task
            print "Next task:",self.currentTask,"added."
            self.finished = False
            return True
        else:
            print "No such task."
            return False

    def sendNewPath(self, path): # For plans given by user
        self.path = path
        print "plan:", self.path
        self.finished = False
        self.planPub.publish(self.path)

    def dijkstra(self):
        #initialization
        for nodename in self.nodes:
            if nodename == self.currentNode:
                self.nodes[nodename].initializeDijkstra(True)
            else:
                self.nodes[nodename].initializeDijkstra(False)

        # Form priority queue
        priorQ = {}
        for node in self.nodes.keys():
            if self.nodes[node].getDist() in priorQ:
                priorQ[self.nodes[node].getDist()].append(node)
            else:
                priorQ[self.nodes[node].getDist()] = [node]
        pathFound = False
        while len(priorQ) != 0:
            # take the one with smallest distance to the source
            nodesSorted = sorted(priorQ.keys())
            next = priorQ[nodesSorted[0]][0]

            # if next one is the goal we can stop
            if next == self.currentTask:
                pathFound = True
                print "Path found!"
                break

            # remove that one from the priority queue
            if len(priorQ[nodesSorted[0]]) == 1:
                del priorQ[nodesSorted[0]]
            else:
                priorQ[nodesSorted[0]] = priorQ[nodesSorted[0]][1:]

            # Go through neighbors
            newDist = 1 + self.nodes[next].getDist() # distance betweem nodes is always 1
            for neighbor in self.nodes[next].getNeighbors():
                if neighbor != 0:
                    # Update previous and distanse if better option is found
                    if newDist < self.nodes[neighbor].getDist():

                        # Remove neighbor from priorQ with the old key
                        priorQ[self.nodes[neighbor].getDist()].remove(neighbor)

                        # remove key if list is empty now
                        if len(priorQ[self.nodes[neighbor].getDist()]) == 0:
                            del priorQ[self.nodes[neighbor].getDist()]

                        # Update values
                        self.nodes[neighbor].setDist(newDist)
                        self.nodes[neighbor].setPrev(next)
                        print "change",neighbor,"prev to",next

                        # add to priorQ with new key
                        if newDist not in priorQ:
                            priorQ[newDist] = [neighbor]
                        else:
                            priorQ[newDist].append(neighbor)
        return pathFound


    def giveCmd(self, fromWhere, toWhere):
        # Here also up = 0, down = 1, right = 2, left = 3 (same way as the indexes in neighbor list)
        if fromWhere == toWhere:
            # Same direction
            return "S"

        elif (fromWhere == 0 and toWhere == 1) or (fromWhere == 1 and toWhere == 0) \
            or (fromWhere == 2 and toWhere == 3) or (fromWhere == 3 and toWhere == 2):
            # Opposite direction
            return "U"

        elif (fromWhere == 3 and toWhere == 0) or (fromWhere == 2 and toWhere == 1) \
            or (fromWhere == 0 and toWhere == 2) or (fromWhere == 1 and toWhere == 3):
            # Turning to the right
            return "R"

        elif (fromWhere == 3 and toWhere == 1) or (fromWhere == 2 and toWhere == 0) \
                or (fromWhere == 0 and toWhere == 3) or (fromWhere == 1 and toWhere == 2):
            # Turning to the left
            return "L"
        else:
            print "Something went wrong!"
            return "ERROR"

    def makePlan(self):

        if self.currentTask == self.currentNode:
            print "Agent is already at the goal."
            self.finished = True
            return

        if self.dijkstra():
            # Plan is a string containing path instructions separated with ':'
            # R = turn right, L = turn left, S = straight, U = turn around
            self.path = ""

            nextNode = self.currentTask
            prev = self.nodes[self.currentTask].getPrev()
            newDirection = 0

            while True:
                # the last move determinates direction
                if nextNode == self.currentTask:
                    # Update direction
                    ind = (self.nodes[prev].getNeighbors()).index(nextNode)
                    newDirection = ind
                    print "Going from", prev, "to", nextNode, "updated direction:", ind

                # One move left
                if prev == self.currentNode:
                    # check in which direction the next node is
                    ind = (self.nodes[prev].getNeighbors()).index(nextNode)
                    self.path = self.giveCmd(self.direction, ind) + self.path

                    # Now direction can be updated
                    self.direction = newDirection
                    break

                # we need to know what is the previous command
                prev2 = self.nodes[prev].getPrev()

                # get directions
                ind1 = (self.nodes[prev].getNeighbors()).index(nextNode)
                ind2 = (self.nodes[prev2].getNeighbors()).index(prev)
                self.path = self.giveCmd(ind2, ind1) + self.path

                nextNode = prev
                prev = self.nodes[nextNode].getPrev()

            # send plan
            print "plan:",self.path
            self.finished = False
            self.planPub.publish(self.path)
        else:
            print "No path"

    def setTaskFinished(self, data):
        if data:
            # update currenNode
            self.updateCurrentNode()
            self.finished = True
            print "Task done, current node:",self.currentNode, "and direction:",self.direction
        else:
            print "Unable to finish task."

    def isFinished(self):
        return self.finished


def readGraphFile(pathPlanner):
    graphfilename = os.getenv('HOME') + "/catkin_ws/src/ohjaus/src/Graafit/graph4.txt"

    # 0 in file means no neighbor on that side
    file = open(graphfilename, 'r')

    # skip the first line
    lines = file.readlines()[1:]
    for line in lines:
        info = line.split(':')
        name = int(info[0])
        type = info[1]
        neighbors = [int(neighbor) for neighbor in info[2:] ]

        # create new node
        n = Node(name,type,neighbors)
        # give it to planner
        pathPlanner.addNewNode(n)
    file.close()


def ask_task(pathPlanner):
    task = input("Give next task: ")

    return pathPlanner.setNewTask(int(task))


def main():

    # make planner
    pathPlanner = Planner()

    # ROS initialization
    rospy.init_node("planner", anonymous=True)

    # read graph file
    readGraphFile(pathPlanner)

    while not rospy.is_shutdown():

        try:
            cmd = int(input("Stop (0), use graph (1) or give instructions (2)? "))
            if cmd == 1:
                # ask a task
                taskGot = ask_task(pathPlanner)
                while not taskGot:
                    taskGot = ask_task(pathPlanner)

                # make the plan
                pathPlanner.makePlan()

            elif cmd == 2:
                print "Give instructions: S=straight, R=right, L=left, U=U-turn"
                inst = raw_input("(stop = 0): ")

                if inst == "0":
                    quit()

                inst = inst.upper()
                correct = True

                for letter in inst:
                    if letter != 'S' and letter != 'R' and letter != 'L' and letter != 'U':
                        print("Uncorrect instructions!")
                        correct = False
                        break
                if correct:
                    pathPlanner.sendNewPath(inst)
                else:
                    continue

            elif cmd == 0:
                quit()
            else:
                print "Give 0, 1 or 2."
                continue

            # wait untill it is done
            r = rospy.Rate(20)
            try:
                while not pathPlanner.isFinished():
                    #r.sleep()
                    sleep(0.05)
            except KeyboardInterrupt:
                rospy.on_shutdown(h=0)
        except NameError:
            print "Options are 0, 1 and 2."
            continue

main()
