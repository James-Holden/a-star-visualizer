import sys
import time 
import math
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import *
from random import seed
from random import randint 

from PyQt5 import QtTest

 

#global variables to hard code app parameters 
height = 23
width = 40
startx = 1
starty = 1
endx = 30
endy = 20


class MainWindow(QMainWindow):

	def __init__(self, *args, **kwargs):
		super(MainWindow, self).__init__(*args, **kwargs)

		self.setWindowTitle("Path Planner")

		self.setGeometry(100, 100, 800, 500)

		#create toolbar
		toolbar = QToolBar("My main toolbar")
		toolbar.setIconSize(QSize(16,16))
		self.addToolBar(toolbar)

		#combobox algorithm selection 
		self.combo = QComboBox()
		toolbar.addWidget(self.combo)
		self.combo.insertItems(1,["Uniform Cost","A*"])
		self.combo.setStatusTip("Select Algorithm to Execute")

		#playbutton to execute algorithm
		play_button_action = QAction(QIcon("play.png"),"Your button", self)
		play_button_action.setStatusTip("Start Algorithm")
		play_button_action.triggered.connect(self.onPlayButtonClick)
		toolbar.addAction(play_button_action)

		#reset button to reset world 
		reset_button_action = QAction(QIcon("reset.png"),"Your button", self)
		reset_button_action.setStatusTip("Reset Environment")
		reset_button_action.triggered.connect(self.onResetButtonClick)
		toolbar.addAction(reset_button_action)

		#random world generation
		random_button_action = QAction(QIcon("random.jpg"),"Your button", self)
		random_button_action.setStatusTip("Generate Random Environment")
		random_button_action.triggered.connect(self.onRandomButtonClick)
		toolbar.addAction(random_button_action)

		#text lable displays amount of nodes generated to the algorithm
		self.nodesExpandedLabel = QLabel("Nodes Expanded: 0")
		self.nodesExpandedLabel.setAlignment(Qt.AlignRight)
		toolbar.addWidget(self.nodesExpandedLabel)

		#list of all button in environment 
		self.worldButtons = []

		#create environment of buttons 
		for j in range(0, height):
			for i in range(0, width):
				button = QPushButton(self)
				button.setGeometry(i*20, j*20+30, 20, 20) 
				button.setFont(QFont(QFont('Times', 17))) 
				button.clicked.connect(self.onEnvironmentClick)
				button.setStatusTip("Create Obstacle") 
				self.worldButtons.append(button)

		#hard coded coordinates start and end nodes 
		self.startx = startx
		self.starty = starty
		self.endx = endx
		self.endy = endy

		self.worldButtons[self.startx + self.starty*width].setStyleSheet("background-color: blue")
		self.worldButtons[self.endx + self.endy*width].setStyleSheet("background-color: red")

		#status bar for hovering over widgets 
		self.setStatusBar(QStatusBar(self))


	#execute selected algorithm
	def onPlayButtonClick(self, s):
		algo = self.combo.currentText()
		print("play", algo)
		#convert the colors in gui to formate for algo 
		#execute selected algo 
		if(algo == "A*"):
			self.planPath((self.startx, self.starty), (self.endx, self.endy), self.astar, self.parseEnvironment())
		elif(algo == "Uniform Cost"):
			self.planPath((self.startx, self.starty), (self.endx, self.endy), self.uniformCost, self.parseEnvironment())


	#convert UI world into form for algorithm
	def parseEnvironment(self):
		environment = [] #@_##__$
		#return 1d environment expression
		for button in self.worldButtons:
			color = button.palette().button().color().name()
			if(color == "#efefef"):
				environment.append('_')
			elif(color == "#000000"):
				environment.append('#')
			elif(color == "#ff0000"):
				environment.append('_')#$
			elif(color == "#0000ff"):
				environment.append('@')

		return environment


	#reset world environment 
	def onResetButtonClick(self, s):
		print("reset", s)
		for i in range(0,height*width):
			if(self.worldButtons[i].palette().button().color().name() == "#000000"):
				continue
			self.worldButtons[i].setStyleSheet("background-color: #efefef")
		self.worldButtons[self.startx + self.starty*width].setStyleSheet("background-color: blue")
		self.worldButtons[self.endx + self.endy*width].setStyleSheet("background-color: red")
		self.nodesExpandedLabel.setText("Nodes Expanded: 0")


	#add random geneworld environment 
	def onRandomButtonClick(self, s):
		print("Adding Random Nodes")
		seed()
		for i in range(0,height*width):
			value = randint(0, 10)
			if(value == 0):
				self.worldButtons[i].setStyleSheet("background-color: black")
		self.worldButtons[self.startx + self.starty*width].setStyleSheet("background-color: blue")
		self.worldButtons[self.endx + self.endy*width].setStyleSheet("background-color: red")

	#click for custon obstables 
	def onEnvironmentClick(self): 
		button = self.sender() 
		button.setStyleSheet("background-color: black")
		self.worldButtons[self.startx + self.starty*width].setStyleSheet("background-color: blue")
		self.worldButtons[self.endx + self.endy*width].setStyleSheet("background-color: red")


	def astar(self, start, goal, environment):
		print("definitly astar")
		closedList = [] #list of tuples of all explored nodes 
		openList = [] #nodes that have been touched but not expanded yet 

		startNode = Node(start[0], start[1], None)

		cur = startNode
		
		while (cur.x, cur.y) != (goal[0], goal[1]):
			cur.expand(environment)
			for child in cur.children:
				if (child.x,child.y) not in closedList:
					if (child.x, child.y) == (goal[0], goal[1]):
						solutionNodes = []
						while(cur.parent != None):
							solutionNodes.append((cur.x,cur.y))
							cur = cur.parent 
						self.displayVisualization(closedList, solutionNodes, len(closedList))
						return						

					openList.append(child)
					closedList.append((child.x, child.y))
			openList = sorted(openList, key=lambda node: node.g + node.h(goal))
			if(len(openList)):
				cur = openList.pop(0)
			else:
				break






	def uniformCost(self, start, goal, environment):

		closedList = [] #list of tuples of all explored nodes 
		openList = [] #nodes that have been touched but not expanded yet 

		startNode = Node(start[0], start[1], None)

		cur = startNode
		
		while (cur.x, cur.y) != (goal[0], goal[1]):
			cur.expand(environment)
			for child in cur.children:
				if (child.x,child.y) not in closedList:
					if (child.x, child.y) == (goal[0], goal[1]):
						# print("Depth of sol node ", child.g)
						# print("h of startNode", startNode.h())
						solutionNodes = []
						while(cur.parent != None):
							solutionNodes.append((cur.x,cur.y))
							cur = cur.parent 
						self.displayVisualization(closedList, solutionNodes, len(closedList))
						return						

					openList.append(child)
					closedList.append((child.x, child.y))
			if(len(openList)):
				cur = openList.pop()
			else:
				break





			


	def planPath(self, start, goal, algorithm, environment):
		algorithm(start, goal, environment)


	#passed planner results to qt for visualization
	#remember g value of a ndoe can be used for step playback 
	#but also nodes in closed list are in order of use 
	def displayVisualization(self, expandedNodes, solutionNodes, numNodesExpanded):

		for node in expandedNodes:
			QtTest.QTest.qWait(10)
			self.worldButtons[node[0] + node[1]*width].setStyleSheet("background-color: blue")

		# print(len(expandedNodes))
		for node in solutionNodes:
			self.worldButtons[node[0] + node[1]*width].setStyleSheet("background-color: green")
			self.worldButtons[self.endx + self.endy*width].setStyleSheet("background-color: red")

		print("len(solutionNodes)", len(solutionNodes))	
		self.nodesExpandedLabel.setText("Nodes Expanded: " + str(numNodesExpanded))






class Node():
	def __init__(self, x, y, parent):
		self.x = x 
		self.y = y
		self.parent = parent
		if parent != None:
			self.g = parent.g + 1
		else:
			self.g = 0
		self.children = []

	def h(self, goal):
		return math.sqrt((self.x - goal[0])**2 + (self.y - goal[1])**2)

	def expand(self, environment):
		#up 
		if(self.y - 1 >= 0):
			up = environment[self.x + ((self.y-1) * width)]
			if(up == '_'):
				self.children.append(Node(self.x, self.y - 1, self))
		
		#down 
		if(self.y + 1 < width):
			down = environment[self.x + ((self.y+1) * width)]
			if(down == '_'):
				self.children.append(Node(self.x, self.y + 1, self))
		
		#left 
		if(self.x - 1 >= 0):
			left = environment[self.x - 1 + (self.y * width)]
			if(left == '_'):
				self.children.append(Node(self.x - 1, self.y, self))

		#right
		if(self.x + 1 < width):
			right = environment[self.x + 1 + (self.y * width)]
			if(right == '_'):
				self.children.append(Node(self.x + 1, self.y, self))





#run application
app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()