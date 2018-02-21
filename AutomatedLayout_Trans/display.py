import numpy as np
import cv2
import math
class plotRoom():
    def __init__(self, winSize=(1000,800),roomSize=(800,600), resfile_name = "recommendation.txt"):
        try:
            fp = open(resfile_name, 'r')
            contents = fp.readlines()
        finally:
            fp.close()
        self.win = np.zeros([winSize[1], winSize[0], 3], np.uint8)
        self.roomSize = roomSize
        self.win_room_offx = (winSize[0] - roomSize[0])/2
        self.win_room_offy = (winSize[1] - roomSize[1])/2
        self.unexpected_chars = ['[',']','from','x','(',')','\n',',']
        state = -1
        i=0
        while i<len(contents):
            line = contents[i]
            words = line.split('\t|\t')
            if(words[0] == "WALL_Id"):
                state = 0
            elif(words[0]=="OBJ_Id"):
                state = 1
            elif(words[0]=="FocalPoint"):
                state = 2
            else:
                if(state == 0):
                    self.draw_wall(words)
                elif(state == 1):
                    recommands = contents[i+3].split('\t|\t')
                    #self.draw_boundingbox(words, recommands)
                    self.draw_object(words,recommands)
                    i+=3
                else:
                    self.draw_focal(words)
            i+=1
        cv2.imshow("result", self.win)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def getPureNum(self, word): 
        for char in self.unexpected_chars:
            word = word.replace(char,"")
        res = word.split()
        x = float(res[0])
        y = float(res[1])
        return x,y

    def TransferToGraph(self, word):
        x,y = self.getPureNum(word)
        xg = x+self.roomSize[0]/2 + self.win_room_offx
        yg = self.roomSize[1]/2-y + self.win_room_offy
        return xg,yg

    def draw_wall(self, words):
        ax,ay = self.TransferToGraph(words[2])
        bx,by = self.TransferToGraph(words[3])
        wall = cv2.line(self.win,(int(ax),int(ay)),(int(bx),int(by)),(255,0,0),3)

    def draw_boundingbox(self, vertices):
        miny, minx = np.min(vertices, axis = 0)
        maxy, maxx = np.max(vertices, axis=0)

        cv2.rectangle(self.win, (miny,minx), (maxy,maxx), (0,255,0),1)

    def draw_object(self, words, recommands):
        #print(words[4])
        # get 2 original vertices
        ax,ay = self.getPureNum(words[3])
        bx,by = self.getPureNum(words[4])

        transx, transy = self.TransferToGraph(recommands[1])
        rot = float(recommands[2])/math.pi * 180
        center = np.array([transx,transy])
        size = np.array([abs(bx-ax), abs(by-ay)])
        vertices = np.int0(cv2.boxPoints((center,size,rot)))
        cv2.drawContours(self.win, [vertices], 0, color=(255,255,0), thickness=2)
        self.draw_boundingbox(vertices)

    def draw_focal(self, words):
        x, y = self.TransferToGraph(words[1])
        cv2.circle(self.win, (int(x),int(y)), 20, (0,0,255), -1)

plot_handle = plotRoom()
