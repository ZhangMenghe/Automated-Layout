import numpy as np
import cv2
class plotRoom():
    def __init__(self, winSize=(1000,1000),roomSize=(800,600), resfile_name = "recommendation.txt"):
        try:
            fp = open(resfile_name, 'r')
            contents = fp.readlines()
        finally:
            fp.close()
        self.win = np.zeros(winSize+(3,), np.uint8)
        self.roomSize = roomSize
        self.win_room_offx = (winSize[0] - roomSize[0])/2
        self.win_room_offy = (winSize[1] - roomSize[1])/2
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
                    self.draw_object(words, contents[i+3].split('\t|\t'))
                    i+=3
                else:
                    self.draw_focal(words)
            i+=1
        cv2.imshow("result", self.win)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    def getPureNum(self, word):
        unexpected_chars = ['[',']','from','x','(',')','\n']
        for char in unexpected_chars:
            word = word.replace(char,"")
        # print(word)
        res = word.split(',')
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
    def draw_object(self, words, words2):
        ax,ay = self.TransferToGraph(words[4])
        bx,by = self.TransferToGraph(words[6])
        offsetx, offsety = self.getPureNum(words2[1])
        cv2.rectangle(self.win, (int(ax+offsetx),int( ay+offsety)), (int(bx+offsetx), int(by+offsety)), (0,255,0), 2)
    def draw_focal(self, words, win):
        print("nothing")

plot_handle = plotRoom()
