#!/usr/bin/env python

from getch import getch
class Keypress:
    def __init__(self):
        print "Are you using a:\n [0] keyboard \n [1] PS4 controller"
        self.mode = int(raw_input())


    def wait_for_input(self):
        key = getch()
        print "you pressed:", key
        if key == 'a':
            return 'circle' #turn left
        elif key == 'd':
            return 'pentagon' # turn right
        elif key == 'w':
            return 'triangle' #forwards
        elif key == 's' :
            return 'hexagon' #backwards
        elif key == 'e': 
            exit() #exit the program
        else:
            print "no valid keypress"


if __name__ == '__main__':
    keys = Keypress()
    keys.wait_for_input()
