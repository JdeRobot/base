#!/usr/bin/env python
# -*- coding: utf-8 -*-

class MyList:
    def __init__(self):
        self.mylist=[]

    def add(self,thing,list):
        self.mylist.insert(0,thing)

    def returnItem(self,pos,list):
        a=tuple(self.mylist[0])
        return a[pos]

    def removeItem(self,pos):
        del self.mylist[pos]

    def len(self):
        return len(self.mylist)

    def show(self):
        print(self.mylist)
