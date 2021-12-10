#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

def func_aaa():
    time.sleep(1)

t1 = time.time()
func_aaa()
t2 = time.time()
print('time elapsed: ' + str(round(t2-t1, 2)) + ' seconds')
