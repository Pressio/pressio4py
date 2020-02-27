
import pytest
import numpy as np
import test_wrapper_module as m

class CallBackHelper:
    def __init__(self): pass

    def cb(self, a, cppAddress):
        a_add = a.__array_interface__['data'][0] 
        print("a: ", a_add)
        assert(a_add==cppAddress)

############################################
def test1():
    a = np.zeros(5)
    a_add = a.__array_interface__['data'][0] 
    print("a: ", hex(a_add))
    assert(m.constructWrapper1(a, a_add) == True)

def test2():
    a = np.zeros(5)
    a_add = a.__array_interface__['data'][0] 
    print("a: ", hex(a_add))
    assert(m.constructWrapper2(a, a_add) == True)

def test3():
    a = np.zeros(5)
    a_add = a.__array_interface__['data'][0] 
    print("a: ", hex(a_add))
    cb = CallBackHelper()
    m.constructWrapper3(a, cb)