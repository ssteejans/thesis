# -*- coding: utf-8 -*-
## @file BitUtilities.py
#  Contains function to take the twos complement
#  @author: mecha31

## @fn GetTwosComplement(value, bits)
#  Takes the twos complement
def GetTwosComplement(value, bits):
    if (value & (1<<(bits-1))) != 0:
        value = value - (1<<bits)
    return value
    


