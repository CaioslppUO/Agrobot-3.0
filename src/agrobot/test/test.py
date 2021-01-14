#!/usr/bin/env python3

"""
@package test
Ativa o modo de testes.
"""

import rosparam

rosparam.set_param("testing","True")
print("RUNNING IN TEST MODE")