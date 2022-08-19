#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 한국어 주석 적기 위함

# import matplotlib
import matplotlib.pyplot as plt
# matplotlib.use('Agg')

a = [1,4,7,8,13,17]
plt.plot(a)
# plt.show()
plt.savefig('matplotlib_test.png')

