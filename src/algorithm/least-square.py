# coding: utf-8

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt  # 绘图库
from scipy.optimize import leastsq  # 最小二乘

# 样本数据(Xi,Yi)，需要转换成数组(列表)形式
Xi = np.array([6.19, 2.51, 7.29, 7.01, 5.7, 2.66, 3.98, 2.5, 9.1, 4.2])
Yi = np.array([5.25, 2.83, 6.41, 6.71, 5.1, 4.23, 5.05, 1.98, 10.5, 6.3])


# 需要拟合的函数
def func(p, x):
    a, b = p
    return a*x+b


# 偏差函数
def error(p, x, y):
    return func(p, x)-y


# a,b的初始值，可以任意设定,经过几次试验，发现p0的值会影响cost的值：Para[1]
p0 = [1, 20]

# 把error函数中除了p0以外的参数打包到args中(使用要求)
Para = leastsq(error, p0, args=(Xi, Yi))

# 读取结果
a, b = Para[0]
print("a=", a, "b=", b)
print("cost："+str(Para[1]))
print("求解的拟合直线为:")
print("y="+str(round(a, 2))+"x+"+str(round(b, 2)))

'''
   绘图，看拟合效果.
   matplotlib默认不支持中文，label设置中文的话需要另行设置
   如果报错，改成英文就可以
'''

# 画样本点
plt.figure(figsize=(8, 6))  # 指定图像比例： 8：6
plt.scatter(Xi, Yi, color="green", label="Sample data", linewidth=2)

# 画拟合直线
x = np.linspace(0, 12, 100)  # 在0-12直接画100个连续点
y = a*x+b  # 函数式
plt.plot(x, y, color="red", label="Fitting line", linewidth=2)
plt.legend(loc='lower right')  # 绘制图例
plt.show()
