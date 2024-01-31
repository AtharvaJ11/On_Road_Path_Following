import numpy as np

a = np.array([2,2,2])
print(a.mean())
print(a.std())
x = np.zeros(10,np.float)
while 1:
    newvalue = input()
    x[:-1] = x[1:]; x[-1] = newvalue
    print("mean:{} std:{} {}".format(x.mean(), x.std(), x))