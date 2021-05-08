import os
res = os.popen('rosparam list').read()
res = [e for e in res.split('\n')[:-1]] # if e.find('kimera_interfacer') != -1]
for k in res:
    val = os.popen(f'rosparam get {k}').read()
    val = [e for e in val.split('\n')[:-1]]
    print( f"rosparam set {k} {val[0]}")