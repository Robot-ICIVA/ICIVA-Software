import numpy as np
import yaml

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

mtxloaded = loadeddict.get('camera_matrix')
distloaded = loadeddict.get('dist_coeff')


print(mtxloaded)
print(distloaded)
