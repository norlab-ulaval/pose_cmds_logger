#!/usr/bin/env python
import numpy as np
import pandas as pd

array = np.array(([1, 2, 3]))

df = pd.DataFrame(array)
df.to_csv('test.csv')