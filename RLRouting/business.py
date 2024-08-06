# library
import seaborn as sns
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
# Create a dataset
df = pd.DataFrame(np.random.random((5,5)), columns=["a","b","c","d","e"])
df.head()
sns.heatmap(df)
plt.show()
