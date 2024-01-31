import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('10.bag')

# replace the topic name as per your need
LASER_MSG = b.message_by_topic('/steering_command')
# LASER_MSG
df_laser = pd.read_csv(LASER_MSG)
# df_laser # prints laser data in the form of pandas dataframe
print(df_laser)