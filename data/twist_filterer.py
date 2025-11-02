#!/usr/bin/env python3 
import pandas as pd

servo_csv = 'aruco-testing_2025-11-02-14-51-09-updated_twist_topic'

servo_dataframe = pd.read_csv(twist_csv)

columns_to_keep = ['.linear.z', '.angular.y']
twist_dataframe = twist_dataframe[columns_to_keep]

servo_error_dataframe = pd.read_csv(servo_error_csv)

columns_to_keep = ['time', '.header.stamp.secs', '.msg']
servo_error_dataframe = servo_error_dataframe[columns_to_keep]

'' pd.to_csv()