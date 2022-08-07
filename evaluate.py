import os
import subprocess
import numpy as np
import pandas as pd

def str_to_num(ttc):
    ttc=ttc.strip()
    if(ttc == 'nan'):
        ttc = np.nan
    elif(ttc == '-inf'):
        ttc = -np.inf 
    elif(ttc == 'inf'):
        ttc = np.inf 
    else:
        ttc = float(ttc)
    return ttc

def output_to_dataframe(output):
    arr_measurement = []
    str_detector = output.decode('utf-8')
    result = str_detector.split("\n")
    str_type =result[0].split(' ')
    detector_type=str_type[1]
    descriptor_type = str_type[-1]
    for idx, res in enumerate(result[1:-1],start=1):
        ttc_lidar, ttc_cam = res.strip().split(';')
        ttc_lidar = str_to_num(ttc_lidar)
        ttc_cam = str_to_num(ttc_cam)
        arr_measurement.append([detector_type, descriptor_type, idx, ttc_lidar, ttc_cam])
    return pd.DataFrame(arr_measurement, columns=["Detector", "Descriptor", "Frame", "TTC_Lidar", "TTC_Camera"])

def eval_image(detector, descriptor):
    output = subprocess.run(["./3D_object_tracking", detector, descriptor], capture_output=True)
    return output_to_dataframe(output.stdout)
    
if __name__ == "__main__":
    os.chdir('build')
    df_merged = pd.DataFrame(columns=["Detector", "Descriptor", "Frame", "TTC_Lidar", "TTC_Camera"])

    detectorTypes = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
    descriptorTypes = ["BRIEF", "BRISK", "ORB", "FREAK", "SIFT"]

    for detectorType in detectorTypes: 
        for descriptorType in descriptorTypes:
            # ORB descriptor doesn't work with SIFT detector
            print("detectotType :{} descriptor: {}".format(detectorType, descriptorType))
            if (detectorType == "SIFT") and (descriptorTypes == "ORB"):
                continue
            df_meas = eval_image(detectorType, descriptorType)
            df_merged = pd.concat([df_merged, df_meas])
            df_merged.to_csv("result.csv")
    # AKAZE Descriptor only works with AKAZE Detector
    df_meas = eval_image("AKAZE", "AKAZE")
    df_merged = pd.concat([df_merged, df_meas])
    df_merged.to_csv("result.csv")