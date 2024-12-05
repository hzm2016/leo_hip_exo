import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks


# Set up parameters
rootdir = "./data/Controller_100Hz_Gain_0x20_20240905_Yuming"  # folder for the current data
dt_controller = 1 / 100
dt_logging = 1 / 100
torque_gain = 0.20 / 20  # torque command is divided by 20 on RPi4
right_leg_sign = -1
all_sign_inverse = 1

# Load files
filelist = []
for root, dirs, files in os.walk(rootdir):
    for file in files:
        if file.endswith('.csv'):
            filelist.append(os.path.join(root, file))

data_total = {}
subject_info = {}  
# print("data_total :", data_total)  
# print("data_total :", data_total)  
print(filelist)  

# Loop through each file
for file_path in filelist:
    print(f"Processing file: {os.path.basename(file_path)}")
    
    file_name = os.path.basename(file_path)
    str_list = file_name.split("-")
    print("str_list :", str_list)  
    
    trial_date = str_list[0]
    trial_time = str_list[1]
    subject_name = str_list[2]
    activity_name = str_list[3]
    speed_name = str_list[4]  
    
    trial_idx = int(str_list[5][5:7])  
    print("trial_idx :", trial_idx)
    
    # Load the data
    data = pd.read_csv(file_path).values   #  header=None 

    # Compute relevant signals
    torque_left = -data[:, 4] * torque_gain * all_sign_inverse
    torque_right = -data[:, 5] * torque_gain * all_sign_inverse
    hip_angle_left = -data[:, 0] * all_sign_inverse
    hip_angle_right = -data[:, 1] * all_sign_inverse
    hip_ang_velocity_left = -data[:, 2] * all_sign_inverse
    hip_ang_velocity_right = -data[:, 3] * all_sign_inverse

    # Ensure same length for all signals
    data_length = min(len(torque_left), len(torque_right), len(hip_angle_left), len(hip_angle_right),
                      len(hip_ang_velocity_left), len(hip_ang_velocity_right))
    torque_left = torque_left[:data_length]
    torque_right = torque_right[:data_length] * right_leg_sign
    hip_angle_left = hip_angle_left[:data_length]
    hip_angle_right = hip_angle_right[:data_length]
    hip_ang_velocity_left = hip_ang_velocity_left[:data_length]
    hip_ang_velocity_right = hip_ang_velocity_right[:data_length]

    hip_power_left = torque_left * hip_ang_velocity_left / 180 * np.pi
    hip_power_right = torque_right * hip_ang_velocity_right / 180 * np.pi

    # Low-pass filter for hip angles
    def butter_lowpass_filter(data, cutoff, fs, order=2):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y

    Fs = 1 / dt_controller
    Fc = 10
    hip_angle_filtered_left = butter_lowpass_filter(hip_angle_left, Fc, Fs)
    hip_angle_filtered_right = butter_lowpass_filter(hip_angle_right, Fc, Fs)

    # Gait segmentation using peak detection
    min_peak_height = 24
    min_gait_duration = int(0.4 / dt_controller)
    max_gait_duration = int(2 / dt_controller)

    if speed_name == 'S2x00':
        min_gait_duration = int(0.22 / dt_controller)
    
    # Additional conditions based on specific trials
    if trial_date == '20240830' and speed_name == 'S2x00':
        min_gait_duration = int(0.25 / dt_controller)

    if trial_date == '20240830':
        min_peak_height = 20

    if trial_date == '20240905' and speed_name == 'S2x00':
        min_peak_height = 45

    if trial_date == '20240905' and subject_name == 'Yuming' and speed_name == 'S2x00':
        min_peak_height = 30
        min_gait_duration = int(0.30 / dt_controller)

    # Left leg gait segmentation
    peaks_left, _ = find_peaks(hip_angle_filtered_left, height=min_peak_height, distance=min_gait_duration)
    gait_head_left = []
    gait_tail_left = []
    
    for i in range(5, len(peaks_left) - 3):
        if peaks_left[i + 1] - peaks_left[i] <= max_gait_duration:
            gait_head_left.append(peaks_left[i])
            gait_tail_left.append(peaks_left[i + 1])

    gait_duration_left = np.array(gait_tail_left) - np.array(gait_head_left)
    print(f"Left leg: {len(gait_head_left)} candidate gaits found. Mean = {gait_duration_left.mean()}, std = {gait_duration_left.std()}")

    # Right leg gait segmentation
    peaks_right, _ = find_peaks(hip_angle_filtered_right, height=min_peak_height, distance=min_gait_duration)
    gait_head_right = []
    gait_tail_right = []

    for i in range(5, len(peaks_right) - 3):
        if peaks_right[i + 1] - peaks_right[i] <= max_gait_duration:
            gait_head_right.append(peaks_right[i])
            gait_tail_right.append(peaks_right[i + 1])

    gait_duration_right = np.array(gait_tail_right) - np.array(gait_head_right)
    print(f"Right leg: {len(gait_head_right)} candidate gaits found. Mean = {gait_duration_right.mean()}, std = {gait_duration_right.std()}")

    # Store data in a dictionary
    if subject_name not in data_total:
        data_total[subject_name] = {}

    if activity_name not in data_total[subject_name]:
        data_total[subject_name][activity_name] = {}

    data_total[subject_name][activity_name][speed_name] = {
        'torque_left': torque_left,
        'torque_right': torque_right,
        'hip_angle_left': hip_angle_left,
        'hip_angle_right': hip_angle_right,
        'hip_angle_filtered_left': hip_angle_filtered_left,
        'hip_angle_filtered_right': hip_angle_filtered_right,
        'hip_ang_velocity_left': hip_ang_velocity_left,
        'hip_ang_velocity_right': hip_ang_velocity_right,
        'hip_power_left': hip_power_left,
        'hip_power_right': hip_power_right,
        'gait_head_left': gait_head_left,
        'gait_tail_left': gait_tail_left,
        'gait_head_right': gait_head_right,
        'gait_tail_right': gait_tail_right,
    }


# Visualization of hip angles and torque
activity_list = ['Walk', 'Walk', 'Walk', 'Run']
speed_list = ['S0x75', 'S1x25', 'S1x75', 'S2x00']
speed_display_list = ['0.75 m/s', '1.25 m/s', '1.75 m/s', '2.00 m/s']

subjectName = 'Yuming'
dt_logging = 0.01
dt_controller = 0.02

# fig, axes = plt.subplots(4, 2, figsize=(12, 12))
# fig.suptitle('Hip Angles and Command Torques', fontsize=16)
# subject_name = 'Yuming'

# for i, (activity_name, speed_name, speed_display_name) in enumerate(zip(activity_list, speed_list, speed_display_list)):
#     data = data_total[subject_name][activity_name][speed_name]
#     time_list = np.arange(0, len(data['hip_angle_left'])) * dt_logging

#     # Plot left leg
#     axes[i, 0].plot(time_list, data['hip_angle_left'], label='Raw')
#     axes[i, 0].plot(time_list, data['hip_angle_filtered_left'], label='Filtered')
#     axes[i, 0].set_ylabel('Hip Angle (deg)')
#     axes[i, 0].legend(loc='best')
#     axes[i, 0].set_title(f"Left leg: {activity_name} @ {speed_display_name}")
    
#     # Plot right leg
#     axes[i, 1].plot(time_list, data['hip_angle_right'], label='Raw')
#     axes[i, 1].plot(time_list, data['hip_angle_filtered_right'], label='Filtered')
#     axes[i, 1].set_ylabel('Hip Angle (deg)')
#     axes[i, 1].legend(loc='best')
#     axes[i, 1].set_title(f"Right leg: {activity_name} @ {speed_display_name}")

#     if i == len(activity_list) - 1:
#         axes[i, 0].set_xlabel('Time (sec)')
#         axes[i, 1].set_xlabel('Time (sec)')

# plt.tight_layout()
# plt.show()


# Plot Figures
# fig, axs = plt.subplots(4, 2, figsize=(15, 10))
# fig.tight_layout(pad=3.0)

# for idx, activityName in enumerate(activity_list):
#     speedName = speed_list[idx]
#     speedDisplayName = speed_display_list[idx]

#     # Extract relevant data from data_total
#     hipAngleLeftList = data_total[subjectName][activityName][speedName]['hip_angle_left']  
#     hipAngleRightList = data_total[subjectName][activityName][speedName]['hip_angle_right']
#     hipAngleFilteredLeftList = data_total[subjectName][activityName][speedName]['hip_angle_filtered_left']
#     hipAngleFilteredRightList = data_total[subjectName][activityName][speedName]['hip_angle_filtered_right']
#     torqueLeftList = data_total[subjectName][activityName][speedName]['torque_left']
#     torqueRightList = data_total[subjectName][activityName][speedName]['torque_right']
#     gaitHeadLeftList = data_total[subjectName][activityName][speedName]['gait_head_left']
#     gaitTailLeftList = data_total[subjectName][activityName][speedName]['gait_tail_left']
#     gaitHeadRightList = data_total[subjectName][activityName][speedName]['gait_head_right']
#     gaitTailRightList = data_total[subjectName][activityName][speedName]['gait_tail_right']

#     timeList = np.arange(len(hipAngleLeftList)) * dt_logging

#     # Left leg
#     ax = axs[idx, 0]
#     ax.plot(timeList, hipAngleLeftList, label='Raw')
#     ax.plot(timeList, hipAngleFilteredLeftList, label='Filtered')
#     ax.plot(timeList[gaitHeadLeftList], np.array(hipAngleLeftList)[gaitHeadLeftList], 'go')
#     ax.set_ylabel('Hip Angle (deg)')
#     ax.legend(loc='best')
#     ax.set_title(f'Left leg: {activityName} @ {speedDisplayName}' if idx == 0 else f'{activityName} @ {speedDisplayName}')
    
#     ax2 = ax.twinx()
#     ax2.plot(timeList, torqueLeftList, color='red')
#     ax2.set_ylabel('Command Torque (Nm)')
#     if idx == len(activity_list) - 1:
#         ax.set_xlabel('Time (sec)')

#     # Right leg
#     ax = axs[idx, 1]
#     ax.plot(timeList, hipAngleRightList, label='Raw')
#     ax.plot(timeList, hipAngleFilteredRightList, label='Filtered')
#     ax.plot(timeList[gaitHeadRightList], np.array(hipAngleRightList)[gaitHeadRightList], 'go')
#     ax.set_ylabel('Hip Angle (deg)')
#     ax.legend(loc='best')
#     ax.set_title(f'Right leg: {activityName} @ {speedDisplayName}' if idx == 0 else f'{activityName} @ {speedDisplayName}')

#     ax2 = ax.twinx()
#     ax2.plot(timeList, torqueRightList, color='red')
#     ax2.set_ylabel('Command Torque (Nm)')
#     if idx == len(activity_list) - 1:
#         ax.set_xlabel('Time (sec)')

# plt.tight_layout()  
# plt.show()


# # Plot Figures
# fig, axs = plt.subplots(4, 2, figsize=(15, 10))
# fig.tight_layout(pad=3.0)

# for idx, activity_name in enumerate(activity_list):
#     speed_name = speed_list[idx]
#     speed_display_name = speed_display_list[idx]

#     # Extract relevant data from data_total
#     hip_angle_left = data_total[subject_name][activity_name][speed_name]['hip_angle_left']
#     hip_angle_right = data_total[subject_name][activity_name][speed_name]['hip_angle_right']
#     hip_angle_filtered_left = data_total[subject_name][activity_name][speed_name]['hip_angle_filtered_left']
#     hip_angle_filtered_right = data_total[subject_name][activity_name][speed_name]['hip_angle_filtered_right']
#     torque_left = data_total[subject_name][activity_name][speed_name]['torque_left']
#     torque_right = data_total[subject_name][activity_name][speed_name]['torque_right']
#     gait_head_left = data_total[subject_name][activity_name][speed_name]['gait_head_left']
#     gait_tail_left = data_total[subject_name][activity_name][speed_name]['gait_tail_left']
#     gait_head_right = data_total[subject_name][activity_name][speed_name]['gait_head_right']
#     gait_tail_right = data_total[subject_name][activity_name][speed_name]['gait_tail_right']
    
#     # Time list for plotting
#     time_list = np.arange(0, len(hip_angle_left)) * dt_logging

#     # Plot Left leg data
#     ax_left = axs[idx, 0]
#     ax_left.plot(time_list, hip_angle_left, label='Hip Angle', color='b')
#     ax_left.set_ylabel('Hip Angle (deg)', color='b')
#     ax_left.tick_params(axis='y', labelcolor='b')
    
#     ax_left_twin = ax_left.twinx()
#     ax_left_twin.plot(time_list, data_total[subject_name][activity_name][speed_name]['hip_power_left'], label='Hip Power', color='r')
#     ax_left_twin.set_ylabel('Power (W)', color='r')
#     ax_left_twin.tick_params(axis='y', labelcolor='r')

#     if idx == len(activity_list) - 1:
#         ax_left.set_xlabel('Time (sec)')

#     if idx == 0:
#         ax_left.set_title(f'Left leg: {activity_name} @ {speed_display_name}')
#     else:
#         ax_left.set_title(f'{activity_name} @ {speed_display_name}')

#     # Plot Right leg data
#     ax_right = axs[idx, 1]
#     ax_right.plot(time_list, hip_angle_right, label='Hip Angle', color='b')
#     ax_right.set_ylabel('Hip Angle (deg)', color='b')
#     ax_right.tick_params(axis='y', labelcolor='b')
    
#     ax_right_twin = ax_right.twinx()
#     ax_right_twin.plot(time_list, data_total[subject_name][activity_name][speed_name]['hip_power_right'], label='Hip Power', color='r')
#     ax_right_twin.set_ylabel('Power (W)', color='r')
#     ax_right_twin.tick_params(axis='y', labelcolor='r')

#     if idx == len(activity_list) - 1:
#         ax_right.set_xlabel('Time (sec)')
    
#     if idx == 0:
#         ax_right.set_title(f'Right leg: {activity_name} @ {speed_display_name}')
#     else:
#         ax_right.set_title(f'{activity_name} @ {speed_display_name}')

# plt.tight_layout()  
# plt.show()


# fig, axs = plt.subplots(4, 2, figsize=(15, 10))
# fig.patch.set_facecolor('white')
# fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
# fig.suptitle('Hip Angles and Power for Left and Right Legs', fontsize=16)

# for idx, activity_name in enumerate(activity_list):
#     speed_name = speed_list[idx]
#     speed_display_name = speed_display_list[idx]
    
#     # Extract relevant data from data_total
#     hip_angle_left = data_total[subject_name][activity_name][speed_name]['hip_angle_left']
#     hip_angle_right = data_total[subject_name][activity_name][speed_name]['hip_angle_right']
#     hip_power_left = data_total[subject_name][activity_name][speed_name]['hip_power_left']
#     hip_power_right = data_total[subject_name][activity_name][speed_name]['hip_power_right']
    
#     # Time list for plotting
#     time_list = np.arange(0, len(hip_angle_left)) * dt_logging
    
#     # Left leg subplot (odd columns)
#     ax_left = axs[idx, 0]
#     ax_left.plot(time_list, hip_angle_left, color='b')
#     ax_left.set_ylabel('Hip Angle (deg)', color='b')
#     ax_left.tick_params(axis='y', labelcolor='b')
    
#     ax_left_twin = ax_left.twinx()
#     ax_left_twin.plot(time_list, hip_power_left, color='r')
#     ax_left_twin.set_ylabel('Power (W)', color='r')
#     ax_left_twin.tick_params(axis='y', labelcolor='r')
    
#     if idx == len(activity_list) - 1:
#         ax_left.set_xlabel('Time (sec)')
    
#     if idx == 0:
#         ax_left.set_title(f'Left leg: {activity_name} @ {speed_display_name}')
#     else:
#         ax_left.set_title(f'{activity_name} @ {speed_display_name}')

#     # Right leg subplot (even columns)
#     ax_right = axs[idx, 1]
#     ax_right.plot(time_list, hip_angle_right, color='b')
#     ax_right.set_ylabel('Hip Angle (deg)', color='b')
#     ax_right.tick_params(axis='y', labelcolor='b')
    
#     ax_right_twin = ax_right.twinx()
#     ax_right_twin.plot(time_list, hip_power_right, color='r')
#     ax_right_twin.set_ylabel('Power (W)', color='r')
#     ax_right_twin.tick_params(axis='y', labelcolor='r')
    
#     if idx == len(activity_list) - 1:
#         ax_right.set_xlabel('Time (sec)')
    
#     if idx == 0:
#         ax_right.set_title(f'Right leg: {activity_name} @ {speed_display_name}')
#     else:
#         ax_right.set_title(f'{activity_name} @ {speed_display_name}')

# # Show plot
# plt.show()


# Assuming dataTotal is a dictionary with the necessary structure
# data_total = {
#     subject_name: {
#         'activity1': {
#             'speed1': {
#                 'hip_angle_left_list': np.sin(np.linspace(0, 2*np.pi, 100)),
#                 'hip_angle_right_list': np.cos(np.linspace(0, 2*np.pi, 100)),
#                 'hip_power_left_list': np.sin(np.linspace(0, 2*np.pi, 100)) * 10,
#                 'hip_power_right_list': np.cos(np.linspace(0, 2*np.pi, 100)) * 10
#             }
#         },
#         'activity2': {
#             'speed2': {
#                 'hip_angle_left_list': np.sin(np.linspace(0, 2*np.pi, 100)),
#                 'hip_angle_right_list': np.cos(np.linspace(0, 2*np.pi, 100)),
#                 'hip_power_left_list': np.sin(np.linspace(0, 2*np.pi, 100)) * 10,
#                 'hip_power_right_list': np.cos(np.linspace(0, 2*np.pi, 100)) * 10
#             }
#         }
#         # Add more activities as needed
#     }
# }

# Create the figure and set size
fig, axs = plt.subplots(4, 2, figsize=(10, 8))
fig.suptitle('Hip Angles and Power over Time', fontsize=16)

# Iterate over activities and speed
for idx, (activity_name, speed_name, speed_display_name) in enumerate(zip(activity_list, speed_list, speed_display_list)):

    # Extract relevant data
    hip_angle_left_list = data_total[subject_name][activity_name][speed_name]['hip_angle_left']
    hip_angle_right_list = data_total[subject_name][activity_name][speed_name]['hip_angle_right']
    hip_power_left_list = data_total[subject_name][activity_name][speed_name]['hip_power_left']
    hip_power_right_list = data_total[subject_name][activity_name][speed_name]['hip_power_right']
    time_list = np.arange(0, len(hip_angle_left_list) * dt_logging, dt_logging)

    # Left leg subplot
    ax_left = axs[idx, 0]
    ax_left.set_title(f'Left Leg: {activity_name} @ {speed_display_name}')
    ax_left.set_xlabel('Time (sec)' if idx == len(activity_list)-1 else '')
    ax_left.set_ylabel('Hip Angle (deg)', color='b')
    ax_left.plot(time_list, hip_angle_left_list, 'b')
    ax_left.tick_params(axis='y', labelcolor='b')

    ax_left_power = ax_left.twinx()
    ax_left_power.set_ylabel('Power (W)', color='r')
    ax_left_power.plot(time_list, hip_power_left_list, 'r')
    ax_left_power.tick_params(axis='y', labelcolor='r')

    # Right leg subplot
    ax_right = axs[idx, 1]
    ax_right.set_title(f'Right Leg: {activity_name} @ {speed_display_name}')
    ax_right.set_xlabel('Time (sec)' if idx == len(activity_list)-1 else '')
    ax_right.set_ylabel('Hip Angle (deg)', color='b')
    ax_right.plot(time_list, hip_angle_right_list, 'b')
    ax_right.tick_params(axis='y', labelcolor='b')

    ax_right_power = ax_right.twinx()
    ax_right_power.set_ylabel('Power (W)', color='r')
    ax_right_power.plot(time_list, hip_power_right_list, 'r')
    ax_right_power.tick_params(axis='y', labelcolor='r')

# Adjust layout
plt.tight_layout()
plt.subplots_adjust(top=0.9)  # Add space for title
plt.show()


# Create the figure and set size
fig, axs = plt.subplots(4, 2, figsize=(15, 12))
fig.suptitle('Averaged Metrics and Injected Work', fontsize=16)

# Iterate over activities and speeds
for idx, (activity_name, speed_name, speed_display_name) in enumerate(zip(activity_list, speed_list, speed_display_list)):
    # Extract relevant data
    averaged_metrics_left = data_total[subject_name][activity_name][speed_name]['averagedMetricsLeft']
    averaged_metrics_right = data_total[subject_name][activity_name][speed_name]['averagedMetricsRight']
    injected_work_left_list = data_total[subject_name][activity_name][speed_name]['injectedWorkLeftList']
    injected_work_right_list = data_total[subject_name][activity_name][speed_name]['injectedWorkRightList']
    
    # Left leg metrics
    gait_torque_left_avg = averaged_metrics_left['gaitTorqueNormalizedAvgList']
    gait_power_left_avg = averaged_metrics_left['gaitHipPowerNormalizedAvgList']
    gait_torque_left_std = averaged_metrics_left['gaitTorqueNormalizedStdList']
    gait_power_left_std = averaged_metrics_left['gaitHipPowerNormalizedStdList']
    
    # Right leg metrics
    gait_torque_right_avg = averaged_metrics_right['gaitTorqueNormalizedAvgList']
    gait_power_right_avg = averaged_metrics_right['gaitHipPowerNormalizedAvgList']
    gait_torque_right_std = averaged_metrics_right['gaitTorqueNormalizedStdList']
    gait_power_right_std = averaged_metrics_right['gaitHipPowerNormalizedStdList']

    # Left leg subplot
    ax_left = axs[idx, 0]
    peak_torque_left = np.max(np.abs(gait_torque_left_avg))
    rms_torque_left = np.sqrt(np.mean(np.square(gait_torque_left_avg)))
    peak_power_left = np.max(np.abs(gait_power_left_avg))
    rms_power_left = np.sqrt(np.mean(np.square(gait_power_left_avg)))

    # Plotting command torque
    ax_left.fill_between(normalized_gait_cycle_list, 
                         gait_torque_left_avg + gait_torque_left_std, 
                         gait_torque_left_avg - gait_torque_left_std, 
                         color='blue', alpha=0.1)
    ax_left.plot(normalized_gait_cycle_list, gait_torque_left_avg, 'b', label='Torque')
    ax_left.set_ylabel('Command Torque (Nm)', color='b')
    ax_left.tick_params(axis='y', labelcolor='b')

    # Plotting power
    ax_power_left = ax_left.twinx()
    ax_power_left.fill_between(normalized_gait_cycle_list, 
                                gait_power_left_avg + gait_power_left_std, 
                                gait_power_left_avg - gait_power_left_std, 
                                color=[0.8500, 0.3250, 0.0980], alpha=0.1)
    ax_power_left.plot(normalized_gait_cycle_list, gait_power_left_avg, 'r', label='Power')
    ax_power_left.set_ylabel('Power (W)', color='r')
    ax_power_left.tick_params(axis='y', labelcolor='r')

    ax_left.set_title(f'Left Leg: {activity_name} @ {speed_display_name}, '
                      f'Peak Torque = {round(peak_torque_left, 2)} Nm, '
                      f'RMS Torque = {round(rms_torque_left, 2)} Nm, '
                      f'Peak Power = {round(peak_power_left, 2)} W, '
                      f'RMS Power = {round(rms_power_left, 2)} W, '
                      f'Mean Work = {round(np.mean(injected_work_left_list), 2)} J')

    # Right leg subplot
    ax_right = axs[idx, 1]
    peak_torque_right = np.max(np.abs(gait_torque_right_avg))
    rms_torque_right = np.sqrt(np.mean(np.square(gait_torque_right_avg)))
    peak_power_right = np.max(np.abs(gait_power_right_avg))
    rms_power_right = np.sqrt(np.mean(np.square(gait_power_right_avg)))

    # Plotting command torque
    ax_right.fill_between(normalized_gait_cycle_list, 
                          gait_torque_right_avg + gait_torque_right_std, 
                          gait_torque_right_avg - gait_torque_right_std, 
                          color='blue', alpha=0.1)
    ax_right.plot(normalized_gait_cycle_list, gait_torque_right_avg, 'b')
    ax_right.set_ylabel('Command Torque (Nm)', color='b')
    ax_right.tick_params(axis='y', labelcolor='b')

    # Plotting power
    ax_power_right = ax_right.twinx()
    ax_power_right.fill_between(normalized_gait_cycle_list, 
                                 gait_power_right_avg + gait_power_right_std, 
                                 gait_power_right_avg - gait_power_right_std, 
                                 color=[0.8500, 0.3250, 0.0980], alpha=0.1)
    ax_power_right.plot(normalized_gait_cycle_list, gait_power_right_avg, 'r')
    ax_power_right.set_ylabel('Power (W)', color='r')
    ax_power_right.tick_params(axis='y', labelcolor='r')

    ax_right.set_title(f'Right Leg: {activity_name} @ {speed_display_name}, '
                       f'Peak Torque = {round(peak_torque_right, 2)} Nm, '
                       f'RMS Torque = {round(rms_torque_right, 2)} Nm, '
                       f'Peak Power = {round(peak_power_right, 2)} W, '
                       f'RMS Power = {round(rms_power_right, 2)} W, '
                       f'Mean Work = {round(np.mean(injected_work_right_list), 2)} J')

# Adjust layout
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Add space for title
plt.show()