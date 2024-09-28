# import datetime as dt
# import RPi.GPIO as GPIO
# import datetime
# import numpy as np   

import time
from math import *
import kqExoskeletonIO as kqio
import ReadIMU_torch as ReadIMU
from DNN_torch import DNN


def SendCmdTorque(cmd1, cmd2):
    Ant.Cmd.Loop_L  = kqio.TOR_LOOP
    Ant.Cmd.Loop_R  = kqio.TOR_LOOP
    Ant.Cmd.Value_L = cmd1
    Ant.Cmd.Value_R = cmd2
    
def rad2deg(rad):
    deg = rad*180.0/pi
    return deg

def deg2rad(deg):
    rad = deg*pi/180.0
    return rad

def saturate(Cmd,sat):
    if Cmd>sat: 
        Cmd=sat
    if Cmd<-sat:
        Cmd=-sat
    return Cmd   

def impedance_control(
    Kp=0.0, 
    Kd=0.0, 
    ref_pos=0.0, ref_vel=0.0, 
    real_pos=0.0, real_vel=0.0,  
    tau_ff=0.0  
):  
    Cmd_tau = 0.0  
    Cmd_tau = (ref_pos - real_pos) * Kp + (ref_vel - real_vel) * Kd + tau_ff  

    return Cmd_tau  


ctl_mode = 1         # 1 for with IMU 0 for without IMU   
nn_mode = 1  
kcontrol = 0.025     # 1.5 para running. 2 para climbing.  
max_cmd = 2.0  

#########################################
# IMU settings  
######################################### 
if ctl_mode == 0:   
    ComPort = '/dev/ttyUSB0'  
    imu = ReadIMU.READIMU(ComPort)     

# network setup  
dnn = DNN(18,128,64,2)     
# network setup   

now   = 0
t_pr1 = 0
t_pr2 = 0
t_pr3 = 0

L_Cmd = 0
R_Cmd = 0
pk    = 0

date  = time.localtime(time.time())
dateh = date.tm_hour
datem = date.tm_min
dates = date.tm_sec

def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13):
    with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
        log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))

# write data into csv file 
with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
    log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(
        "t",
        "L_Cmd", "R_Cmd",
        "L_tau", "R_tau",
        "L_IMU_angle", "R_IMU_angle",
        "L_IMU_vel", "R_IMU_vel",
        "L_encoder", "R_encoder",
        "L_encoder_vel", "R_encoder_vel" 
    ))  


print("Initializing the comunication with the Exoskeleton")
GetSec = kqio.GetSec
Ant = kqio.AntCH("/dev/ttyAMA0")                # This is the comport that connects the Raspberry Pi 4 to the LEO
Ant.Cmd.CmdMode  = kqio.CMD_SERVO_OVERRIDE
StartSec         = GetSec()
UpdateSec        = StartSec
UpdateState      = Ant.ComState
UpdateSuccessSec = StartSec
AntConnected     = (Ant.ComState == 1)

ComTotalCnt = 1
ComErrorCnt = 0
print("Succesful initialization")

input("Getting initial angular position values for encoders and press enter")  
# The follwing sign definition follow the rigth hand side rule assuming that the rotation axis is pointing outside of the exoskeleton (for each motor)
StartHipAngle_L = -rad2deg(Ant.Data.HipAngle_L)      
StartHipAngle_R = rad2deg(Ant.Data.HipAngle_R)    

# reference position 
pos_ampl = 30 
pos_fre = 0.5   
# reference position    

start = time.time()   

while(AntConnected):  
    if UpdateState == 1:  
        UpdateSuccessSec = GetSec()  
        CurrentSec = UpdateSuccessSec - StartSec   

        now = (time.time()-start)   
        ref_pos = pos_ampl * sin(2 * pi * 1/pos_fre * now)    

        if ctl_mode == 0:  
            imu.read()  
            imu.decode()   
        
        # sensing feedback  
        L_tau = Ant.Data.HipTor_L
        R_tau = Ant.Data.HipTor_R
        
        L_encoder     = -rad2deg(Ant.Data.HipAngle_L) - StartHipAngle_L
        R_encoder     = rad2deg(Ant.Data.HipAngle_R) - StartHipAngle_R
        L_encoder_vel = -rad2deg(Ant.Data.HipSpeed_L)    
        R_encoder_vel = rad2deg(Ant.Data.HipSpeed_R)   

        if ctl_mode == 0:  
            L_IMU_angle = imu.XIMUL
            R_IMU_angle = imu.XIMUR
            L_IMU_vel   = imu.XVIMUL
            R_IMU_vel   = imu.XVIMUR  
        else: 
            L_IMU_encoder = L_encoder 
            R_IMU_angle = R_encoder  
            L_IMU_vel = L_encoder_vel  
            R_IMU_vel = R_encoder_vel    

        if (now - t_pr3 > 3):   
            t_pr3 = now  
            pk = 0  

        if (now - t_pr1 > 0.001):  
            t_pr1 = now  
            
            kp = 10
            kd = 400

            dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, kp, kd)    
            
            L_Cmd     = dnn.hip_torque_L*kcontrol  
            R_Cmd     = dnn.hip_torque_R*kcontrol  

            L_Cmd_sat = saturate(L_Cmd, max_cmd)   
            R_Cmd_sat = saturate(R_Cmd, max_cmd)    
                
            if(L_Cmd>pk or R_Cmd>pk):  
                if(R_Cmd>L_Cmd):  
                    pk=R_Cmd  
                if(L_Cmd>R_Cmd):  
                    pk=L_Cmd   
            
            # send torque cmd  
            # SendCmdTorque(L_Cmd_sat, R_Cmd_sat)    
            SendCmdTorque(0.0, 0.0)    
            
            print(f" Time: {UpdateSuccessSec:^8.3f}, L_IMU: {L_IMU_angle:^8.3f} | R_IMU: {R_IMU_angle:^8.3f} | L_CMD: {L_Cmd_sat:^8.3f} | R_CMD: {R_Cmd_sat:^8.3f} | Peak: {pk:^8.3f} ")
            
            # Save to CSV 
            write_csv(now, 
                      ref_pos, 
                      L_Cmd, R_Cmd,
                      L_tau, R_tau,
                      L_IMU_angle, R_IMU_angle,
                      L_IMU_vel, R_IMU_vel,
                      L_encoder, R_encoder,
                      L_encoder_vel, R_encoder_vel
                    )
            
    if UpdateSec - StartSec > 5:
        Ant.Disconnect()  
        print('Run Finish')
        break  
    elif UpdateState == -1:
        print('Com Error')
        break
    else:
        ComErrorCnt += 1
        if GetSec() - UpdateSuccessSec > 0.3:
            print('Error: Com Lost')
            Ant.Cmd.CmdMode = kqio.CMD_OBS_ONLY
            Ant.Disconnect()
            break

    UpdateState = Ant.Update()   
    ComTotalCnt += 1   