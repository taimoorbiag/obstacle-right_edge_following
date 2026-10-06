#ROBOT 16

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def right_edge(ROBOT_REG):

   def BASIC_DE_FUZ(f,output_S,output_D):
       sum_frontside = 0 
       sum_frontside2 = 0
       summation_n = 0
       summation_n2 = 0
       output = 0
       output2 = 0

       for m,t in f.items():
           if t > 0:

               if (m[0]=="near") & (m[1]=="near"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["left"][1]*t

               if (m[0]=="near") & (m[1]=="medium"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["left"][1]*t

               if (m[0]=="near") & (m[1]=="far"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["left"][1]*t

               if (m[0]=="medium") & (m[1]=="near"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["left"][1]*t

               if (m[0]=="medium") & (m[1]=="medium"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["zero"][1]*t

               if (m[0]=="medium") & (m[1]=="far"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["left"][1]*t

               if (m[0]=="far") & (m[1]=="near"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["right"][1]*t

               if (m[0]=="far") & (m[1]=="medium"):
                   summation_n += output_S["slow"][1]*t
                   summation_n2 += output_D["right"][1]*t

               if (m[0]=="far") & (m[1]=="far"):
                   summation_n += output_S["medium"][1]*t
                   summation_n2 += output_D["right"][1]*t

               sum_frontside += t    
               sum_frontside2 += t   
           if(sum_frontside > 0):     
               output  = summation_n/sum_frontside

           if(sum_frontside2 > 0):     
               output2 = summation_n2/sum_frontside2

       a = {"linear": output, "angular": output2,}     

       return a
 
   sensor_FR = {"near": [0,0.6,0.65],"medium": [0.6,0.65,0.7],"far": [0.65,0.7,1]}
   sensor_B =  {"near": [0,0.6,0.65],"medium": [0.6,0.65,0.7],"far": [0.65,0.7,1]}
   output_S = {"slow": [0.01,0.05,0.1], "medium": [0.1,0.15,0.2], "fast": [0.2,0.25,0.3]}
   output_D = {"right": [0.1,0.5,0.3],"zero": [-1,0,1],"left": [-0.3,-0.2,-0.1]}
   
   def BASIC_FUZ(ROBOT_REG,sensor_FR,sensor_B):

       f = {("near","near"):0, ("near","medium"):0, ("near","far"):0, ("medium","near"):
            0, ("medium","medium"):0, ("medium","far"):0, ("far","near"):0, ("far","medium"):0, ("far","far"): 0,
            }
   
       sensor_rf_near = 0
       sensor_rf_medium = 0
       sensor_rf_far = 0
       sensor_rb_near = 0
       sensor_rb_medium = 0
       sensor_rb_far = 0
 
       for m,t in ROBOT_REG.items():
           if m == "fright":
               for s,b in sensor_FR.items():
                   if (t >= b[0]) & (t <= b[2]):
                       
                       if s == "near":
                           if t <= b[1]:
                               sensor_rf_near = (t-b[0])/(b[1]-b[0])    
                           else:
                               sensor_rf_near = (b[2]-t)/(b[2]-b[0])       

                       elif s == "medium":
                           if t <= b[1]:
                               sensor_rf_medium = (t-b[0])/(b[1]-b[0])
                           else:
                               sensor_rf_medium = (b[2]-t)/(b[2]-b[0])

                       elif s == "far":
                           if t == 1:
                               sensor_rf_far = 1
                           elif t <= b[1]:
                               sensor_rf_far = (t-b[0])/(b[1]-b[0]) 
                           else:  
                               sensor_rf_far = (b[2]-t)/(b[2]-b[0])

           if m == "bright":
               for s,b in sensor_B.items():
                   if (t >= b[0]) & (t <= b[2]):
 
                       if s == "near":
                           if t <= b[1]:
                               sensor_rb_near = (t-b[0])/(b[1]-b[0])    
                           else:
                               sensor_rb_near = (b[2]-t)/(b[2]-b[0])    

                       elif s == "medium":
                           if t <= b[1]:
                               sensor_rb_medium = (t-b[0])/(b[1]-b[0])
                           else:
                               sensor_rb_medium = (b[2]-t)/(b[2]-b[0])

                       elif s == "far":
                           if t == 1:
                               sensor_rb_far = 1
                           elif t <= b[1]:
                               sensor_rb_far = (t-b[0])/(b[1]-b[0]) 
                           else:  
                               sensor_rb_far = (b[2]-t)/(b[2]-b[0])

       if (sensor_rf_near>0) & (sensor_rb_near>0):
           m = min(sensor_rf_near,sensor_rb_near)
           f[("near","near")] = m

       if (sensor_rf_near>0) & (sensor_rb_medium>0):
           m = min(sensor_rf_near,sensor_rb_medium)
           f[("near","medium")] = m

       if (sensor_rf_near>0) & (sensor_rb_far>0):
           m = min(sensor_rf_near,sensor_rb_far)
           f[("near","far")] = m

       if (sensor_rf_medium>0) & (sensor_rb_near>0):
           m = min(sensor_rf_medium,sensor_rb_near)
           f[("medium","near")] = m

       if (sensor_rf_medium>0) & (sensor_rb_medium>0):
           m = min(sensor_rf_medium,sensor_rb_medium)
           f[("medium","medium")] = m

       if (sensor_rf_medium>0) & (sensor_rb_far>0):
           m = min(sensor_rf_medium,sensor_rb_far)
           f[("medium","far")] = m

       if (sensor_rf_far>0) & (sensor_rb_near>0):
           m = min(sensor_rf_far,sensor_rb_near)
           f[("far","near")] = m

       if (sensor_rf_far>0) & (sensor_rb_medium>0):
           m = min(sensor_rf_far,sensor_rb_medium)
           f[("far","medium")] = m

       if (sensor_rf_far>0) & (sensor_rb_far>0):
           m = min(sensor_rf_far,sensor_rb_far)
           f[("far","far")] = m

       return f    

   f = BASIC_FUZ(ROBOT_REG,sensor_FR,sensor_B)
 
   linear = (BASIC_DE_FUZ(f,output_S,output_D)["linear"])
   angular = -BASIC_DE_FUZ(f,output_S,output_D)["angular"]

   return {"linear": linear, "angular": angular}    

def obstacle_avoid(ROBOT_REG):

   def BASIC_FUZ(ROBOT_REG,sensor_FR,sensor_FL,sensor_F): 
       sensor_rf_near = sensor_rf_medium = sensor_rf_far = 0
       f_near = f_medium = f_far = 0
       lf_near = lf_medium = lf_far = 0

       f = {
            ("near", "near", "near"): 0, ("near", "near", "medium"): 0, ("near", "near", "far"): 0,
            ("near", "medium", "near"): 0, ("near", "medium", "medium"): 0, ("near", "medium", "far"): 0,
            ("near", "far", "near"): 0, ("near", "far", "medium"): 0, ("near", "far", "far"): 0,
            ("medium", "near", "near"): 0, ("medium", "near", "medium"): 0, ("medium", "near", "far"): 0,
            ("medium", "medium", "near"): 0, ("medium", "medium", "medium"): 0, ("medium", "medium", "far"): 0,
            ("medium", "far", "near"): 0, ("medium", "far", "medium"): 0, ("medium", "far", "far"): 0,
            ("far", "near", "near"): 0, ("far", "near", "medium"): 0, ("far", "near", "far"): 0,
            ("far", "medium", "near"): 0, ("far", "medium", "medium"): 0, ("far", "medium", "far"): 0,
            ("far", "far", "near"): 0, ("far", "far", "medium"): 0, ("far", "far", "far"): 0
           }
  
       for m,t in ROBOT_REG.items():
           if m == "fright":
               for s,b in sensor_FR.items():
                   if (t >= b[0]) & (t <= b[2]):

                       if s == "near":
                           if t <= b[1]:
                               sensor_rf_near = (t-b[0])/(b[1]-b[0]) #(X-A/B-A)    
                           else:
                               sensor_rf_near = (b[2]-t)/(b[2]-b[0]) # c-x/c-a    
                       
                       elif s == "medium":
                           if t <= b[1]:
                               sensor_rf_medium = (t-b[0])/(b[1]-b[0])
                           else:
                               sensor_rf_medium = (b[2]-t)/(b[2]-b[0])

                       elif s == "far":
                           if t == 1:
                               sensor_rf_far = 1

                           elif t <= b[1]:
                               sensor_rf_far = (t-b[0])/(b[1]-b[0])

                           else:
                               sensor_rf_far = (b[2]-t)/(b[2]-b[0])
 
           if m == "front":              
               for s,b in sensor_F.items():
                   if (t >= b[0]) & (t <= b[2]):
                       if s == "near":
                           if t <= b[1]:
                               f_near = (t-b[0])/(b[1]-b[0])   
                           else:
                               f_near = (b[2]-t)/(b[2]-b[0])   

                       elif s == "medium":
                           if t <= b[1]:
                               f_medium = (t-b[0])/(b[1]-b[0])    
                           else:
                               f_medium = (b[2]-t)/(b[2]-b[0])    

                       elif s == "far":                          
                           if t == 1:
                               f_far = 1                        
                           elif t <= b[1]:
                               f_far = (t-b[0])/(b[1]-b[0])   
                           else:
                               f_far = (b[2]-t)/(b[2]-b[0])    
                                                    
           if m == "fleft":
               for s,b in sensor_FL.items():
                   if (t >= b[0]) & (t <= b[2]):
                       if s == "near":
                           if t <= b[1]:
                               lf_near = (t-b[0])/(b[1]-b[0])    
                               lf_near = (b[2]-t)/(b[2]-b[0])      

                       elif s == "medium":
                           if t <= b[1]:
                               lf_medium = (t-b[0])/(b[1]-b[0])    
                           else:
                               lf_medium = (b[2]-t)/(b[2]-b[0])     

                       elif s == "far":
                           if t <= b[1]:
                               lf_far = (t-b[0])/(b[1]-b[0]) 
                           elif t == 1:
                               lf_far = 1
                           else:  
                               lf_far = (b[2]-t)/(b[2]-b[0])
                                             
       if (sensor_rf_near > 0) & (f_near > 0) & (lf_near > 0):
           m = min(sensor_rf_near, f_near, lf_near)     
           f[("near", "near", "near")] = m      

       if (sensor_rf_near > 0) & (f_near > 0) & (lf_medium > 0):
           m = min(sensor_rf_near, f_near, lf_medium)
           f[("near", "near", "medium")] = m

       if (sensor_rf_near > 0) & (f_near > 0) & (lf_far > 0):
           m = min(sensor_rf_near, f_near, lf_far)
           f[("near", "near", "far")] = m

       if (sensor_rf_near > 0) & (f_medium > 0) & (lf_near > 0):
           m = min(sensor_rf_near, f_medium, lf_near)
           f[("near", "medium", "near")] = m

       if (sensor_rf_near > 0) & (f_medium > 0) & (lf_medium > 0):
           m = min(sensor_rf_near, f_medium, lf_medium)
           f[("near", "medium", "medium")] = m

       if (sensor_rf_near > 0) & (f_medium > 0) & (lf_far > 0):
           m = min(sensor_rf_near, f_medium, lf_far)
           f[("near", "medium", "far")] = m

       if (sensor_rf_near > 0) & (f_far > 0) & (lf_near > 0):
           m = min(sensor_rf_near, f_far, lf_near)
           f[("near", "far", "near")] = m

       if (sensor_rf_near > 0) & (f_far > 0) & (lf_medium > 0):
           m = min(sensor_rf_near, f_far, lf_medium)
           f[("near", "far", "medium")] = m

       if (sensor_rf_near > 0) & (f_far > 0) & (lf_far > 0):
           m = min(sensor_rf_near, f_far, lf_far)
           f[("near", "far", "far")] = m

       if (sensor_rf_medium > 0) & (f_near > 0) & (lf_near > 0):
           m = min(sensor_rf_medium, f_near, lf_near)
           f[("medium", "near", "near")] = m

       if (sensor_rf_medium > 0) & (f_near > 0) & (lf_medium > 0):
           m = min(sensor_rf_medium, f_near, lf_medium)
           f[("medium", "near", "medium")] = m

       if (sensor_rf_medium > 0) & (f_near > 0) & (lf_far > 0):
           m = min(sensor_rf_medium, f_near, lf_far)
           f[("medium", "near", "far")] = m

       if (sensor_rf_medium > 0) & (f_medium > 0) & (lf_near > 0):
           m = min(lf_far, f_medium, sensor_rf_near)
           f[("medium", "medium", "near")] = m

       if (sensor_rf_medium > 0) & (f_medium > 0) & (lf_medium > 0):
           m = min(lf_far, f_medium, lf_medium)
           f[("medium", "medium", "medium")] = m

       if (sensor_rf_medium > 0) & (f_medium > 0) & (lf_far > 0):
           m = min(lf_far, f_medium, lf_far)
           f[("medium", "medium", "far")] = m

       if (sensor_rf_medium > 0) & (f_far > 0) & (lf_near > 0):
           m = min(sensor_rf_medium, f_far, lf_near)
           f[("medium", "far", "near")] = m

       if (sensor_rf_medium > 0) & (f_far > 0) & (lf_medium> 0):
           m = min(sensor_rf_medium, f_far, lf_medium)
           f[("medium", "far", "medium")] = m

       if (sensor_rf_medium > 0) & (f_far > 0) & (lf_far > 0):
           m = min(sensor_rf_medium, f_far, lf_far)
           f[("medium", "far", "far")] = m

       if (sensor_rf_far > 0) & (f_near > 0) & (lf_near > 0):
           m = min(sensor_rf_far, f_near, lf_near)
           f[("far", "near", "near")] = m

       if (sensor_rf_far > 0) & (f_near > 0) & (lf_medium > 0):
           m = min(lf_far, f_near, lf_medium)
           f[("far", "near", "medium")] = m

       if (sensor_rf_far > 0) & (f_near > 0) & (lf_far > 0):
           m = min(lf_far, f_near, lf_far)
           f[("far", "near", "far")] = m

       if (sensor_rf_far > 0) & (f_medium > 0) & (lf_near > 0):
           m = min(lf_far, f_medium, lf_near)
           f[("far", "medium", "near")] = m

       if (sensor_rf_far > 0) & (f_medium > 0) & (lf_medium > 0):
           m = min(lf_far, f_medium, lf_medium)
           f[("far", "medium", "medium")] = m

       if (sensor_rf_far > 0) & (f_medium > 0) & (lf_far > 0):
           m = min(lf_far, f_medium, lf_far)
           f[("far", "medium", "far")] = m

       if (sensor_rf_far > 0) & (f_far > 0) & (lf_near > 0):
           m = min(sensor_rf_far, f_far, lf_near)
           f[("far", "far", "near")] = m

       if (sensor_rf_far > 0) & (f_far > 0) & (lf_medium> 0):
           m = min(sensor_rf_far, f_far, lf_medium)
           f[("far", "far", "medium")] = m

       if (sensor_rf_far > 0) & (f_far > 0) & (lf_far > 0):
           m = min(sensor_rf_far, f_far, lf_far)
           f[("far", "far", "far")] = m

       return f
   
   def BASIC_DE_FUZ(f,OUT_SPED,output_D):
       sum_frontside = 0
       sum_frontside2 = 0
       summation_n = 0
       summation_n2 = 0
       output = 0
       output2 = 0
       
       for m, t in f.items():
           if t > 0:   
                      
               if (m[0] == "near") & (m[1] == "near") & (m[2] == "near"):
                   summation_n += OUT_SPED["slow"][1] * t
                   summation_n2 += output_D["left"][1] * t

               elif (m[0] == "near") & (m[1] == "near") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t

               elif (m[0] == "near") & (m[1] == "near") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t

               elif (m[0] == "near") & (m[1] == "medium") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "near") & (m[1] == "medium") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t

               elif (m[0] == "near") & (m[1] == "medium") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t
       
               elif (m[0] == "near") & (m[1] == "far") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "near") & (m[1] == "far") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t

               elif (m[0] == "near") & (m[1] == "far") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t         

               elif (m[0] == "medium") & (m[1] == "near") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "medium") & (m[1] == "near") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "medium") & (m[1] == "near") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t

               elif (m[0] == "medium") & (m[1] == "medium") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "medium") & (m[1] == "medium") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "medium") & (m[1] == "medium") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t

               elif (m[0] == "medium") & (m[1] == "far") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "medium") & (m[1] == "far") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "medium") & (m[1] == "far") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["right"][1] * t      

               elif (m[0] == "far") & (m[1] == "near") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "near") & (m[2] == "medium"):
                       summation_n += OUT_SPED["fast"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "near") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "medium") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "medium") & (m[2] == "medium"):
                       summation_n += OUT_SPED["fast"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "medium") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "far") & (m[2] == "near"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "far") & (m[2] == "medium"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["left"][1] * t

               elif (m[0] == "far") & (m[1] == "far") & (m[2] == "far"):
                       summation_n += OUT_SPED["slow"][1] * t
                       summation_n2 += output_D["zero"][1] * t

               sum_frontside += t

               sum_frontside2 += t

       if(sum_frontside > 0):
           output  = summation_n/sum_frontside    
               
       if(sum_frontside2 > 0):
           output2 = summation_n2/sum_frontside2    

       a = {"linear": output, "angular": output2,}

       return a

   sensor_FR = {"near": [0,0.25,0.6],"medium": [0.25,0.6,0.8],"far": [0.6,0.7,1]}
   sensor_F = {"near": [0,0.25,0.75],"medium": [0.25,0.75,0.85],"far": [0.75,0.85,1]}
   sensor_FL = {"near": [0,0.25,0.6],"medium": [0.25,0.6,0.8],"far": [0.6,0.7,1]}
   output_S = {"slow": [0.01,0.05,0.1], "medium": [0.1,0.15,0.2], "fast": [0.2,0.25,0.3]}
   output_D = {"right": [0.1,0.5,0.3],"zero": [-1,0,1],"left": [-0.3,-0.2,-0.1]}

   ROBOT_REG["front"] = min(ROBOT_REG["front1"] , ROBOT_REG["front2"]) 

   f = BASIC_FUZ(ROBOT_REG,sensor_FR,sensor_FL,sensor_F) 
   a = BASIC_DE_FUZ(f,output_S,output_D)

   return a

mynode_ = None
pub_ = None

regions_ = { 
     'right': 0, "front": 0, 'fright': 0, 'front1': 0, 'front2': 0, 'fleft': 0, 'left': 0, 
     }

twstmsg_ = None

def timer_callback():
   global pub_, twstmsg_

   if ( twstmsg_ != None ):
       pub_.publish(twstmsg_)

def clbk_laser(msg):
   global regions_, twstmsg_

   regions_ = { 
       'front1':  find_nearest (msg.ranges[0:10]),
       'front2':  find_nearest (msg.ranges[350:360]),
       'fright': find_nearest(msg.ranges[320:340]),
       'bright': find_nearest(msg.ranges[200:220]),
       'fleft':  find_nearest (msg.ranges[20:50]),
       'right':  find_nearest(msg.ranges[265:275]),
       'left':   find_nearest (msg.ranges[85:95])
   }

   twstmsg_= movement()

def find_nearest(list):
   f_list = filter(lambda item: item > 0.0, list) 
   return min(min(f_list, default=1), 1)   

def movement():
   global regions_,mynode_,m,e,sensor_FR,br_sensor,output_S,output_D
   ROBOT_REG = regions_   
   ROBOT_REG["front"] = min(ROBOT_REG["front1"] , ROBOT_REG["front2"])  

   msg = Twist() 
  
   d1 = min( ROBOT_REG["fright"], ROBOT_REG["bright"])   
   d2 = min( ROBOT_REG["fright"], min(ROBOT_REG["front1"],ROBOT_REG["front2"]), ROBOT_REG["fleft"])    
   d1_range = [0.6, 0.65, 0.7] 
   d2_range = [0,0.25,0.6]
   md1n = 0    
   md2n = 0    

   if (d1 > d1_range[0]) & (d1 <= d1_range[2]):
       md1n = 1 

   elif (d2 >= d2_range[0]) & (d2 <= d2_range[2]):
       md2n = 1  
   
   right_e = right_edge(ROBOT_REG)
   obstacle_a = obstacle_avoid(ROBOT_REG)
    
   print(md1n)
   print(md2n)

   if md1n !=0 or md2n != 0: 
       s =( (md1n * right_e["linear"] )+ (md2n * obstacle_a["linear"]))/ (md1n+md2n)     
       b =( (md1n * right_e["angular"] )+ (md2n * obstacle_a["angular"]))/ (md1n+md2n)

   else:
       s=right_e["linear"]    
       b = right_e["angular"]    
   
   msg.linear.x = s
   msg.angular.z = b

   return msg    

def stop():
   global pub_

   msg = Twist()     
   msg.angular.z = 0.0    
   msg.linear.x = 0.0    

   pub_.publish(msg)   

def main():
   global pub_, mynode_

   rclpy.init()    
   mynode_ = rclpy.create_node('reading_laser')    

   qos = QoSProfile(
       depth=10,   
       reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,    
         )

   pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)
   sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

   timer_period = 0.2  
   timer = mynode_.create_timer(timer_period, timer_callback)

   try:
       rclpy.spin(mynode_)   

   except KeyboardInterrupt:
       stop()  

   except:
       stop() 

   finally:
       mynode_.destroy_timer(timer)
       mynode_.destroy_node()

       rclpy.shutdown()

if __name__ == '__main__':
   main()
