# -*- coding: utf-8 -*-
"""
V01 Baseline for Project Hayley V4
V02 Attempt to send encoder data to Pi ==> success with "Project_Hayley_ESP32_V3.ino" 
V03 Attempt to convert encoder data from long to float and tabulate in dash table with new table id "encoder_position_table"
V04 Add homing capability ==> success with "Project_Hayley_ESP32_V4.ino" 
V05 Working baseline as of 7-May-25
V06 Update with actual physical linkage for Hayley V4. Attempt to add in box entry for fine stepper movement 
    Compaitible with "Project_Hayley_ESP32_V5.ino"
V07 Update to box entry for angles entry for FK
    Update kinematics movement calculations to use with "return_encoder = get_encoder_data()"
    This is make movements base on actual encoder readings before movement - this version do not make use of feedback loop yet
    Compaitible with "Project_Hayley_ESP32_V5.ino"
    Compaitible with "Project_Hayley_ESP32_V6.ino"
    Compaitible with "Project_Hayley_ESP32_V7.ino"

V08 Correct homing logic for encoders based. Remove calibration button (calibration shall be done after home with BNO055)
    Removed Project Hayley V3 data
    optimize DH_matrix e.g. let all theta/180*np.pi be theta_rad
    
V09 major change attempt to switch steps calculation from python/raspberry side to ESP32
    get_encoder_data() updated to process incoming data as float instead of long datatype
    function call change from "writeNumbers(steps)" to "writeAngles(angles_list)"
    Compaitible with "Project_Hayley_ESP32_V9.ino" 

V10 Update interface to have both angular and steps display on same main page
    removed callback "update_stpper_position_div(btn4)" and attempt to add auto update of encoder data instead
    Compaitible with "Project_Hayley_ESP32_V9.ino" 
    
V11 Remove function "Inverse_Kinematics_CCD" as the focus is on raspberry Pi to function as user interface
    The IK solver should remain on a PC 

V12 Cleanup   
    Update step page to reflect absolute stepper position (steps) after each travel
    Removed method of 'Cubic Polynomial' and 'Quintic Polynomial'as this should be part of the IK solver seperately
    
    
Features to be added
multi angle calibration capability

"""

import dash
from dash import dcc
from dash import html
import plotly.express as px
import plotly.graph_objs as go
import pandas as pd
import numpy as np
import smbus2 as smbus
import struct
import time
import math
from skspatial.objects import Plane, Point, Vector
import base64
import io
from dash import dash_table
import subprocess
from dash_extensions.enrich import Output, DashProxy, Input, MultiplexerTransform

# device address
arduino_IOT = 0x04
ESP32_IOT = 0x05

# initialize serial commumication
bus = smbus.SMBus(1)
time.sleep(1)

angle_slider_resolution = 0.1 # degrees


gear_reduction = [-30, -30, -30, -30, -30, -1] #-ve sign to flip direction of physical motor
steps_per_rotation = [200, 200, 200, 200, 200, 200]
micro_steps = [8, 8, 8, 8, 8, 8]

hinge_length = 14 # for illustrating the axis hinge in 3d view
max_iter = 500 # iterations allow for IK CCD
err_min = 2.5 # acceptable error in mm in inverse kinematics calculation

figure_axis_limit = 500

# config file to limit the max min angles of the axis
config_df = pd.read_csv('config - project Hayley V5.csv')
lower_limit = list(config_df["Absolute low angle limit"])
upper_limit = list(config_df["Absolute high angle limit"])


angle_lower_limit = list(np.round((np.array(lower_limit) + 180) % 360 - 180,0))   
angle_upper_limit = list(np.round((np.array(upper_limit) + 180) % 360 - 180,0))   



start_angles_list = list(config_df["Start angle"])
angles_list = start_angles_list.copy()

list_of_thetas = [0] * 32 # To be updated

index_of_angles = [2, 8, 11, 18, 25, 30] #hinge for project Hayley v4
index_of_hinges = index_of_angles.copy()

angle_max = list_of_thetas.copy()
angle_min = list_of_thetas.copy()
list_of_blockers = list_of_thetas.copy()


# Each row represent orientation of linkage in spatial coordinates
# (theta, alpha, delta, rho)
# data for project hayley V4 - V2 links
local_linkage_data = [
    [0,0,0,0,0], #0
    [1,0,0,54.5,0], #1 
    [1,0,0,0,0], #theta_1 - Axis1, anti-clockwise with positive theta #2
    [2,0,0,84,0], #3
    [2,0,-90,0,0], #4
    [2,0,0,61,0], #5
    [2,0,180,0,0], #6
    [2,90,0,0,0], #7 - Z right, X Up
    [2,0,0,0,0], #theta_2 - Axis2, down with positive theta #8
    [3,0,-180,0,0], #9 - Z left, X Up
    [3,0,0,0,70],   #10    
    [3,0,0,0,0], #theta_3 - Axis3, down with positive theta #11
    [4,0,0,-61,0],     #12
    [4,0,0,0,52],     #13
    [4,90,0,0,0],     #14 - Z left, X front
    [4,0,0,0,51.5],   #15 
    [4,90,0,0,0],     #16 - Z left, X down
    [4,0,90,0,0],     #17 - Z front, X down
    [4,0,0,0,0], #theta_4 - Axis4, anti-clockwise from top view with positive theta  #18
    [5,90,0,0,0],   #19    - Z front, X left
    [5,0,0,0,47],   #20
    [5,0,0,50,0],   #21
    [5,-90,0,0,0],     #22 - Z front, X down
    [5,0,-90,0,0],     #23 - Z left, X down
    [5,-90,0,0,0],     #24 - Z left, X front    
    [5,0,0,0,0], #theta_5 - Axis5, down with positive theta #25
    [6,0,0,-47,0],    #26
    [6,0,0,0,100],    #27
    [6,-90,0,0,0],     #28 - Z left, X up
    [6,0,-90,0,0],     #29 - Z front, X up    
    [6,0,0,0,0], #theta_6 - Axis6, anti-clockwise with positive theta #30
    [7,0,0,10.5,0], # dummy 20 mm extension #31
    [7,0,90,0,0], #32 - Z left, X up
    [7,90,0,0,0], #33 - Z left, X front
    [7,0,90,0,0], #34 - Z up, X front    
    ]


#This headers are for the encoders'df
column_headers = [
                  'Axis 1', 'Axis 2', 'Axis 3', 'Axis 4', 'Axis 5', 'Axis 6'#, 'dummy'
                  ]
    
    
list_of_thetas = [0] * len(local_linkage_data)
angle_max = list_of_thetas.copy()
angle_min = list_of_thetas.copy()
list_of_blockers = list_of_thetas.copy()

for i, value in enumerate(index_of_angles):
    angle_max[value] = upper_limit[i]
    angle_min[value] = lower_limit[i]
    list_of_blockers[value] = 1
    

# special provision for the blockers to have theta to rotate the coodinate frame that is not due to hinge i.e theta due to mechanical link
for i, linkage in enumerate(local_linkage_data):
    if linkage[1] != 0:
        list_of_blockers[i] = 2
        list_of_thetas[i] = local_linkage_data[i][1]
        


def check_list(input_list):
    lower_exceed_count = 0
    upper_exceed_count = 0
    for i in range(len(input_list)):
        if input_list[i] < lower_limit[i]: lower_exceed_count=lower_exceed_count+1
        if input_list[i] > upper_limit[i]: upper_exceed_count=upper_exceed_count+1
    
    if lower_exceed_count>0 or upper_exceed_count>0: 
        return_string = "Angles out of range"
    else:
        return_string = "Angles in range"        

    return return_string


def DH_matrix(theta, alpha, delta, rho):

    transient_matrix = np.eye(4)
    theta_rad = theta/180*np.pi
    alpha_rad = alpha/180*np.pi
    
    cos_theta_rad = np.cos(theta_rad)
    sin_theta_rad = np.sin(theta_rad)
    cos_alpha_rad = np.cos(alpha_rad)
    sin_alpha_rad = np.sin(alpha_rad)    
    
    # Handle DH parameters, row-by-row, left-to-right
    transient_matrix[0,0]=cos_theta_rad
    transient_matrix[0,1]=-sin_theta_rad   
    transient_matrix[0,2]=0    
    transient_matrix[0,3]=rho    

    transient_matrix[1,0]=sin_theta_rad*cos_alpha_rad
    transient_matrix[1,1]=cos_theta_rad*cos_alpha_rad
    transient_matrix[1,2]=-sin_alpha_rad   
    transient_matrix[1,3]=-sin_alpha_rad * delta    

    transient_matrix[2,0]=sin_theta_rad*sin_alpha_rad
    transient_matrix[2,1]=cos_theta_rad*sin_alpha_rad
    transient_matrix[2,2]=cos_alpha_rad     
    transient_matrix[2,3]=cos_alpha_rad * delta   

    return transient_matrix

def get_closest_coordinate_params(quadrant_coordinates_df):
    
    closest_coordinate = quadrant_coordinates_df[quadrant_coordinates_df['distance'] == quadrant_coordinates_df['distance'].min()]

    return(closest_coordinate)


def input_linkage_angles(list_of_thetas):
    
    for i in range(len(list_of_thetas)):
        # if list_of_blockers[i] == 2: # This is for special provision at axis 5
        #     list_of_thetas[i] = 90
        local_linkage_data[i][1] = list_of_thetas[i]

    array_matrix = []
    transformation_matrix = None
    
    for i, linkage in enumerate(local_linkage_data):
        
        # Rotations first
        transient_rotation = DH_matrix(linkage[1], linkage[2], 0, 0)
        
        if transformation_matrix is None:
            transformation_matrix = transient_rotation
        else:
            transformation_matrix = np.matmul(transformation_matrix, transient_rotation)
        
        # then the translations
        transient_translation = DH_matrix(0, 0, linkage[3], linkage[4])
        
        if transformation_matrix is None:
            transformation_matrix = transient_translation
        else:
            transformation_matrix = np.matmul(transformation_matrix, transient_translation)
        
        array_matrix.append(transformation_matrix)

    return(array_matrix)


def sqrt_sum_aquare(input_list):
    sum_square = 0
    for value in input_list:
        sum_square += value*value
    return(math.sqrt(sum_square))

def Inverse_Kinematics_CCD(target):
    solved = False
    err_end_to_target = math.inf
    minimum_error = math.inf
    
    for loop in range(max_iter):
        for i in range(len(local_linkage_data)-1, -1, -1):
            
            if list_of_blockers[i] != 0: # == 1 or 2 

                P = input_linkage_angles(list_of_thetas) # forward kinematics 
                end_to_target = target - P[-1][:3, 3]
                err_end_to_target = sqrt_sum_aquare(end_to_target)
                #error_list.append([loop, err_end_to_target])
                
                # record the angles of the best minimal error so far; yes the error can increase in further iterations
                if err_end_to_target < minimum_error:
                    minimum_error = err_end_to_target
                    least_error_angles = list_of_thetas.copy()                  
                
                if err_end_to_target < err_min:
                    solved = True
                else:
                    
                    if list_of_blockers[i] != 2:
                    
                        # Calculate distance between i-joint position to end effector position
                        # P[i] is position of current joint
                        # P[-1] is position of end effector
                        
                        # reviewed and change code here to improve since normal vector is always Z-axis if theta is always used as rotation
                        # use the DH array matrix, because in the top-left 3x3 sub-matrix, it already contains the vectors 
                        # for all 3 axis, top-row = X axis, middle = Y axis and last row = Z axis 
                        # this is with great hint from chatGPT with key question
                        # "find z-axis vector from denavit hartenberg matrix"
                        # attempt this update from V19
    
                        # find normal of rotation plane, aka hinge axis (hinge is always normal to rotation plane)
                        normal_vector = list(P[i][2, :3])
                        plane = Plane(point=P[i][:3, 3], normal=normal_vector)
                        
                        # find projection of tgt onto rotation plane
                        # https://scikit-spatial.readthedocs.io/en/stable/gallery/projection/plot_point_plane.html
                        target_point_projected = plane.project_point(target)
                        end_point_projected = plane.project_point(P[-1][:3, 3])
                        
                        # find angle between projected tgt and cur_to_end
                        cur_to_end_projected = end_point_projected - P[i][:3, 3]
                        cur_to_target_projected = target_point_projected - P[i][:3, 3]
    
                        # end_target_mag = |a||b|    
                        cur_to_end_projected_mag = sqrt_sum_aquare(cur_to_end_projected)
                        cur_to_target_projected_mag = sqrt_sum_aquare(cur_to_target_projected)
                        end_target_mag = cur_to_end_projected_mag * cur_to_target_projected_mag
    
                        # if the 2 vectors current-effector and current-target is already very close 
                        if end_target_mag <= 0.0001:    
                            cos_rot_ang = 1
                            sin_rot_ang = 0
                        else:
                            # dot product rule - https://en.wikipedia.org/wiki/Dot_product
                            # To solve for angle magnitude between 2 vectors
                            # dot product of two Euclidean vectors a and b
                            # a.b = |a||b|cos(lambda)
                            # cos_rot_ang = cos(lambda) = a.b / |a||b|
                            cos_rot_ang = (cur_to_end_projected[0] * cur_to_target_projected[0] + cur_to_end_projected[1] * cur_to_target_projected[1] + cur_to_end_projected[2] * cur_to_target_projected[2]) / end_target_mag
                            
                            # cross product rule - https://en.wikipedia.org/wiki/Cross_product
                            # https://www.mathsisfun.com/algebra/vectors-cross-product.html
                            # cross product of two Euclidean vectors a and b
                            # a X b = |a||b|sin(lambda)
                            # sin_rot_ang = sin(lambda) = [a X b] / |a||b|
                            # To solve for direction of angle A->B or B->A
                            # for theta rotation (about Z axis) in right hand rule, keep using [0] and [1] for finding Z direction
                            # cross product of 3d vectors has i, j, k components
                            # after we do the projections onto the plane level, we will focus on the k component                      
                            sin_rot_ang = (cur_to_end_projected[0] * cur_to_target_projected[1] - cur_to_end_projected[1] * cur_to_target_projected[0]) / end_target_mag   
                            
                        rot_ang = math.acos(max(-1, min(1,cos_rot_ang)))
        
                        if sin_rot_ang < 0.0:
                            rot_ang = -rot_ang
        
                        # Update current joint angle values
                        list_of_thetas[i] = list_of_thetas[i] + (rot_ang * 180 / math.pi)
                        list_of_thetas[i] = (list_of_thetas[i] + 180) % 360 - 180
    
                        # clamp angle
                        if list_of_thetas[i] > angle_max[i]: list_of_thetas[i] = angle_max[i]
                        if list_of_thetas[i] < angle_min[i]: list_of_thetas[i] = angle_min[i]
                        
                    elif list_of_blockers[i] == 2:  
                        #list_of_thetas[i] = 90
                        # there was a bug here befoew where the blockers force it to only positive 90 deg
                        # now I have update to adopt whatever linkage data is needed e.g. -90 deg aka 270 deg
                        list_of_thetas[i] = local_linkage_data[i][1] 

        if solved:
            break

    if solved == False:
        for i in range(len(list_of_thetas)):
            list_of_thetas[i] = least_error_angles[i] #return least error
            err_end_to_target = minimum_error
            P = input_linkage_angles(list_of_thetas) # forward kinematics
            
    return P, list_of_thetas, err_end_to_target, solved, loop


def add_trace(array_of_transformation_matrix):
    
    traces = []
    
    x_coordinate = []
    y_coordinate = []
    z_coordinate = []
        
    for i, transformation_matrix in enumerate(array_of_transformation_matrix):
        
        x_coordinate.append(transformation_matrix[0,3])
        y_coordinate.append(transformation_matrix[1,3])
        z_coordinate.append(transformation_matrix[2,3])
               
        
        if len(x_coordinate)==2:

            if x_coordinate[1]!=x_coordinate[0] or y_coordinate[1]!=y_coordinate[0] or z_coordinate[1]!=z_coordinate[0]:
            
                # print("Difference found at " + str(i))
                traces.append(go.Scatter3d(x=x_coordinate, y=y_coordinate, z=z_coordinate,
                                            #opacity=0.9,
                                            mode='lines',
                                            
                                            marker=dict(
                                                size=2
                                                ),
                                            line=dict(
                                                #name = linkage[0],
                                                #color=next(colors),
                                                color=px.colors.qualitative.Plotly[local_linkage_data[i][0]],
                                                width=15
                                              )
                                            ))
            x_coordinate.pop(0)
            y_coordinate.pop(0)
            z_coordinate.pop(0)

    return traces


def writeAngles(values):
    # sending an float to the arduino
    # consider changing d to f for float precision
    # python float f is 4 bytes
    # python double d is 8 bytes
                  
    print("angles_list")
    print(values)
    
    byteList = []
    for value in values:
        value = float(value)
        byteList += list(struct.pack('f', value))
    print(byteList)
    byteList.append(0)  # fails to send last byte over I2C, hence this needs to be added 
    try:
        bus.write_i2c_block_data(ESP32_IOT, byteList[0], byteList[1:len(byteList)])#[0:11]) byteList[0], 
    except IOError:
        subprocess.call(['i2cdetect', '-y', '1'])
    time.sleep(1)



def end_of_sequence():
    # create a dummy angles_list df to signify end of sequence, so it will of 6 theta values of "888"      
    last_row_data = [888.8, 888.8, 888.8, 888.8, 888.8, 888.8]               
    #last_row_df = pd.DataFrame(data, columns=['theta1', 'theta2','theta3', 'theta4','theta5', 'theta6'])
    writeAngles(last_row_data)
                    
    
app = DashProxy(__name__, suppress_callback_exceptions=True, title='6 DOF Kinematics', transforms=[MultiplexerTransform()]) 


app.layout = html.Div([
    
    dcc.Store(id='memory-output'),

    html.Div([   
        
    html.Div([   
        
        html.Div([
            
            html.Div([   
                html.Img(src=app.get_asset_url('Jason_H_Engineering_Logo_100_100_V1.png')),
            ], className='one columns'),
            
            html.Br(),
        ], className='row'), 
    
        html.Div([
        
                dcc.RadioItems(
                    id='angle_or_coordinate_radio',
                    options=[{'label': i, 'value': i} for i in ['Angles', 'Steppers', 'Path Plan']],
                    #options=['Angles', 'Target Coordinate'],
                    value='Angles',
                    labelStyle={'display': 'inline-block'}
                ),
                ], className='row'),
        
   
    ], className='three columns'),

    ], className='row'),
    
    html.Div([ 
        html.Div(id='div-content'),
    ], className='row'),

    ])



def angles_to_steps(angular_list):
    step_list = [0]*len(angular_list)
    for i in range(len(angular_list)):
        step_list[i] = round((angular_list[i] * gear_reduction[i] * micro_steps[i] / (360/steps_per_rotation[i])),0)
    return step_list

def div_header(value):
    
    if value == "Angles":
        html_string = "Enter Target Angles"
    else:
        html_string = "Enter Target Coordinates"

    appended_layout = html.Div([

        html.Div([
            html.H4(html_string)
        ], className='row'),
        
    ], className='row'),
        
    return appended_layout


def div_angles(axis, axis_data, selected_state):
       
    if selected_state == "Angles": 
        boolean_state = False
    else:
        boolean_state = True
    
    html_string = "A" + str(axis+1) + " ∠:"
    id_string = "angle_" + str(axis+1)
    
    body_layout = html.Div([
        html.B(html_string,style={'display':'inline-block','margin-right':5}),
        
        dcc.Input(
            id=id_string, 
            type="number",
            debounce=True, 
            value=axis_data,
            disabled=boolean_state,
            # placeholder=axis_encoder_data[axis],
            min=angle_lower_limit[axis], 
            max=angle_upper_limit[axis], 
            # step=1,
        ),
        
    ], className='row')
        
    return body_layout





def div_stepper(axis, step_data, selected_state):


    if selected_state == "Steppers": 
        boolean_state = False
    else:
        boolean_state = True
    
    html_string = "A" + str(axis+1) + " steps:"
    id_stepper_string = "stepper_" + str(axis+1)
    
    body_layout = html.Div([
        html.B(html_string,style={'display':'inline-block','margin-right':5}),
        
        dcc.Input(
            id=id_stepper_string, 
            type="number",
            debounce=True, 
            value=step_data,
            disabled=boolean_state,
            # placeholder=0,
            min=stepper_lower_limit[axis], 
            max=stepper_upper_limit[axis], 
            # step=1,
        ),
        
    ], className='row')
        
    return body_layout



def add_axis_to_trace(traces, colour_string, axis_start, axis_end):

    rotate_axis = [axis_start, axis_start, axis_end]
    
    for i in range(len(colour_string)):

        traces.append(go.Scatter3d(x=rotate_axis[0], y=rotate_axis[1], z=rotate_axis[2],
                                   #opacity=0.7,
                                   mode='lines',
                                   marker=dict(
                                       color=colour_string[i],
                                       size=12
                                       )
                                   ))
        
        rotate_axis.append(rotate_axis.pop(0))
    
    return traces

def DH_array_to_hinge_list(traces, array_matrix):
    
    # hinge_type = ["delta","rho","rho","rho","delta"]
    
    for j in index_of_hinges:
        transformation_matrix = array_matrix[j]

        transient_translation = DH_matrix(0, 0, hinge_length, 0)      
        positive_z_matrix = np.matmul(transformation_matrix, transient_translation)
        transient_translation = DH_matrix(0, 0, -hinge_length, 0)
        #negative_z_matrix = np.matmul(transformation_matrix, transient_translation)

        # grab the +/- coordinates into array
        x_hinge = np.array([array_matrix[j][0,3],positive_z_matrix[0,3]])
        y_hinge = np.array([array_matrix[j][1,3],positive_z_matrix[1,3]])
        z_hinge = np.array([array_matrix[j][2,3],positive_z_matrix[2,3]])
        

        traces.append(go.Scatter3d(x=x_hinge, y=y_hinge, z=z_hinge,
                                   #opacity=0.7,
                                   mode='lines',
                                   marker=dict(
                                       #symbol="arrow",
                                       color="black",
                                       size=20
                                       ),
                                   line=dict(
                                    #color='purple',
                                    width=20)
                                   ))        

    return(traces)

def append_target_to_trace(traces, target):
    
    # append target point
    traces.append(go.Scatter3d(x=[target[0]], y=[target[1]], z=[target[2]],
                               opacity=0.7,
                               mode='markers',
                               marker=dict(
                                   color='black',
                                   size=12
                                   )
                               ))
    
    return(traces)


def parse_data(contents, filename):
    content_type, content_string = contents.split(",")

    decoded = base64.b64decode(content_string)
    try:
        if "csv" in filename:
            # Assume that the user uploaded a CSV or TXT file
            # df = pd.read_csv(io.StringIO(decoded.decode("utf-8")))
            df = pd.read_csv(io.StringIO(decoded.decode("ISO-8859-1")))
        elif "xls" in filename:
            # Assume that the user uploaded an excel file
            df = pd.read_excel(io.BytesIO(decoded))
        elif "txt" or "tsv" in filename:
            # Assume that the user upl, delimiter = r'\s+'oaded an excel file
            df = pd.read_csv(io.StringIO(decoded.decode("utf-8")), delimiter=r"\s+")
    except Exception as e:
        print(e)
        return html.Div(["There was an error processing this file."])

    return df




def get_encoder_data():
       
    try:
        data=bus.read_i2c_block_data(ESP32_IOT,0x00,24) #read_i2c_block_data(i2c_addr, register, length, force=None)
        byte_data = bytearray(data) # have to convert to array of bytes
        the_struct = struct.unpack('ffffff', byte_data)
        #print(the_struct)
        encoder_df = pd.DataFrame([the_struct], columns = column_headers)

        raw_encoder_data = []
        for i in range(len(encoder_df.columns)):
            raw_encoder_data.append(round(encoder_df[encoder_df.columns[i]][0],2))

        return raw_encoder_data
    
    except IOError:
        print("remote i/o error")
        subprocess.call(['i2cdetect', '-y', '1'])
        time.sleep(1)    
        



#Div handler
@app.callback(dash.dependencies.Output('div-content', 'children'),
              [dash.dependencies.Input('angle_or_coordinate_radio', 'value')])
def display_page(value):

    
    internal_layout_angles = []
    internal_layout_stepper = []
              
    axis_encoder_data = get_encoder_data()    
    
    # update here in V7 to feed encoder data to reflect current angular position with respect to zero position
    for axis in range(len(start_angles_list)):
        internal_layout_angles.append(div_angles(axis, axis_encoder_data[axis],value))
            
    axis_steps_data = angles_to_steps(axis_encoder_data) # convert encoder data to absolute steps (from zero position) 
    
    for axis in range(len(start_angles_list)):
        internal_layout_stepper.append(div_stepper(axis, axis_steps_data[axis],value))
            

    if (value == "Angles"):
        header_string = 'Forward Kinematics - Angles (°)'
    else: 
        header_string = 'Manual Stepping (Note: Absolute Position)'
    
    if (value == "Angles") or (value == "Steppers"):
    
        div_layout = html.Div([

            html.Div([
                
            html.H5(header_string),
            html.Br(),
            
            html.Div([
            
                html.Div(internal_layout_angles, className='three columns'),
                html.Div(internal_layout_stepper, className='three columns'),
            
            ], className='row'),
            
            html.Br(),
            html.Button('Run', id='btn-1',style={'width': '280px'}),
            html.Br(),
            html.Button('Home Steppers', id='btn-8',style={'width': '280px'}),
            html.Br(),
            
            html.Div([
                html.B(id='target_coordinates',style={'color': 'grey', 'fontSize': 16}),
            ], className='row'),
            html.Br(),
            html.Div([
                html.B(id='target_angles',style={'color': 'grey', 'fontSize': 16}),
            ], className='row'),
            
            ], className='row'),
        
        ])


    if value == "Path Plan":
        
        div_layout = html.Div([
            
        html.Div([
        html.Div([
            
        html.H5('Select/drop IK solution file!'),
            dcc.Upload(
                id="upload_data_1",
                children=html.Div(["Drag and Drop or ", html.A("Select Files")]),
                style={
                    "width": "100%",
                    "height": "60px",
                    "lineHeight": "60px",
                    "borderWidth": "1px",
                    "borderStyle": "dashed",
                    "borderRadius": "5px",
                    "textAlign": "center",
                    #"margin": "10px",
                },
                # Allow multiple files to be uploaded
                multiple=False,
            ),

        #], style={'marginTop': 25, 'marginLeft': 50, 'marginRight': 50}),
        ], className='six columns'),
        ], className='row'),
        

        html.Div([

            html.Div([
                 html.Div(id="raw-data-1-upload"),
                  ], className='nine columns'),

        ],className='row'),

    ]),
        
    return div_layout


# file upload handler
@app.callback(
    dash.dependencies.Output("upload_data_1", "children"), 
    dash.dependencies.Output("raw-data-1-upload", "children"),
    [dash.dependencies.Input("upload_data_1", "contents"), 
     dash.dependencies.Input("upload_data_1", "filename"),
     ],
)

# for displaying raw table
def update_children_1(contents_1, filename_1):

    if contents_1:

        df_1 = parse_data(contents_1, filename_1)
        return_div = html.Div([filename_1])
        html_string_1 = "Raw file: " + str(filename_1)
        
        sequence_id = df_1["Rows"]

        table = html.Div([
                html.Div([        
                                        
                    html.Br(),
                    html.Button('Run all sequence', id='btn-6',style={'width': '280px'}),
                    html.Br(),
                    
                    html.H6('Or select a specific target:'),
                       html.Div([
                           dcc.Dropdown(
                               id='IK_target_ID',
                               options=[{'label': x, 'value': x} for x in sequence_id],
                               value=None,
                               placeholder="Select target number",
                       )],style={"width": "60%"},),
                    html.Br(),                    
                    html.Button('Run Sequence', id='btn-7',style={'width': '280px'}),
                    html.Br(),                    
                      ], className='three columns'),

                html.Div([        
                
                html.Br(), 
                html.Div(html_string_1),
                dash_table.DataTable(
                    id="table",
                    data=df_1.to_dict("records"),
                    columns=[{"name": i, "id": i} for i in df_1.columns],
                    style_table={'height': '600px', 'overflowY': 'auto', 'overflowX': 'auto'},
                    style_cell={'textAlign': 'left',
                                'font_size': '16px'},
                    style_data={
                        'whiteSpace': 'normal',
                        'height': 'auto',
                    },
                    fill_width=False
                ),
            ], className='nine columns'),
                
            html.Div(id='running_status'),
                
            ])

    else:
        return_div = html.Div(["Drag and Drop or ", html.A("Select Files")])
        table = None

    return return_div, table



# callback for running full sequence from CSV file
@app.callback(
    #dash.dependencies.Output("table", "style_data_conditional"), #https://community.plotly.com/t/highlighting-selected-rows/49595/5
    dash.dependencies.Output(component_id='running_status', component_property='children'),    
    [dash.dependencies.Input('btn-6', 'n_clicks'),
    dash.dependencies.Input("upload_data_1", "contents"), 
    dash.dependencies.Input("upload_data_1", "filename"),
    ], prevent_initial_call=True)
def sequence_run(btn6, contents_1, filename_1):

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    if 'btn-6' in changed_id:
        
        sequence_df = parse_data(contents_1, filename_1)
        
        theta_start_column = sequence_df.columns.get_loc('theta1')
        solve_status_column = sequence_df.columns.get_loc('Solve Status')
            
        for rows in range(len(sequence_df.index)):
            angles_list = sequence_df.iloc[rows][theta_start_column:(theta_start_column+len(index_of_angles))]
            solve_status = sequence_df.iloc[rows][solve_status_column]
            
            if solve_status == True:
                check_string = check_list(angles_list)
                if check_string == "Angles in range" :
                    writeAngles(angles_list)
                    
        time.sleep(5)                      
        #end_of_sequence()

        return "Sequence run complete" 
                        
           


# callback for running selected sequence from CSV file
@app.callback(
    dash.dependencies.Output("table", "style_data_conditional"), #https://community.plotly.com/t/highlighting-selected-rows/49595/5
    [dash.dependencies.Input('btn-7', 'n_clicks'),
    dash.dependencies.Input('IK_target_ID', 'value'),
    dash.dependencies.Input("upload_data_1", "contents"), 
    dash.dependencies.Input("upload_data_1", "filename"),
    ])
def move_to_point(btn7, sequence_id_value, contents_1, filename_1):

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    if 'btn-7' in changed_id:
        
        sequence_df = parse_data(contents_1, filename_1)
        theta_start_column = sequence_df.columns.get_loc('theta1')
        selected_row_index = sequence_df[(sequence_df["Rows"] == sequence_id_value)].index
        angles_list = sequence_df.iloc[selected_row_index.values[0]][theta_start_column:(theta_start_column+len(index_of_angles))]      
    
        if sequence_id_value>-1: 
            check_string = check_list(angles_list)

            if check_string == "Angles in range" :

                writeAngles(angles_list)
                time.sleep(1)  
                #end_of_sequence()
                
                return [
                    {"if": {"row_index": sequence_id_value-1}, "backgroundColor": "dodgerblue",}
                ]            
                        
            if check_string == "Angles out of range" :
                   
                return [
                    {"if": {"row_index": sequence_id_value-1}, "backgroundColor": "tomato",}
                ]



# callback btn-8 for homing
@app.callback(
    dash.dependencies.Output('angle_1', 'value'),
    dash.dependencies.Output('angle_2', 'value'),
    dash.dependencies.Output('angle_3', 'value'),
    dash.dependencies.Output('angle_4', 'value'),
    dash.dependencies.Output('angle_5', 'value'),
    dash.dependencies.Output('angle_6', 'value'),
    dash.dependencies.Output('stepper_1', 'value'),
    dash.dependencies.Output('stepper_2', 'value'),
    dash.dependencies.Output('stepper_3', 'value'),
    dash.dependencies.Output('stepper_4', 'value'),
    dash.dependencies.Output('stepper_5', 'value'),
    dash.dependencies.Output('stepper_6', 'value'),
    dash.dependencies.Output('target_coordinates', 'children'),
    [dash.dependencies.Input('btn-8', 'n_clicks'),
], prevent_initial_call=True)
#])
def home_stepper_callback(btn8):
       
    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    if 'btn-8' in changed_id:
        
        angles_list = [0,0,0,0,0,0]
        writeAngles(angles_list)

        
    
        for i, row in enumerate(index_of_angles):
            list_of_thetas[row] = angles_list[i]
    
        array_matrix = input_linkage_angles(list_of_thetas)
        
        end_x_coordinate = round(array_matrix[-1][0,3],1)
        end_y_coordinate = round(array_matrix[-1][1,3],1)
        end_z_coordinate = round(array_matrix[-1][2,3],1)
        
        time.sleep(1)  
        #end_of_sequence()
        
        axis_encoder_data = get_encoder_data()  
        axis_steps_data = angles_to_steps(axis_encoder_data) # convert encoder data to absolute steps (from zero position) 
        
        return (
        axis_encoder_data[0],
        axis_encoder_data[1],
        axis_encoder_data[2],
        axis_encoder_data[3],
        axis_encoder_data[4],
        axis_encoder_data[5],
        axis_steps_data[0],
        axis_steps_data[1],
        axis_steps_data[2],        
        axis_steps_data[3],
        axis_steps_data[4],
        axis_steps_data[5],   
        f'X = {end_x_coordinate} mm, Y = {end_y_coordinate} mm, Z = {end_z_coordinate} mm'
        )


# handler for angles input (forward kinematics)
@app.callback(
    dash.dependencies.Output('angle_1', 'value'),
    dash.dependencies.Output('angle_2', 'value'),
    dash.dependencies.Output('angle_3', 'value'),
    dash.dependencies.Output('angle_4', 'value'),
    dash.dependencies.Output('angle_5', 'value'),
    dash.dependencies.Output('angle_6', 'value'),
    dash.dependencies.Output('stepper_1', 'value'),
    dash.dependencies.Output('stepper_2', 'value'),
    dash.dependencies.Output('stepper_3', 'value'),
    dash.dependencies.Output('stepper_4', 'value'),
    dash.dependencies.Output('stepper_5', 'value'),
    dash.dependencies.Output('stepper_6', 'value'),
    dash.dependencies.Output('target_coordinates', 'children'),
    [dash.dependencies.Input('angle_or_coordinate_radio', 'value'),
    dash.dependencies.Input('btn-1', 'n_clicks'),
    dash.dependencies.Input('stepper_1', 'value'),
    dash.dependencies.Input('stepper_2', 'value'),
    dash.dependencies.Input('stepper_3', 'value'),
    dash.dependencies.Input('stepper_4', 'value'),
    dash.dependencies.Input('stepper_5', 'value'),
    dash.dependencies.Input('stepper_6', 'value'),
    dash.dependencies.Input('angle_1', 'value'),
    dash.dependencies.Input('angle_2', 'value'),
    dash.dependencies.Input('angle_3', 'value'),
    dash.dependencies.Input('angle_4', 'value'),
    dash.dependencies.Input('angle_5', 'value'),
    dash.dependencies.Input('angle_6', 'value')
    ],prevent_initial_call=True)
def forward_kinematics(value, btn1, 
                       stepper_1, stepper_2, stepper_3, stepper_4, stepper_5, stepper_6, 
                       angle_1, angle_2, angle_3, angle_4, angle_5, angle_6):
    
    # method to check if button click, so that fig update only upon click
    # https://stackoverflow.com/questions/62671226/plotly-dash-how-to-reset-the-n-clicks-attribute-of-a-dash-html-button
    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    if 'btn-1' in changed_id:
        # start of reading encoder angles for working out the kinematics
        axis_encoder_data = get_encoder_data()
        
        # axis_steps_data = angles_to_steps(axis_encoder_data) # convert encoder data to absolute steps (from zero position) 
        
        if value == "Steppers":
        
            steps = [stepper_1, stepper_2, stepper_3, stepper_4, stepper_5, stepper_6]
            steps = [0 if v is None else v for v in steps]
            
            # convert steps to angles and to check angle limits
            steps_to_angles = np.array(steps) / (np.array(gear_reduction) * np.array(micro_steps)) * (360/np.array(steps_per_rotation))

            angles_list = list(steps_to_angles)
                        
        
        if value == "Angles":

            angles_list = [angle_1, angle_2, angle_3, angle_4, angle_5, angle_6] # these are desired relative angles from user entry
            
        check_string = check_list(angles_list)
        
        if check_string == "Angles in range" :
            
            for i, row in enumerate(index_of_angles):
                list_of_thetas[row] = angles_list[i]

            array_matrix = input_linkage_angles(list_of_thetas)
            
            end_x_coordinate = round(array_matrix[-1][0,3],1)
            end_y_coordinate = round(array_matrix[-1][1,3],1)
            end_z_coordinate = round(array_matrix[-1][2,3],1)
        
            # writeNumbers(steps)       
            writeAngles(angles_list)



        if check_string == "Angles out of range" :

            end_x_coordinate = 0
            end_y_coordinate = 0
            end_z_coordinate = 0
        
        time.sleep(1)  
        #end_of_sequence()
        axis_encoder_data = get_encoder_data()  
        axis_steps_data = angles_to_steps(axis_encoder_data) # convert encoder data to absolute steps (from zero position) 
     

        return (
        axis_encoder_data[0],
        axis_encoder_data[1],
        axis_encoder_data[2],
        axis_encoder_data[3],
        axis_encoder_data[4],
        axis_encoder_data[5],
        axis_steps_data[0],
        axis_steps_data[1],
        axis_steps_data[2],        
        axis_steps_data[3],
        axis_steps_data[4],
        axis_steps_data[5],        
        f'X = {end_x_coordinate} mm, Y = {end_y_coordinate} mm, Z = {end_z_coordinate} mm'
        )


if __name__ == '__main__':
    
    stepper_lower_limit = angles_to_steps(angle_lower_limit)
    stepper_upper_limit = angles_to_steps(angle_upper_limit)
    
    app.run_server(host= '0.0.0.0',debug=False)
        