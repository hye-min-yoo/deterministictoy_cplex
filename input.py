
#node (num_leg)+(num_airprot)
#arc = (num_leg)+(num_airprot)*(num_leg)+(num_airprot)
import pandas as pd
import numpy as np

horizon=168

num_airport=5

maintenance=[1,2,3,4,5]

#leg 정보
leg_info=pd.read_csv('C:/Users/scm/PycharmProjects/deterministic_toy/leg_info_40.csv',engine='python')
leg_info = leg_info.sort_values(by='dep_t', ascending=True)
num_leg=len(leg_info)
dep_t=leg_info['dep_t'].tolist()+[0]*num_airport*2
arr_t=leg_info['arr_t'].tolist()+[0]*num_airport+[horizon]*num_airport
dep_s=leg_info['dep_s'].tolist()+[num_airport+1]*num_airport*2
arr_s=leg_info['arr_s'].tolist()+[0]*num_airport

#beginning_station=leg_info['dep_s'].tolist()
'''
del beginning_station[len(beginning_station)-1]
'''

num_aircraft=6

num_node=num_leg+2*num_airport

tripfuel_array=[[0,20,30,30,45],[0,0,10,20,35],[0,0,0,35,40],[0,0,0,0,20],[0,0,0,0,0]]

#cost
c=[[0]*num_node for i in range(num_node)]
penalty=500

turn = 1
sit=2
m_time=3

#2nd stage params
fuel=[5.48,4.97,4.57,5.1,4.84]+[0]
DOW=5
load_factor=0
avg_weight=0
PL=load_factor*avg_weight
taxi_fuel=0
min_holding_fuel=0
min_final_reserve_fuel=0
SF=[0]*num_leg
for i in range(num_leg):
    if dep_s[i] < arr_s[i]:
        SF[i]=taxi_fuel+min_holding_fuel+min_final_reserve_fuel+tripfuel_array[dep_s[i] - 1][arr_s[i] - 1]*0.04
    else:
        SF[i]=taxi_fuel+min_holding_fuel+min_final_reserve_fuel+tripfuel_array[arr_s[i] - 1][dep_s[i] - 1]*0.04
MTOW=90
MTOTW=MTOW-DOW-PL
MLW= 70
MLTW=MLW-DOW-PL
TANK= 70
a=0.02

max_tripfuel=max(map(max,tripfuel_array))
min_tripfuel=max_tripfuel
for j in range(num_airport-1):
    k=j
    while k<num_airport-1:
        k=k+1
        if tripfuel_array[j][k]<min_tripfuel:
            min_tripfuel=tripfuel_array[j][k]

M=(1-a)*max_tripfuel+TANK-min_tripfuel
