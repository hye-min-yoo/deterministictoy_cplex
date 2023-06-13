import pandas as pd
from docplex.mp.model import Model
from utils import *
from input import *
from datetime import datetime
import cplex
import numpy

model=cplex.Cplex()
model.parameters.read.datacheck.set(model.parameters.read.datacheck.values.off)

model.objective.set_sense(model.objective.sense.minimize)

cost, arc,m_arc, nm_arc, p_arc = generate_connection(num_leg,num_airport, c, dep_t,arr_t,dep_s,arr_s,turn, sit, penalty, maintenance, m_time,horizon)


y=[]
X=[]
R=[]
for i in range(num_aircraft):
    y.append([])
    for j in range(num_node-num_airport):
        y[i].append([])
        for k in range(num_node):
            var_name = "y." + str(i) + "." + str(j) + "." + str(k)
            y[i][j].append(model.variables.get_num())
            model.variables.add(lb=[0.0], ub=[1.0], types=["B"],names=[var_name])

for i in range(num_aircraft):
    X.append([])
    for j in range(num_node-num_airport):
        X[i].append([])
        for k in range(num_node):
            var_name = "X." + str(i) + "." + str(j) + "." + str(k)
            X[i][j].append(model.variables.get_num())
            model.variables.add(lb=[0.0], ub=[TANK], types=["C"],names=[var_name])
for i in range(num_aircraft):
    R.append([])
    for j in range(num_node-num_airport):
        R[i].append([])
        for k in range(num_node):
            var_name = "R." + str(i) + "." + str(j) + "." + str(k)
            R[i][j].append(model.variables.get_num())
            model.variables.add(lb=[0.0], ub=[TANK], types=["C"],
                              names=[var_name])

#초기해 지정
lhs = [
    cplex.SparsePair(ind=[y[0][ dep_s[0] + num_leg - 1][0]], val=[1.0]),
    cplex.SparsePair(ind=[y[1][dep_s[1] + num_leg - 1][1]], val=[1.0]),
    cplex.SparsePair(ind=[y[2][dep_s[2] + num_leg - 1][2]], val=[1.0])
]
rhs = [1.0, 1.0, 1.0]
model.linear_constraints.add(lin_expr=lhs, senses=["E", "E", "E"], rhs=rhs)

# formulation의 symmetry를 제거하기 위한 제약
for i in range(num_aircraft - 1):
    lhs = cplex.SparsePair(ind=[y[i][j][k] for j, k in arc] + [y[i + 1][j][k] for j, k in arc],
                           val=[1.0] * len(arc) + [-1.0] * len(arc))
    rhs = [0.0]
    senses = ["G"]
    model.linear_constraints.add(lin_expr=[lhs], senses=senses, rhs=rhs)


# con1 : 모든 flight leg는 cover되어야 한다.
for j in range(num_leg):
    temp = []
    for k in range(num_node):
        if [j, k] in arc:
            temp.append([j, k])
    lhs = [cplex.SparsePair(ind=[y[i][temp[k][0]][temp[k][1]] for i in range(num_aircraft) for k in range(len(temp))], val=[1.0]*len(temp)*num_aircraft)]
    rhs = [1.0]
    model.linear_constraints.add(lin_expr=lhs, senses=["E"], rhs=rhs)

# con2 : flow continuity
for i in range(num_aircraft):
    for j in range(num_leg):
        temp1 = []
        temp2 = []
        for k in range(num_node):
            if [j, k] in arc:
                temp1.append([j, k])
            if [k, j] in arc:
                temp2.append([k, j])
        lhs = cplex.SparsePair(ind=[y[i][j][temp1[k][1]] for k in range(len(temp1))]+[y[i][temp2[k][0]][j] for k in range(len(temp2))], val=[1.0]*len(temp1)+[-1.0] * len(temp2))
        rhs = [0.0]
        model.linear_constraints.add(lin_expr=[lhs], senses=["E"], rhs=rhs)

# con3 : 모든 항공기는 시작한 공항에서 끝날 것
for i in range(num_aircraft):
    #모든 공항 통틀어서 나가는 아크
    temp = []
    for j in range(num_airport):
        #j 공항에서 나가는 아크
        temp1 = []
        #j 공항으로 들어오는 아크
        temp2 = []
        for k in range(num_leg):
            if [j + num_leg, k] in arc:
                temp1.append([j + num_leg, k])
                temp.append([j + num_leg, k])
            if [k, j + num_leg + num_airport] in arc:
                temp2.append([k, j + num_leg + num_airport])
        lhs = [cplex.SparsePair(ind=[y[i][temp1[j][0]][temp1[j][1]] for j in range(len(temp1))]+[y[i][temp2[k][0]][temp2[k][1]] for k in range(len(temp2))], val=[1.0]*len(temp1)+[-1.0]*len(temp2))]
        rhs = [0.0]
        model.linear_constraints.add(lin_expr=lhs, senses=["E"], rhs=rhs)
    #모든 항공기는 최대 하나의 공항을 통해 나갈 수 있다.
    lhs = [cplex.SparsePair(ind=[y[i][temp[j][0]][temp[j][1]] for j in range(len(temp))], val=[1.0]*len(temp))]
    rhs = [1.0]
    model.linear_constraints.add(lin_expr=lhs, senses=["L"], rhs=rhs)


# con4 : maintenance constraint
for i in range(num_aircraft):
    lhs = [cplex.SparsePair(ind=[y[i][j][k] for j, k in m_arc], val=[1.0]*len(m_arc))]
    rhs = [1.0]
    model.linear_constraints.add(lin_expr=lhs, senses=["G"], rhs=rhs)


# con5,6,7,8,9 : TANK, MTOW, MLW, (y==0이면 x,r은 0으로 죽여,) balance equation
for i in range(num_aircraft):
    for j, k in arc:
        if k < num_leg:
            # Constraint 1
            lhs = cplex.SparsePair(ind=[X[i][j][k], R[i][j][k],y[i][j][k]], val=[1.0, 1.0, -MTOTW])
            rhs = [0]
            model.linear_constraints.add(lin_expr=[lhs], senses=["L"], rhs=rhs)

            lhs = cplex.SparsePair(ind=[X[i][j][k], R[i][j][k], y[i][j][k]],
                                   val=[1.0, 1.0, -TANK])
            rhs = [0]
            model.linear_constraints.add(lin_expr=[lhs], senses=["L"], rhs=rhs)

            # Constraint 2
            lhs = [cplex.SparsePair(ind=[X[i][j][k], R[i][j][k],y[i][j][k]], val=[1.0 - a, 1.0 - a,-MLTW])]
            rhs = [compute_tripfuel(tripfuel_array, dep_s,arr_s,k,num_leg)*(1-a)]
            model.linear_constraints.add(lin_expr=lhs, senses=["L"], rhs=rhs)

            # Constraint 3
            lhs = [cplex.SparsePair(ind=[X[i][j][k], R[i][j][k],y[i][j][k]], val=[(1.0 - a), (1.0 - a),-(SF[k]+(1-a)*compute_tripfuel(tripfuel_array, dep_s, arr_s, k, num_leg))])]
            rhs = [0]
            model.linear_constraints.add(lin_expr=lhs, senses=["G"], rhs=rhs)

            # Constraint 4
            lhs = [cplex.SparsePair(ind=[X[i][j][k],y[i][j][k] ], val=[1.0,-TANK])]
            rhs = [0]
            model.linear_constraints.add(lin_expr=lhs, senses=["L"], rhs=rhs)

            lhs = [cplex.SparsePair(ind=[R[i][j][k], y[i][j][k]], val=[1.0, -TANK])]
            model.linear_constraints.add(lin_expr=lhs, senses=["L"], rhs=rhs)

            #balance equation
            temp = []
            for l in range(num_node):
                if [k, l] in arc:
                    temp.append([k, l])
            lhs=[cplex.SparsePair(ind=[X[i][j][k],R[i][j][k],y[i][j][k]]+[R[i][p][q] for p,q in temp], val=[1-a,1-a,-(1-a)*compute_tripfuel(tripfuel_array, dep_s, arr_s, k, num_leg)]+[-1]*len(temp))]
            rhs = [0]
            model.linear_constraints.add(lin_expr=lhs, senses=["L"], rhs=rhs)


            # Constraint 6
            lhs = [cplex.SparsePair(ind=[X[i][j][k], R[i][j][k],y[i][j][k]]+[R[i][p][q] for p,q in temp], val=[1.0 - a, 1.0 - a,-M]+[-1]*len(temp))]
            rhs = [(1-a)*compute_tripfuel(tripfuel_array, dep_s, arr_s, k, num_leg)-M]
            model.linear_constraints.add(lin_expr=lhs, senses=["G"], rhs=rhs)

# con10 : 시작할 때 tank는 비어 있다.
for i in range(num_aircraft):
    for j, k in arc:
        if j >= num_leg and j < num_leg + num_airport:
            # Constraint
            lhs = [cplex.SparsePair(ind=[R[i][j][k]], val=[1.0])]
            rhs = [0.0]
            model.linear_constraints.add(lin_expr=lhs, senses=["E"], rhs=rhs)



for i in range(num_aircraft):
    for j,k in arc:

        #model.objective.set_linear(y[i][j][k], cost[j][k])
        model.objective.set_linear(X[i][j][k],fuel[dep_s[k]-1])

num_threads = model.get_num_cores()
model.parameters.threads.set(num_threads)

model.parameters.benders.strategy = 2
model.cleanup(1e-6)
model.solve()

solution=model.solution
print("Solution status: ", solution.get_status())
print("Objective value: ", solution.get_objective_value())
no_tankering_cost=0
for l in range(num_leg):
    no_tankering_cost= no_tankering_cost + compute_tripfuel(tripfuel_array, dep_s,arr_s,l,num_leg) * fuel[dep_s[l]-1]

print("fuel cost with no tankering :",round(no_tankering_cost,2))
print("reduced rate :",round(100*(no_tankering_cost-solution.get_objective_value())/solution.get_objective_value(),2),"%")

results_y = []
results_X = []
results_R = []

# 모든 i, j, k에 대한 값 반복
for i in range(num_aircraft):
    for j, k in arc:
        value = solution.get_values(y[i][j][k])  # X 변수의 값 가져오기
        results_y.append([i, j, k, value])  # 결과 리스트에 [i, j, k, value] 추가
        value=solution.get_values(R[i][j][k])
        results_R.append(value)
        value = solution.get_values(X[i][j][k])
        results_X.append(value)

# 결과를 데이터프레임으로 변환
df = pd.DataFrame(results_y, columns=['i', 'j', 'k', 'value'])
df['fuel']=results_X
df['fuel']=df['fuel'].round(2)
df['remain']=results_R
df['remain']=df['remain'].round(2)
df.columns = ['aircraft', 'node_1', 'node_2', 'y', 'x', 'r']

df['uplifted fuel']=df['x']+df['r']
df['trip_fuel']=df.apply(lambda row:compute_tripfuel(tripfuel_array, dep_s,arr_s,row['node_2'],num_leg),axis=1)
df = df[(df['y'] == 1)]

#df['fuel consumption']=df['trip_fuel']+a*(df['x']+df['r']-df['trip_fuel'])
#df['fuel consumption']=df['fuel consumption'].round(2)

# 결과 확인
now = datetime.now()
file_name = f"Output_{now.strftime('%m%d')}_{'%02d' % now.hour}{'%02d' % now.minute}.xlsx"
df.to_excel(file_name)

# CPLEX 객체 종료
model.end()
