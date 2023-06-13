import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

'''
1.generate_connection : 주어진 flight leg에 대해 turn time을 고려하여 feasible connection 생성
2.compute_BOF : 각 flight leg 별 연료 소모량 계산
3.gen_optimal_route : optimal solution에 포함된 leg를 시간 순으로 sequential하게 정렬
4.print_optimal_solution : optimal solution을 각 aircraft 별로 sequential하게 정렬해서 엑셀로 출력 
5.net_pos : leg 개수와 공항 개수에 따라 시각화할 때 노드 position 결정
6.visu_given_connection : input으로 주어진 네트워크 시각화
7.visu_optimal_route : 각 항공기 별 optimal route 시각화
'''

def generate_connection(num_leg,num_airport, c, dep_t,arr_t,dep_s,arr_s,turn, sit, penalty,maintenance, m_time,horizon):
    arc=[]
    m_arc = []
    p_arc=[]
    for j in range(num_leg):
        for k in range(num_leg):
            if arr_s[j]==dep_s[k]:
                if arr_t[j] + turn <= dep_t[k]:
                    arc.append([j,k])
                    if arr_t[j]+ sit > dep_t[k]:
                        p_arc.append([j,k])
                        c[j][k] = penalty
                    if arr_t[j] + m_time <= dep_t[k] and arr_s[j] in maintenance:
                        m_arc.append([j,k])


    for j in range(num_leg):
        arc.append([num_leg+dep_s[j]-1,j])
        arc.append([j,num_leg+num_airport+arr_s[j]-1])
        if dep_t[j]>=m_time and dep_s[j] in maintenance:
            m_arc.append([num_leg+dep_s[j]-1,j])
        if horizon-arr_t[j]>=m_time and arr_s[j] in maintenance:
            m_arc.append([j, num_leg + num_airport + arr_s[j] - 1])

    nm_arc = [value for value in m_arc if value not in m_arc]

    return c, arc, m_arc, nm_arc, p_arc

def compute_tripfuel(tripfuel_array, dep_s,arr_s,leg,num_leg):
    leg=int(leg)
    if leg>=num_leg:
        return 0
    else:
        if dep_s[leg]<arr_s[leg]:
            return tripfuel_array[dep_s[leg] - 1][arr_s[leg] - 1]
        else :
            return tripfuel_array[arr_s[leg]-1][dep_s[leg]-1]

def gen_optimal_route(opt_output, num_leg,num_aircraft,num_airport):
    optimal_route = [[]] * num_aircraft
    opt_output = opt_output.values.tolist()
    for i in range(num_aircraft):
        for j in range(len(opt_output)):
            if (opt_output[j][0] == i):
                if (opt_output[j][1] >= num_leg):
                    optimal_route[i] = opt_output[j][1]
                    optimal_route[i] = np.append(optimal_route[i], np.array([opt_output[j][2]]))

    for i in range(num_aircraft):
        if len(optimal_route[i]) > 0:
            while (optimal_route[i][0] + num_airport) not in optimal_route[i]:
                for k in range(len(opt_output)):
                    if optimal_route[i][len(optimal_route[i]) - 1] == opt_output[k][1]:
                        optimal_route[i] = np.append(optimal_route[i], np.array([opt_output[k][2]]))

    return optimal_route

def print_optimal_solution(sol,y,X,R,num_leg,num_aircraft,num_airport):
    opt_routing = sol.get_value_df(y).round(0)
    opt_fuel = sol.get_value_df(X)
    opt_remain = sol.get_value_df(R)

    opt_output = pd.concat([opt_routing,  opt_fuel['value'], opt_remain['value']], axis=1)
    opt_output.columns = ['aircraft', 'node_1', 'node_2', 'y', 'x', 'r']



    opt_output = opt_output[(opt_output['y'] == 1)]

    optimal_route = gen_optimal_route(opt_output, num_leg, num_aircraft, num_airport)

    # optimal route를 순서대로 정렬해서 데이터프레임에 저장
    opt_output['sorting'] = 0
    for i in range(num_aircraft):
        for j in range(len(optimal_route[i])):
            for k in range(len(optimal_route[i]) - 1):
                if optimal_route[i][j] == opt_output.iloc[k + sum(np.max([len(optimal_route[l]) - 1, 0]) for l in range(i)), 1]:
                    opt_output.iloc[k + sum(np.max([len(optimal_route[l]) - 1, 0]) for l in range(i)), 7] = j + sum(
                        np.max([len(optimal_route[l]) - 1, 0]) for l in range(i))

    opt_output = opt_output.sort_values(by='sorting', ascending=True)
    opt_output = opt_output.drop('sorting', axis=1)

    opt_output.to_excel('optimal solution.xlsx')
    return optimal_route, opt_output

def net_pos(num_leg,num_airport):
    pos = {}
    for i in range(num_leg):
        if i % 2 == 0:
            pos[i] = (0.1, (num_leg - i) * 0.1)
        else:
            pos[i] = (-0.1, (num_leg - i) * 0.1)
    for i in range(num_airport):
        pos[num_leg + i] = [-1, (num_airport - i) * 0.1 * num_leg / num_airport]
        pos[num_leg + num_airport + i] = [1, (num_airport - i) * 0.1 * num_leg / num_airport]

    return pos

def visu_given_connection(arc,num_leg, num_airport):
    pos = net_pos(num_leg, num_airport)
    G_CN = nx.DiGraph(directed=True)
    for i in range(len(arc)):
        G_CN.add_edge(arc[i][0], arc[i][1])


    options1 = {
        "font_size": 6,
        "node_size": 300,
        "node_color": "white",
        "edgecolors": "black",
        "linewidths": 2,
        "width": 2,
        'arrowstyle': '-|>',
        'arrowsize': 5,
    }

    nx.draw_networkx(G_CN, pos, arrows=True, **options1)
    ax = plt.gca()
    ax.set_title('Given connection network')
    ax.margins(0.20)
    plt.axis("off")
    plt.show()

def visu_optimal_route(optimal_route, opt_output, num_leg, num_airport, num_aircraft):
    pos = net_pos(num_leg, num_airport)
    G_CN_opt = nx.MultiDiGraph(directed=True)
    edge_colors = [[]] * num_aircraft
    clrs = []
    for i in range(num_aircraft):
        temp_color = np.random.rand(3)
        clrs.append(temp_color)
        if len(optimal_route[i]) > 0:
            for j in range(len(optimal_route[i]) - 1):
                G_CN_opt.add_edge(optimal_route[i][j].astype(int), optimal_route[i][j + 1].astype(int),relation=i)
                edge_colors[i].append(temp_color)

    # labels=nx.get_edge_attributes(G_CN_opt,'weight')
    nx.draw_networkx_labels(G_CN_opt, pos, font_size=7)
    nx.draw_networkx_nodes(G_CN_opt, pos, node_size=300, node_color='white', linewidths=2, edgecolors='black')

    for node1, node2, index in G_CN_opt.edges:
        for i in range(num_aircraft):
            temp = opt_output.index[(opt_output['aircraft'] == i) & (opt_output['node_1'] == node1) & (
                        opt_output['node_2'] == node2)].tolist()
            x1, y1 = pos[node1]

            if len(temp) == 1:
                x2, y2 = pos[node2]

                plt.annotate(
                    "",
                    (x2, y2),
                    xytext=(x1, y1),
                    arrowprops=dict(
                        arrowstyle="->",
                        color=clrs[i],
                        shrinkA=10,
                        shrinkB=10
                    )
                )

            '''temp_x=sol.get_value(X[i,node1,node2])
            plt.text((x1+x2)*2/3, (y1+y2)*2/3,
                     int(temp_x),
                     verticalalignment='center',
                     horizontalalignment='center',
                     fontsize=5
                     )'''

    labels = ["aircraft {}".format(i) for (i) in range(num_aircraft)]

    legend_elements = []
    for i in range(len(clrs)):
        legend_elements.append(Line2D([0, 1], [0, 1], color=clrs[i]))

    plt.legend(legend_elements, labels)

    ax = plt.gca()
    ax.set_title('Optimal route')
    ax.margins(0.20)
    plt.axis("off")

    plt.show()

#반영 1 : 일반 arc와 maintenance arc의 색/형태 구분