import json
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
import networkx as nx
from topo_transform import transform_data
# from agv_path_planning import plan_path
from agv_path_planning_positionv6 import plan_path

def optimize_agv_paths(data, waybills, agv_positions):
    # 创建有向图并添加边和节点
    G = nx.DiGraph()
    G.add_nodes_from(data["nodes"])
    for edge in data["edges"]:
        G.add_edge(edge["start"], edge["end"], weight=edge["cost"])

    orders = [{"orderId": waybill["orderId"], "siteList": [node["nodeId"] for node in waybill["siteList"]]} for waybill
              in waybills]

    # 计算成本矩阵
    cost_matrix = []
    for order in orders:
        task_sequence = order["siteList"]
        order_costs = []
        for agv_id, agv_start in agv_positions.items():
            total_cost = 0
            for i in range(len(task_sequence)):
                if i == 0:
                    length, _ = nx.single_source_dijkstra(G, source=agv_start, target=task_sequence[i])
                else:
                    length, _ = nx.single_source_dijkstra(G, source=task_sequence[i - 1], target=task_sequence[i])
                total_cost += length
            order_costs.append(total_cost)
        cost_matrix.append(order_costs)

    cost_matrix = np.array(cost_matrix)
    order_idx, agv_idx = linear_sum_assignment(cost_matrix)

    # 构建最终的任务数据格式
    optimized_orders = []
    for o, a in zip(order_idx, agv_idx):
        assignment = {
            "orderId": orders[o]["orderId"],
            "areaId": waybills[o]["areaId"],
            "missionKey": 1,
            "agvId": list(agv_positions.keys())[a],
            "targetList": orders[o]["siteList"]
        }
        optimized_orders.append(assignment)
    # 对每个订单执行路径规划并更新结果
    for order in optimized_orders:
        # result = plan_path(order, data)
        result = plan_path(order, data, agv_positions)
        if result:
            order.update(result)

    # 返回订单分配和路径规划的结果
    return optimized_orders


def demo(RG1,ARG2,ARG3):

    return optimized_orders

if __name__ == '__main__':
    with open('topo_map_data.json', 'r') as file:
        data = json.load(file)
    data = transform_data(data)


    with open('order.json', 'r') as file:
        order_dict = json.load(file)


    # 从字典中提取 AGV 位置信息
    agv_list = order_dict['agv_position']['agvList']
    start_positions = order_dict['agv_position']['startPosition']

    # 创建一个新的字典来存储 AGV 的位置信息
    agv_positions = {}
    for agv, position in zip(agv_list, start_positions):
        agv_positions[agv] = position

#     agv_positions = {
#         # "Agv001": "LM7",
#         "Agv002": "LM7"
# }

    # waybills = [order_data]
    waybills = [
        {
            "orderId": "00001",
            "areaId": "Area001",
            "createTime": "2024/03/18 14:09",
            "siteList": [
                {"nodeId": "LM3", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM2", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        },

        {
            "orderId": "00002",
            "areaId": "Area001",
            "createTime": "2024/03/18 14:09",
            "siteList": [
                {"nodeId": "LM1", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM2", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        }

    ]
    optimized_orders = optimize_agv_paths(data, waybills, agv_positions)
    for order in optimized_orders:
        print(order)
