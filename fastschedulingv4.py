import json
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
import networkx as nx
from topo_transform import transform_data
# from agv_path_planning import plan_path
from agv_path_planning_positionv6 import plan_path


#间接到达完成版本

def optimize_agv_paths(data, waybills, agv_positions):
    # 创建有向图并添加边和节点
    G = nx.DiGraph()
    G.add_nodes_from(data["nodes"])
    for edge in data["edges"]:
        G.add_edge(edge["start"], edge["end"], weight=edge["cost"])

    # 打印图的信息
    print("Graph nodes:", G.nodes)
    print("Graph edges:", G.edges(data=True))

    orders = [{"orderId": waybill["orderId"], "siteList": [node["nodeId"] for node in waybill["siteList"]]} for waybill
              in waybills]

    # 打印订单信息
    print("Orders:", orders)

    # 计算成本矩阵
    cost_matrix = []#初始化一个空列表 cost_matrix，用于存储每个订单对每个AGV的总成本。
    for order in orders:
        task_sequence = order["siteList"]#遍历所有订单 orders。对于每个订单，提取其任务序列 task_sequence，即订单中需要访问的一系列站点。
        order_costs = []#初始化一个空列表 order_costs，用于存储当前订单对每个AGV的总成本。
        for agv_id, agv_start in agv_positions.items():
            total_cost = 0#遍历所有AGV的位置 agv_positions，其中agv_id是AGV的标识，agv_start是AGV的起始位置。初始化一个变量 total_cost 为0，用于累计当前AGV执行该订单的总成本。
            for i in range(len(task_sequence)):
                if i == 0:
                    length, _ = nx.single_source_dijkstra(G, source=agv_start, target=task_sequence[i])#遍历订单中的任务序列 task_sequence。对于任务序列中的第一个任务：使用Dijkstra算法计算从AGV的起始位置 agv_start 到任务序列中的第一个站点 task_sequence[i] 的最短路径长度 length。
                else:
                    length, _ = nx.single_source_dijkstra(G, source=task_sequence[i - 1], target=task_sequence[i])#对于任务序列中的后续任务:计算从上一个任务站点 task_sequence[i - 1] 到当前任务站点 task_sequence[i] 的最短路径长度 length。
                total_cost += length#将当前任务的路径长度 length 累加到 total_cost 中。
            order_costs.append(total_cost)#将当前AGV执行该订单的总成本 total_cost 添加到 order_costs 列表中。
        print(order_costs)
        cost_matrix.append(order_costs)#将当前订单对应的 order_costs 添加到 cost_matrix 中。


#这段代码的作用是为每个订单计算每个AGV的总成本，并将这些成本值存储在 cost_matrix 中。最终，cost_matrix 是一个二维列表，其中每一行对应一个订单，每一列对应一个AGV的成本。

    cost_matrix = np.array(cost_matrix)

    print(cost_matrix)

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
    # with open('topo_map_data.json', 'r') as file:
    with open('topo_map_data_unreachable.json', 'r') as file:
    # with open('topo_map_data_indirect.json', 'r') as file:
        data = json.load(file)
    data = transform_data(data)

    with open('order.json', 'r') as file:
    # with open('order_unreachable.json', 'r') as file:
    # with open('order_indirect.json', 'r') as file:
        order_dict = json.load(file)

    # 从字典中提取 AGV 位置信息
    agv_list = order_dict['agv_position']['agvList']
    start_positions = order_dict['agv_position']['startPosition']

    # 创建一个新的字典来存储 AGV 的位置信息
    agv_positions = {agv: position for agv, position in zip(agv_list, start_positions)}

    waybills = [
        {
            "orderId": "00001",
            "areaId": "Area001",
            "createTime": "2024/03/18 14:09",
            "siteList": [
                {"nodeId": "LM3", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM4", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        },
        {
            "orderId": "00002",
            "areaId": "Area001",
            "createTime": "2024/03/18 14:09",
            "siteList": [
                {"nodeId": "LM5", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM4", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        }
    ]

    optimized_orders = optimize_agv_paths(data, waybills, agv_positions)
    for order in optimized_orders:
        print(order)
