import json
import numpy as np
from scipy.optimize import linear_sum_assignment
import networkx as nx
from topo_transform import transform_data
from agv_path_planning_positionv6 import plan_path
from networkx import astar_path, NetworkXNoPath, NodeNotFound

#不可达能抛出异常,已完成版本，还差打印的优化

class UnreachablePathError(Exception):
    def __init__(self, agv_id, order_id):
        self.agv_id = agv_id
        self.order_id = order_id
        super().__init__(f"Order {order_id} is not reachable by AGV {agv_id}")

def optimize_agv_paths(data, waybills, agv_positions):
    try:
        G = nx.DiGraph()
        G.add_nodes_from(data["nodes"])
        for edge in data["edges"]:
            G.add_edge(edge["start"], edge["end"], weight=edge["cost"])

        # 打印图的信息
        print("Graph nodes:", G.nodes)
        print("Graph edges:", G.edges(data=True))

        orders = [{"orderId": waybill["orderId"], "siteList": [node["nodeId"] for node in waybill["siteList"]]} for
                  waybill
                  in waybills]

        # 打印订单信息
        print("Orders:", orders)

        # 计算成本矩阵
        cost_matrix = []
        unreachable_pairs = []  # 用于记录不可达的AGV和订单

        for order in orders:
            task_sequence = order["siteList"]
            order_costs = []

            for agv_id, agv_start in agv_positions.items():
                total_cost = 0

                try:
                    for i in range(len(task_sequence)):
                        if i == 0:
                            if agv_start not in G or task_sequence[i] not in G:
                                raise NodeNotFound
                            path = astar_path(G, agv_start, task_sequence[i])
                            length = nx.path_weight(G, path, weight='weight')
                        else:
                            if task_sequence[i - 1] not in G or task_sequence[i] not in G:
                                raise NodeNotFound
                            path = astar_path(G, task_sequence[i - 1], task_sequence[i])
                            length = nx.path_weight(G, path, weight='weight')
                        total_cost += length
                    order_costs.append(total_cost)
                except (NetworkXNoPath, NodeNotFound):
                    # 如果路径不可达，抛出异常并记录该AGV和订单
                    unreachable_pairs.append((agv_id, order["orderId"]))
                    raise UnreachablePathError(agv_id, order["orderId"])

            print(order_costs)
            cost_matrix.append(order_costs)

        # 打印计算结果
        for i, order_cost in enumerate(cost_matrix):
            print(f"Order {i + 1} costs: {order_cost}")

        # 使用匈牙利算法进行订单分配
        try:
            order_idx, agv_idx = linear_sum_assignment(cost_matrix)
            print("Optimal assignment:")
            for o, a in zip(order_idx, agv_idx):
                if cost_matrix[o][a] >= 1e6:
                    print(f"Order {orders[o]['orderId']} is not reachable by AGV {list(agv_positions.keys())[a]}")
                else:
                    print(f"Order {orders[o]['orderId']} assigned to AGV {list(agv_positions.keys())[a]} with cost {cost_matrix[o][a]}")
        except ValueError as e:
            print("Error in assignment:", e)

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
            result = plan_path(order, data, agv_positions)
            if result:
                order.update(result)

        # 返回订单分配和路径规划的结果
        return optimized_orders

    except UnreachablePathError as e:
        print(e)
        return None

if __name__ == '__main__':
    with open('topo_map_data.json', 'r') as file:
    # with open('topo_map_data_unreachable.json', 'r') as file:
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
    if optimized_orders:
        for order in optimized_orders:
            print(order)
