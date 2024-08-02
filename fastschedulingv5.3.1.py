import json
import numpy as np
from scipy.optimize import linear_sum_assignment
import networkx as nx
from topo_transform import transform_data
from agv_path_planning_positionv6 import plan_path
from networkx import astar_path, NetworkXNoPath, NodeNotFound

#在这个版本中，如果某条路径不可达，将成本设置为一个非常大的数值（例如1e6），确保匈牙利算法不会选择这些不可达的AGV进行匹配。
#之后，打印所有不可达的AGV和订单对的信息，并继续对可达的AGV和订单进行匹配。最终返回和打印的结果将只包含可达的订单和AGV匹配对。

#5.3更新：订单返回一个值，如果拿取的订单和所有车都不匹配，那么把这个值保存
#5.3.1更新，订单返回的不再是ID值，而是不能分配的整个对象


def optimize_agv_paths(data, waybills, agv_positions):
    # 创建有向图并添加边和节点
    G = nx.DiGraph()
    G.add_nodes_from(data["nodes"])
    for edge in data["edges"]:
        G.add_edge(edge["start"], edge["end"], weight=edge["cost"])

    # 打印图的信息
    print("Graph nodes:", G.nodes)
    print("Graph edges:", G.edges(data=True))

    orders = [{"orderId": waybill["orderId"], "siteList": [node["nodeId"] for node in waybill["siteList"]]} for waybill in waybills]

    # 打印订单信息
    print("Orders:", orders)

    # 计算成本矩阵
    cost_matrix = []
    unreachable_pairs = []  # 用于记录不可达的AGV和订单
    unassigned_orders = []  # 记录无法进行分配的完整订单对象

    for order in orders:
        task_sequence = order["siteList"]
        order_costs = []
        all_unreachable = True

        for agv_id, agv_start in agv_positions.items():
            total_cost = 0
            unreachable = False

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
                all_unreachable = False
            except (NetworkXNoPath, NodeNotFound):
                order_costs.append(1e6)  # 用一个大的数值表示不可达
                unreachable_pairs.append((agv_id, order["orderId"]))

        cost_matrix.append(order_costs)
        if all_unreachable:
            unassigned_orders.append(order)  # 如果所有路径都不可达，记录此订单对象

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
        if cost_matrix[o][a] < 1e6:  # 仅添加可达的订单
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
        print("")
        if result:
            order.update(result)

    # 打印不可达的AGV和订单
    if unreachable_pairs:
        print("Unreachable pairs:")
        for agv_id, order_id in unreachable_pairs:
            print(f"Order {order_id} is not reachable by AGV {agv_id}")

    # 返回订单分配和路径规划的结果，以及未分配的完整订单对象列表
    return optimized_orders, unassigned_orders



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
                {"nodeId": "LM1", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM2", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        },
        {
            "orderId": "00002",
            "areaId": "Area001",
            "createTime": "2024/03/18 14:09",
            "siteList": [
                {"nodeId": "LM2", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM1", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        },
        {
            "orderId": "00003",
            "areaId": "Area001",
            "createTime": "2024/03/18 14:09",
            "siteList": [
                {"nodeId": "LM3", "postionX": 190.33, "postionY": 270.2, "nodeTime": 0},
                {"nodeId": "LM4", "postionX": 190.33, "postionY": 270.2, "nodeTime": 12}
            ]
        }

    ]

    optimized_orders,unassigned_orders= optimize_agv_paths(data, waybills, agv_positions)


    print("最终输出结果打印")
    if optimized_orders:
        for order in optimized_orders:
            print(order)


    if unassigned_orders:
        for order in unassigned_orders:
            print(order)