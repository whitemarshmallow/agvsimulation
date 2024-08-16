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
#5.3.2更新，订单返回的是原始的订单对象，不是路径规划和任务分配的对象
#5.3.3更新，增加check_order_is_executable()可调用的函数

def optimize_agv_paths(graphdata, waybills, agv_positions):#data是拓扑图信息,waybills是订单信息，包括订单ID，在哪个区域以及要去的起始点和终点，agvposition是agv的位置
    # 创建有向图并添加边和节点
    G = nx.DiGraph()
    G.add_nodes_from(graphdata["nodes"])
    for edge in graphdata["edges"]:
        G.add_edge(edge["start"], edge["end"], weight=edge["cost"])

    # 打印图的信息
    print("Graph nodes:", G.nodes)
    print("Graph edges:", G.edges(data=True))

    # 构建基本订单信息
    orders = [{"orderId": waybill["orderId"], "siteList": [node["nodeId"] for node in waybill["siteList"]]} for waybill in waybills]
    print("Orders:", orders)

    #看看车的位置长什么样
    print("agv_positions",agv_positions)#是具备AGV名字和位置的字典

    # 计算成本矩阵
    cost_matrix = []
    unreachable_pairs = []  # 用于记录不可达的AGV和订单
    unreachable_order_ids = []  # 记录所有不可达的订单ID

    for order in orders:#先对订单做循环,获得每个订单针对所有车辆的成本信息，然后把结果汇总到cost_matrix上，作为匈牙利算法的输出
        task_sequence = order["siteList"]#找到订单要从哪到哪去
        order_costs = []
        all_unreachable = True

        for agv_id, agv_start in agv_positions.items():#针对每个订单，对AGV做循环
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

        print("当前订单针对每辆车的成本")
        print(order_costs)

        cost_matrix.append(order_costs)
        if all_unreachable:
            unreachable_order_ids.append(order["orderId"])  # 仅记录ID

    print("打印获得的整体成本矩阵")
    print(cost_matrix)
    # 打印计算结果
    for i, order_cost in enumerate(cost_matrix):
        print(f"Order {i + 1} costs: {order_cost}")

    #成本矩阵计算完毕

    # 使用匈牙利算法进行订单分配
    try:
        print("使用匈牙利算法进行分配")
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

    # 使用订单ID从waybills中提取未分配的完整订单对象
    unassigned_orders = [waybill for waybill in waybills if waybill["orderId"] in unreachable_order_ids]

    # 返回订单分配和路径规划的结果，以及未分配的完整订单对象列表
    return optimized_orders, unassigned_orders

def check_order_is_executable(graphdata, order, agv_positions):
    """
    根据订单信息、AGV位置和图数据判断订单是否可执行。
    :param graphdata: 包含节点和边的图数据
    :param order: 当前订单信息，包括siteList
    :param agv_positions: AGV位置的字典，键为AGV ID，值为位置
    :return: 如果有AGV可以执行该订单，返回 True；否则返回 False
    """
    # 创建图数据
    G = nx.DiGraph()
    G.add_nodes_from(graphdata["nodes"])
    for edge in graphdata["edges"]:
        G.add_edge(edge["start"], edge["end"], weight=edge["cost"])

    task_sequence = order["siteList"]  # 找到订单要从哪到哪去

    # 遍历所有AGV，判断是否有AGV能执行该订单
    for agv_start in agv_positions.values():
        try:
            for i in range(len(task_sequence)):
                start = agv_start if i == 0 else task_sequence[i - 1]
                end = task_sequence[i]

                # 检查起点和终点是否存在于图中，并计算路径
                if start not in G or end not in G:
                    raise nx.NetworkXNoPath
                path = nx.astar_path(G, start, end)
                length = nx.path_weight(G, path, weight='weight')

            return True  # 如果路径存在，表示订单可执行
        except (NetworkXNoPath, nx.NodeNotFound):
            continue  # 尝试下一个AGV

    return False  # 如果所有AGV都不可达，返回 False

if __name__ == '__main__':
    # with open('topo_map_data.json', 'r') as file:
    with open('topo_map_data_unreachable.json', 'r') as file:
    # with open('topo_map_data_indirect.json', 'r') as file:
        data = json.load(file)
    data = transform_data(data)#transformer去掉data中所有无关的数据，只留下了点和边以及边的代价信息，具备了基本构成图的条件

    print("data transform之后是\n")
    print(data)
    print("transformer完毕")

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