import networkx as nx
import json
from typing import Dict, List, Any
from networkx import astar_path
from topo_transform import transform_data

class GraphData:
    def __init__(self, graph_data: Dict[str, Any]):
        self.graph_data = graph_data
        self.nodes = []
        self.edges = []
        self.load_graph_data()

    def load_graph_data(self):
        try:
            self.nodes = self.graph_data['nodes']
            self.edges = self.graph_data['edges']
        except KeyError as e:
            raise ValueError(f"Invalid graph data: missing key {e}")

def get_graph_data(graph_data: Dict[str, Any]) -> GraphData:
    return GraphData(graph_data)

def plan_path(orderid_data: Dict[str, Any], graph_data: Dict[str, Any]) -> str:

    """
    根据提供的 orderid 数据字典，使用 NetworkX 库进行路径规划，并返回路径规划结果。
    """
    agv_id = orderid_data["agvId"]
    target_list = orderid_data["targetList"]

    if len(target_list) < 2:
        raise ValueError(f"目标列表至少需要两个节点 ({agv_id})")

    graph_data_obj = get_graph_data(graph_data)

    # 创建有向图
    G = nx.DiGraph()
    G.add_nodes_from(graph_data_obj.nodes)
    G.add_weighted_edges_from([(edge['start'], edge['end'], edge['cost']) for edge in graph_data_obj.edges])

    # 检查目标列表是否包含有效的节点
    if not all(node in graph_data_obj.nodes for node in target_list):
        raise ValueError(f"目标列表中的节点不在节点列表中 ({agv_id})")

    # 使用 A* 算法规划路径
    path = astar_path(G, target_list[0], target_list[-1], weight="cost")  # 确保与图数据中的键名一致

    # 构建输出字典
    result_dict = {
        "orderId": orderid_data["orderId"],
        "areaId": orderid_data.get("areaId"),  # 新增字段，如果输入数据中没有则为 None
        "missionKey": orderid_data["missionKey"],
        "agvId": agv_id,
        "targetList": target_list,
        "siteList": list(map(str, path)),  # 假设输入数据中已有 siteList 字段
        "segmentList": None,  # 路径规划结果，将路径节点转换为字符串列表
    }

    return result_dict
    # return f"Path for AGV {agv_id}: {' -> '.join(path)}"

# # # 示例
# order_data_example = {
#     "orderId": "00001",
#     "areaId": "Area001",
#     "missionKey": 1,
#     "agvId": "Agv001",
#     "targetList": ['CP3', 'LM1'],
# }

# # 读取JSON文件
# with open('../HTTP/topo_map_data.json', 'r') as file:
#     data = json.load(file)
# transformed_data = transform_data(data)
# print(transformed_data)
# #
# # # 定义图数据
# # data = {
# #   "nodes": [
# #     "siteA",
# #     "siteB",
# #     "siteC",
# #     "siteD",
# #     "siteE",
# #     "siteF",
# #     "siteG"
# #   ],
# #   "edges": [
# #     {"start": "siteA", "end": "siteD", "cost": 1.2},
# #     {"start": "siteA", "end": "siteC", "cost": 1.2},
# #     {"start": "siteA", "end": "siteE", "cost": 9.1},
# #     {"start": "siteA", "end": "siteG", "cost": 0.5},
# #     {"start": "siteB", "end": "siteD", "cost": 5},
# #     {"start": "siteB", "end": "siteF", "cost": 3.1},
# #     {"start": "siteB", "end": "siteG", "cost": 2},
# #     {"start": "siteC", "end": "siteF", "cost": 4},
# #     {"start": "siteC", "end": "siteG", "cost": 1.5},
# #     {"start": "siteD", "end": "siteA", "cost": 1.2},
# #     {"start": "siteD", "end": "siteB", "cost": 5},
# #     {"start": "siteD", "end": "siteE", "cost": 6.7},
# #     {"start": "siteE", "end": "siteA", "cost": 9.2},
# #     {"start": "siteE", "end": "siteD", "cost": 6.7},
# #     {"start": "siteE", "end": "siteF", "cost": 15.6},
# #     {"start": "siteF", "end": "siteB", "cost": 3.1},
# #     {"start": "siteF", "end": "siteC", "cost": 4},
# #     {"start": "siteF", "end": "siteE", "cost": 15.6},
# #     {"start": "siteG", "end": "siteA", "cost": 0.5},
# #     {"start": "siteG", "end": "siteB", "cost": 2},
# #     {"start": "siteG", "end": "siteC", "cost": 1.5}
# #   ]
# # }
# #
# result = plan_path(order_data_example, transformed_data)
# print(result)
