import networkx as nx
import json
from typing import Dict, Any
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


def plan_path(orderid_data: Dict[str, Any], graph_data: Dict[str, Any], agv_positions: Dict[str, str]) -> str:
    agv_id = orderid_data["agvId"]
    target_list = orderid_data["targetList"]
    original_target_list = target_list.copy()

    if not target_list:
        raise ValueError(f"目标列表不能为空 ({agv_id})")

    graph_data_obj = get_graph_data(graph_data)

    # 获取AGV当前位置
    agv_current_position = agv_positions.get(agv_id)
    if agv_current_position is None or agv_current_position not in graph_data_obj.nodes:
        raise ValueError(f"无法找到AGV({agv_id})的当前位置({agv_current_position})在节点列表中")

    # 创建有向图
    G = nx.DiGraph()
    G.add_nodes_from(graph_data_obj.nodes)
    G.add_weighted_edges_from([(edge['start'], edge['end'], edge['cost']) for edge in graph_data_obj.edges])

    target_list.insert(0, agv_current_position)
    full_path = [target_list[0]]  # 初始化路径为AGV当前位置
    for i in range(1, len(target_list)):
        next_point = target_list[i]
        next_path = astar_path(G, full_path[-1], next_point, weight="cost")
        if not next_path:
            raise ValueError(f"无法规划从{full_path[-1]}到{next_point}的路径")
        full_path.extend(next_path[1:])  # 添加除了当前节点外的所有节点

    # 构建输出字典并转换为JSON字符串
    result_dict = {
        "orderId": orderid_data["orderId"],
        "areaId": orderid_data.get("areaId"),
        "missionKey": orderid_data["missionKey"],
        "agvId": agv_id,
        "targetList": original_target_list,
        "siteList": list(map(str, full_path)),
    }

    return result_dict


if __name__ == "__main__":
    order_data_example = {
        "orderId": "00001",
        "areaId": "Area001",
        "missionKey": 1,
        "agvId": "Agv001",
        "targetList": ['LM6', 'LM4'],
    }

    # 读取JSON文件
    with open('topo_map_data.json', 'r') as file:
        data = json.load(file)
    transformed_data = transform_data(data)

    agv_positions = {
        "Agv001": "LM1",
        "Agv002": "LM4",
        "Agv003": "LM1",
        "Agv004": "LM2"
    }

    # 调用路径规划函数
    result = plan_path(order_data_example, transformed_data, agv_positions)
    print(result)


