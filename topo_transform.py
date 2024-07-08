import json

# 读取JSON文件
with open('topo_map_data.json', 'r') as file:
    data = json.load(file)

def transform_data(json_data):
    # 提取nodes的id
    nodes = [node["instanceName"] for node in json_data["nodeList"]]

    # 提取lines的信息并转换
    links = []
    for line in json_data["lineList"]:
        links.append({
            "start": line["lineSource"],
            "end": line["lineTarget"],
            "cost": line["lineLength"]
        })

    # 构造data
    data = {
        "nodes": nodes,
        "edges": links
    }

    return data

# # 转换数据
# transformed_data = transform_data(data)
# # 打印转换后的数据以供检查
# print(json.dumps(transformed_data, indent=4))
