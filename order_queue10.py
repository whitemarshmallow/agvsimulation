import json
import threading
import queue
import time
import random
import networkx as nx
import numpy as np
from scipy.optimize import linear_sum_assignment
from agv import AGV  # 导入AGV类
from fastschedulingv3 import optimize_agv_paths

#修改订单格式与前后端匹配，高新宇本地调测版本

#testgit
# 生产者函数
def producer(order_queue, orders):
    #如果订单
    for order in orders:
        print(f"Producing {order['orderId']}")  # 打印正在生成的订单ID
        order_queue.put(order)  # 将订单放入队列中
        # time.sleep(random.uniform(0.1, 0.5))  # 模拟生产订单的时间间隔，随机等待0.1到0.5秒
    print("All orders have been produced")  # 所有订单生产完成后打印消息

# 消费者函数
def consumer(order_queue, data, agvs, result_list):
    while True:
        # 过滤出可用的AGV
        available_agv_positions = {agv.agv_id: agv.get_position() for agv in agvs if agv.status == 0}
        num_available_agvs = len(available_agv_positions)

        if num_available_agvs == 0:
            print("No available AGVs. Waiting for AGVs to become available.")
            time.sleep(1)  # 等待一段时间后重试
            continue

        #返回任务是正确的，但是没有空闲的车

        waybills = []
        for _ in range(num_available_agvs):
            order = order_queue.get()
            if order is None:
                order_queue.put(None)  # 重新放回终止信号，以便其他消费者能检测到
                break
            waybills.append(order)
            order_queue.task_done()

        if waybills:
            optimized_orders = optimize_agv_paths(data, waybills, available_agv_positions)#及时post
            result_list.extend(optimized_orders)

        if order is None:
            break

    print("Consumer has finished processing orders")

# 封装的AGV调度系统函数
def agv_scheduling_system(data, orders, order_queue,agvs):
    # order_queue = queue.Queue()  # 创建一个订单队列
    result_list = []  # 初始化结果列表

    #检查orders是否是空，如果是空则post

    producer_thread = threading.Thread(target=producer, args=(order_queue, orders))  # 创建生产者线程
    consumer_thread = threading.Thread(target=consumer, args=(order_queue, data, agvs, result_list))  # 创建消费者线程

    producer_thread.start()  # 启动生产者线程
    consumer_thread.start()  # 启动消费者线程

    producer_thread.join()  # 等待生产者线程完成
    order_queue.put(None)  # 向队列中添加终止信号，通知消费者退出
    consumer_thread.join()  # 等待消费者线程结束

    print("All processing is complete")  # 打印消息，表示所有处理完成
    return result_list  # 返回结果列表

if __name__ == "__main__":
    # 模拟输入数据
    data = {
        "nodes": [f"Node-{i}" for i in range(1, 11)],
        "edges": [{"start": f"Node-{i}", "end": f"Node-{j}", "cost": random.randint(1, 10)}
                  for i in range(1, 11) for j in range(1, 11) if i != j]
    }
    # data= {
    #     "nodes": ["LM1", "LM2", "LM3"],
    #     "edges": [
    #         {"start": "LM1", "end": "LM2", "cost": 1.0},
    #         {"start": "LM2", "end": "LM3", "cost": 1.0},
    #         {"start": "LM3", "end": "LM1", "cost": 1.0}
    #     ]
    # }

    orders = []
    for i in range(10):
        order_id = f"Order-{i + 1}"
        site_list = []
        num_sites = random.randint(2, 5)

        start_node = f"Node-{random.randint(1, 10)}"
        site_list.append({"nodeId": start_node, "postionX": random.uniform(100, 200), "postionY": random.uniform(100, 200), "nodeTime": 0})

        for _ in range(1, num_sites - 1):
            node_id = f"Node-{random.randint(1, 10)}"
            site_list.append({"nodeId": node_id, "postionX": random.uniform(100, 200), "postionY": random.uniform(100, 200), "nodeTime": random.randint(1, 20)})

        while True:
            end_node = f"Node-{random.randint(1, 10)}"
            if end_node != start_node:
                site_list.append({"nodeId": end_node, "postionX": random.uniform(100, 200), "postionY": random.uniform(100, 200), "nodeTime": random.randint(1, 20)})
                break

        area_id = f"Area-{random.randint(1, 3)}"
        agv_list = [f"AGV-{i}" for i in range(1, random.randint(2, 5))]
        start_positions = [f"Node-{random.randint(1, 10)}" for _ in agv_list]

        order = {
            "orderId": order_id,
            "siteList": site_list,
            "areaId": area_id,
            "agv_position": {
                "agvList": agv_list,
                "startPosition": start_positions
            }
        }
        orders.append(order)

    agvs = [
        AGV(f"AGV-{i}", f"Node-{random.randint(1, 10)}", 0)
        for i in range(1, 5)
    ]

    # 调用调度系统
    results = agv_scheduling_system(data, orders, agvs)
    print(f"Scheduling Results: {results}")

    # 将结果保存为JSON文件
    with open('jiaotongtest.json', 'w') as json_file:
        json.dump(results, json_file, indent=4)
