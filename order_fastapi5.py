from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import threading
import queue
import time
import random
import networkx as nx
import numpy as np
from scipy.optimize import linear_sum_assignment
from agv import AGV  # 导入AGV类
import json
from order_queue10 import agv_scheduling_system

#高新宇postman成功

app = FastAPI()

class Site(BaseModel):
    nodeId: str
    postionX: float
    postionY: float
    nodeTime: int

class AGVPosition(BaseModel):
    agvList: List[str]
    startPosition: List[str]

class Order(BaseModel):
    orderId: str
    areaId: str
    siteList: List[Site]
    agv_position: AGVPosition


@app.post("/process_order")
async def process_order(order: Order):
    # 模拟输入数据

    #固定拓扑数据
    data = {
        "nodes": ["LM1", "LM2", "LM3"],
        "edges": [
            {"start": "LM1", "end": "LM2", "cost": 1.0},
            {"start": "LM2", "end": "LM3", "cost": 1.0},
            {"start": "LM3", "end": "LM1", "cost": 1.0}
        ]
    }

    agvs = [
        AGV(agv, start_node, 0)
        for agv, start_node in zip(order.agv_position.agvList, order.agv_position.startPosition)
    ]

    # 调用调度系统

    order_queue = queue.Queue()#队列放到主函数中

    results = agv_scheduling_system(data, [order.dict()], order_queue, agvs)
    print(f"Scheduling Results: {results}")

    # 将结果保存为JSON文件
    with open('jiaotongtest.json', 'w') as json_file:
        json.dump(results, json_file, indent=4)

    return {"status": "Order processed", "results": results}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
