import eventlet
import eventlet.wsgi
import socketio
import queue
import json
import pdb
import os

import globals
import utils
import simScript
from order_allocation_and_path_planning.fastscheduling import optimize_agv_paths

save_file_path = "simData"
resource_save_file = utils.append_timestamp_to_filename("initJavaData.json")
order_save_file = utils.append_timestamp_to_filename("orderJavaData.json")

# 仿真脚本作为函数直接调用，运行之后一直有数据发送，死循环，注意socket_flag = False
sio = socketio.Server(async_mode='eventlet', cors_allowed_origins='*')
app = socketio.WSGIApp(sio)

# 创建队列
# 存放仿真脚本生成的AGV动态数据, (线程process_data)调用交管算法处理，将处理后的数据放入send_queue
agv_dynamic_data_queue = queue.Queue(10000)
#  (线程send_data)按照约定数据格式处理后发送给拓扑图，再调用仿真脚本生成下一份AGV数据放入agv_dynamic_queue，形成闭环
send_queue = queue.Queue(10000)

# socketio监听订单下发，存储到order_queue，（线程process_order)调用任务分配和路径规划算法处理后放到order_res_queue,
# （线程monitor_order）监听order_res_queue,有数据则同步调用仿真脚本生成新数据
order_queue = queue.Queue(10000)
order_res_queue = queue.Queue(10000)

# 进程池管理：分开管理的原因是如果地图和AGV初始位置发生变化，所有线程需要重启，如果订单更新，只需要重启 pool1 的处理数据和发送数据的线程
# pool_1：process_data, send_data
pool_1 = eventlet.GreenPool(5)
# pool_2: monitor_order, process_order
pool_2 = eventlet.GreenPool(5)

# 控制所有线程执行标志位，当资源更新时，需要重启所有线程
running_flag = True
# 控制订单更新线程执行标志位
order_update_flag = False
socket_flag = False

# 全局变量，保存地图数据和AGV初始位置，只经过json.load()
GRAPHDATA = None
AGV_INIT_STATIC_DATA = None


def process_agv_data():
    global running_flag
    while running_flag and not order_update_flag:
        if not agv_dynamic_data_queue.empty():
            agv_dynamic_data = agv_dynamic_data_queue.get()
            # 交管处理数据，生成调度指令
            cmds = utils.traffic_control(agv_dynamic_data)
            send_queue.put((agv_dynamic_data, cmds))
        eventlet.sleep(globals.UPDATE_TIME_INTERVAL)  # 使用 gevent.sleep() 代替 sio.sleep()


def send_agv_data():
    global running_flag
    while running_flag and not order_update_flag:
        if not send_queue.empty():
            data = send_queue.get()
            agv_dynamic_data = data[0]
            cmds = data[1]
            # 按照约定数据格式生成拓扑图所需 AGV 数据
            topo_result = utils.generate_agv_display_result(agv_dynamic_data, cmds)
            # topo_result_json = json.dumps(topo_result)
            sio.emit('response', topo_result, namespace='/topo')
            # sio.emit('simScriptRes', sim_result, namespace='/simScript')
            agv_new_data = simScript.update_agv(agv_dynamic_data, cmds,
                                                update_time_interval=globals.UPDATE_TIME_INTERVAL,
                                                edges_dict=globals.EDGE_INFO, nodes_dict=globals.NODE_INFO)
            agv_dynamic_data_queue.put(agv_new_data)
        eventlet.sleep(globals.UPDATE_TIME_INTERVAL)


def process_order():
    global running_flag, order_queue, order_res_queue
    while running_flag:
        if not order_queue.empty():
            print("=" * 50, "收到新订单", "=" * 50)
            available_agv_positions = {agv.id: agv.start_node_id for agv in globals.AGV_VEHICLES.values() if agv.status == 0}
            print(f"当前空闲状态的AGV: {available_agv_positions}")
            num_available_agvs = len(available_agv_positions)

            if num_available_agvs == 0:
                print("No available AGVs. Waiting for AGVs to become available.")
            else:
                order_batch_size = min(num_available_agvs, order_queue.qsize())
                print(f"订单数量：{order_queue.qsize()}，空闲状态的AGV数量：{num_available_agvs}，当前批次处理订单的数量：{order_batch_size}")

                waybills = []
                for _ in range(order_batch_size):
                    order = order_queue.get()
                    waybills.append(order)
                print("当前批次处理订单 waybills: ", waybills)

                if waybills:
                    utils.parse_data_to_init_orders(waybills, globals.ORDER_INFO)
                    global GRAPHDATA
                    optimized_orders, unassigned_order_ids = optimize_agv_paths(GRAPHDATA, waybills, available_agv_positions)
                    print("任务分配和路径规划结果：")
                    for order in optimized_orders:
                        print(f"订单编号：{order['orderId']}, 目标节点：{order['targetList']}, "
                              f"执行AGV编号：{order['agvId']}, 路径规划结果：{order['siteList']}")

                    if unassigned_order_ids:
                        print("以下订单无法分配，将重新加入队列:")
                        for orderId in unassigned_order_ids:
                            # 查找原始订单信息并重新加入队列
                            original_order = next((order for order in waybills if order['orderId'] == orderId), None)
                            if original_order:
                                order_queue.put(original_order)
                                print(f"订单编号：{orderId} 重新加入队列")

                    utils.dynamic_update_orders(optimized_orders, globals.ORDER_INFO)
                    order_path_res = utils.generate_order_path_schedule_res(optimized_orders)
                    sio.emit('scheduled_order', order_path_res, namespace='/update')
                    order_res_queue.put(optimized_orders)
                    print("=" * 50, "新订单任务分配和路径规划结果已生成", "=" * 50)

            eventlet.sleep(globals.UPDATE_TIME_INTERVAL)
        eventlet.sleep(globals.UPDATE_TIME_INTERVAL)



def monitor_order():
    # 监听订单结果队列，暂停process和send线程，保存未更新订单的AGV位置，更改有新订单的AGV位置，生成agv_init_dynamic_data, 再重启线程
    global running_flag
    while running_flag:
        if not order_res_queue.empty():
            # 线程暂停！
            global order_update_flag, pool_1
            print("=" * 50, "订单结果处理", "=" * 50)
            print("正在等待当前处理数据和发送数据线程结束...")
            # print(pool_1.running())
            # print(pool_1.coroutines_running)
            # print(pool_1.waiting())
            # print("*" * 100)
            order_update_flag = True
            pool_1.waitall()
            print("当前处理和发送线程已结束，正在生成新的仿真数据...")
            # print("=" * 100)
            # print(agv_dynamic_data_queue.qsize())
            # print(send_queue.qsize())
            new_order = order_res_queue.get()
            # print(new_order)
            if not send_queue.empty():
                agv_init_static_data = send_queue.get()[0]
            elif not agv_dynamic_data_queue.empty():
                agv_init_static_data = agv_dynamic_data_queue.get()
            else:
                agv_init_static_data = AGV_INIT_STATIC_DATA

            # print(agv_init_static_data)
            # pdb.set_trace()
            agv_init_dynamic_data = utils.generate_agv_first_simdata(agv_init_static_data, new_order)
            print("新的仿真数据已生成")
            # print(agv_init_dynamic_data)
            # print(agv_init_dynamic_data)
            # print("=" * 100)
            print("正在清理原始队列数据...")
            agv_dynamic_data_queue.queue.clear()
            send_queue.queue.clear()
            print("队列清理完毕，正在重启处理数据和发送数据线程...")

            # 复位控制线程执行的flag，重启线程
            order_update_flag = False
            agv_dynamic_data_queue.put(agv_init_dynamic_data)

            pool_1.spawn_n(send_agv_data)
            pool_1.spawn_n(process_agv_data)
            print("线程重启完毕！")
            # print("重启后：当前所有线程的名称：")
            # print(pool_1.running())
            # print(pool_1.coroutines_running)
            # print(pool_1.waiting())
            print("=" * 50, "订单结果处理完毕", "=" * 50)
        eventlet.sleep(globals.UPDATE_TIME_INTERVAL)


def print_agv_info():
    for agv_vehicle in globals.AGV_VEHICLES.values():
        agv_vehicle.info()


def print_order_info():
    for order in globals.ORDER_INFO.values():
        order.info()


# 监听后端接口，获取更新的地图资源以及AGV初始位置数据
def initialize_resources(init_data=None):
    with open(os.path.join(save_file_path, resource_save_file), 'w', encoding='utf-8') as f:
        json.dump(init_data, f, ensure_ascii=False)

    graphdata = init_data['graphData']
    agv_init_static_data = init_data['agvData']
    # 增加任务等待时间相关数据字段，之后如果有新增字段可以在不改变后端数据接口的基础上在该函数中修改
    agv_init_static_data = utils.generate_agv_first_initdata(agv_init_static_data)

    global GRAPHDATA, AGV_INIT_STATIC_DATA
    GRAPHDATA = graphdata
    AGV_INIT_STATIC_DATA = agv_init_static_data
    globals.GRAPHDATA = graphdata
    globals.AGV_INIT_STATIC_DATA = agv_init_static_data

    utils.init_all_resources(graphdata, agv_init_static_data, socket_flag=socket_flag)


def restart_all_coroutines():
    global running_flag, pool_1, pool_2
    # 等待当前线程结束
    running_flag = False
    pool_1.waitall()
    pool_2.waitall()
    # 清空所有数据队列
    agv_dynamic_data_queue.queue.clear()
    send_queue.queue.clear()
    order_queue.queue.clear()
    order_res_queue.queue.clear()
    # 复位控制线程执行的flag，重启线程
    running_flag = True
    pool_1.spawn_n(send_agv_data)
    pool_1.spawn_n(process_agv_data)
    pool_2.spawn_n(monitor_order)
    pool_2.spawn_n(process_order)


def run_server(fd):
    try:
        eventlet.wsgi.server(fd, app)
    except Exception:
        print('Error starting thread.')
        raise SystemExit(1)


@sio.on('connect', namespace='/update')
def connect(sid, environ):
    print('Java Client connected:', sid)


@sio.on('connect', namespace='/topo')
def connect(sid, environ):
    print('Topo Client connected:', sid)


# @sio.on('connect', namespace='/order')
# def connect(sid, environ):
#     print('Order Client connected:', sid)
#
#
# @sio.on('connect', namespace='/')
# def connect(sid, environ):
#     print('Default Client connected:', sid)


# 监听地图或AGV等静态资源更新事件
@sio.on('resource_update', namespace='/update')
def restart(sid, resource_data):
    print("=" * 50, "资源更新", "=" * 50)
    # 重启所有线程
    print("资源更新，正在重启所有线程...")
    init_data = json.loads(resource_data)
    restart_all_coroutines()
    print("所有线程重启完毕！")
    print("正在初始化地图和AGV数据...")
    # 地图和AGV初始化
    initialize_resources(init_data)
    print("地图及AGV初始化完成！")
    print(f"地图数据(类型{type(globals.GRAPHDATA)}):")
    print(globals.GRAPHDATA)
    print("AGV初始位置信息：")
    print_agv_info()
    print("=" * 50, "资源更新完毕", "=" * 50)


# 监听订单更新
@sio.on('order_update', namespace='/update')
def order_update(sid, order):
    order_data = json.loads(order)
    with open(os.path.join(save_file_path, order_save_file), 'a', encoding='utf-8') as f:
        json.dump(order_data, f, ensure_ascii=False)
        f.write(',\n')
    order_queue.put(order_data)


# 运行服务器
if __name__ == '__main__':
    pool_1.spawn_n(send_agv_data)
    pool_1.spawn_n(process_agv_data)
    pool_2.spawn_n(monitor_order)
    pool_2.spawn_n(process_order)

    # listen_fd = eventlet.listen(('10.9.58.239', 5000))
    # listen_fd = eventlet.listen(('172.30.132.78', 5000))
    listen_fd = eventlet.listen(('172.30.131.86', 5000))

    run_server(listen_fd)
