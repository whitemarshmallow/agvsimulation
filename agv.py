# agv.py

class AGV:
    def __init__(self, agv_id, start_node_id, status=0):
        self.agv_id = agv_id
        self.start_node_id = start_node_id
        self.status = status

    def get_position(self):
        return self.start_node_id

    def is_available(self):
        return self.status == 0

    def set_position(self, new_position):
        self.start_node_id = new_position

    def set_status(self, status):
        self.status = status

    def __repr__(self):
        return f"AGV(id={self.agv_id}, start_node_id={self.start_node_id}, status={self.status})"
