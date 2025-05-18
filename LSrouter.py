####################################################
# LSrouter.py
# Name:Tran Van Minh
# HUID: 23020119
#####################################################

import heapq
import json
from packet import Packet
from router import Router


class LSrouter(Router):
    def __init__(self, addr, heartbeat_time):
        super().__init__(addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        self.seq_num = 0

        self.neighbors = {}              # neighbor -> cost (của chính router này)
        self.seq_nums = {}               # router -> latest seq_num
        self.graph = {}                  # toàn bộ đồ thị mạng {router: {neighbor: cost}}
        self.forwarding_table = {}       # destination -> port

        self.port_to_neighbor = {}       # port -> neighbor
        self.neighbor_to_port = {}       # neighbor -> port

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            if packet.dst_addr in self.forwarding_table:
                out_port = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)

        elif packet.is_routing:
            try:
                data = json.loads(packet.content)
                src = data["src"]
                seq_num = data["seq_num"]
                neighbors = data["neighbors"]

                current_seq = self.seq_nums.get(src)
                if current_seq is None or seq_num > current_seq:
                    self.seq_nums[src] = seq_num
                    self.graph[src] = neighbors
                    self.update_forwarding_table()
                    self.flood_except(packet, exclude_port = port)
            except Exception as e:
                print(f"[{self.addr}] Error parsing LSP from {packet.src_addr}: {e}")

    def handle_new_link(self, port, endpoint, cost):
        self.neighbors[endpoint] = cost
        self.port_to_neighbor[port] = endpoint
        self.neighbor_to_port[endpoint] = port

        self.update_graph()
        self.update_forwarding_table()
        self.broadcast_link_state()

    def handle_remove_link(self, port):
        if port in self.port_to_neighbor:
            neighbor = self.port_to_neighbor.pop(port)
            self.neighbor_to_port.pop(neighbor, None)
            self.neighbors.pop(neighbor, None)

            self.update_graph()
            self.update_forwarding_table()
            self.broadcast_link_state()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.update_graph()
            self.broadcast_link_state()

    def broadcast_link_state(self):
        content = json.dumps({
            "src": self.addr,
            "seq_num": self.seq_num,
            "neighbors": self.neighbors
        })
        packet = Packet(Packet.ROUTING, self.addr, None, content)

        for neighbor, port in self.neighbor_to_port.items():
            self.send(port, packet)

    def flood_except(self, packet, exclude_port):
        for neighbor, port in self.neighbor_to_port.items():
            if port != exclude_port:
                self.send(port, packet)

    def update_graph(self):
        self.seq_num += 1
        self.seq_nums[self.addr] = self.seq_num
        self.graph[self.addr] = dict(self.neighbors)

    def update_forwarding_table(self):
        d = {self.addr: 0} #distance
        prev = {}
        visited = set()
        heap = [(0, self.addr)]

        while heap:
            _, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)

            for v, w in self.graph.get(u, {}).items(): # {v,w} : {neighbor, cost}
                if v not in d or d[u] + w < d[v]:
                    d[v] = d[u] + w
                    prev[v] = u
                    heapq.heappush(heap, (d[v], v))

        self.forwarding_table.clear()
        for dest in d:
            if dest == self.addr:
                continue
            next_hop = dest
            while prev.get(next_hop) != self.addr:
                next_hop = prev.get(next_hop)
                if next_hop is None:
                    break
            port = self.neighbor_to_port.get(next_hop)
            if port is not None:
                self.forwarding_table[dest] = port

    def __repr__(self):
        return f"[{self.addr}] FWD: {self.forwarding_table}"
