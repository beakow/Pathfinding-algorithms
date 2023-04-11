import math
import datetime
import heapq
import random


def dijkstra(gg, start, goal, start_time):
    prev_nodes = {node: None for node in gg}
    arrival_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    departure_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    lines = {node: 0 for node in gg}
    cost_so_far = {node: float('inf') for node in gg}

    arrival_times[start] = start_time
    cost_so_far[start] = 0
    front = [(0, start, start_time)]
    done = []

    while front:
        _, curr_node, curr_time = heapq.heappop(front)

        if not done.__contains__(curr_node):
            done.append(curr_node)

            if curr_node == goal:
                break

            for neighbor, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in gg[curr_node]:
                if (curr_node == start and curr_time <= departure_time) or \
                        (line != lines[curr_node] and curr_time <= departure_time) or \
                        (line == lines[curr_node] and curr_time == departure_time):
                    new_cost = (arrival_time - start_time).seconds / 60
                    if (not done.__contains__(neighbor)) and new_cost < cost_so_far[neighbor]:
                        prev_nodes[neighbor] = curr_node
                        arrival_times[neighbor] = arrival_time
                        departure_times[neighbor] = departure_time
                        lines[neighbor] = line
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost
                        heapq.heappush(front, (priority, neighbor, arrival_time))

    path, arrival_time, departure_time, line = create_path(goal, arrival_times, departure_times, lines, prev_nodes)
    return cost_so_far[goal], path, arrival_time, departure_time, line


def astar_t(gg, start, goal, start_time):
    heuristic_fn = lambda a, b: euclidean_distance(a, b)
    prev_nodes = {node: None for node in gg}
    arrival_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    departure_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    lines = {node: 0 for node in gg}
    cost_so_far = {node: float('inf') for node in gg}

    arrival_times[start] = start_time
    cost_so_far[start] = 0
    front = [(0, start, start_time)]
    done = []

    while front:
        _, curr_node, curr_time = heapq.heappop(front)

        if not done.__contains__(curr_node):
            done.append(curr_node)

            if curr_node == goal:
                break

            for neighbor, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in gg[curr_node]:
                if (curr_node == start and curr_time <= departure_time) or \
                        (line != lines[curr_node] and curr_time <= departure_time) or \
                        (line == lines[curr_node] and curr_time == departure_time):
                    new_cost = (arrival_time - start_time).seconds / 60
                    if (not done.__contains__(neighbor)) and new_cost < cost_so_far[neighbor]:
                        prev_nodes[neighbor] = curr_node
                        arrival_times[neighbor] = arrival_time
                        departure_times[neighbor] = departure_time
                        lines[neighbor] = line
                        cost_so_far[neighbor] = new_cost
                        goal_coordinates = (float(gg[goal][0][6]), float(gg[goal][0][7]))
                        neighbor_coordinates = (float(end_stop_lat), float(end_stop_lon))
                        priority = new_cost + heuristic_fn(goal_coordinates, neighbor_coordinates)
                        heapq.heappush(front, (priority, neighbor, arrival_time))

    path, arrival_time, departure_time, line = create_path(goal, arrival_times, departure_times, lines, prev_nodes)
    return cost_so_far[goal], path, arrival_time, departure_time, line


def astar_p(gg, start, goal, start_time):
    available_lines = []
    for neighbor, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in gg[start]:
        if not available_lines.__contains__(line):
            available_lines.append(line)

    final_cost = float('inf')
    final_arrival_times = {}
    final_departure_times = {}
    final_lines = {}
    final_prev_nodes = {}

    for line in available_lines:
        goal, arrival_times, departure_times, lines, prev_nodes, cost_so_far = astar_p_help(gg, start, goal, start_time, line)
        if final_cost > cost_so_far[goal]:
            final_cost = cost_so_far[goal]
            final_arrival_times = arrival_times
            final_departure_times = departure_times
            final_lines = lines
            final_prev_nodes = prev_nodes

    path, arrival_time, departure_time, line = create_path(goal, final_arrival_times, final_departure_times, final_lines, final_prev_nodes)
    return cost_so_far[goal], path, arrival_time, departure_time, line


def astar_p_help(gg, start, goal, start_time, start_line):
    heuristic_fn = lambda a, b: euclidean_distance(a, b)
    prev_nodes = {node: None for node in gg}
    arrival_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    departure_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    lines = {node: 0 for node in gg}
    cost_so_far = {node: float('inf') for node in gg}
    transfers = {node: float('inf') for node in gg}

    arrival_times[start] = start_time
    cost_so_far[start] = 0
    lines[start] = start_line
    transfers[start] = 0
    front = [(0, start, start_time)]
    done = []

    while front:
        _, curr_node, curr_time = heapq.heappop(front)

        if not done.__contains__(curr_node):
            done.append(curr_node)

            if curr_node == goal:
                break

            for neighbor, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in gg[curr_node]:
                if (curr_node == start and curr_time <= departure_time) or \
                        (line != lines[curr_node] and curr_time<= departure_time) or \
                        (line == lines[curr_node] and curr_time == departure_time):
                    prev_transfers = transfers[curr_node]
                    new_cost = (prev_transfers * 100) + ((arrival_time - start_time).seconds / 60)
                    if line != lines[curr_node]:
                        new_cost += 100
                    if (not done.__contains__(neighbor)) and new_cost < cost_so_far[neighbor]:
                        prev_nodes[neighbor] = curr_node
                        arrival_times[neighbor] = arrival_time
                        departure_times[neighbor] = departure_time
                        lines[neighbor] = line
                        cost_so_far[neighbor] = new_cost
                        transfers[neighbor] = prev_transfers
                        if line != lines[curr_node]:
                            transfers[neighbor] += 1
                        goal_coordinates = (float(gg[goal][0][6]), float(gg[goal][0][7]))
                        neighbor_coordinates = (float(end_stop_lat), float(end_stop_lon))
                        priority = new_cost + heuristic_fn(goal_coordinates, neighbor_coordinates)
                        heapq.heappush(front, (priority, neighbor, arrival_time))

    return goal, arrival_times, departure_times, lines, prev_nodes, cost_so_far


def astar_t_p(gg, start, goal, start_time):
    available_lines = []
    for neighbor, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in gg[start]:
        if not available_lines.__contains__(line):
            available_lines.append(line)

    final_cost = float('inf')
    final_arrival_times = {}
    final_departure_times = {}
    final_lines = {}
    final_prev_nodes = {}

    for line in available_lines:
        goal, arrival_times, departure_times, lines, prev_nodes, cost_so_far = astar_p_help(gg, start, goal, start_time, line)
        if final_cost > cost_so_far[goal]:
            final_cost = cost_so_far[goal]
            final_arrival_times = arrival_times
            final_departure_times = departure_times
            final_lines = lines
            final_prev_nodes = prev_nodes

    path, arrival_time, departure_time, line = create_path(goal, final_arrival_times, final_departure_times, final_lines, final_prev_nodes)
    return cost_so_far[goal], path, arrival_time, departure_time, line


def astar_t_p_help(gg, start, goal, start_time, start_line):
    heuristic_fn = lambda a, b: euclidean_distance(a, b)
    prev_nodes = {node: None for node in gg}
    arrival_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    departure_times = {node: datetime.timedelta(hours=23, minutes=59, seconds=59) for node in gg}
    lines = {node: 0 for node in gg}
    cost_so_far = {node: float('inf') for node in gg}
    transfers = {node: float('inf') for node in gg}

    arrival_times[start] = start_time
    cost_so_far[start] = 0
    lines[start] = start_line
    transfers[start] = 0
    front = [(0, start, start_time)]
    done = []

    while front:
        _, curr_node, curr_time = heapq.heappop(front)

        if not done.__contains__(curr_node):
            done.append(curr_node)

            if curr_node == goal:
                break

            for neighbor, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in gg[curr_node]:
                if (curr_node == start and curr_time <= departure_time) or \
                        (line != lines[curr_node] and curr_time + datetime.timedelta(hours=0, minutes=1, seconds=0) <= departure_time) or \
                        (line == lines[curr_node] and curr_time == departure_time):
                    prev_transfers = transfers[curr_node]
                    new_cost = (prev_transfers * 10) + ((arrival_time - start_time).seconds / 60)
                    if line != lines[curr_node]:
                        new_cost += 10
                    if (not done.__contains__(neighbor)) and new_cost < cost_so_far[neighbor]:
                        prev_nodes[neighbor] = curr_node
                        arrival_times[neighbor] = arrival_time
                        departure_times[neighbor] = departure_time
                        lines[neighbor] = line
                        cost_so_far[neighbor] = new_cost
                        transfers[neighbor] = prev_transfers
                        if line != lines[curr_node]:
                            transfers[neighbor] += 1
                        goal_coordinates = (float(gg[goal][0][6]), float(gg[goal][0][7]))
                        neighbor_coordinates = (float(end_stop_lat), float(end_stop_lon))
                        priority = new_cost + heuristic_fn(goal_coordinates, neighbor_coordinates)
                        heapq.heappush(front, (priority, neighbor, arrival_time))

    return goal, arrival_times, departure_times, lines, prev_nodes, cost_so_far


def manhattan_distance(a, b):
    return sum([abs(x - y) for x, y in zip(a, b)]) * 1000


def euclidean_distance(a, b):
    return math.sqrt(sum([(x - y) ** 2 for x, y in zip(a, b)])) * 1000


def tabu(gg, start, stops, opt, start_time):
    stops_count = len(stops)
    current_solution = stops
    random.shuffle(current_solution)
    best_solution = current_solution
    best_solution_cost, best_path, best_arrival_time, best_departure_time, best_line = get_path_cost(gg, start, best_solution, opt, start_time)

    max_iterations = math.ceil(1.1*(stops_count**2))
    turns_improved = 0
    improve_thresh = 2 * math.floor(math.sqrt(max_iterations))

    tabu_tenure = stops_count
    tabu_list = []

    for iteration in range(max_iterations):
        if turns_improved > improve_thresh:
            break

        best_neighbor = None
        best_neighbor_cost = float('inf')
        best_neighbor_path = []
        best_neighbor_arrival_time = []
        best_neighbor_departure_time = []
        best_neighbor_line = []
        coordA, coordB = 0, 0

        for i in range(stops_count):
            for j in range(i + 1, stops_count):
                neighbor = current_solution
                neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
                neighbor_cost, neighbor_path, neighbor_arrival_time, neighbor_departure_time, neighbor_line = get_path_cost(gg, start, neighbor, opt, start_time)
                if (i, j) not in tabu_list:
                    if neighbor_cost < best_neighbor_cost:
                        best_neighbor = neighbor
                        best_neighbor_cost = neighbor_cost
                        best_neighbor_path = neighbor_path
                        best_neighbor_arrival_time = neighbor_arrival_time
                        best_neighbor_departure_time = neighbor_departure_time
                        best_neighbor_line = neighbor_line
                        coordA, coordB = i, j
            tabu_list.append((coordA, coordB))
        if best_neighbor is not None:
            current_solution = best_neighbor
            tabu_list.append((coordA, coordB))

            if len(tabu_list) > tabu_tenure:
                tabu_list.pop(0)
            if best_neighbor_cost < best_solution_cost:
                best_solution = best_neighbor
                best_solution_cost = best_neighbor_cost
                best_path = best_neighbor_path
                best_arrival_time = best_neighbor_arrival_time
                best_departure_time = best_neighbor_departure_time
                best_line = best_neighbor_line
                turns_improved = 0
            else:
                turns_improved = turns_improved + 1

    return best_solution, best_solution_cost, best_path, best_arrival_time, best_departure_time, best_line


def get_path_cost(gg, start, stops, opt, start_time):
    curr_time = start_time
    curr_stop = start
    final_cost = 0
    final_path = [start]
    final_arrival_time = ['']
    final_departure_time = ['']
    final_line = ['']

    for stop in stops:
        if opt == 't':
            cost, path, arrival_time, departure_time, line = astar_t(gg, curr_stop, stop, curr_time)
        elif opt == 'p':
            cost, path, arrival_time, departure_time, line = astar_p(gg, curr_stop, stop, curr_time)

        final_cost += cost
        final_path = final_path + path[1:]
        final_arrival_time = final_arrival_time + arrival_time[1:]
        final_departure_time = final_departure_time + departure_time[1:]
        final_line = final_line + line[1:]

        curr_stop = stop
        curr_time = datetime.timedelta(hours=arrival_time[len(path) - 1].hour, minutes=arrival_time[len(path) - 1].minute, seconds=arrival_time[len(path) - 1].second)

    cost, path, arrival_time, departure_time, line = dijkstra(gg, curr_stop, start, curr_time)

    final_cost += cost
    final_path = final_path + path[1:]
    final_arrival_time = final_arrival_time + arrival_time[1:]
    final_departure_time = final_departure_time + departure_time[1:]
    final_line = final_line + line[1:]
    return final_cost, final_path, final_arrival_time, final_departure_time, final_line


def create_path(goal, arrival_times, departure_times, lines, prev_nodes):
    path = []
    arrival_time = []
    departure_time = []
    curr_node = goal
    line = []

    while curr_node is not None:
        path.append(curr_node)
        arrival_time.append((datetime.datetime.min + arrival_times[curr_node]).time())
        departure_time.append((datetime.datetime.min + departure_times[curr_node]).time())
        line.append(lines[curr_node])
        curr_node = prev_nodes[curr_node]

    path.reverse()
    arrival_time.reverse()
    departure_time.reverse()
    line.reverse()

    return path, arrival_time, departure_time, line


def print_path(cost, path, arrival_times, departure_times, lines):
    if len(path) > 1:
        print(path[0])
        print(lines[1], ": ", departure_times[1], " - ", end='')

        prev_line = lines[1]
        prev_arrival_time = arrival_times[1]
        prev_stop = path[1]

        for i in range(len(path)):
            if i > 0:
                if prev_line != lines[i]:
                    if i > 1:
                        print(prev_arrival_time)
                        print(prev_stop)
                    print(lines[i], ": ", departure_times[i], " - ", end='')
                prev_line = lines[i]
                prev_arrival_time = arrival_times[i]
                prev_stop = path[i]
        print(arrival_times[len(path) - 1])
        print(prev_stop)


def print_path_all(cost, path, arrival_times, departure_times, lines):
    if len(path) > 1:
        print(path[0])
        for i in range(len(path)):
            if i > 0:
                print(lines[i], ": ", departure_times[i], " - ", arrival_times[i])
                print(path[i])