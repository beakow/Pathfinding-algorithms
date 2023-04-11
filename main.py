import pandas as pd
from algorithms import *
import time


def get_graph_dict():
    file = 'connection_graph.csv'
    data = pd.read_csv(file, delimiter=",", dtype='unicode')
    data = data[['company', 'line', 'departure_time', 'arrival_time', 'start_stop', 'end_stop',
                 'start_stop_lat', 'start_stop_lon', 'end_stop_lat', 'end_stop_lon']]
    data['departure_time'] = [time.time() for time in pd.to_datetime(data['departure_time'], format='%H:%M:%S')]
    data['arrival_time'] = [time.time() for time in pd.to_datetime(data['arrival_time'], format='%H:%M:%S')]
    dataframe = pd.DataFrame(data, columns=['company', 'line', 'departure_time', 'arrival_time', 'start_stop', 'end_stop',
                                            'start_stop_lat', 'start_stop_lon', 'end_stop_lat', 'end_stop_lon'])
    edges = []
    for index, row in dataframe.iterrows():
        arrival_time = row["arrival_time"]
        departure_time = row["departure_time"]
        arrival_time_delta = datetime.timedelta(hours=arrival_time.hour, minutes=arrival_time.minute, seconds=arrival_time.second)
        departure_time_delta = datetime.timedelta(hours=departure_time.hour, minutes=departure_time.minute, seconds=departure_time.second)
        edges.append((row["start_stop"], row["end_stop"], row["line"], departure_time_delta, arrival_time_delta,
                      row["start_stop_lat"], row["start_stop_lon"], row["end_stop_lat"], row["end_stop_lon"]))

    graph_dict = {}
    for start, end, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon in edges:
        if start in graph_dict:
            graph_dict[start].append((end, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon))
        else:
            graph_dict[start] = [(end, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon)]
        if end in graph_dict:
            graph_dict[end].append((start, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon))
        else:
            graph_dict[end] = [(start, line, departure_time, arrival_time, start_stop_lat, start_stop_lon, end_stop_lat, end_stop_lon)]

    return graph_dict


def run_dijkstra(gg, A, B, time):
    print("---------- Dijkstra ------------------------------------")
    cost, path, arrival_time, departure_time, line = dijkstra(gg, A, B, time)
    print_path(cost, path, arrival_time, departure_time, line)


def run_astar(gg, A, B, opt, time):
    if opt == 'all':
        run_astar(gg, A, B, 't', time)
        run_astar(gg, A, B, 'p', time)
        run_astar(gg, A, B, 'both', time)
    elif opt == 't':
        print("\n---------- A* - time ----------------------------------")
        cost, path, arrival_time, departure_time, line = astar_t(gg, A, B, time)
        print_path(cost, path, arrival_time, departure_time, line)
    elif opt == 'p':
        print("\n---------- A* - transfers -----------------------------")
        cost, path, arrival_time, departure_time, line = astar_p(gg, A, B, time)
        print_path(cost, path, arrival_time, departure_time, line)
    else:
        print("\n---------- A* - time & transfers ----------------------")
        cost, path, arrival_time, departure_time, line = astar_t_p(gg, A, B, time)
        print_path(cost, path, arrival_time, departure_time, line)


def run_tabu(gg, A, L, opt, time):
    if opt == 'all':
        run_tabu(gg, A, L, 't', time)
        run_tabu(gg, A, L, 'p', time)
    else:
        if opt == 't':
            print("\n---------- TABU SEARCH - time ------------------------")
        elif opt == 'p':
            print("\n---------- TABU SEARCH - transfers -------------------")

        solution, cost, path, arrival_time, departure_time, line = tabu(gg, A, L, opt, time)
        print(solution)
        print_path(cost, path, arrival_time, departure_time, line)


def run_user_queries(gg):
    start = input("Przystanek początkowy A: ")
    goal = input("Przystanek końcowy B: ")
    opt = input("Kryterium optymalizacyjne (t - czas, p - przesiadki): ")
    time = input("Czas pojawienia się na przystanku A (hh:mm): ").split(':')
    hour, minutes = [int(item) for item in time]
    run_dijkstra(gg, start, goal, datetime.timedelta(hours=hour, minutes=minutes, seconds=0))
    run_astar(gg, start, goal, opt, datetime.timedelta(hours=hour, minutes=minutes, seconds=0))

    start = input("Przystanek początkowy A: ")
    stops = input("Przystanki do odwiedzenia: ").split(", ")
    opt = input("Kryterium optymalizacyjne (t - czas, p - przesiadki): ")
    time = input("Czas pojawienia się na przystanku A (hh:mm): ").split(':')
    hour, minutes = [int(item) for item in time]
    run_tabu(gg, start, stops, opt, datetime.timedelta(hours=hour, minutes=minutes, seconds=0))


def run_automatic_queries(gg):
    B = 'KSIĘŻE MAŁE'
    A = 'LEŚNICA'
    # run_dijkstra(gg, A, B, datetime.timedelta(hours=7, minutes=30, seconds=0))
    #run_astar(gg, A, B, 'all', datetime.timedelta(hours=7, minutes=30, seconds=0))
    run_tabu(gg, 'PL. GRUNWALDZKI', ['KROMERA', 'PILCZYCE', 'Hutmen', 'Volvo'], 'all', datetime.timedelta(hours=12, minutes=0, seconds=0))


def time_comparison(gg):
    # little distance
    A = 'PL. GRUNWALDZKI'
    B = 'GALERIA DOMINIKAŃSKA'
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)

    exe_start_time = time.time()
    dijkstra(gg, A, B, start_time)
    exe_end_time = time.time()
    print("Dijkstra: ", exe_end_time - exe_start_time)

    exe_start_time = time.time()
    astar_t(gg, A, B, start_time)
    exe_end_time = time.time()
    print("A* - time: ", exe_end_time - exe_start_time)

    exe_start_time = time.time()
    astar_p(gg, A, B, start_time)
    exe_end_time = time.time()
    print("A* - transfers: ", exe_end_time - exe_start_time, "\n")

    # medium distance
    A = 'Katedra'
    B = 'Niedźwiedzia'
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)

    exe_start_time = time.time()
    dijkstra(gg, A, B, start_time)
    exe_end_time = time.time()
    print("Dijkstra: ", exe_end_time - exe_start_time)

    exe_start_time = time.time()
    astar_t(gg, A, B, start_time)
    exe_end_time = time.time()
    print("A* - time: ", exe_end_time - exe_start_time)

    exe_start_time = time.time()
    astar_p(gg, A, B, start_time)
    exe_end_time = time.time()
    print("A* - transfers: ", exe_end_time - exe_start_time, "\n")

    # big distance
    A = 'KROMERA'
    B = 'Parafialna'
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)

    exe_start_time = time.time()
    dijkstra(gg, A, B, start_time)
    exe_end_time = time.time()
    print("Dijkstra: ", exe_end_time - exe_start_time)

    exe_start_time = time.time()
    astar_t(gg, A, B, start_time)
    exe_end_time = time.time()
    print("A* - time: ", exe_end_time - exe_start_time)

    exe_start_time = time.time()
    astar_p(gg, A, B, start_time)
    exe_end_time = time.time()
    print("A* - transfers: ", exe_end_time - exe_start_time, "\n")

    # tabu
    start = 'KROMERA'
    stops = ['GALERIA DOMINIKAŃSKA']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 't', start_time)
    exe_end_time = time.time()
    print("1 stop: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['PL. GRUNWALDZKI', 'Rynek']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 't', start_time)
    exe_end_time = time.time()
    print("2 stops: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['Katedra', 'GALERIA DOMINIKAŃSKA', 'Niedźwiedzia']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 't', start_time)
    exe_end_time = time.time()
    print("3 stops: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['Katedra', 'GALERIA DOMINIKAŃSKA', 'Niedźwiedzia', 'Przybyszewskiego']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 't', start_time)
    exe_end_time = time.time()
    print("4 stops: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['Katedra', 'GALERIA DOMINIKAŃSKA', 'Niedźwiedzia', 'Parafialna', 'Przybyszewskiego']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 't', start_time)
    exe_end_time = time.time()
    print("5 stops: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['GALERIA DOMINIKAŃSKA']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 'p', start_time)
    exe_end_time = time.time()
    print("1 stop: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['PL. GRUNWALDZKI', 'Rynek']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 'p', start_time)
    exe_end_time = time.time()
    print("2 stops: ", exe_end_time - exe_start_time, "\n")

    start = 'KROMERA'
    stops = ['Katedra', 'GALERIA DOMINIKAŃSKA', 'Niedźwiedzia']
    start_time = datetime.timedelta(hours=10, minutes=00, seconds=00)
    exe_start_time = time.time()
    tabu(gg, start, stops, 'p', start_time)
    exe_end_time = time.time()
    print("3 stops: ", exe_end_time - exe_start_time, "\n")



gg = get_graph_dict()
run_automatic_queries(gg)
# run_user_queries(gg)
