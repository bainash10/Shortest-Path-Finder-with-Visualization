import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from matplotlib.animation import FuncAnimation

def dijkstra_step(graph, start, target):
    distances = {node: float('infinity') for node in graph}
    previous_nodes = {node: None for node in graph}
    distances[start] = 0
    nodes = list(graph.nodes)
    path_edges = []

    while nodes:
        current_node = min(nodes, key=lambda node: distances[node])
        nodes.remove(current_node)

        if distances[current_node] == float('infinity'):
            break

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor]['weight']
            alternative_route = distances[current_node] + weight
            if alternative_route < distances[neighbor]:
                distances[neighbor] = alternative_route
                previous_nodes[neighbor] = current_node

        if current_node == target:
            break

    path = []
    current = target
    while previous_nodes[current] is not None:
        path.insert(0, current)
        path_edges.append((current, previous_nodes[current]))
        current = previous_nodes[current]
    if path:
        path.insert(0, start)

    return path, distances, path_edges

def update(num, path_edges, pos, ax, graph, path, distances):
    ax.clear()
    nx.draw(graph, pos, with_labels=True, node_color='lightblue', node_size=500, edge_color='gray', font_size=15, font_weight='bold', ax=ax)

    # Draw explored edges in red
    nx.draw_networkx_edges(graph, pos, edgelist=path_edges[:num], edge_color='red', width=2, ax=ax)

    for node in graph.nodes:
        ax.text(pos[node][0], pos[node][1] + 0.05, f"{distances[node]:.1f}", fontsize=12, ha='center')

    # Draw the final path in blue once completed
    if num == len(path_edges):
        nx.draw_networkx_edges(graph, pos, edgelist=list(zip(path, path[1:])), edge_color='blue', width=3, ax=ax)

def main():
    G = nx.Graph()
    edges = [
        ('A', 'B', 4),
        ('A', 'C', 2),
        ('A', 'D', 7),
        ('B', 'E', 5),
        ('B', 'F', 10),
        ('C', 'E', 3),
        ('C', 'G', 6),
        ('D', 'F', 4),
        ('D', 'H', 8),
        ('E', 'I', 2),
        ('E', 'J', 7),
        ('F', 'J', 1),
        ('F', 'K', 5),
        ('G', 'I', 4),
        ('G', 'L', 3),
        ('H', 'J', 6),
        ('H', 'M', 9),
        ('I', 'N', 8),
        ('J', 'N', 4),
        ('J', 'O', 3),
        ('K', 'O', 7),
        ('L', 'N', 2),
        ('L', 'P', 5),
        ('M', 'Q', 4),
        ('N', 'P', 1),
        ('N', 'R', 7),
        ('O', 'R', 6),
        ('O', 'S', 2),
        ('P', 'T', 3),
        ('Q', 'T', 6),
        ('Q', 'U', 2),
        ('R', 'V', 5),
        ('S', 'V', 4),
        ('S', 'W', 1),
        ('T', 'U', 7),
        ('T', 'X', 4),
        ('U', 'Y', 3),
        ('V', 'W', 6),
        ('W', 'Z', 2),
        ('X', 'Y', 5),
        ('Y', 'Z', 1),
        ('P', 'A', 8),
        ('D', 'O', 3),
        ('H', 'X', 5),
        ('C', 'N', 6)
    ]

    G.add_weighted_edges_from(edges)

    source = 'A'
    target = 'Z'

    path, distances, path_edges = dijkstra_step(G, source, target)
    pos = nx.spring_layout(G)

    fig, ax = plt.subplots(figsize=(15, 10))
    ani = FuncAnimation(fig, update, frames=len(path_edges) + 1, fargs=(path_edges, pos, ax, G, path, distances), interval=1000, repeat=False)

    plt.title(f"Dijkstra's Algorithm Visualization\nSource: {source} to Target: {target}")
    plt.show()

if __name__ == "__main__":
    main()
