import xml.etree.ElementTree as ET
from pathlib import Path


def parse_graphml(file_path):
    """
    Parsează un fișier GraphML și extrage coordonatele nodurilor și informațiile despre muchii.

    Args:
        file_path: Calea către fișierul GraphML.

    Returns:
        Un dicționar cu următoarele chei:
            - nodes: Un dicționar cu id-ul nodului ca cheie și un dicționar cu coordonatele ca valoare.
            - edges: O listă de tupluri (source, target, data).
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()

        namespace = {'graphml': "http://graphml.graphdrawing.org/xmlns"}

        nodes = {}
        edges = []

        for node in root.findall(".//graphml:node", namespace):
            node_id = node.get('id')
            coords = {}
            for data in node.findall('graphml:data', namespace):
                key = data.get('key')
                value = data.text
                if key in ['d0', 'd1']:  # Adaptați la cheile dvs.
                    coords[key] = float(value)
            nodes[node_id] = coords

        for edge in root.findall(".//graphml:edge", namespace):
            source = edge.get('source')
            target = edge.get('target')
            data = {}
            for edge_data in edge.findall('data'):
                key = edge_data.get('key')
                value = edge_data.text
                data[key] = value
            edges.append((source, target, data.get('d2', False)))


        return {'nodes': nodes, 'edges': edges}
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except ET.ParseError as e:
        print(f"Parsing error: {e}")



def create_adjacency_matrix(nodes, edges):
    """
    Creează o matrice de adiacență din noduri și muchii.

    Args:
        nodes: Dicționar de noduri cu id-uri.
        edges: Listă de muchii (source, target).

    Returns:
        O matrice de adiacență (listă de liste).
    """
    # Asociem fiecărui nod un index unic
    node_ids = list(nodes.keys())
    node_index = {node_id: idx for idx, node_id in enumerate(node_ids)}
    n = len(node_ids)

    # Inițializăm matricea de adiacență cu 0
    adjacency_matrix = [[0] * n for _ in range(n)]

    # Parcurgem muchiile și completăm matricea
    for source, target, _ in edges:
        if source in node_index and target in node_index:
            adjacency_matrix[node_index[source]][node_index[target]] = 1

    return adjacency_matrix, node_index

def get_vertex_list(file_path):
    """
    Получает список вершин и их координат из файла GraphML.
    """
    graph_data = parse_graphml(file_path)
    if graph_data is None:
        return []

    nodes = graph_data['nodes']
    vertex_list = []

    for node_id, coords in nodes.items():
        x = coords.get('d0', 0.0)
        y = coords.get('d1', 0.0)
        vertex_list.append((node_id, x, y))

    return vertex_list