import networkx as nx


g = nx.Graph()

g.add_node(1)

g.nodes[1]['x'] = 1
g.nodes[1]['y'] = 5


for node in g.nodes:
    print(g.nodes[node]['x'])