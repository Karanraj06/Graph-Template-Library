import matplotlib.pyplot as plt
import networkx as nx
G = nx.DiGraph()
G.add_edge(0, 3, weight = 3)
G.add_edge(1, 0, weight = 5)
G.add_edge(1, 2, weight = 3)
G.add_edge(2, 3, weight = 8)
G.add_edge(3, 0, weight = 3)
G.add_edge(3, 1, weight = 2)
subax2 = plt.subplot(122)
nx.draw_shell(G, nlist=[range(5, 10), range(5)], with_labels=True, font_weight='bold')
plt.show()