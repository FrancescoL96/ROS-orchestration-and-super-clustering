#!/usr/bin/env python

#import rospy
#import rostopic
#import rosnode

'''
To better understand this code, you should probably check the order in which these function and classes are used starting from the main function at the end of the file
'''

try:
    import cPickle as pickle
except ModuleNotFoundError:
    import pickle

node_list = []			# List for the names of the nodes
Topic_list = []			# List of "Topic" objects, where each object contains the name of the topic and publishers/subscribers for the topic
Graph = {}				# A graph composed by "graph_node" objects that contains the node name and the connected nodes
Graph_b = {}			# Like "Graph", but used as a temporary swap 

# This class is used to represent a cluster of nodes: as the algorithm for clustering continues, it creates a new graph (Graph_b) that contains less nodes, each of these nodes contains several nodes for the original graph (Graph)
class cluster_node:
	def __init__(self, node_list):
		self.cluster_name = ''
		for node in node_list:
			self.cluster_name += node.node_name + ' +++ '
		self.cluster_name = self.cluster_name[:-5] # Updates the cluster name by removing the last five characters (added by the previous 2 lines, ' +++ ')
		self.node_name = self.cluster_name
		self.node_list = node_list
		self.connections = self.get_all_connections()
		self.rank = len(self.node_list)
	
	# Recreates edges exiting from the cluster node starting from the edges inside the contained nodes
	def get_all_connections(self):
		connections = {}
		for node in self.node_list:
			for connection in node.connections:
				if connection in connections:
					connections[connection] += node.connections[connection]
				else:
					connections[connection] = node.connections[connection]
		# Edges between nodes inside the cluster are removed because they are not needed
		for node in self.node_list:
			if node.node_name in connections.keys():
				del connections[node.node_name]
		return connections
	
	# Sorts the edges by their cost and then returns the nth (indicated by counter)
	def get_max_connection_pos(self, counter):
		connection_list = []
		for connection in self.connections:
			connection_list.append([connection, self.connections[connection]])
		sorted_connections = sorted(connection_list, key=lambda x : x[1], reverse=True)
		#print(sorted_connections)
		#print(counter)
		if (len(sorted_connections) > counter):
			return sorted_connections[counter][0]
		else:
			return None
		
#------------------------------------------------------------------------------------------
# This class represent a node of the original graph (Graph), there are no clusters, just the node name and the outgoing edges
class graph_node:
	def __init__ (self, node_name):
		self.node_name = node_name
		self.connections = {}
		
	# When we add an edge we only care about the direction of the edge and the cost, each topic has cost 1 (this should be improved to represent bandwidth)
	def add_connection(self, connected_node_name):
		if (connected_node_name in self.connections):
			self.connections[connected_node_name] += 1
		else:
			self.connections[connected_node_name] = 1
#------------------------------------------------------------------------------------------
# This class represents a topic by saving its topic name and all the publishers/subscribers names
class Topic:	
	def __init__ (self, topic_name):
		self.topic_name = topic_name
		self.publishers = []
		self.subscribers = []
	
	def add_publisher(self, publisher):
		#print('pub' + publisher)
		if (publisher != '' and 'rosout' not in publisher):
			self.publishers.append(publisher.split(' ')[2])
		
	def add_subscriber(self, subscriber):
		#print('sub' + subscriber)
		if (subscriber != '' and 'rosout' not in subscriber):
			self.subscribers.append(subscriber.split(' ')[2])
#------------------------------------------------------------------------------------------
# This function gets the topics/nodes data from a live ROS environment (excludes rosout)
def collect_data():
	topics = rospy.get_published_topics()
	for topic in topics:
		if ('rosout' in topic[0]):
			continue
		cur_topic = Topic(topic[0])
		
		# This manipulation is required because the rostopic.get_info_text(...) is a human readable string that needs some manipulation
		topic_info = rostopic.get_info_text(topic[0]).split('\n')
		#print(topic_info)

		subscribers = False		
		for i in range(3, len(topic_info)):

			if ('Subscribers' in topic_info[i]):
				subscribers = True
				continue
			#print(subscribers)
			#print(topic_info[i])
			if subscribers:
				cur_topic.add_subscriber(topic_info[i])
			else:
				cur_topic.add_publisher(topic_info[i])
				
		Topic_list.append(cur_topic)
		#print(cur_topic.publishers)
		#print(cur_topic.subscribers)
		#print('\n\n\n')

# Uses the data collected by the collect_data function and creates a graph that connects each node with an edge that represents the amount of bandwidth between the two
def graph_data():
	global node_list
	node_list = rosnode.get_node_names()
	node_list.remove('/rosout')
	for node in node_list:
		Graph[node] = graph_node(node)
	for topic in Topic_list:
		for node_s in topic.subscribers:
			for node_p in topic.publishers:
				if (node_p != node_s):
					# We add the connection in both directions, to create a non-directed graph
					Graph[node_p].add_connection(node_s)
					Graph[node_s].add_connection(node_p)
					
# Edge-less nodes are culled, they are not needed for a clustering algorithm based on edges
def remove_zero_connections_nodes():
	global Graph, Graph_b
	for node in Graph:
		if len(Graph[node].connections) != 0:
			Graph_b[node] = Graph[node]
	Graph = Graph_b

# This function is a wrapper that transforms each node in the graph into a "cluster_node", capable of hosting multiple nodes inside each vertex
def create_clusters(N_CONT):
	global Graph_b
	for node in Graph:
		Graph_b[node] = cluster_node([Graph[node]])

	# We keep clustering nodes until the total number of nodes is lower than the total number of containers, this works because in our graph we are actually clustering nodes together and creating a new graph at each iteration
	while len(Graph_b) > N_CONT:
		Graph_b = create_clusters_impl(Graph_b)
	
# This function is the actual implementation that handles the creation of clusters
def create_clusters_impl(Graph_b):
	in_clusters = []		# This list contains the list of nodes that were clustered in this iteration
	Graph_c = {}			# Another temporary variable for Graph_b
	nodes_to_refactor = []	# These nodes have been clustered in this iteration, this means that we have to update all the incoming/outgoing edges
	
	# We order the nodes by the rank, the first iteration will have basically casual sorting as all nodes have rank=1, lexicographic sorting could be used
	ordered_list = get_ordered_by_rank_node_list(Graph_b)

	# At each iteration, we try cluster every node with the node with the highest cost edge
	for node_rank in ordered_list:
		node = node_rank[0]
		counter = 0
		# To get the node with the highest edge cost, we use a counter that keeps asking for next highest cost edge until we find one that has not been clustered in this iteration yet
		candidate_cluster_name = Graph_b[node].get_max_connection_pos(counter)
		while candidate_cluster_name in in_clusters:
			counter += 1
			candidate_cluster_name = Graph_b[node].get_max_connection_pos(counter)
			if (candidate_cluster_name == None):
				break
			#print('candidate: ' + candidate_cluster_name)
		#print('node: ' + node)
		#print('candidate: ' + candidate_cluster_name)
		#print('in_clusters: ' + str(in_clusters))
		#print('refactor: ' + str(nodes_to_refactor))
		#for key in Graph_b:
		#	print(key)
		# If the node was not already clustered with some other node (in previous iterations of the for cycle)
		# If there is no valid candidate
		if (candidate_cluster_name == None or candidate_cluster_name == '') and node not in in_clusters:
			# We add the node to the new graph as it was, without clustering anything
			Graph_c[node] = Graph_b[node]
			in_clusters.append(node)
		# If the node and the candidate were not already used in a cluster in this iteration, we cluster them together and flag them for refactoring
		elif candidate_cluster_name not in in_clusters and node not in in_clusters:
			Graph_c[node+' +++ '+candidate_cluster_name] = cluster_node([Graph_b[node], Graph_b[candidate_cluster_name]])
			nodes_to_refactor.append([node, candidate_cluster_name])
			in_clusters.append(node)
			in_clusters.append(candidate_cluster_name)
		# This is basically an else, should never reach here
		elif node not in in_clusters:
			Graph_c[node] = Graph_b[node]
			in_clusters.append(node)
	# We refactor the edges of the nodes that we flagged during this iteration
	refactor_graph(Graph_c, nodes_to_refactor)
	
	#print(len(Graph_c))
	#print('------------------')
	return Graph_c

# Sorts the nodes by their rank
def get_ordered_by_rank_node_list(graph):
	ordered_list = []
	for node in Graph_b:
		ordered_list.append([node, Graph_b[node].rank])

	return sorted(ordered_list, key=lambda x : x[1])
	
# This function updates the edges on the nodes by summing together edges that go into the same cluster or edges that come into the same cluster
def refactor_graph(graph, nodes_to_refactor):
	for node in graph:
		for nodes_refactor in nodes_to_refactor:
			#print(nodes_refactor)
			if (nodes_refactor[0] in graph[node].connections):
				if (nodes_refactor[1] in graph[node].connections):
					#print(nodes_refactor[0])
					#print(nodes_refactor[1])
					#print(graph[node].connections)
					graph[node].connections[nodes_refactor[0] + ' +++ ' + nodes_refactor[1]] = graph[node].connections[nodes_refactor[0]] + graph[node].connections[nodes_refactor[1]]
					del graph[node].connections[nodes_refactor[0]]
					del graph[node].connections[nodes_refactor[1]]
				else:
					graph[node].connections[nodes_refactor[0] + ' +++ ' + nodes_refactor[1]] = graph[node].connections[nodes_refactor[0]]
					del graph[node].connections[nodes_refactor[0]]
			elif (nodes_refactor[1] in graph[node].connections):
				graph[node].connections[nodes_refactor[0] + ' +++ ' + nodes_refactor[1]] = graph[node].connections[nodes_refactor[1]]
				del graph[node].connections[nodes_refactor[1]]

# ALGORITHM A -------------------------------------------		
# Kruskal		
def get_maximum_spanning_tree():
	global Graph_b
	Graph_b = {}
	
	edge_list = []
	for node in Graph:
		for edge in Graph[node].connections:
			reverse_edge = [Graph[node].connections[edge], edge, node]
			if reverse_edge not in edge_list:
				edge_list.append([Graph[node].connections[edge], node, edge])
	edge_list_sorted = sorted(edge_list, key=lambda x: x[0], reverse=True)
	
	for edge in edge_list_sorted:
		if (edge[1] in Graph_b.keys() and edge[2] in Graph_b.keys()):
			continue
		if (not check_loop(Graph_b, edge)):
			if (edge[1] in Graph_b.keys()):
				Graph_b[edge[1]].connections[edge[2]] = edge[0]
				if (edge[2] not in Graph_b.keys()):
					Graph_b[edge[2]] = graph_node(edge[2])
					Graph_b[edge[2]].connections[edge[1]] = edge[0]
				else:
					Graph_b[edge[2]].connections[edge[1]] = edge[0]
			elif (edge[2] in Graph_b.keys()):
				Graph_b[edge[2]].connections[edge[1]] = edge[0]
				if (edge[1] not in Graph_b.keys()):
					Graph_b[edge[1]] = graph_node(edge[1])
					Graph_b[edge[1]].connections[edge[2]] = edge[0]
				else:
					Graph_b[edge[1]].connections[edge[2]] = edge[0]
			else:
				Graph_b[edge[1]] = graph_node(edge[1])
				Graph_b[edge[1]].connections[edge[2]] = edge[0]
				Graph_b[edge[2]] = graph_node(edge[2])
				Graph_b[edge[2]].connections[edge[1]] = edge[0]
				
	return edge_list
				
# Check for loops inside the MST
def check_loop(graph, edge):
	if len(graph.keys()) < 1:
		return False

	graph_b = graph.copy()
	if (edge[1] in graph_b.keys()):
		graph_b[edge[1]].connections[edge[2]] = edge[0]
		if (edge[2] not in graph_b.keys()):
			graph_b[edge[2]] = graph_node(edge[2])
			graph_b[edge[2]].connections[edge[1]] = edge[0]
		else:
			graph_b[edge[2]].connections[edge[1]] = edge[0]
	elif (edge[2] in graph_b.keys()):
		graph_b[edge[2]].connections[edge[1]] = edge[0]
		if (edge[1] not in graph_b.keys()):
			graph_b[edge[1]] = graph_node(edge[1])
			graph_b[edge[1]].connections[edge[2]] = edge[0]
		else:
			graph_b[edge[1]].connections[edge[2]] = edge[0]
	else:
		graph_b[edge[1]] = graph_node(edge[1])
		graph_b[edge[1]].connections[edge[2]] = edge[0]
		graph_b[edge[2]] = graph_node(edge[2])
		graph_b[edge[2]].connections[edge[1]] = edge[0]
	
	return dfs(graph_b, graph_b[graph.keys()[0]].node_name, graph_b[graph.keys()[0]].node_name, [])

# DFS
def dfs(graph, node, father, visited, print_name=False, node_list = []):
	visited.append(node)
#	if (print_name):
#		print(node)
#		print(graph[node].connections)
	for edge in graph[node].connections:
		if edge in visited and edge != father:
			#print('CYCLE DETECTED')
			#print(edge)
			return True
		elif (edge != father):
			if (print_name):
				print('\t' + edge)
#				print('------')
				node_list.remove(edge)
				dfs(graph, edge, node, visited, True, node_list)
			else:			
				dfs(graph, edge, node, visited)
	
	return False
	
def cut_k_max_edges(n_cont):
	n_cont -= 1
	
	edge_list = []
	for node in Graph_b:
		for edge in Graph_b[node].connections:
			reverse_edge = [Graph_b[node].connections[edge], edge, node]
			if reverse_edge not in edge_list:
				edge_list.append([Graph_b[node].connections[edge], node, edge])
	edge_list_sorted = sorted(edge_list, key=lambda x: x[0])
	
	for i in range(0, n_cont):
		del Graph_b[edge_list_sorted[i][1]].connections[edge_list_sorted[i][2]]
		del Graph_b[edge_list_sorted[i][2]].connections[edge_list_sorted[i][1]]
		
def print_clusters():
	node_list = Graph_b.keys()
	while (len(node_list) > 0):
		node = node_list[0]
		node_list.remove(node)
		print(node)
		dfs(Graph_b, node, node, [], True, node_list)
# END ALGORITHM A -------------------------------------------

# Stores all the data collected by the function collect_data into a pickle file
def dump_data():
	with open('data_store', 'wb') as data_store_file:
		pickle.dump(node_list, data_store_file, pickle.HIGHEST_PROTOCOL)
		pickle.dump(Topic_list, data_store_file, pickle.HIGHEST_PROTOCOL)
		pickle.dump(Graph, data_store_file, pickle.HIGHEST_PROTOCOL)
	
# Loads the stored data from the pickle file
def load_data():
	global node_list, Topic_list, Graph
	with open('data_store', 'rb') as data_store_file:
		node_list = pickle.load(data_store_file)
		Topic_list = pickle.load(data_store_file)
		Graph = pickle.load(data_store_file)

# Prints each node in the given graph and the edges (default: nodes only)
# Note: Some "replace" functions have been added to perform some formatting specific to my test case output
def print_graph(graph, EDGES=False):
	counter = 0
	for node in graph:
		print(counter, graph[node].node_name.replace("/cram/", "").replace("/localhost", ""))
		if EDGES:
			for connection in graph[node].connections:
				print('\t'+connection.replace("/cram/", "").replace("/localhost", "")[0:20] + ': ' + str(graph[node].connections[connection]))
		counter += 1

if __name__ == '__main__':
	#rospy.init_node('data_collection')
	try:	
		# Data collection/loading phase
		# We either load the graph to cluster, or collect the data and graph it before clustering
		load_data()
		#collect_data()
		#graph_data()
		
		# Saves the collected data
		#dump_data()
		
		# Alg A --- Cuts the N lowest cost edges from the Maximum Spanning Tree of the graph, uses DFS, Kruskal
		#remove_zero_connections_nodes()
		#get_maximum_spanning_tree()
		#cut_k_max_edges(10)
		#print_clusters()
		
		# Alg B, based on Hierarchical Agglomerative Clustering, custom
		remove_zero_connections_nodes()
		create_clusters(15)
		print_graph(Graph_b, True)
	except rospy.ROSInterruptException: pass
