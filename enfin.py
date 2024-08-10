#!/usr/bin/env python

'Setting the position of nodes and providing mobility'

import sys
import numpy as np
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mininet.link import TCLink

def create_nodes(net):
    "Create nodes in the network."
    info("*** Creating nodes\n")
    h1 = net.addHost('h1', mac='00:00:00:00:00:01', ip='10.0.0.1/8')
    h2 = net.addHost('h2', mac='00:00:00:00:00:05', ip='10.0.0.5/8')
    sta1 = net.addStation('sta1', mac='00:00:00:00:00:02', ip='10.0.0.2/8')
    sta2 = net.addStation('sta2', mac='00:00:00:00:00:03', ip='10.0.0.3/8')
    sta3 = net.addStation('sta3', mac='00:00:00:00:00:04', ip='10.0.0.4/8')
    s1 = net.addSwitch('s1')
    s2 = net.addSwitch('s2')
    s3 = net.addSwitch('s3')
    s4 = net.addSwitch('s4')
    s5 = net.addSwitch('s5')
    s6 = net.addSwitch('s6')
    c1 = net.addController('c1')
    return h1, h2, sta1, sta2, sta3, s1, s2, s3, s4, s5, s6, c1

def create_access_points(net, rows, cols):
    "Create access points in the network based on grid dimensions."
    info("*** Creating access points\n")
    aps = []
    spacing_x = 50
    spacing_y = 50
    for i in range(rows):
        for j in range(cols):
            ap = net.addAccessPoint(f'ap{i * cols + j + 1}', ssid=f'ap{i * cols + j + 1}-wlan0', mode='g', channel=str(i * cols + j + 1), position=f'{(j + 1) * spacing_x},{(i + 1) * spacing_y},0')
            aps.append(ap)
    return aps

def configure_network(net):
    "Configure propagation model and nodes."
    info("*** Configuring propagation model\n")
    net.setPropagationModel(model="logDistance", exp=4.5)

    info("*** Configuring nodes\n")
    net.configureNodes()

def create_links(net, h1, h2, aps, s1, s2, s5, s6):
    "Create links between nodes."
    info("*** Associating and Creating links\n")
    net.addLink(s1, aps[0], cls=TCLink, bw=300)
    net.addLink(h1, s1, cls=TCLink, bw=100)
    net.addLink(h2, s6, cls=TCLink, bw=100)
    net.addLink(s6, aps[-1], cls=TCLink, bw=300)
    net.addLink(s1, s2, cls=TCLink, bw=300)
    net.addLink(s5, s6, cls=TCLink, bw=400)
    net.addLink(s2, s5, cls=TCLink, bw=200)

def setup_mobility(net, args, sta1, sta2, sta3):
    "Setup mobility for stations."
    if '-p' not in args:
        net.plotGraph(max_x=200, max_y=200)

    if '-c' in args:
        sta1.coord = ['40.0,30.0,0.0', '31.0,10.0,0.0', '50.0,25.0,0.0']
        sta2.coord = ['40.0,40.0,0.0', '55.0,31.0,0.0', '75.0,75.0,0.0']
        sta3.coord = ['40.0,40.0,0.0', '55.0,31.0,0.0', '155.0,125.0,0.0']

    net.startMobility(time=0, mob_rep=1, reverse=False)

    p1, p2, p3, p4, p5, p6 = {}, {}, {}, {}, {}, {}
    if '-c' not in args:
        p1 = {'position': '40.0,30.0,0.0'}
        p2 = {'position': '40.0,40.0,0.0'}
        p3 = {'position': '155.0,25.0,0.0'}
        p4 = {'position': '50.0,40.0,0.0'}
        p5 = {'position': '75.0,75.0,0.0'}
        p6 = {'position': '155.0,120.0,0.0'}

    net.mobility(sta1, 'start', time=1, **p1)
    net.mobility(sta2, 'start', time=2, **p2)
    net.mobility(sta3, 'start', time=4, **p3)
    net.mobility(sta1, 'stop', time=12, **p4)
    net.mobility(sta2, 'stop', time=30, **p5)
    net.mobility(sta3, 'stop', time=50, **p6)
    net.stopMobility(time=60)

def start_network(net, c1, aps, s1, s2, s3, s4, s5, s6):
    "Start the network."
    info("*** Starting network\n")
    net.build()
    c1.start()
    for ap in aps:
        ap.start([c1])
    s1.start([c1])
    s2.start([c1])
    s3.start([c1])
    s4.start([c1])
    s5.start([c1])
    s6.start([c1])

def stop_network(net):
    "Stop the network."
    info("*** Stopping network\n")
    net.stop()

def RI_SDN(net):
    "Determine the mobility anchor and choose the route."
    nodes = net.stations + net.aps + net.switches

    def adjacency_matrix():
        "Create the adjacency matrix of the network."
        num_nodes = len(nodes)
        adj_matrix = np.zeros((num_nodes, num_nodes))
        for i, node in enumerate(nodes):
            for j, neighbor in enumerate(nodes):
                if neighbor in node.connectionsTo(nodes):
                    adj_matrix[i, j] = 1
        return adj_matrix

    def most_connected_nodes(adj_matrix):
        "Determine the set of most connected nodes (SMC)."
        connections = adj_matrix.sum(axis=1)
        max_connections = np.max(connections)
        smc = np.where(connections == max_connections)[0]
        return smc

    def determine_anchor(smc, adj_matrix):
        "Determine the mobility anchor."
        if len(smc) == 1:
            return nodes[smc[0]]

        for i in smc:
            for j in smc:
                if i != j and adj_matrix[i, j] == 0:
                    return nodes[i]
        
        distances = np.sum(adj_matrix, axis=1)
        min_distance = np.min(distances)
        sdc = np.where(distances == min_distance)[0]
        
        if len(sdc) == 1:
            return nodes[sdc[0]]
        
        smc_connections = adj_matrix[sdc].sum(axis=1)
        max_smc_connections = np.max(smc_connections)
        anchor_candidates = np.where(smc_connections == max_smc_connections)[0]
        return nodes[anchor_candidates[0]]

    def choose_route():
        "Choose the route for the network traffic."
        # Implement the logic for route selection
        pass

    adj_matrix = adjacency_matrix()
    smc = most_connected_nodes(adj_matrix)
    anchor = determine_anchor(smc, adj_matrix)
    info(f"Mobility anchor is: {anchor.name}")

    choose_route()

def topology(args):
    "Create a network."
    rows = 2
    cols = 3
    for arg in args:
        if arg.startswith('--rows='):
            rows = int(arg.split('=')[1])
        elif arg.startswith('--cols='):
            cols = int(arg.split('=')[1])

    net = Mininet_wifi()

    h1, h2, sta1, sta2, sta3, s1, s2, s3, s4, s5, s6, c1 = create_nodes(net)
    aps = create_access_points(net, rows, cols)

    configure_network(net)
    create_links(net, h1, h2, aps, s1, s2, s5, s6)
    setup_mobility(net, args, sta1, sta2, sta3)
    start_network(net, c1, aps, s1, s2, s3, s4, s5, s6)
    
    info("*** Running RI_SDN\n")
    RI_SDN(net)
    
    info("*** Running CLI\n")
    CLI(net)

    stop_network(net)

if __name__ == '__main__':
    setLogLevel('info')
    topology(sys.argv)
