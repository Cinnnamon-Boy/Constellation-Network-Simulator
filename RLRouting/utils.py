# * -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
# *
# * Copyright (c) 2023 UCAS China
# *
# * This program is free software; you can redistribute it and/or modify
# * it under the terms of the GNU General Public License version 2 as
# * published by the Free Software Foundation;
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program; if not, write to the Free Software
# * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
# *
# * Author: HaiLong Su
# *
'''
                      node5
                        |
             node6---node1---node7
               |       |      |
    node11---node3---node0---node4---node12
               |       |      |
             node9---node2---node10
                       |
                     node8
                         |   intra orbital ISL
                         --- inter orbital ISL
'''
import math
import torch
from torch_geometric.data import Data


num_orbits = 72
num_satellites_per_orbit = 22
height_of_orbit = 550
num_satellites = num_orbits*num_satellites_per_orbit
dim_edge_feature = 9
dim_node_feature = 12
edge_index = torch.tensor([[0, 1], [0, 2], [0, 3], [0, 4], \
                                [1, 5], [1, 6], [1, 7], [2, 8], \
                                [2, 9], [2, 10], [3, 6], [3, 9], \
                                [3, 11], [4, 7], [4, 10], [4, 12]], dtype=torch.long).t().contiguous()


distance_normalized = 2000000
thousand = 1000


def Graph_data_construction(obs, mask_request, mask_actual):
    node_features = list()
    edge_features = list()

    '''node feature: lat, long, service links sent and received, action mask(8)'''
    '''node0'''
    node_features.append([obs[0][0], obs[0][1], obs[0][34]/thousand, obs[0][39]/thousand,
                          mask_request[0], mask_request[1], mask_request[2], mask_request[3],
                          mask_actual[0], mask_actual[1], mask_actual[2], mask_actual[3]])
    '''node1'''
    node_features.append([obs[1][0], obs[1][1], obs[1][34]/thousand, obs[1][39]/thousand,  0, 0, 0, 0, 0, 0, 0, 0])

    '''node2'''
    node_features.append([obs[2][0], obs[2][1], obs[2][34]/thousand, obs[2][39]/thousand,  0, 0, 0, 0, 0, 0, 0, 0])

    '''node3'''
    node_features.append([obs[3][0], obs[3][1], obs[3][34]/thousand, obs[3][39]/thousand,  0, 0, 0, 0, 0, 0, 0, 0])
    '''node4'''
    node_features.append([obs[4][0], obs[4][1], obs[3][34]/thousand, obs[3][39]/thousand,  0, 0, 0, 0, 0, 0, 0, 0])
    '''node5'''
    node_features.append([obs[1][2], obs[1][3], obs[1][35]/thousand, obs[1][40]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])
    '''node6'''
    node_features.append([obs[1][6], obs[1][7], obs[1][37]/thousand, obs[1][42]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])
    '''node7'''
    node_features.append([obs[1][8], obs[1][9], obs[1][38]/thousand, obs[1][43]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])

    '''node8'''
    node_features.append([obs[2][4], obs[2][5], obs[2][36]/thousand, obs[2][41]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])
    '''node9'''
    node_features.append([obs[2][6], obs[2][7], obs[2][37]/thousand, obs[2][42]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])
    '''node10'''
    node_features.append([obs[2][8], obs[2][9], obs[2][38]/thousand, obs[2][43]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])

    '''node11'''
    node_features.append([obs[3][6], obs[3][7], obs[3][37]/thousand, obs[3][42]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])
    '''node12'''
    node_features.append([obs[4][8], obs[4][9], obs[4][38]/thousand, obs[4][43]/thousand, 0, 0, 0, 0, 0, 0, 0, 0])


    '''edge [data_rate(normalized), Queue idle ratio,  distance, relative speed, packet sent packet receive]'''
    '''edge from node0 to node1'''

    edge_features.append([obs[0][10], obs[0][14], obs[0][18] / distance_normalized, obs[0][22], obs[0][26]/thousand, obs[0][30]/thousand, obs[0][44], obs[0][48], obs[0][52]])

    '''edge from node0 to node2'''
    edge_features.append([obs[0][11], obs[0][15], obs[0][19] / distance_normalized, obs[0][23],  obs[0][27]/thousand, obs[0][31]/thousand, obs[0][45], obs[0][49], obs[0][53]])

    '''edge from node0 to node3'''
    edge_features.append([obs[0][12], obs[0][16], obs[0][20] / distance_normalized, obs[0][24], obs[0][28]/thousand, obs[0][32]/thousand, obs[0][46], obs[0][50], obs[0][54]])
    '''edge from node0 to node4'''
    edge_features.append([obs[0][13], obs[0][17], obs[0][21] / distance_normalized, obs[0][25], obs[0][29]/thousand, obs[0][33]/thousand, obs[0][47], obs[0][51], obs[0][55]])
    '''edge from node1 to node5'''
    edge_features.append([obs[1][10], obs[1][14], obs[1][18] / distance_normalized, obs[1][22], obs[1][26]/thousand, obs[1][30]/thousand, obs[1][44], obs[1][48], obs[1][52]])
    '''edge from node1 to node6'''
    edge_features.append([obs[1][12], obs[1][16], obs[1][20] / distance_normalized, obs[1][24], obs[1][28]/thousand, obs[1][32]/thousand, obs[1][46], obs[1][50], obs[1][54]])
    '''edge from node1 to node7'''
    edge_features.append([obs[1][13], obs[1][17], obs[1][21] / distance_normalized, obs[1][25], obs[1][29]/thousand, obs[1][33]/thousand, obs[1][47], obs[1][51], obs[1][55]])
    '''edge from node2 to node8'''
    edge_features.append([obs[2][11], obs[2][15], obs[2][19] / distance_normalized, obs[2][23], obs[2][27]/thousand, obs[2][31]/thousand, obs[2][45], obs[2][49], obs[2][53]])
    '''edge from node2 to node9'''
    edge_features.append([obs[2][12], obs[2][16], obs[2][20] / distance_normalized, obs[2][24], obs[2][28]/thousand, obs[2][32]/thousand, obs[2][46], obs[2][50], obs[2][54]])

    '''edge from node2 to node10'''
    edge_features.append([obs[2][13], obs[2][17], obs[2][21] / distance_normalized, obs[2][25], obs[2][29]/thousand, obs[2][33]/thousand, obs[2][47], obs[2][51], obs[2][55]])

    '''edge from node3 to node6'''
    edge_features.append([obs[3][10], obs[3][14], obs[3][18] / distance_normalized, obs[3][22], obs[3][26]/thousand, obs[3][30]/thousand, obs[3][44], obs[3][48], obs[3][52]])
    '''edge from node3 to node9'''
    edge_features.append([obs[3][11], obs[3][15], obs[3][19] / distance_normalized, obs[3][23], obs[3][27]/thousand, obs[3][31]/thousand, obs[3][45], obs[3][49], obs[3][53]])
    '''edge from node3 to node11'''
    edge_features.append([obs[3][12], obs[3][16], obs[3][20] / distance_normalized, obs[3][24], obs[3][28]/thousand, obs[3][32]/thousand, obs[3][46], obs[3][50], obs[3][54]])
    '''edge from node4 to node7'''
    edge_features.append([obs[4][10], obs[4][14], obs[4][18] / distance_normalized, obs[4][22], obs[4][26]/thousand, obs[4][30]/thousand, obs[4][44], obs[4][48], obs[4][52]])
    '''edge from node4 to node10'''
    edge_features.append([obs[4][11] , obs[4][15], obs[4][19] / distance_normalized, obs[4][23], obs[4][27]/thousand, obs[4][31]/thousand, obs[4][45], obs[4][49], obs[4][53]])
    '''edge from node4 to node12'''
    edge_features.append([obs[4][13], obs[4][17], obs[4][21] / distance_normalized, obs[4][25], obs[4][29]/thousand, obs[4][33]/thousand, obs[4][47], obs[4][51], obs[4][55]])

    mask_actual = torch.tensor(mask_actual).view(1, 4)
    node_features = torch.tensor(node_features, dtype=torch.float)
    edge_features = torch.tensor(edge_features, dtype=torch.float)
    state = Data(x=node_features, edge_index=edge_index, edge_attr=edge_features)
    return (state, mask_actual)



