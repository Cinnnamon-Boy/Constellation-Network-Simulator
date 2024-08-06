
import torch
import torch.nn.functional as F
from torch import nn
from torch_geometric.nn import GATv2Conv,TopKPooling
from torch_geometric.nn import global_max_pool as gmp
from torch_geometric.nn import global_mean_pool as gap
'''Class created to configure the structure for our neural networks'''

class Actor(nn.Module):

    """
    Initialize a neural network for actor network.
    num_route: the number of route which can be chosen
    num_heads: the number of heads of GAT attention layer
    node_feature_dim : the dimension of feature vector
    num_hid : the dimension of hidden layer
    dropout: the ratio of dropout
    alpha:  Alpha for the leaky_relu
    edge_feature_dim: the dimension of edge feature
    """

    def __init__(self, node_feature_dim, num_hid, edge_feature_dim, num_route, dropout, alpha, num_heads):
        super(Actor, self).__init__()
        self.GATconv1 = GATv2Conv(in_channels=node_feature_dim, out_channels=num_hid, concat=False, heads=num_heads,negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool1 = TopKPooling(num_hid, ratio=0.8)
        self.GATconv2 = GATv2Conv(in_channels=num_hid, out_channels=num_hid, concat=False, heads=num_heads, negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool2 = TopKPooling(num_hid, ratio=0.8)
        self.GATconv3 = GATv2Conv(in_channels=num_hid, out_channels=num_hid, concat=False, heads=num_heads, negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool3 = TopKPooling(num_hid, ratio=0.8)
        self.FCLayer1 = nn.Linear(2*num_hid, num_hid)
        self.FCLayer2 = nn.Linear(num_hid, num_hid)
        self.FCLayer3 = nn.Linear(num_hid, num_route)
        self.dropout = dropout
    def forward(self, data):
        x, edge_index, edge_attr, batch = data.x, data.edge_index, data.edge_attr, data.batch
        x = F.relu(self.GATconv1(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool1(x,edge_index=edge_index,edge_attr=edge_attr,batch=batch)
        x1 = torch.concat([gap(x,batch),gmp(x,batch)],dim=1)
        x = F.relu(self.GATconv2(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool2(x, edge_index=edge_index, edge_attr=edge_attr, batch=batch)
        x2 = torch.concat([gap(x,batch),gmp(x,batch)],dim=1)
        x = F.relu(self.GATconv3(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool3(x, edge_index=edge_index, edge_attr=edge_attr, batch=batch)
        x3 = torch.concat([gap(x,batch),gmp(x,batch)],dim=1)
        readout = x1+x2+x3
        x = F.relu(self.FCLayer1(readout))
        F.dropout(x, p=self.dropout, training=self.training)
        x = F.relu(self.FCLayer2(x))
        F.dropout(x, p=self.dropout, training=self.training)
        out = torch.softmax(self.FCLayer3(x),dim=1)
        return out
    
class Critic(nn.Module):

    """
    Initialize a neural network for critic network.
    num_route: the number of route which can be chosen
    num_heads: the number of heads of GAT attention layer
    node_feature_dim : the dimension of feature vector
    num_hid : the dimension of hidden layer
    dropout: the ratio of dropout
    alpha:  Alpha for the leaky_relu
    edge_feature_dim: the dimension of edge feature
    """

    def __init__(self, node_feature_dim, num_hid, edge_feature_dim, num_route, dropout, alpha, num_heads):
        super(Actor, self).__init__()
        self.GATconv1 = GATv2Conv(in_channels=node_feature_dim, out_channels=num_hid, concat=False, heads=num_heads,negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool1 = TopKPooling(num_hid, ratio=0.8)
        self.GATconv2 = GATv2Conv(in_channels=num_hid, out_channels=num_hid, concat=False, heads=num_heads, negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool2 = TopKPooling(num_hid, ratio=0.8)
        self.GATconv3 = GATv2Conv(in_channels=num_hid, out_channels=num_hid, concat=False, heads=num_heads, negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool3 = TopKPooling(num_hid, ratio=0.8)
        self.FCLayer1 = nn.Linear(2*num_hid, num_hid)
        self.FCLayer2 = nn.Linear(num_hid, num_hid)
        self.FCLayer3 = nn.Linear(num_hid, num_route)
        self.dropout = dropout
    def forward(self, data):
        x, edge_index, edge_attr, batch = data.x, data.edge_index, data.edge_attr, data.batch
        x = F.relu(self.GATconv1(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool1(x,edge_index=edge_index,edge_attr=edge_attr,batch=batch)
        x1 = torch.concat([gap(x,batch),gmp(x,batch)],dim=1)
        x = F.relu(self.GATconv2(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool2(x, edge_index=edge_index, edge_attr=edge_attr, batch=batch)
        x2 = torch.concat([gap(x,batch),gmp(x,batch)],dim=1)
        x = F.relu(self.GATconv3(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool3(x, edge_index=edge_index, edge_attr=edge_attr, batch=batch)
        x3 = torch.concat([gap(x,batch),gmp(x,batch)],dim=1)
        readout = x1+x2+x3
        x = F.relu(self.FCLayer1(readout))
        F.dropout(x, p=self.dropout, training=self.training)
        x = F.relu(self.FCLayer2(x))
        F.dropout(x, p=self.dropout, training=self.training)
        out = self.FCLayer3(x)
        return out
        