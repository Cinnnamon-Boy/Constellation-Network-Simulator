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

import torch
import torch.nn.functional as F
from torch import nn
from torch_geometric.nn import GATv2Conv, TopKPooling
from torch.distributions import Categorical
from torch_geometric.nn import global_max_pool as gmp
from torch_geometric.nn import global_mean_pool as gap
from torch.distributions import Normal
import numpy as np

'''Class created to configure the structure for our neural networks'''


def hidden_init(layer):
    fan_in = layer.weight.data.size()[0]
    lim = 1. / np.sqrt(fan_in)
    return (-lim, lim)




class CategoricalMasked(Categorical):
    def __init__(self, probs=None, logits=None, validate_args=None, masks=[]):
        self.masks = masks
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if len(self.masks) == 0:
            super(CategoricalMasked, self).__init__(probs, logits, validate_args)
        else:
            self.masks = masks.type(torch.BoolTensor).to(device)
            if logits != None:
                logits = torch.where(self.masks, logits, torch.tensor(-1e+8).to(device))
            if probs != None:
                probs = torch.where(self.masks, probs, torch.tensor(0).to(device))
            super(CategoricalMasked, self).__init__(probs, logits, validate_args)

    def GetProbs(self):
        return self.probs
    def GetLogits(self):
        return self.logits


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
        self.GATConv1 = GATv2Conv(in_channels=node_feature_dim, out_channels=num_hid, concat=False, heads=num_heads,
                                  negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool1 = TopKPooling(num_hid, ratio=0.8)
        self.GATConv2 = GATv2Conv(in_channels=num_hid, out_channels=num_hid, concat=False, heads=num_heads,
                                  negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool2 = TopKPooling(num_hid, ratio=0.8)
        self.FCLayer1 = nn.Linear(2*num_hid, num_hid)
        self.FCLayer2 = nn.Linear(num_hid, num_hid)
        self.mean = nn.Linear(num_hid, num_route)
        self.log_std = nn.Linear(num_hid, num_route)
        self.LOG_STD_MAX = 2
        self.LOG_STD_MIN = -20
        self.dropout = dropout
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def forward(self, state):
        x, edge_index, edge_attr, batch = state.x, state.edge_index, state.edge_attr, state.batch
        x = F.relu(self.GATConv1(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool1(x, edge_index, edge_attr, batch)
        x = F.relu(self.GATConv2(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool2(x, edge_index, edge_attr, batch)
        x = torch.cat([gmp(x, batch), gap(x, batch)], dim=1)
        x = F.relu(self.FCLayer1(x))
        F.dropout(x, p=self.dropout, training=self.training)
        x = F.relu(self.FCLayer2(x))
        F.dropout(x, p=self.dropout, training=self.training)
        mean = self.mean(x)
        log_std = self.log_std(x)
        log_std = torch.clamp(log_std, self.LOG_STD_MIN, self.LOG_STD_MAX)
        std = torch.exp(log_std)
        dist = Normal(mean, std)
        out = dist.sample()
        out = torch.softmax(out,dim=1)
        return out

    def evaluate(self, state, mask, epsilon=1e-8):
        action_probs = self.forward(state)
        dist = CategoricalMasked(probs=action_probs, masks=mask.view(mask.shape[0], 4))
        action_probs_masked = dist.GetProbs()
        log_prob = dist.GetLogits()
        log_action_pi = torch.sum(log_prob * action_probs_masked, dim=1)
        return action_probs_masked, log_action_pi

    def get_det_action(self, state, mask):
        action_probs = self.forward(state)
        dist = CategoricalMasked(probs=action_probs, masks=mask.view(mask.shape[0], 4))
        action_probs_masked = dist.GetProbs()
        return action_probs_masked


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
        super(Critic, self).__init__()
        self.GATConv1 = GATv2Conv(in_channels=node_feature_dim, out_channels=num_hid, concat=False, heads=num_heads,
                                  negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.GATConv2 = GATv2Conv(in_channels=num_hid, out_channels=num_hid, concat=False, heads=num_heads,
                                  negative_slope=alpha, dropout=dropout, edge_dim=edge_feature_dim)
        self.pool1 = TopKPooling(num_hid, ratio=0.8)
        self.pool2 = TopKPooling(num_hid, ratio=0.8)
        self.FCLayer1 = nn.Linear(2*num_hid + num_route, num_hid)
        self.FCLayer2 = nn.Linear(num_hid, num_hid)
        self.FCLayer3 = nn.Linear(num_hid, 1)
        self.dropout = dropout
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.reset_parameters()

    def reset_parameters(self):
        self.FCLayer1.weight.data.uniform_(*hidden_init(self.FCLayer1))
        self.FCLayer2.weight.data.uniform_(*hidden_init(self.FCLayer2))
        self.FCLayer3.weight.data.uniform_(-3e-3, 3e-3)

    def forward(self, state, action):
        x, edge_index, edge_attr, batch = state.x, state.edge_index, state.edge_attr, state.batch
        x = F.relu(self.GATConv1(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool1(x, edge_index, edge_attr, batch)
        x = F.relu(self.GATConv2(x=x, edge_index=edge_index, edge_attr=edge_attr))
        x, edge_index, edge_attr, batch, _, _ = self.pool2(x, edge_index, edge_attr, batch)
        x = torch.cat([gmp(x, batch), gap(x, batch), action], dim=1)
        x = F.relu(self.FCLayer1(x))
        F.dropout(x, p=self.dropout, training=self.training)
        x = F.relu(self.FCLayer2(x))
        F.dropout(x, p=self.dropout, training=self.training)
        out = self.FCLayer3(x)
        return out
