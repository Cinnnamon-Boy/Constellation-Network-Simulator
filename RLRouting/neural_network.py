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
from ACN import Actor, Critic
from replay_memory import ReplayMemory
from utils import *
import torch
import torch.optim as optim
import torch.nn.functional as F
import torch.nn as nn
import json
import os
import copy
import numpy as np


'''Open file Setting.json which contains learning parameters. '''
with open('Setting.json') as f:
    setting = json.load(f)

def QValueMasked(Value=None, masks=[]):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    masks = masks.type(torch.BoolTensor).to(device).view(masks.shape[0], -1)
    ValueMasked = torch.where(masks, Value, torch.tensor(0.0).to(device))
    return ValueMasked

class NeuralNetwork_SAC(nn.Module):

    def __init__(self, capacity=setting['NETWORK']['memory_bank_size']):
        super(NeuralNetwork_SAC, self).__init__()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        learning_rate = setting['NETWORK']['optimizer_learning_rate']
        dim_node = dim_node_feature
        dim_edge = dim_edge_feature
        num_hid = setting['NETWORK']['num_hidden']
        num_route = setting['NETWORK']['num_out_route']
        dropout = setting['NETWORK']['dropout']
        alpha = setting['NETWORK']['alpha']
        num_heads = setting['NETWORK']['num_heads_attention']

        self.target_entropy = -num_route  # -dim(A)
        self.clip_grad_param = 1
        self.tau = 1e-2
        self.log_alpha = torch.tensor([0.0], requires_grad=True)
        self.alpha = self.log_alpha.exp().detach()
        self.alpha_optimizer = optim.Adam(params=[self.log_alpha], lr=learning_rate)

        self.actor_net = Actor(dim_node, num_hid, dim_edge, num_route, dropout, alpha, num_heads).to(self.device)
        self.actor_optimizer = optim.Adam(self.actor_net.parameters(), lr=learning_rate)
        self.critic1 = Critic(dim_node, num_hid, dim_edge, num_route, dropout, alpha, num_heads).to(self.device)
        self.critic2 = Critic(dim_node, num_hid, dim_edge, num_route, dropout, alpha, num_heads).to(self.device)
        assert self.critic1.parameters() != self.critic2.parameters()

        self.critic1_target = Critic(dim_node, num_hid, dim_edge, num_route, dropout, alpha, num_heads).to(self.device)
        self.critic1_target.load_state_dict(self.critic1.state_dict())

        self.critic2_target = Critic(dim_node, num_hid, dim_edge, num_route, dropout, alpha, num_heads).to(self.device)
        self.critic2_target.load_state_dict(self.critic2.state_dict())

        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=learning_rate)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=learning_rate)

        file_exists_act = os.path.exists("./trained_models/actor_model_parameter.pt")
        if(file_exists_act):
            print("load parameters of actor model")
            self.actor_net.load_state_dict(torch.load("./trained_models/actor_model_parameter.pt"))
        '''copy the parameter'''
        self.replay_memory = ReplayMemory(capacity)

    def get_action(self, state, mask):
        """Returns actions for given state as per current policy."""
        state = state.to(self.device)
        with torch.no_grad():
            action = self.actor_net.get_det_action(state, mask).detach().to('cpu')[0]
        return action

    def calc_policy_loss(self, states, masks, alpha):

        action_probs, log_action_pi = self.actor_net.evaluate(states, masks)
        q1 = self.critic1(states,action_probs)
        q2 = self.critic2(states,action_probs)
        min_Q = torch.min(q1, q2)
        actor_loss = (self.alpha.to(self.device) * log_action_pi - min_Q).mean()
        return actor_loss, log_action_pi


    def learn(self, step, experiences, gamma, d=1):
        """Updates actor, critics and entropy_alpha parameters using given batch of experience tuples.
        Q_targets = r + γ * (min_critic_target(next_state, actor_target(next_state)) - α *log_pi(next_action|next_state))
        Critic_loss = MSE(Q, Q_target)
        Actor_loss = α * log_pi(a|s) - Q(s,a)
        where:
            actor_target(state) -> action
            critic_target(state, action) -> Q-value
        Params
        ======
            experiences (Tuple[torch.Tensor]): tuple of (s, a, r, s', done) tuples
            gamma (float): discount factor
        """
        states, actions, next_states, rewards, dones = experiences


        # ---------------------------- update actor ---------------------------- #
        current_alpha = copy.deepcopy(self.alpha)
        actor_loss, log_pis = self.calc_policy_loss(states[0].to(self.device),states[1].to(self.device),current_alpha.to(self.device))
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Compute alpha loss
        alpha_loss = - (self.log_alpha.exp() * (log_pis.cpu() + self.target_entropy).detach().cpu()).mean()
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()
        self.alpha = self.log_alpha.exp().detach()

        # ---------------------------- update critic ---------------------------- #
        # Get predicted next-state actions and Q values from target models
        with torch.no_grad():
            next_action, log_pis_next = self.actor_net.evaluate(next_states[0].to(self.device), next_states[1].to(self.device))
            Q_target1_next = self.critic1_target(next_states[0].to(self.device), next_action.squeeze(0).to(self.device))
            Q_target2_next = self.critic2_target(next_states[0].to(self.device), next_action.squeeze(0).to(self.device))
            Q_target_next = torch.min(Q_target1_next, Q_target2_next) - self.alpha.to(self.device) * log_pis_next.reshape(-1, 1)

            # Compute Q targets for current states (y_i)
            Q_targets = rewards.reshape(-1, 1).to(self.device) + (gamma * (1 - dones.reshape(-1, 1).to(self.device)) * Q_target_next)

            # Compute critic loss
        Q_1 = self.critic1(states[0].to(self.device), actions.squeeze(0).to(self.device))
        Q_2 = self.critic2(states[0].to(self.device), actions.squeeze(0).to(self.device))
        critic1_loss = 0.5*F.mse_loss(Q_1, Q_targets.detach())
        critic2_loss = 0.5*F.mse_loss(Q_2, Q_targets.detach())

        ## Update critics
        # critic 1
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()
        # critic 2
        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # ----------------------- update target networks ----------------------- #
        self.soft_update(self.critic1, self.critic1_target)
        self.soft_update(self.critic2, self.critic2_target)

        return actor_loss.item(), alpha_loss.item(), critic1_loss.item(), critic2_loss.item(), current_alpha

    def soft_update(self, local_model, target_model):
        """Soft update model parameters.
        θ_target = τ*θ_local + (1 - τ)*θ_target
        Params
        ======
            local_model: PyTorch model (weights will be copied from)
            target_model: PyTorch model (weights will be copied to)
            tau (float): interpolation parameter
        """
        for target_param, local_param in zip(target_model.parameters(), local_model.parameters()):
            target_param.data.copy_(self.tau * local_param.data + (1.0 - self.tau) * target_param.data)

class NeuralNetwork_DQN(nn.Module):
    def __init__(self, capacity=setting['NETWORK']['memory_bank_size']):
        super(NeuralNetwork_DQN, self).__init__()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        learning_rate = setting['NETWORK']['optimizer_learning_rate']
        dim_node = dim_node_feature
        dim_edge = dim_edge_feature
        num_hid = setting['NETWORK']['num_hidden']
        self.num_route = setting['NETWORK']['num_out_route']
        dropout = setting['NETWORK']['dropout']
        alpha = setting['NETWORK']['alpha']
        num_heads = setting['NETWORK']['num_heads_attention']
        self.epsilon_greedy = setting['NETWORK']['epsilon_greedy']
        self.QPolicy = Critic(dim_node, num_hid, dim_edge, self.num_route, dropout, alpha, num_heads).to(self.device)
        self.QTarget = Critic(dim_node, num_hid, dim_edge, self.num_route, dropout, alpha, num_heads).to(self.device)
        assert self.QPolicy.parameters() != self.QTarget.parameters()
        self.optimizer = optim.Adam(self.QPolicy.parameters(), lr=learning_rate)
        file_exists_QPolicy = os.path.exists("./trained_models/QPolicy_model_parameter.pt")
        self.tau = 1e-2
        if(file_exists_QPolicy):
            print("load parameters of QPolicy model")
            self.QPolicy.load_state_dict(torch.load("./trained_models/QPolicy_model_parameter.pt"))
            '''copy the parameter'''
        self.replay_memory = ReplayMemory(capacity)

    def get_action(self, state, mask):
        """Returns actions for given state as per current policy."""
        state = state.to(self.device)
        if np.random.uniform() < self.epsilon_greedy:
            with torch.no_grad():
                QValue = self.QPolicy.forward(state)
                masks = mask.type(torch.BoolTensor).to(self.device)
                QValue = torch.where(masks, QValue, torch.tensor(0.0).to(self.device))
                action = torch.argmax(QValue).item()
        else:
            Feasible_actions = np.nonzero(mask.squeeze(dim=0)).view(1,-1).squeeze(dim=0)
            action = np.random.choice(Feasible_actions)
        return action


    def learn(self, step, experiences, gamma, d=1):
        states, actions, next_states, rewards, dones = experiences
        # Get predicted next-state actions and Q values from target models
        with torch.no_grad():
            next_QValue = QValueMasked(self.QPolicy(next_states[0].to(self.device)), next_states[1].to(self.device))
            next_action = next_QValue.max(1)[1].view(-1, 1)
            Q_target_next = self.QTarget(next_states[0].to(self.device)).gather(1, next_action)
            # Compute Q targets for current states (y_i)
            Q_star = rewards.reshape(-1,1).to(self.device) + gamma *(1-dones.reshape(-1,1).to(self.device)) * Q_target_next

            # Compute critic loss
        Q = self.QPolicy(states[0].to(self.device)).gather(1, actions.reshape(-1,1).to(self.device))


        QLoss = 0.5 * F.mse_loss(Q, Q_star)
        # Update QPolicy
        self.optimizer.zero_grad()
        QLoss.backward()
        self.optimizer.step()

        # ----------------------- update target networks ----------------------- #
        self.soft_update(self.QPolicy, self.QTarget)

        return QLoss

    def soft_update(self, local_model, target_model):
        """Soft update model parameters.
        θ_target = τ*θ_local + (1 - τ)*θ_target
        Params
        ======
            local_model: PyTorch model (weights will be copied from)
            target_model: PyTorch model (weights will be copied to)
            tau (float): interpolation parameter
        """
        for target_param, local_param in zip(target_model.parameters(), local_model.parameters()):
            target_param.data.copy_(self.tau * local_param.data + (1.0 - self.tau) * target_param.data)




