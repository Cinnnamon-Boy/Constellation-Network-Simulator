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
from ns3gym import ns3env
from collections import namedtuple
import json
import neural_network
import random
import numpy as np
import wandb
from collections import deque
from utils import *
from torch_geometric.loader import DataLoader
import Constellation
import glob
import argparse

with open('Setting.json') as f:
    setting = json.load(f)

Experience = namedtuple(
    'Experience',
    ('state', 'action', 'next_state', 'reward','done')
)

def get_config():
    parser = argparse.ArgumentParser(description='RL')
    parser.add_argument("--run_name", type=str, default="SAC", help="Run name, default: SAC")
    parser.add_argument("--episodes", type=int, default=15, help="Number of episodes, default: 50")
    parser.add_argument("--save_every", type=int, default=5, help="Saves the network every x epochs, default: 10")
    args = parser.parse_args()
    return args

class CreateEnviroment(object):
    # Constructor, Get spaces of gym
    def __init__(self, portID=5555, startSim=True, simSeed=1, train=True, offLine=True ) -> None:
        self.env = ns3env.Ns3Env(port=portID, startSim=startSim, simSeed=simSeed)
        self.env.reset()
        self.getSpaces()
        self.finished = False
        self.action_size = setting['NETWORK']['num_out_route']
        self.data = dict()
        self.config = {
            "batch_size": setting['NETWORK']['memory_batch_size'],
            "gamma": setting['AGENT']['gamma_for_next_q_val'],
            "sample_memory": setting['AGENT']['use_random_sample_memory'],
            "recent_memory": setting['AGENT']['use_most_recent_memory'],
            "priority_memory": setting['AGENT']['use_priority_memory'],
        }
        self.config_training = get_config()
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        self.agent_count = 0
        self.step = 0
        self.average10 = deque(maxlen=10)
        self.total_steps = 0
        self.train = train
        self.offLine = offLine
        self.rewards_total = 0
        self.episodes = 0

    def getSpaces(self) -> None:
        observe_space = self.env.observation_space
        action_space = self.env.action_space
        print("Observation space: ", observe_space, observe_space.dtype)
        print("Action space: ", action_space, action_space.dtype)

    def GetSamples(self):
        '''check which type of memories to pll'''
        if self.config['sample_memory']:
            experiences = self.Networks.replay_memory.sample(
                self.config['batch_size'])
        elif self.config['recent_memory']:
            experiences = self.Networks.replay_memory.take_recent(
                self.config['batch_size'])
        elif self.config['priority_memory']:
            experiences = self.Networks.replay_memory.take_priority(
                self.config['batch_size'])
        '''extract values from experiences'''
        return self.extract_tensors(experiences)

    def save(self,save_name, model):
        import os
        save_dir = './trained_models/'
        print('Save The Model')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        torch.save(model.state_dict(), save_dir + save_name)

    ''' helper function to extract values from our stored experiences'''
    def extract_tensors(self, experiences):
        states = list(exps[0] for exps in experiences)
        next_states = list(tuple(exps[2] for exps in experiences))
        current_loader = DataLoader(states, batch_size=self.config['batch_size'], shuffle=False)
        next_loader = DataLoader(next_states,  batch_size=self.config['batch_size'], shuffle=False)
        states = next(iter(current_loader))
        next_states = next(iter(next_loader))
        actions = torch.FloatTensor(np.array(list(exps[1].cpu().detach().numpy() for exps in experiences)))
        rewards = torch.cat(
            tuple(torch.tensor([exps[3]]) for exps in experiences))
        done = torch.cat(
            tuple(torch.tensor([exps[4]]) for exps in experiences))
        return (states, actions, next_states, rewards, done)

    def start_train(self):
        if self.NetType == "SAC":
            self.start_train_SAC()
        else:
            self.start_train_DQN()

    def start_train_SAC(self):
        print("start SAC training")
        for i in range(self.config_training.episodes):
            while (self.step < 20):
                experiences = self.GetSamples()
                policy_loss, alpha_loss, bellmann_error_1, bellmann_error_2,current_alpha = self.Networks.learn(
                    self.step, experiences, gamma=self.config['gamma'])
                self.step += 1
                self.total_steps += 1
                self.average10.append(self.rewards_total)
                # wandb.log({
                #     "Steps": self.total_steps,
                #     "Policy Loss": policy_loss,
                #     "Alpha Loss": alpha_loss,
                #     "Bellmann error 1": bellmann_error_1,
                #     "Bellmann error 2": bellmann_error_2,
                #     "Alpha": current_alpha,
                #     "Episode": self.episodes})
            self.episodes += 1
            print("Episode: {} | Polciy Loss: {} | bellmann_error_1:{}  | bellmann_error_2:{}  | alpha_loss:{} | current alpha:{} |Steps: {} ".format(self.episodes, policy_loss, bellmann_error_1, bellmann_error_2, alpha_loss, current_alpha, self.total_steps))
            self.step = 0
            if self.episodes % self.config_training.save_every == 0:
                self.save(save_name="actor_model_parameter.pt", model=self.Networks.actor_net)
        self.train = False

    def start_train_DQN(self):
        print("start DQN training")
        for i in range(self.config_training.episodes):
            while (self.step < 20):
                self.step += 1
                self.total_steps += 1
                experiences = self.GetSamples()
                Q_loss = self.Networks.learn(
                    self.step, experiences, gamma=self.config['gamma'])
                # wandb.log({
                #     "Steps": self.total_steps,
                #     "Bellmann error 1": Q_loss,
                #     "Episode": self.episodes})
            self.episodes += 1
            print("Episode: {} | bellmann error: {} | Steps: {} ".format(self.episodes, Q_loss, self.total_steps))
            self.step = 0
            if self.episodes % self.config_training.save_every == 0:
                self.save(save_name="QPolicy_model_parameter.pt", model=self.Networks.QPolicy)
        self.Networks.epsilon_greedy = 0.99
        self.train = False

    def simulation(self) -> None:
        obs = self.env.reset()
        simulation_start = False
        '''This the signal that tell agent simulation will start'''
        firstsingal = obs[0]
        if firstsingal == 1111 or firstsingal == 2222:
            print("simulation will start...")
            if(firstsingal == 1111):
                self.NetType = "SAC"
                self.config_training.run_name = 'SAC'
                self.Networks = neural_network.NeuralNetwork_SAC()
                print("Using Soft Actor Critic algorithm")
            if(firstsingal == 2222):
                self.NetType = "DQN"
                self.config_training.run_name = 'DQN'
                self.Networks = neural_network.NeuralNetwork_DQN()
                if not self.train:
                    self.Networks.epsilon_greedy = 0.99
                print("Using Deep Q Learning algorithm")
            if self.offLine and self.train:
                success = self.Networks.replay_memory.read_file()
                if success:
                    print("offline learning")
                else:
                    print("online learning")
                    self.offLine = False
        end_train = 0 if self.train else 1

        # with wandb.init(project="Low earth orbit satellites", name="RL-Routing", config=self.config_training):
        #     wandb.watch(self.Networks, log="gradients", log_freq=10)
        while not self.finished:
            if not simulation_start:
                action = self.env.action_space.sample()
                action['AgentID'][1] = end_train
                obs, reward, done, info = self.env.step(action)
                print("simulation start!")
                simulation_start = True
                if self.train:
                    print("Collecting experience...")
                else:
                    print("Using trained models directly...")
            else:
                obs, reward, done, info = self.env.step(self.action)
                self.rewards_total += reward
                self.average10.append(self.rewards_total)
            agentID = obs[0][0]
            request_actions = [0]*4
            actual_actions = [0]*4
            key_mask = [0]*4
            num_request = 0
            num_actual = 0
            print_high_value = False
            for i in range(4):
                key_mask[i] = obs[0][i+1]
                if(key_mask[i]==2):
                    print_high_value = True
                    num_actual+=1
                    actual_actions[i] = 1
                if(key_mask[i]==1):
                    num_request+=1
                    request_actions[i] = 1
                if(key_mask[i]==3):
                    num_request+=1
                    num_actual+=1
                    request_actions[i] = 1
                    actual_actions[i] = 1
            self.finished = done
            if(self.finished):
                print("finished")
                break
            agent_ID_string = "agent_{0}".format(agentID)
            mask_string = "mask_{0}{1}{2}{3}".format(key_mask[0],key_mask[1],key_mask[2],key_mask[3])
            state = Graph_data_construction(list(obs[1:]), request_actions, actual_actions)
            act = self.Networks.get_action(state[0], state[1])
            self.action = self.env.action_space.sample()
            self.action['AgentID'][0] = agentID
            self.action['AgentID'][1] = end_train
            for i in range(4):
                self.action['Probability'][i] = act[i]
            if print_high_value or np.random.uniform(0,1) <= 0.001:
                print("AgentID ",agentID,"get new probability: ",act,"for action mask: ",actual_actions, "priority actions are:", request_actions, "key",mask_string,"reward is", reward)
            if (self.train):
                if agent_ID_string not in self.data:
                    self.agent_count += 1
                    task = dict()
                    state_list = list()
                    reward_list = list()
                    action_list = list()
                    state_list.append(state)
                    action_list.append(act)
                    task[mask_string] = [state_list,action_list,reward_list]
                    self.data[agent_ID_string] = task
                else:
                    if mask_string not in self.data[agent_ID_string]:
                        state_list = list()
                        reward_list = list()
                        action_list = list()
                        state_list.append(state)
                        action_list.append(act)
                        self.data[agent_ID_string][mask_string] = [state_list, action_list, reward_list]
                    else:
                        self.data[agent_ID_string][mask_string][0].append(state)
                        self.data[agent_ID_string][mask_string][1].append(act)
                        self.data[agent_ID_string][mask_string][2].append(reward)
                        self.average10.append(self.rewards_total)
                        # wandb.log({"rewards": self.rewards_total,
                        #            "Average10": np.mean(self.average10)})
                        if(len(self.data[agent_ID_string][mask_string][0])>=1):
                            last_state = self.data[agent_ID_string][mask_string][0][-2]
                            current_state = self.data[agent_ID_string][mask_string][0][-1]
                            action_will_relay = self.data[agent_ID_string][mask_string][1][-2]
                            prob_weight = self.data[agent_ID_string][mask_string][2][-1]
                            if not self.offLine:
                                self.Networks.replay_memory.push(
                                    prob_weight, last_state, action_will_relay, current_state, reward, 0)
                ''' check if our memory bank has sufficient memories to sample from'''
                if self.Networks.replay_memory.can_provide_sample(self.config['batch_size']):
                    self.average10.append(self.rewards_total)
                    print("rewards: {}  | agent count: {}".format(self.rewards_total, self.agent_count))
                    ''' start train'''
                    self.start_train()


        if self.finished:
            self.env.close()
            print("Simulation End")











