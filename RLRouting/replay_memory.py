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
import random
from collections import namedtuple
from operator import itemgetter
import pickle
import os

'''Tuple class created to contain elements for experiences'''

Experience = namedtuple(
    'Experience',
    ('state', 'action', 'next_state', 'reward','done')
)



class ReplayMemory(object):

    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0
        self.samples = 0
        self.prob_weight = []

    def push(self, prob_weight, *args):
        self.samples += 1
        if(self.__len__()%200 ==0):
            print("Have collected {0} experiences, target size: {1}".format(self.__len__(), self.capacity))
        if len(self.memory) < self.capacity:
            self.memory.append(Experience(*args))
            self.prob_weight.append(prob_weight)
        else:
            self.position = (self.position + 1) % self.capacity
            self.memory[self.position] = Experience(*args)
            self.prob_weight[self.position] = prob_weight

    '''Take a random sample of size batch_size from self.memory'''
    def sample(self, batch_size):
        self.count_samples_provided = self.__len__()
        return random.sample(self.memory, batch_size)

    '''Given size batch_size, take the batch_size most recent Experiences. '''
    def take_recent(self, batch_size):
        self.count_samples_provided = self.__len__()
        return self.memory[-batch_size:]

    '''Take a sample of size batch_size with samples which are more 
        different from our model are more likely to be selected. '''
    def take_priority(self,batch_size):
        self.count_samples_provided = self.__len__()
        ind = random.choices(range(len(self.prob_weight)), k=batch_size, weights=self.prob_weight)
        return list(itemgetter(*ind)(self.memory))

    '''return the length of our memory bank '''
    def __len__(self):
        return len(self.memory)

    '''return a boolean if able to provide experiences for learning'''

    def can_provide_sample(self, batch_size):
        if (self.__len__() >= batch_size) and int(self.__len__() >= self.capacity):
            self.write_file()
            return True
        else:
            return False


    def read_file(self):
        file_exists_exp = os.path.exists("./experiences.bin")
        if file_exists_exp:
            with open('experiences.bin', 'rb') as f:
                experiences = pickle.load(f)
                self.memory = list(experiences)
                return True
        else:
            return False

    def write_file(self):
        with open('experiences.bin', 'wb') as f:
            pickle.dump(self.memory, f)








