/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024 SJTU China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: HaiLong Su
 *
 */

#include "reinforcement-learning-single-forward.h"
#include "satellite-routing-tag.h"
#define eps 1e-12
namespace ns3 {

    NS_OBJECT_ENSURE_REGISTERED (ReinforcementSingleForward);
    NS_LOG_COMPONENT_DEFINE ("ReinforcementSingleForward");

    TypeId
    ReinforcementSingleForward::GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::ReinforcementSingleForward")
                .SetParent<ReinforcementLearningArbiter> ()
                .SetGroupName("RoutingRL")
        ;
        return tid;
    }



    ReinforcementSingleForward::ReinforcementSingleForward(
            Ptr<Node> this_node,
            NodeContainer nodes,
            Ptr<TopologySatellite> satTopology,
            Ptr<MultiAgentGymEnvRouting> agentGymEnv,
            std::vector<std::vector<uint32_t>>  routing_table
    ) : ReinforcementLearningArbiter(this_node, nodes, satTopology)
    {
        //!<ns-3gym environment
        m_agentGymEnv = agentGymEnv;
        //!<Period of sending link states to neighbors
        m_period_gather_neighbors = satTopology->GetPeriodInformationGathering();
        //!<Information from neighbors
        m_information_neighbors = std::vector<double>(56, 0.0);
        m_information_neighbors_1 = std::vector<double>(56, 0.0);
        m_information_neighbors_2 = std::vector<double>(56, 0.0);
        m_information_neighbors_3 = std::vector<double>(56, 0.0);
        m_information_neighbors_0 = std::vector<double>(56, 0.0);
        //!<Static Routing
        m_routing_table = routing_table;
        //!<routing type
        m_rotingType = satTopology->GetRoutingType();
        m_neighbor_ISL_state = {ISLState::WORK,ISLState::WORK,ISLState::WORK,ISLState::WORK};
        m_neighbor_queue_size ={0,0,0,0};
        m_final_mask = {0,0,0,0};
        m_actual_mask = {0,0,0,0};
        m_send_vector = {0,0,0,0,0};
        m_receive_vector = {0,0,0,0,0};
        m_mean_queue_length = {0.0,0.0,0.0,0.0};
        m_count_for_queue_length = 0.0;
        m_times_of_using_RL = 0;
        m_num_masks = 0;
        uint32_t first_device_Id_to_neighbor = m_neighbor_node_id_to_if_idx[m_neighborID.at(0)];
        Ptr<LaserNetDevice> first_device_to_neighbor = m_topology->GetNodes().Get(m_node_id)->GetDevice(first_device_Id_to_neighbor)->GetObject<LaserNetDevice>();
        m_max_queue_size = first_device_to_neighbor->GetQueue()->GetMaxSize().GetValue();
        m_disconnection = false;
        Simulator::Schedule(Seconds(0.0),&ReinforcementSingleForward::BuildSockets,this);
        Simulator::Schedule(MilliSeconds(1.0),&ReinforcementSingleForward::BroadCastLinkState,this);
    }

    ReinforcementSingleForward::~ReinforcementSingleForward() {
        // Left empty intentionally
    }

    void
    ReinforcementSingleForward::RecordInterfaces()
    {
        //!<Avoid calling GetObject<A> repeatedly;
        for (int i = 0; i < 4 ; ++i) {
            Ptr<Node> neighborNode = m_topology->GetSatelliteNodes().Get(m_neighborID.at(i));
            Ptr<ReinforcementSingleForward> reinforceSingleForward = neighborNode->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>();
            m_singleForward_neighbors.push_back(reinforceSingleForward);
            uint32_t the_device_Id_to_neighbor = m_neighbor_node_id_to_if_idx[m_neighborID.at(i)];
            Ptr<LaserNetDevice> the_device_to_neighbor = m_topology->GetSatelliteNodes().Get(m_node_id)->GetDevice(the_device_Id_to_neighbor)->GetObject<LaserNetDevice>();
            m_laserDevice_neighbors.push_back(the_device_to_neighbor);
        }
        m_capacity = m_topology->GetCapacity();
        for (uint32_t i = 0; i < m_capacity; ++i) {
            Ptr<ServiceLinkNetDevice> SLDevice = m_topology->GetServiceLinkNetDevices().Get(i + m_node_id * m_capacity)->GetObject<ServiceLinkNetDevice>();
            int SLDevice_nodeId =  SLDevice->GetNode()->GetId();
            NS_ASSERT(SLDevice_nodeId == m_node_id);
            m_service_linkDevices.push_back(SLDevice);
        }
        NS_ASSERT(m_singleForward_neighbors.size()==4);
        NS_ASSERT(m_laserDevice_neighbors.size()==4);
        NS_ASSERT(m_service_linkDevices.size()==m_capacity);
        UpdatingPacketCount();
        Simulator::Schedule(Seconds(1.0),&ReinforcementSingleForward::ReSetQueueLength,this);
    }


    void
    ReinforcementSingleForward::ReSetQueueLength()
    {
        for (int i = 0; i < 4; ++i) {
            m_mean_queue_length.at(i) = 0.0;
        }
        m_count_for_queue_length = 0.0;
        Simulator::Schedule(Seconds(1.0),&ReinforcementSingleForward::ReSetQueueLength,this);
    }

    bool
    ReinforcementSingleForward::StrategyWithDisconnection(std::vector <uint32_t> PriorityActions, std::vector <uint32_t> AlternateActions)
    {
        bool found = false;
        for (uint32_t i = 0; i < 4; ++i) {
            if(PriorityActions.at(i)==1 && m_neighbor_ISL_state.at(i) == ISLState::WORK)
            {
                m_next_hop = i;
                m_actual_mask.at(i) = 1;
                m_feasible_actions++;
                found = true;
            }
        }
        
        if(!found) {
            for (int i = 0; i < 4; ++i) {
                if (AlternateActions.at(i) == 1 && m_neighbor_ISL_state.at(i) == ISLState::WORK)
                {
                    if (PriorityActions.at(GetInterfaceAtSameDirection(i)) == 0) {
                        m_next_hop = i;
                        m_actual_mask.at(i) = 1;
                        m_feasible_actions++;
                        found = true;
                    }
                }
            }
        }
        if(!found) {
            for (int i = 0; i < 4; ++i) {
                if (AlternateActions.at(i) == 1 && m_neighbor_ISL_state.at(i) == ISLState::WORK)
                {
                    m_next_hop = i;
                    m_actual_mask.at(i) = 1;
                    m_feasible_actions++;
                    found = true;
                }
            }
        }

        return found;
    }

    void
    ReinforcementSingleForward::RLDecisionMaking(std::vector <uint32_t> PriorityActions, std::vector <uint32_t> AlternateActions) {
        NS_LOG_FUNCTION(this);
        m_feasible_actions = 0;
        m_next_hop = -1;
        m_approach = false;
        m_actual_mask = {0, 0, 0, 0};
        m_final_mask = {0, 0, 0, 0};
        m_wait_reward = false;
        m_drop = false;
        //!< read queue length for decision and calculating average queue length
        for (uint32_t i = 0; i < 4; i++) {
            uint32_t queue_size = m_laserDevice_neighbors[i]->GetQueue()->GetCurrentSize().GetValue();
            m_neighbor_ISL_state.at(i) = m_laserDevice_neighbors[i]->GetDeviceState();
            m_mean_queue_length.at(i) += (double) (queue_size) / (double) (GetMaxQueueLength());
            m_neighbor_queue_size.at(i) = queue_size;
        }
        m_count_for_queue_length += 1.0;

        if(!StrategyWithDisconnection(PriorityActions,AlternateActions))
        {
            m_drop = true;
            for (int i = 0; i < 4; ++i) {
                if(PriorityActions.at(i)==1)
                {
                    m_next_hop = i;
                    m_feasible_actions = 1;
                    m_actual_mask.at(i) = 1;
                    break;
                }
            }
            if(m_next_hop==-1){
                for (int i = 0; i < 4; ++i) {
                    if(AlternateActions.at(i)==1)
                    {
                        m_next_hop = i;
                        m_feasible_actions=1;
                        m_actual_mask.at(i) = 1;
                        break;
                    }
                }
            }
        }

        if(m_feasible_actions==1){
            NS_ASSERT(m_next_hop!=-1);
            m_approach = PriorityActions.at(m_next_hop)==1 ? true:false;
            return;
        }

        //!< The action is decided by reinforcement learning
        for (int i = 0; i < 4; ++i) {
            m_final_mask.at(i) = PriorityActions.at(i) + 2 * m_actual_mask.at(i);
        }

        NS_ASSERT(m_feasible_actions > 1);
        m_wait_reward = true;
        std::string string_mask;
        std::ostringstream oss;
        std::copy(m_final_mask.begin(), m_final_mask.end(), std::ostream_iterator<int>(oss));
        string_mask = oss.str();
        m_action_iter = m_action.find(string_mask);
        //!<If the action is found and the action is still valid:
        if (m_action_iter != m_action.end() && Simulator::Now() -
                std::get<1>(m_action_iter->second) < Seconds(GetGatherPeriod())) {
            //!<find next hop directly.
            m_next_hop = GetActionFromProbability(std::get<0>(m_action_iter->second));
        } else {
            //!<If no action is found, create a new map of the reward and action
            if (m_action_iter == m_action.end()) {
                m_reward.insert(std::pair<std::string, double>(string_mask, 0.0));
                m_count.insert(std::pair<std::string, uint32_t>(string_mask, 0));
                //!< Return the state and reward to python
                GetGymEnvRouting()->ObserveNow(m_node_id, 0.0, m_neighborID, m_final_mask);
                //!< get next hop
                std::vector<double> action_probability = GetGymEnvRouting()->GetNewProbability();
                m_next_hop = GetActionFromProbability(action_probability);
                //!< create a new map of <mask, <action,time>>
                m_action.insert(std::pair < std::string, std::tuple < std::vector < double > ,
                                Time >> (string_mask, std::make_tuple(action_probability, Simulator::Now())));
                m_times_of_using_RL++;
                m_num_masks++;
            } else {
                //!< Dynamic routing entry is expired.
                m_reward_iter = m_reward.find(string_mask);
                NS_ASSERT(m_reward_iter != m_reward.end());
                m_count_iter = m_count.find(string_mask);
                NS_ASSERT(m_count_iter != m_count.end());
                double Reward = m_reward_iter->second;
                //!< Prevents divisor from being 0
                if (m_count_iter->second >= 1) {
                    Reward = Reward / (double) (m_count_iter->second);
                }
                //!< Return the state and reward to agent
                //!< Call the policy network.
                GetGymEnvRouting()->ObserveNow(m_node_id, Reward, m_neighborID, m_final_mask);
                //!< get next hop
                std::vector<double> action_probability = GetGymEnvRouting()->GetNewProbability();
                m_next_hop = GetActionFromProbability(action_probability);
                //!< Set the reward to 0 and prepare to count the reward in the next time period.
                m_reward_iter->second = 0.0;
                m_count_iter->second = 0;
                //!< erase packet reward tracer
                for (m_packet_action_iter = m_packet_action.begin(); m_packet_action_iter != m_packet_action.end();) {
                    if (m_packet_action_iter->second == string_mask) {
                        m_packet_action_iter = m_packet_action.erase(m_packet_action_iter);
                    } else {
                        ++m_packet_action_iter;
                    }
                }
                //!< set new entry with  <action_mask <action, time>> pair
                m_action_iter->second = std::make_tuple(action_probability, Simulator::Now());
                m_times_of_using_RL++;
            }
        }
        NS_ASSERT(m_next_hop >= 0 && m_next_hop <= 3);
        m_approach = PriorityActions.at(m_next_hop)==1 ? true:false;
        m_drop = m_neighbor_queue_size.at(m_next_hop)+1>=GetMaxQueueLength();
        return;
    }

    int32_t
    ReinforcementSingleForward::TopologySatelliteDecide(
            int32_t source_node_id,
            int32_t target_node_id,
            const std::set<int64_t>& neighbor_node_ids,
            ns3::Ptr<const ns3::Packet> pkt,
            ns3::Ipv4Header const &ipHeader,
            bool is_socket_request_for_source_ip,
            bool read_static_route_directly)
    {

        NS_LOG_FUNCTION (this);
        NS_ASSERT(m_topology->IsSatelliteId(source_node_id)&&m_topology->IsSatelliteId(target_node_id));
        if(read_static_route_directly){
             return m_routing_table.at(target_node_id).at(0);
        }
        SatelliteRoutingTag routingTag;
        NS_ASSERT(pkt->PeekPacketTag(routingTag));
        int loop_action =-1;
        //!<From which neighbor.
        uint32_t neighbor_id = routingTag.GetLastNodeID();
        resultLastDecision result;

        //!< If the last hop is ground equipment then do not consider
        if(!m_topology->IsUserTerminalId(neighbor_id)){
            std::vector<uint32_t>::iterator it=find(m_neighborID.begin(),m_neighborID.end(),neighbor_id);
            NS_ASSERT(it!=m_neighborID.end());
            //!<Remember that blocking this neighbor cause packets cannot generate loops.
            loop_action = it - m_neighborID.begin();
        }

        //!<Create two sets of mask, one that can approach the target node and one cannot
        //!< up down left right
        std::vector <uint32_t> mask_approach ={0,0,0,0};
        std::vector <uint32_t> mask_away ={0,0,0,0};
        //!<Identify behaviors that can approach the target by looking up the static routing table
        size_t size = m_routing_table.at(target_node_id).size();
        for (size_t i = 0; i < size; ++i) {
            std::vector<uint32_t>::iterator it=find(m_neighborID.begin(),m_neighborID.end(),m_routing_table.at(target_node_id).at(i));
            NS_ASSERT(it!=m_neighborID.end());
            mask_approach.at(it-m_neighborID.begin()) = 1;
        }

        NS_ASSERT(mask_approach.size()==4);
        //!<Assigning values to mask_away
        for(int i=0; i<4; ++i)
        {
            if (mask_approach.at(i)==0)
            {
                mask_away.at(i) =1;

            }
            if(mask_approach.at(i)==1)
            {
                mask_away.at(i) =0;
            }
        }
        NS_ASSERT(mask_away.size()==4);
        //!<Blocking loop behavior
        if(loop_action!=-1){
            mask_approach.at(loop_action) = 0;
            mask_away.at(loop_action) = 0;
        }

        //!<Agent needs to track every action， mask and reward
        RLDecisionMaking(mask_approach, mask_away);
        BroadcastTag broadcastTag;
        //!< wait for feedback of reward
        if(GetGymEnvRouting()->Training()&&!(pkt->PeekPacketTag(broadcastTag)) && m_wait_reward && CalculateRemainSteps(m_node_id, target_node_id) >= 2){
            std::string string_mask;
            std::ostringstream oss;
            std::copy(m_final_mask.begin(), m_final_mask.end(), std::ostream_iterator<int>(oss));
            string_mask = oss.str();
            //!< Record the packet and wait for the feedback of reward
            m_packet_action.insert(std::pair<uint32_t, std::string>(routingTag.GetId(), string_mask));
        }


        if(m_drop == true)
        {
            result = resultLastDecision::Drop;
        }else{
            if(m_approach == true)
                result = resultLastDecision::ApproachingTarget;
            else
                result = resultLastDecision::AwayFromTarget;
        }
        UpdatingRoutingTagReturnReward(pkt,result,m_next_hop);
        uint32_t next_satellite = m_neighborID.at(m_next_hop);
        return next_satellite;
    }


    void
    ReinforcementSingleForward::UpdatingRoutingTagReturnReward
            (ns3::Ptr<const ns3::Packet> pkt,
             resultLastDecision result, uint32_t nextSatellite)
    {
        SatelliteRoutingTag routingTag;
        NS_ASSERT(pkt->PeekPacketTag(routingTag));
        //In order to return a reward for two steps we need to do a reading,
        //if this is time interval has been recorded in the tag, then a two-step reward
        //will be returned to the satellite counting backwards two steps
        bool will_return_reward = false;
        uint32_t reward_receiver_1=0;
        uint32_t reward_receiver_2=0;
        uint32_t reward_receiver_count=0;
        uint32_t time_interval_1=0;
        uint32_t time_interval_2=0;
        uint32_t channel_quality_1=0;
        uint32_t channel_quality_2=0;
        routingTag.SetSteps(routingTag.GetSteps()+1);
        //!< This packet from a ground equipment
        if(m_topology->IsUserTerminalId(routingTag.GetLastNodeID()))
        {
            routingTag.SetTimeInterval_1(0);
            routingTag.SetSatellite_1(m_node_id);
            channel_quality_1 = (uint32_t)(m_laserDevice_neighbors.at(nextSatellite)->GetDataRateDecayFactor()*10000);
            routingTag.SetChannelQuality_1(channel_quality_1);
            if(result==resultLastDecision::Drop)
            {
                //return bad decision
                will_return_reward = true;
                reward_receiver_1 = routingTag.GetSatelliteID_1();
                reward_receiver_count=1;
            }
        }
            //!< This package from the a satellite
        else{
            //!<this is the second satellite
            if(routingTag.GetTimeInterval_1() == 0)
            {
                routingTag.SetTimeInterval_1((uint32_t)Simulator::Now().GetMicroSeconds() - routingTag.GetTimeStamp());
                routingTag.SetSatellite_2(m_node_id);
                channel_quality_2 = (uint32_t)(m_laserDevice_neighbors.at(nextSatellite)->GetDataRateDecayFactor()*10000);
                routingTag.SetChannelQuality_2(channel_quality_2);
                if(result==resultLastDecision::AwayFromTarget||result==resultLastDecision::Drop)
                {
                    //return bad decision
                    will_return_reward = true;
                    reward_receiver_count=2;
                    reward_receiver_1 = routingTag.GetSatelliteID_1();
                    reward_receiver_2 = routingTag.GetSatelliteID_2();
                }
            }
            //!<This packet has went through two satellites,this the third satellite
            else if(routingTag.GetTimeInterval_1() != 0 && routingTag.GetTimeInterval_2() == 0)
            {
                will_return_reward = true;
                routingTag.SetTimeInterval_2((uint32_t)Simulator::Now().GetMicroSeconds() - routingTag.GetTimeStamp());
                routingTag.SetSatellite_3(m_node_id);
                reward_receiver_1 = routingTag.GetSatelliteID_1();
                reward_receiver_count=1;
                time_interval_1 = routingTag.GetTimeInterval_1();
                time_interval_2 = routingTag.GetTimeInterval_2();
                channel_quality_1 = routingTag.GetChannelQuality_1();
                channel_quality_2 = routingTag.GetChannelQuality_2();
                routingTag.SetChannelQuality_1(routingTag.GetChannelQuality_2());
                routingTag.SetChannelQuality_2((uint32_t)(m_laserDevice_neighbors.at(nextSatellite)->GetDataRateDecayFactor()*10000));
            }
            else {
                //!< back shift
                will_return_reward = true;
                reward_receiver_1 = routingTag.GetSatelliteID_1();
                reward_receiver_count=1;
                time_interval_1 = routingTag.GetTimeInterval_1();
                time_interval_2 = routingTag.GetTimeInterval_2();
                channel_quality_1 = routingTag.GetChannelQuality_1();
                channel_quality_2 = routingTag.GetChannelQuality_2();
                routingTag.SetTimeInterval_1(routingTag.GetTimeInterval_2());
                routingTag.SetTimeInterval_2((uint32_t)Simulator::Now().GetMicroSeconds() - routingTag.GetTimeStamp());
                routingTag.SetChannelQuality_1(routingTag.GetChannelQuality_2());
                routingTag.SetChannelQuality_2((uint32_t)(m_laserDevice_neighbors.at(nextSatellite)->GetDataRateDecayFactor()*10000));
                routingTag.SetSatellite_1(routingTag.GetSatelliteID_2());
                routingTag.SetSatellite_2(routingTag.GetSatelliteID_3());
                routingTag.SetSatellite_3(m_node_id);

            }
        }
        //!<Set new time stamp
        routingTag.SetTimeStamp ((uint32_t)Simulator::Now().GetMicroSeconds());
        //!<Set new last node
        routingTag.SetLastNodeID(m_node_id);

        if(will_return_reward&&GetGymEnvRouting()->Training()){
            //!<return this two-step reward
            uint32_t packet_Id = routingTag.GetId();

            m_topology->GetSatelliteNodes().Get(reward_receiver_1)->GetObject<Ipv4>()
                ->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()
                    ->GetObject<ReinforcementSingleForward>()->ReceiveReward
                        (packet_Id, time_interval_1,time_interval_2, channel_quality_1,channel_quality_2, result);

            if(reward_receiver_count==2){
                m_topology->GetSatelliteNodes().Get(reward_receiver_2)->GetObject<Ipv4>()
                        ->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()
                        ->GetObject<ReinforcementSingleForward>()->ReceiveReward(packet_Id, time_interval_1,
                                                                                 time_interval_2, channel_quality_1,channel_quality_2, result);
            }
        }

        //!<Prepare to replace the new tag
        m_laserDevice_neighbors.at(nextSatellite)->SetRoutingTag(routingTag);
    }


    void
    ReinforcementSingleForward::ReceiveReward(uint32_t packet_Id, uint32_t time_interval_1, uint32_t time_interval_2,uint32_t channel_quality_1, uint32_t channel_quality_2 ,resultLastDecision result)
    {
        m_packet_action_iter = m_packet_action.find(packet_Id);
        double max_delay = 18000.0; //us
        double min_delay = 2200.0; //us
       // double max_quality = 200.0; // in this version we do not use packet error rate anymore, because we have the channel capacity(transmission rate)
        if(m_packet_action_iter!=m_packet_action.end()){
            double reward = 0.0;
            double reward_delay = 0.0;
            //double reward_quality= 0.0;
            //!<The distance of inter-orbit ISL is only one third of that of
            //!< intra-orbit ISL. If the distance weight is set too large,
            //!< the algorithm will degenerate to the shortest distance.
            if(result == resultLastDecision::ApproachingTarget)
            {

                if((double)time_interval_1<max_delay)
                    reward_delay += (max_delay-(double)time_interval_1)/(max_delay - min_delay);
                else
                    reward_delay += 0.0;
                if((double)time_interval_2<max_delay)
                    reward_delay += (max_delay-(double)time_interval_2)/(max_delay - min_delay);
                else
                    reward_delay += 0.0;
            }
            //punishment for selfish routing
            else if(result == resultLastDecision::AwayFromTarget)
            {
                reward_delay = -1.0;
            }else{
                NS_ASSERT(result == resultLastDecision::Drop);
                //std::cout<<"drop one packet in reward"<<std::endl;
                reward_delay = -6.0;
            }
            reward = 0.5*reward_delay;
            std::string mask_string = m_packet_action_iter->second;
            m_reward_iter = m_reward.find(mask_string);
            m_count_iter = m_count.find(mask_string);
            //!< This mask string is sure to be found
            NS_ASSERT(m_reward_iter != m_reward.end());
            NS_ASSERT (m_count_iter != m_count.end());
            //!< Cumulative reward
            m_reward_iter->second += reward;
            m_count_iter->second +=1;
            //!< delete this packet-action item
            m_packet_action.erase(m_packet_action_iter);
        }
    }


    std::vector<uint32_t>
    ReinforcementSingleForward::GetPacketsSentCount()const
    {
        return m_send_vector;
    }

    std::vector<uint32_t>
    ReinforcementSingleForward::GetPacketsReceivedCount()const
    {
        return m_receive_vector;
    }

    void
    ReinforcementSingleForward::UpdatingPacketCount()
    {
        for (int i = 0; i < 4; ++i) {
            std::vector<uint32_t> new_count = m_laserDevice_neighbors[i]->GetTotalPacketCount();
            m_receive_vector[i] = new_count[0] - m_receive_vector[i];
            m_send_vector[i] = new_count[1] - m_send_vector[i];
        }
        uint32_t received_count_SL = 0;
        uint32_t sent_count_SL = 0;
        for(uint32_t i = 0; i < m_capacity; ++i)
        {
            std::vector<uint32_t> new_count = m_service_linkDevices[i] -> GetTotalPacketCount();
            received_count_SL += new_count[0];
            sent_count_SL += new_count[1];
        }
        m_receive_vector[4] = received_count_SL - m_receive_vector[4];
        m_send_vector[4] = sent_count_SL - m_send_vector[4];
        Simulator::Schedule(Seconds(1.0), &ReinforcementSingleForward::UpdatingPacketCount,this);
    }


    double
    ReinforcementSingleForward::GetGatherPeriod() const
    {
        return m_period_gather_neighbors;
    }

    void
    ReinforcementSingleForward::NotifyDisconnection(bool disconnection)
    {
        m_disconnection = disconnection;
    }

    void
    ReinforcementSingleForward::GatherInformation()
    {
        //!<latitude,longitude date rate to neighbor, packet_queue_length,  relative_distance, relative_speed, ISL_state
        //!<north south west east
        NS_LOG_FUNCTION (this);
        Ptr<Node> currentNode = m_topology->GetSatelliteNodes().Get(m_node_id);
        int feature_vector_position = 0;
        // read latitude and longitude of this satellite and  four neighbor satellites
        for (int i = 0; i < 5 ; ++i) {
            if(i==0){
                Ptr<Satellite> currentSatellite  = m_topology->GetSatellite(m_node_id);
                JulianDate curTime = currentSatellite->GetTleEpoch () + Simulator::Now ();
                Vector3D currentPosition  = currentSatellite->GetGeographicPosition(curTime);
                m_information_neighbors[feature_vector_position++]=currentPosition.x/90.0;
                m_information_neighbors[feature_vector_position++]=currentPosition.y/180.0;
            }else{
                Ptr<Satellite> neighborSatellite = m_topology->GetSatellite(m_neighborID.at(i-1));
                JulianDate curTime = neighborSatellite->GetTleEpoch () + Simulator::Now ();
                Vector3D neighborPosition = neighborSatellite -> GetGeographicPosition(curTime);
                m_information_neighbors[feature_vector_position++]=neighborPosition.x/90.0;
                m_information_neighbors[feature_vector_position++]=neighborPosition.y/180.0;
            }
        }

        // read data rate of neighbors
        for (int i = 0; i < 4 ; ++i) {
            if(m_neighbor_ISL_state.at(i) == ISLState::SHUTDOWN)
                m_information_neighbors[feature_vector_position++] = 0.0;
            else
                m_information_neighbors[feature_vector_position++] = (double)(m_laserDevice_neighbors[i]->GetDataRate().GetBitRate())/10000000.0;
        }
        // read mean idle ratio of packet queues
        for (int i = 0; i < 4 ; ++i) {
            if(m_count_for_queue_length > 0.0)
            {
                m_information_neighbors[feature_vector_position++] = 1.0 - m_mean_queue_length.at(i)/m_count_for_queue_length;
            }else
            {
                m_information_neighbors[feature_vector_position++] = 0.0;
            }
        }

        // read relative distance
        for (int i = 0; i < 4 ; ++i) {
            Ptr<MobilityModel> currentMobility  = currentNode->GetObject<MobilityModel>();
            Ptr<Node> neighborNode = m_topology->GetSatelliteNodes().Get(m_neighborID.at(i));
            Ptr<MobilityModel> neighborMobility = neighborNode->GetObject<MobilityModel>();
            m_information_neighbors[feature_vector_position++] = currentMobility->GetDistanceFrom(neighborMobility);
        }

        // read relative velocity
        for (int i = 0; i < 4 ; ++i) {
            Ptr<MobilityModel> currentMobility  = currentNode->GetObject<MobilityModel>();
            m_information_neighbors[feature_vector_position++] = m_laserDevice_neighbors[i]->GetRelativeSpeed()/280.0;
        }

        // read packet sent of four ISLs
        for (int i = 0; i < 4; ++i) {
            m_information_neighbors[feature_vector_position++] = GetPacketsSentCount()[i];
        }

        // read packet received of four ISLs
        for (int i = 0; i < 4; ++i) {
            m_information_neighbors[feature_vector_position++] = GetPacketsReceivedCount()[i];
        }

        // read packet sent of Service links (mine and neighbors)
        for (int i = 0; i < 5 ; ++i)
        {   if(i==0) {
                m_information_neighbors[feature_vector_position++] = GetPacketsSentCount()[4];
            }else {
                m_information_neighbors[feature_vector_position++] = m_singleForward_neighbors[i-1]-> GetPacketsSentCount()[4];
            }
        }

        // read packet received of Service links (mine and neighbors)
        for (int i = 0; i < 5 ; ++i)
        {
            if(i==0){
                m_information_neighbors[feature_vector_position++] = GetPacketsReceivedCount()[4];
            }else
            {
                m_information_neighbors[feature_vector_position++] = m_singleForward_neighbors[i-1]-> GetPacketsReceivedCount()[4];
            }
        }
        // read dynamic routes
        std::vector<double> busyness = CountDynamicRoutes();
        NS_ASSERT(busyness.size()==4);
        for (int i = 0; i < 4 ; ++i)
        {
            m_information_neighbors[feature_vector_position++] = busyness.at(i);
        }
        // read current queue length
        for (int i = 0; i < 4 ; ++i)
        {
            m_information_neighbors[feature_vector_position++] = (double)(m_neighbor_queue_size.at(i))/(double) (GetMaxQueueLength());
        }

        // read channel quality of neighbors
        for (int i = 0; i < 4 ; ++i) {
            m_information_neighbors[feature_vector_position++] = m_laserDevice_neighbors[i]->GetDataRateDecayFactor();
        }

    }

    Ptr<MultiAgentGymEnvRouting>
    ReinforcementSingleForward::GetGymEnvRouting() {
        return m_agentGymEnv;
    }

    std::vector <double>
    ReinforcementSingleForward::CountDynamicRoutes()
    {

        std::vector <double> busyness = {0.0,0.0,0.0,0.0};
        std::map <std::string, std::tuple<std::vector<double>, Time>>::iterator action_iter = m_action.begin();
        while(action_iter != m_action.end())
        {
            Time record_time = std::get<1>(action_iter->second);
            std::vector<double> action_probability = std::get<0>(action_iter->second);
            if(Simulator::Now() - record_time < Seconds(GetGatherPeriod()))
            {
                for (int i = 0; i < 4; ++i) {
                    busyness.at(i)+=action_probability.at(i);
                }
            }
            action_iter++;
        }
        return busyness;
    }


    std::vector<uint32_t>
    ReinforcementSingleForward::GetNeighborSatellites(){
        return m_neighborID;
    }

    std::vector<double>
    ReinforcementSingleForward::GetNeighborInformation(){
        GatherInformation();
        return m_information_neighbors;
    }

    std::vector<double>
    ReinforcementSingleForward::GetLinkStateTable(uint32_t neighbor)
    {
        switch (neighbor)
        {
            case 0:
                return m_information_neighbors_0;
            case 1:
                return m_information_neighbors_1;
            case 2:
                return m_information_neighbors_2;
            case 3:
                return m_information_neighbors_3;
            default:
                throw std::runtime_error(format_string(
                        "satellite %d get wrong neighbor input Id: %d.", m_node_id, neighbor
                ));
        }
    }

    uint32_t
    ReinforcementSingleForward::GetMaxQueueLength()const {
        return m_max_queue_size;
    }

    uint32_t
    ReinforcementSingleForward::GetTimesUsingRL()const {
        return m_times_of_using_RL;
    }

    uint32_t
    ReinforcementSingleForward::GetNumberOfMasks()const {
        return m_num_masks;
    }

    void
    ReinforcementSingleForward::BuildSockets()
    {
        int count = 0;
        const std::vector<std::pair<uint32_t, uint32_t>>& interface_idxs_for_undirected_edges = m_topology->GetInterfaceIdxsForUndirectedEdges();
        //!< four mappings <neighbor id,interface number to neighbor>.
        for (int i = 0; i < m_topology->GetNumUndirectedEdges(); i++) {
            std::pair<int64_t, int64_t> edge = m_topology->GetUndirectedEdges().at(i);
            if (edge.first == m_node_id) {
                uint32_t remoteId = edge.second;
                Ipv4Address localIpAddress = m_nodes.Get(m_node_id)->GetObject<Ipv4>()->GetAddress(interface_idxs_for_undirected_edges.at(i).first, 0).GetLocal();
                Ipv4Address remoteIpAddress = m_nodes.Get(edge.second)->GetObject<Ipv4>()->GetAddress(interface_idxs_for_undirected_edges.at(i).second, 0).GetLocal();
                m_peersSockets[remoteId] = Socket::CreateSocket (m_nodes.Get(m_node_id), UdpSocketFactory::GetTypeId ());
                m_peersSockets[remoteId]->Bind(InetSocketAddress (localIpAddress, m_RLRoutingPort));
                m_peersSockets[remoteId]->SetAllowBroadcast (false);
                m_peersSockets[remoteId]->SetRecvCallback (MakeCallback (&ReinforcementSingleForward::StoreLinkStateFromNeighbor,this));
                m_peersSockets[remoteId]->Connect (InetSocketAddress (remoteIpAddress, m_RLRoutingPort));
                count ++;

            } else if (edge.second == m_node_id) {
                uint32_t remoteId = edge.first;
                Ipv4Address localIpAddress = m_nodes.Get(m_node_id)->GetObject<Ipv4>()->GetAddress(interface_idxs_for_undirected_edges.at(i).second, 0).GetLocal();
                Ipv4Address remoteIpAddress = m_nodes.Get(edge.first)->GetObject<Ipv4>()->GetAddress(interface_idxs_for_undirected_edges.at(i).first, 0).GetLocal();
                m_peersSockets[remoteId] = Socket::CreateSocket (m_nodes.Get(m_node_id), UdpSocketFactory::GetTypeId ());
                m_peersSockets[remoteId]->Bind(InetSocketAddress (localIpAddress, m_RLRoutingPort));
                m_peersSockets[remoteId]->SetAllowBroadcast (false);
                m_peersSockets[remoteId]->SetRecvCallback (MakeCallback (&ReinforcementSingleForward::StoreLinkStateFromNeighbor,this));
                m_peersSockets[remoteId]->Connect (InetSocketAddress (remoteIpAddress, m_RLRoutingPort));
                count ++;
            }
        }
        if(count!=4){
            throw std::runtime_error(format_string(
                    "The satellites %d didn't record neighbor satellites interface correctly.", m_node_id
            ));
        }
    }

    void
    ReinforcementSingleForward::BroadCastLinkState()
    {
        std::vector<double> neighbor_information = GetNeighborInformation();
        uint32_t count = neighbor_information.size();
        double bData[neighbor_information.size()];
        memcpy(bData, &neighbor_information[0], count * sizeof(double_t));
        Ptr<Packet> packet = Create<Packet> (reinterpret_cast<uint8_t *> (bData), count * sizeof(double_t));
        BroadcastTag broadcastTag;
        broadcastTag.SetSource(m_node_id);
        packet->AddPacketTag(broadcastTag);
        std::map<uint32_t, Ptr<Socket>>::iterator iter = m_peersSockets.begin();
        while(iter!=m_peersSockets.end())
        {
            broadcastTag.SetTarget(iter->first);
            packet->ReplacePacketTag(broadcastTag);
            iter->second->Send(packet);
            iter++;
        }
        Simulator::Schedule(Seconds(GetGatherPeriod()/2.0),&ReinforcementSingleForward::BroadCastLinkState,this);
    }

    void
    ReinforcementSingleForward::StoreLinkStateFromNeighbor(Ptr<Socket> socket)
    {
        Ptr<Packet> pkt;
        while ((pkt = socket->Recv ())) {
            BroadcastTag broadcastTag;
            if(!pkt->PeekPacketTag(broadcastTag))
                continue;
            if ((int)broadcastTag.GetTarget() != m_node_id) {
                continue;
            } else {
                int port;
                for (port = 0; port < 4; port++) {
                    if (m_neighborID.at(port) == broadcastTag.GetSource())
                        break;
                }
                NS_ASSERT(pkt->GetSize()/sizeof(double)==m_information_neighbors_0.size());
                uint8_t bData[pkt->GetSize()];
                pkt->CopyData(bData, sizeof(bData));
                double *bData_double = reinterpret_cast<double *> (bData);
                uint32_t array_size = m_information_neighbors.size();
                switch (port) {
                    case 0:
                        memcpy(&m_information_neighbors_0[0], &bData_double[0], array_size * sizeof(double));
                        break;
                    case 1:
                        memcpy(&m_information_neighbors_1[0], &bData_double[0], array_size * sizeof(double));
                        break;
                    case 2:
                        memcpy(&m_information_neighbors_2[0], &bData_double[0], array_size * sizeof(double));
                        break;
                    case 3:
                        memcpy(&m_information_neighbors_3[0], &bData_double[0], array_size * sizeof(double));
                        break;
                    default:
                        break;
                }
                continue;
            }
        }
    }

    std::string ReinforcementSingleForward::StringReprOfForwardingState() {
        NS_LOG_FUNCTION (this);
        std::ostringstream res;
        res << "State of static routing table: " << m_node_id << std::endl;
        for (int i = 0; i < m_topology->GetNumSatellites(); i++) {
            res << "  -> " << i << ": {";
            bool first = true;
            for (int j : m_routing_table.at(i)) {
                if (!first) {
                    res << ",";
                }
                res << j;
                first = false;
            }
            res << "}" << std::endl;
        }
        return res.str();
    }

}