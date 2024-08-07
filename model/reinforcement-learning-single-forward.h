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


#ifndef REINFORCEMENT_LEARNING_SINGLE_FORWARD_H
#define REINFORCEMENT_LEARNING_SINGLE_FORWARD_H


#include "reinforcement-learning-arbiter.h"
#include "multi-agent-env.h"
#include "on-off-isl.h"



namespace ns3 {
    enum class resultLastDecision : uint32_t{
        ApproachingTarget = 0,
        AwayFromTarget = 1,
        Drop = 2
    };


    class MultiAgentGymEnvRouting;
    class ReinforcementSingleForward : public ReinforcementLearningArbiter
    {
    public:
        static TypeId GetTypeId (void);

        //!< Constructor for single forward next-hop forwarding state
        ReinforcementSingleForward(
                Ptr<Node> this_node,
                NodeContainer nodes,
                Ptr<TopologySatellite> satTopology,
                Ptr<MultiAgentGymEnvRouting> agentGymEnv,
                std::vector<std::vector<uint32_t>>  routing_table
        );

        virtual ~ReinforcementSingleForward();

        //!< Single forward next-hop implementation
        int32_t TopologySatelliteDecide(
                int32_t source_node_id,
                int32_t target_node_id,
                const std::set<int64_t>& neighbor_node_ids,
                ns3::Ptr<const ns3::Packet> pkt,
                ns3::Ipv4Header const &ipHeader,
                bool is_socket_request_for_source_ip,
                bool read_static_route_directly
        );
        //!< print static routing table
        std::string StringReprOfForwardingState();
        //!<Get the ID of neighbor satellites to fill observe space.
        std::vector<uint32_t> GetNeighborSatellites();
        //!<Return vector of neighbor information.
        std::vector<double> GetNeighborInformation();
        //!<The real function to gather information.
        void GatherInformation();
        //!<Get environment of ns3-gym
        Ptr<MultiAgentGymEnvRouting> GetGymEnvRouting();
        //!<Get period of gather information
        double GetGatherPeriod() const;
        //!< Get packet sent and received of four ISLs and service links
        std::vector<uint32_t> GetPacketsSentCount()const;
        std::vector<uint32_t> GetPacketsReceivedCount()const;


        /**
        * Decide where the packet to be delivered from feasible actions.
        * @param PriorityActions actions that can close to target
        * @param AlternateActions actions that will deviate from target to bypass obstacle.
        */
        void RLDecisionMaking(std::vector <uint32_t> PriorityActions,
                              std::vector <uint32_t> AlternateActions);

        /**
        * Evenly share packets based on queue length.
        * @param PriorityActions actions that can close to target
        * @param AlternateActions actions that will deviate from target.
        * @return Does it find feasible actions.
        */
        bool StrategyWithoutDisconnection(std::vector <uint32_t> PriorityActions, std::vector <uint32_t> AlternateActions);
        bool StrategyWithDisconnection(std::vector <uint32_t> PriorityActions, std::vector <uint32_t> AlternateActions);

        /**
        * Return the reward to the agent, while updating the routingTag.
        * @param pkt pointer of packet
        * @param resultLastDecision Result of this route caused by the last decision
        * @param nextSatellite satellite id
        */
        void UpdatingRoutingTagReturnReward(ns3::Ptr<const ns3::Packet> pkt,
                                            resultLastDecision result, uint32_t nextSatellite);
        /**
         * Record the reward received by the agent to the corresponding action.
         * @param packet_Id unique Id of packet.
         * @param time_interval_1 first interval caused by decision of this agent.
         * @param time_interval_2 second interval caused by decision of next agent.
         * @param channel_quality_1 first channel quality caused by decision of this agent.
         * @param channel_quality_2 second channel quality caused by decision of next agent.
         * @param result  result caused by this agent, approach, away, lose  (chain reaction)
        */
        void
        ReceiveReward(uint32_t packet_Id, uint32_t time_interval_1, uint32_t time_interval_2,uint32_t channel_quality_1, uint32_t channel_quality_2 ,resultLastDecision result);


        /**
        * return the reward after normalization.
        * @param time_interval time interval in microseconds.
        * @return reward.
        */
        double CalculateReward(uint32_t time_interval);

        /**
        * The counter is updated every 1.0 seconds
        *
        */
        void UpdatingPacketCount();


        /**
        * Easy to read neighbor information.
        */
        void RecordInterfaces();

        /**
        * Reset queue length tracer every 1.0 seconds.
        */
        void ReSetQueueLength();

        /**
        * Get maximum queue length.
        * @return Max queue length.
        */
        uint32_t GetMaxQueueLength()const;

        /**
        * Count weights of dynamic routes assigned to each ISL.
        * @return count vector
        */
        std::vector <double>  CountDynamicRoutes();

        /**
        * Verify that RL is not being called frequently.
        * @return the times of using RL.
        */
        uint32_t GetTimesUsingRL()const;

        /**
        * Verify that RL is not being called frequently.
        * @return the number of masks.
        */
        uint32_t GetNumberOfMasks()const;

        /**
         * When disconnection attacks, Start obstacle avoidance
         */
        void NotifyDisconnection(bool interference);

        /**
         * Broadcast Link State periodically
         */
        void BroadCastLinkState();

        /**
        * Store link state from neighbor.
        */
        void StoreLinkStateFromNeighbor(Ptr<Socket> socket);
        void BuildSockets();

        std::vector<double> GetLinkStateTable(uint32_t neighbor);



    private:
        //!<ns3-gym environment
        Ptr<MultiAgentGymEnvRouting> m_agentGymEnv;
        //!<Link information from neighbors
        std::vector<double> m_information_neighbors;
        //!<information of neighbors collected by neighbor 0 (second-order)
        std::vector<double> m_information_neighbors_0;
        //!<information of neighbors collected by neighbor 1 (second-order)
        std::vector<double> m_information_neighbors_1;
        //!<information of neighbors collected by neighbor 2 (second-order)
        std::vector<double> m_information_neighbors_2;
        //!<information of neighbors collected by neighbor 3 (second-order)
        std::vector<double> m_information_neighbors_3;
        std::vector<Ptr<ReinforcementSingleForward>> m_singleForward_neighbors;
        std::vector<Ptr<LaserNetDevice>> m_laserDevice_neighbors;
        std::vector<Ptr<ServiceLinkNetDevice>> m_service_linkDevices;
        uint32_t m_capacity;
        //!<period for acquiring new strategies at the end of training
        double m_period_gather_neighbors;
        //!<static routing table
        typedef std::vector<std::vector<uint32_t>> routing_table;
        routing_table m_routing_table;
        routing_table::iterator m_routes_iter;
        //!<m_rotingType
        RoutingProtocol m_rotingType;
        //!< mapping for masks and Corresponding rewards.
        std::map <std::string, double> m_reward;
        std::map <std::string, double>::iterator m_reward_iter;
        std::map <std::string, uint32_t> m_count;
        std::map <std::string, uint32_t>::iterator m_count_iter;
        //!< mapping for masks, Corresponding action and valid time. That is, dynamic routing tables
        std::map <std::string, std::tuple<std::vector<double>, Time>> m_action;
        std::map <std::string, std::tuple<std::vector<double>, Time>>::iterator m_action_iter;
        //!< The number of packet sent and received of service links and four ISL links.
        std::vector<uint32_t> m_send_vector;
        std::vector<uint32_t> m_receive_vector;
        //!< mapping for packet id and Corresponding action.
        //!< trace rewards.
        std::map <uint32_t, std::string> m_packet_action;
        std::map <uint32_t, std::string>::iterator m_packet_action_iter;
        //!< Only actions that require a choice from two or three direction will need a reward return
        bool m_wait_reward;
        //!< Record ISL state of four neighbors
        std::vector <ISLState> m_neighbor_ISL_state;
        //!< Record Queue size state of four neighbors
        std::vector <uint32_t> m_neighbor_queue_size;
        uint32_t  m_max_queue_size;
        //!< key for finding routing entry.
        std::vector <uint32_t> m_final_mask;
        //!< For calculating mean queue length.
        std::vector <double> m_mean_queue_length;
        double m_count_for_queue_length;
        uint32_t m_times_of_using_RL;
        uint32_t m_num_masks;
        //!< next hop for single forward.
        int m_next_hop;
        //!< Is next hop will approach target.
        bool m_approach;
        //!< drop this packet
        bool m_drop;
        //!< next hop will be chosen form m_actual_mask.
        std::vector <uint32_t> m_actual_mask;
        //!< Number of feasible next.
        uint32_t m_feasible_actions;
        //!< Is ISLs are disturbed.
        bool m_disconnection;
    };
}
#endif //REINFORCEMENT_LEARNING_SINGLE_FORWARD_H