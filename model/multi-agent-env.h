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

#ifndef MULTI_AGENT_ENV_H
#define MULTI_AGENT_ENV_H
#include "ns3/opengym-module.h"
#include "ns3/core-module.h"
#include "topology-satellites.h"
#include "ns3/ipv4-arbiter-routing.h"
#include <map>


namespace ns3 {
    class TopologySatellite;
    class MultiAgentGymEnvRouting : public OpenGymEnv
    {
    public:
        MultiAgentGymEnvRouting ();
        MultiAgentGymEnvRouting (Ptr<TopologySatellite> SatelliteTopology);
        virtual ~MultiAgentGymEnvRouting ();
        static TypeId GetTypeId (void);
        virtual void DoDispose ();
        Ptr<OpenGymSpace> GetActionSpace();
        Ptr<OpenGymSpace> GetObservationSpace();
        bool GetGameOver();
        Ptr<OpenGymDataContainer> GetObservation();
        Ptr<OpenGymBoxContainer<double>> ConvertInformation(std::vector<double> dataVector,std::vector<uint32_t> featureVectorShape);
        float GetReward();
        std::string GetExtraInfo();
        bool ExecuteActions(Ptr<OpenGymDataContainer> action);
        void ObserveNow (uint32_t agent_ID, double reward,
                         std::vector<uint32_t> neighborNodeList,
                         std::vector <uint32_t> action_mask);
        static uint32_t GetUniqueId();//packet ID
        void Initialization();
        void SetSimulationStart();
        bool Training()const;
        std::vector<double> GetNewProbability()const;

    private:
        Ptr<TopologySatellite> m_satelliteTopology;
        std::vector<uint32_t> m_neighborNodeList;
        double    m_reward;
        uint32_t  m_agentID;
        std::vector<double>  m_probability;
        std::vector <uint32_t> m_action_mask;
        bool      m_decideSuccessfully;
        bool      m_initialized;
        bool      m_training_network;
    };

}
#endif //MULTI_AGENT_ENV_H