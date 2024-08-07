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
#include "multi-agent-env.h"
#include "ns3/queue.h"
#include "ns3/data-rate.h"
#include "satellite-routing-tag.h"
#include "reinforcement-learning-single-forward.h"
namespace ns3 {
    uint32_t packetGlobalID = 0;
    NS_LOG_COMPONENT_DEFINE ("ns3::MultiAgentGymEnvRouting");
    NS_OBJECT_ENSURE_REGISTERED (MultiAgentGymEnvRouting);
    TypeId
    MultiAgentGymEnvRouting::GetTypeId (void)
    {
        static TypeId tid = TypeId ("MultiAgentGymEnvRouting")
                .SetParent<OpenGymEnv> ()
                .SetGroupName ("OpenGym")
                .AddConstructor<MultiAgentGymEnvRouting> ()
        ;
        return tid;
    }

    MultiAgentGymEnvRouting::MultiAgentGymEnvRouting ()
    {
        NS_LOG_FUNCTION (this);
    }

    MultiAgentGymEnvRouting::MultiAgentGymEnvRouting (Ptr<TopologySatellite> SatelliteTopology)
    {
        NS_LOG_FUNCTION (this);
        m_satelliteTopology  = SatelliteTopology;
        m_decideSuccessfully = false;
        m_initialized        = false;
        m_training_network   = false;
        m_probability = {1.0, 0.0, 0.0, 0.0};
        m_action_mask ={0,0,0,0};
    }

    MultiAgentGymEnvRouting::~MultiAgentGymEnvRouting ()
    {
        NS_LOG_FUNCTION (this);
    }

    void
    MultiAgentGymEnvRouting::DoDispose ()
    {
        NS_LOG_FUNCTION (this);
    }

    void
    MultiAgentGymEnvRouting::SetSimulationStart(){
        m_initialized = true;
    }

    uint32_t
    MultiAgentGymEnvRouting::GetUniqueId(){
        return packetGlobalID++;
    }

    void
    MultiAgentGymEnvRouting::Initialization()
    {
        m_openGymInterface->SetGetActionSpaceCb( MakeCallback (&MultiAgentGymEnvRouting::GetActionSpace, this) );
        m_openGymInterface->SetGetObservationSpaceCb( MakeCallback (&MultiAgentGymEnvRouting::GetObservationSpace, this ) );
        m_openGymInterface->SetGetGameOverCb( MakeCallback (&MultiAgentGymEnvRouting::GetGameOver, this) );
        m_openGymInterface->SetGetObservationCb( MakeCallback (&MultiAgentGymEnvRouting::GetObservation, this) );
        m_openGymInterface->SetGetRewardCb( MakeCallback (&MultiAgentGymEnvRouting::GetReward,this) );
        m_openGymInterface->SetGetExtraInfoCb( MakeCallback (&MultiAgentGymEnvRouting::GetExtraInfo, this) );
        m_openGymInterface->SetExecuteActionsCb( MakeCallback (&MultiAgentGymEnvRouting::ExecuteActions ,this) );
    }

    bool
    MultiAgentGymEnvRouting::Training() const
    {
        return  m_training_network;
    }


    Ptr<OpenGymSpace>
    MultiAgentGymEnvRouting::GetObservationSpace()
    {
        /*The number of inter satellite links(ISL) per satellite is 4.
       We adopt this structure ,each satellite has four neighbors.*/
        uint32_t dim_num_feature_vector = 56;
        uint32_t agentID_mask = 5;


        std::vector<uint32_t> agent_ID_shape = {agentID_mask,};
        std::vector<uint32_t> feature_vector_shape = {dim_num_feature_vector,};

        std::string dtype_double = TypeNameGet<double> ();
        std::string dtype_uint_64  = TypeNameGet<uint64_t> ();

        Ptr<OpenGymBoxSpace> space_agent_ID = CreateObject<OpenGymBoxSpace> (0,UINT64_MAX, agent_ID_shape, dtype_uint_64);

        // Five feature vectors
        Ptr<OpenGymBoxSpace> space_local_satellite_feature_vector = CreateObject<OpenGymBoxSpace> (-999.0, 999.0, feature_vector_shape, dtype_double);
        Ptr<OpenGymBoxSpace> space_neighbor1_feature_vector = CreateObject<OpenGymBoxSpace> (-999.0, 999.0, feature_vector_shape, dtype_double);
        Ptr<OpenGymBoxSpace> space_neighbor2_feature_vector = CreateObject<OpenGymBoxSpace> (-999.0, 999.0, feature_vector_shape, dtype_double);
        Ptr<OpenGymBoxSpace> space_neighbor3_feature_vector = CreateObject<OpenGymBoxSpace> (-999.0, 999.0, feature_vector_shape, dtype_double);
        Ptr<OpenGymBoxSpace> space_neighbor4_feature_vector = CreateObject<OpenGymBoxSpace> (-999.0, 999.0, feature_vector_shape, dtype_double);


        Ptr<OpenGymDictSpace> state_space = CreateObject<OpenGymDictSpace> ();
        state_space->Add("agent ID and mask", space_agent_ID);
        state_space->Add("Feature vector of current satellite", space_local_satellite_feature_vector);
        state_space->Add("Feature vector of first neighbor satellite", space_neighbor1_feature_vector);
        state_space->Add("Feature vector of second neighbor satellite", space_neighbor2_feature_vector);
        state_space->Add("Feature vector of third neighbor satellite", space_neighbor3_feature_vector);
        state_space->Add("Feature vector of fourth neighbor satellite", space_neighbor4_feature_vector);
        NS_LOG_UNCOND ("MyGetObservationSpace: " << state_space);
        return state_space;
    }

    Ptr<OpenGymSpace>
    MultiAgentGymEnvRouting::GetActionSpace()
    {
        std::vector<uint32_t> agent_ID_shape = {2,}; //agent_id[0-num of agents-1],  training_end(0 is training 1 denotes end training)
        std::vector<uint32_t> probability_vector_shape = {4,};
        std::string dtype_uint_32  = TypeNameGet<uint32_t> ();
        std::string dtype_float = TypeNameGet<float> ();
        Ptr<OpenGymBoxSpace> agent_ID = CreateObject<OpenGymBoxSpace> (0, 99999, agent_ID_shape, dtype_uint_32);
        Ptr<OpenGymBoxSpace> probability = CreateObject<OpenGymBoxSpace> (0, 1.0, probability_vector_shape, dtype_float);
        Ptr<OpenGymDictSpace> action_space = CreateObject<OpenGymDictSpace> ();
        action_space->Add("AgentID", agent_ID);
        action_space->Add("Probability", probability);
        return action_space;
    }

    void
    MultiAgentGymEnvRouting::ObserveNow (uint32_t agent_ID,double reward, std::vector<uint32_t> neighborNodeList,
                                         std::vector <uint32_t> action_mask)
    {
        m_reward = reward;
        m_agentID = agent_ID;
        m_action_mask = action_mask;
        m_neighborNodeList = neighborNodeList;
        m_openGymInterface->NotifyCurrentState();
    }

    bool
    MultiAgentGymEnvRouting::GetGameOver()
    {
        return Simulator::IsFinished();
    }

    Ptr<OpenGymDataContainer>
    MultiAgentGymEnvRouting::GetObservation(void)
    {

        uint32_t dim_num_feature_vector = 56;
        uint32_t agentID_mask = 5;
        std::vector<uint32_t> agentID_shape = {agentID_mask,};
        std::vector<uint32_t> feature_vector_shape = {dim_num_feature_vector,};
        Ptr<OpenGymBoxContainer<uint64_t>> space_agent_ID = CreateObject<OpenGymBoxContainer<uint64_t>>(agentID_shape);

        if(!m_initialized){
            std::vector<uint32_t> start_singal_shape = {1,};
            Ptr<OpenGymBoxContainer<uint64_t>> notify_simulation_start = CreateObject<OpenGymBoxContainer<uint64_t>>(start_singal_shape);
            if(m_satelliteTopology->GetRoutingType() == RoutingProtocol::SoftActorCritic)
                notify_simulation_start->AddValue(1111);
            return notify_simulation_start;
        }
        //It is guaranteed that the next hop address returned by DQN is based on the judgment of the current task(Trajectory)
        space_agent_ID->AddValue(m_agentID);
        for (int i = 0; i < 4; ++i) {
            space_agent_ID->AddValue(m_action_mask[i]);
        }
        /************Read observe space *************/
        Ptr<Node> currentNode  = m_satelliteTopology->GetSatelliteNodes().Get(m_agentID);
        std::vector<double> local_features  = currentNode->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>()->GetNeighborInformation();
        std::vector<double> Neigh1_features = currentNode->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>()->GetLinkStateTable(0);
        std::vector<double> Neigh2_features = currentNode->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>()->GetLinkStateTable(1);
        std::vector<double> Neigh3_features = currentNode->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>()->GetLinkStateTable(2);
        std::vector<double> Neigh4_features = currentNode->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>()->GetLinkStateTable(3);
        Ptr<OpenGymBoxContainer<double>> space_local_feature_vector     = ConvertInformation(local_features , feature_vector_shape);
        Ptr<OpenGymBoxContainer<double>> space_neighbor1_feature_vector = ConvertInformation(Neigh1_features, feature_vector_shape);
        Ptr<OpenGymBoxContainer<double>> space_neighbor2_feature_vector = ConvertInformation(Neigh2_features, feature_vector_shape);
        Ptr<OpenGymBoxContainer<double>> space_neighbor3_feature_vector = ConvertInformation(Neigh3_features, feature_vector_shape);
        Ptr<OpenGymBoxContainer<double>> space_neighbor4_feature_vector = ConvertInformation(Neigh4_features, feature_vector_shape);
        /************End of Read*************/
        Ptr<OpenGymTupleContainer> state = CreateObject<OpenGymTupleContainer> ();
        state->Add(space_agent_ID);
        state->Add(space_local_feature_vector);
        state->Add(space_neighbor1_feature_vector);
        state->Add(space_neighbor2_feature_vector);
        state->Add(space_neighbor3_feature_vector);
        state->Add(space_neighbor4_feature_vector);
        return state;
    }

    Ptr<OpenGymBoxContainer<double>>
    MultiAgentGymEnvRouting::ConvertInformation(std::vector<double> dataVector, std::vector<uint32_t> featureVectorShape)
    {
        //target satellite
        Ptr<OpenGymBoxContainer<double>> feature_vector = CreateObject<OpenGymBoxContainer<double>>(featureVectorShape);
        for(std::vector<double>::iterator it = dataVector.begin(); it != dataVector.end(); ++it)
        {
            feature_vector->AddValue((*it));
        }
        return feature_vector;
    }

    float
    MultiAgentGymEnvRouting::GetReward(void)
    {
        if(!m_initialized){
            return 0.0;
        }else{
            //std::cout<<"GetReward"<<m_reward<<std::endl;
            return m_reward;
        }
    }


    bool
    MultiAgentGymEnvRouting::ExecuteActions(Ptr<OpenGymDataContainer> action)
    {

        Ptr<OpenGymDictContainer> dict = DynamicCast<OpenGymDictContainer>(action);
        Ptr<OpenGymBoxContainer<uint32_t>> agent_ID = DynamicCast<OpenGymBoxContainer<uint32_t> > (dict->Get("AgentID"));
        Ptr<OpenGymBoxContainer<float >> Probability = DynamicCast<OpenGymBoxContainer<float> > (dict->Get("Probability"));
        m_training_network = (agent_ID->GetValue(1) == 0);
        if(!m_initialized){
            m_training_network = (agent_ID->GetValue(1) == 0);
            if(m_training_network)
                std::cout<<"  > Training Neural Network"<<std::endl;
            else
                std::cout<<"  > Using trained models directly"<<std::endl;
            return true;
        }
        if(m_agentID == agent_ID->GetValue(0))
        {
            m_decideSuccessfully = true;
            for (int i = 0; i < 4; ++i) {
                m_probability.at(i) = Probability->GetValue(i);
                //std::cout<<" "<<m_probability.at(i);
            }
            //std::cout<<" "<<std::endl;
        }
        else{
            m_decideSuccessfully = false;
        }

        return m_decideSuccessfully;
    }

    std::string
    MultiAgentGymEnvRouting::GetExtraInfo ()
    {
        std::string Extra_info = "ExtraInfo";
        NS_LOG_INFO("ExtraInfo Simulation time : " << Simulator::Now().GetSeconds()<<" s");
        return Extra_info;
    }

    std::vector<double>
    MultiAgentGymEnvRouting::GetNewProbability() const
    {
        if(m_decideSuccessfully){
            return m_probability;
        }else{
            throw std::runtime_error("Decision making failure!");
        }
    }

}