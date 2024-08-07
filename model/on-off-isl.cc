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

#include "on-off-isl.h"
#include "ns3/laser-channel.h"
#include "ns3/laser-net-device.h"
#include "ns3/satellite-position-helper.h"

#include "ns3/reinforcement-learning-single-forward.h"


namespace ns3{

    NS_LOG_COMPONENT_DEFINE("PowerLossModel");
    NS_OBJECT_ENSURE_REGISTERED(PowerLossModel);
    NS_OBJECT_ENSURE_REGISTERED(FreeSpaceOpticsLossModel);


    TypeId
    FreeSpaceOpticsLossModel::GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::FreeSpaceOpticsLossModel")
                .SetParent<PropagationLossModel> ()
                .SetGroupName ("Propagation")
                .AddConstructor<FreeSpaceOpticsLossModel> ()
                .AddAttribute ("Wavelength",
                               "The carrier Wavelength (in nm) at which propagation occurs  (default is 1150 nm).",
                               DoubleValue (1150.0e-9),
                               MakeDoubleAccessor (&FreeSpaceOpticsLossModel::SetWavelength,
                                                   &FreeSpaceOpticsLossModel::GetWavelength),
                               MakeDoubleChecker<double> ())
                .AddAttribute ("Resolution",
                               "The resolution of FPO device (rad).",
                               DoubleValue (0.5e-6),
                               MakeDoubleAccessor (&FreeSpaceOpticsLossModel::SetResolution,
                                                   &FreeSpaceOpticsLossModel::GetResolution),
                               MakeDoubleChecker<double> ())
                .AddAttribute ("ApertureDiameter",
                               "The aperture diameter of FPO device (m).",
                               DoubleValue (0.05),
                               MakeDoubleAccessor (&FreeSpaceOpticsLossModel::SetApertureDiameter,
                                                   &FreeSpaceOpticsLossModel::GetApertureDiameter),
                               MakeDoubleChecker<double> ())
                .AddAttribute ("Bandwidth",
                               "A band of frequencies used for sending electronic signals.",
                               DoubleValue (1.0e9),
                               MakeDoubleAccessor (&FreeSpaceOpticsLossModel::SetBandwidth,
                                                   &FreeSpaceOpticsLossModel::GetBandwidth),
                               MakeDoubleChecker<double> ())
        ;
        return tid;
    }

    FreeSpaceOpticsLossModel::FreeSpaceOpticsLossModel ()
    {
        m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
        double min =0.0;
        double max =1.0;
        m_uniformRandomVariable->SetAttribute("Min",DoubleValue(min));
        m_uniformRandomVariable->SetAttribute("Max",DoubleValue(max));
    }
    FreeSpaceOpticsLossModel::~FreeSpaceOpticsLossModel()
    {

    }
    void
    FreeSpaceOpticsLossModel::SetResolution(double resolution)
    {
        m_resolution = resolution;
    }

    double
    FreeSpaceOpticsLossModel::GetResolution() const
    {
        return m_resolution;
    }

    void
    FreeSpaceOpticsLossModel::SetApertureDiameter(double apertureDiameter)
    {
        m_apertureDiameter = apertureDiameter;
    }

    double
    FreeSpaceOpticsLossModel::GetApertureDiameter() const
    {
        return m_apertureDiameter;
    }

    void
    FreeSpaceOpticsLossModel::SetBandwidth(double bandwidth)
    {
        m_bandwidth = bandwidth;
    }
    double
    FreeSpaceOpticsLossModel::GetBandwidth() const
    {
        return m_bandwidth;
    }

    void
    FreeSpaceOpticsLossModel::SetWavelength(double wavelength)
    {
        m_lambda = wavelength;
        static const double C = 299792458.0; // speed of light in vacuum
        m_frequency = C / m_lambda;
    }

    double
    FreeSpaceOpticsLossModel::GetWavelength() const
    {
        return m_lambda;
    }

    double
    FreeSpaceOpticsLossModel::DoCalcSNR (Ptr<MobilityModel> a, Ptr<MobilityModel> b)
    {
        double distance = a->GetDistanceFrom (b);
        double numerator = m_lambda * m_lambda;
        double L = numerator / (16 * M_PI * M_PI * distance * distance);
        double Gt = (M_PI * M_PI*m_apertureDiameter*m_apertureDiameter)/numerator;
        double Gr = Gt;
        double uv_1 = m_uniformRandomVariable->GetValue();
        double theta_t = std::sqrt(-2*m_resolution*m_resolution*std::log(1-uv_1));
        double uv_2 = m_uniformRandomVariable->GetValue();
        double theta_r = std::sqrt(-2*m_resolution*m_resolution*std::log(1-uv_2));
        double L_t = std::exp(-1*Gt*theta_t*theta_t);
        double L_r = std::exp(-1*Gr*theta_r*theta_r);
        return L*Gt*Gr*L_t*L_r;
    }

    double
    FreeSpaceOpticsLossModel::GetChannelCapacity(Ptr<MobilityModel> a, Ptr<MobilityModel> b, bool is_intraLISL)
    {
        if(is_intraLISL)
        {

            double ratio_Pt_N = 9.285859635948615e4;
            return m_bandwidth*std::log2(1+ratio_Pt_N*DoCalcSNR(a,b));
        }

        else
        {
            double ratio_Pt_N = 1.20716175267332e4;
            return m_bandwidth*std::log2(1+ratio_Pt_N*DoCalcSNR(a,b));
        }
    }



    TypeId
    PowerLossModel::GetTypeId (void)
    {
        static TypeId tid = TypeId("ns3::PowerLossModel").SetParent<Object>().SetGroupName("RoutingRL");
        return tid;
    }

    PowerLossModel::PowerLossModel(Ptr<LaserChannel> laserChannel, double meanInterval, double meanDuration, bool usingSPOF, bool usingLoss)
    {
        NS_ASSERT(laserChannel->GetLaserDevice(0));
        m_Device_a = laserChannel->GetLaserDevice(0);
        NS_ASSERT(laserChannel->GetLaserDevice(1));
        m_Device_b = laserChannel->GetLaserDevice(1);
        /**<meanInterval and meanDuration of SPoF>**/
        m_faultInterval = CreateObject<ExponentialRandomVariable> ();
        m_FPO_model = CreateObject<FreeSpaceOpticsLossModel>();
        m_faultInterval->SetAttribute("Mean",DoubleValue (meanInterval));
        m_faultDuration = CreateObject<NormalRandomVariable>();
        m_faultDuration->SetAttribute("Mean",DoubleValue (meanDuration));
        m_faultDuration->SetAttribute("Variance",DoubleValue (25));
        m_state = ISLState::WORK;
        m_isIntraOrbitISL = false;
        m_usingLossModel = usingLoss;
        m_usingSPOFModel = usingSPOF;
        if(m_usingSPOFModel)
            ScheduleSPOF();
        if(m_usingLossModel)
            Simulator::Schedule(Seconds(0.0),&PowerLossModel::CalculateChannelCapacity,this);
    }

    PowerLossModel::~PowerLossModel(){
        // Left empty intentionally
    }

    void
    PowerLossModel::SetIntraOrInterOrbitISL(bool intraOrbitISL)
    {
        m_isIntraOrbitISL = intraOrbitISL;
    }

    bool
    PowerLossModel::IsIntraOrbitISL() const
    {
        return m_isIntraOrbitISL;
    }

    void
    PowerLossModel::CalculateChannelCapacity()
    {

        Ptr <MobilityModel> mobility_a = m_Device_a->GetNode()->GetObject<MobilityModel>();
        Ptr <MobilityModel> mobility_b = m_Device_b->GetNode()->GetObject<MobilityModel>();
        m_transmissionRate = m_FPO_model->GetChannelCapacity(mobility_a, mobility_b, m_isIntraOrbitISL);
        //std::cout<<m_Device_a->GetNode()->GetId()<<" -> "<<m_Device_b->GetNode()->GetId()<<" distance: "<< mobility_a->GetDistanceFrom (mobility_b)<<" m"
        //<<" m_transmissionRate "<<m_transmissionRate/1e6<<" Mbps"<<std::endl;
        m_Device_a->SetNewDataTransmissionRate(m_transmissionRate);
        m_Device_b->SetNewDataTransmissionRate(m_transmissionRate);
        Simulator::Schedule(Seconds(10.0), &PowerLossModel::CalculateChannelCapacity, this);

    }

    void
    PowerLossModel::ScheduleSPOF() {
        /**<Schedule next break of LISL**/
        Time errorTime = Seconds(m_faultInterval->GetValue());
        double duration = 0.0;
        while (duration<=0.0){
            duration = m_faultDuration->GetValue();
        }

        Time firstDuration = Seconds(duration);
        Simulator::Schedule(errorTime, &PowerLossModel::SetISLSPOF,this,firstDuration);
    }

    void
    PowerLossModel::SetISLSPOF(Time duration)
    {
        if( m_state == ISLState::WORK){
            ShutDownISL();
            //std::cout<<m_Device_a->GetNode()->GetId()<<std::endl;
            m_Device_a->CumulateBreakTime(duration);
            m_Device_b->CumulateBreakTime(duration);
            UpdatingRoutingStrategy(true);
        }

        Simulator::Schedule(duration, &PowerLossModel::SPOFEnd,this);
        Time nextErrorTime = Seconds(m_faultInterval->GetValue());

        double next_duration = 0.0;
        while (next_duration <= 0.0){
            next_duration = m_faultDuration->GetValue();
        }
        Time nextDuration = Seconds(next_duration);
        nextErrorTime = duration + nextErrorTime;
        Simulator::Schedule(nextErrorTime, &PowerLossModel::SetISLSPOF, this, nextDuration);
    }
    void
    PowerLossModel::SPOFEnd() {
        if (m_state == ISLState::SHUTDOWN)
        {
            BootUpISL();
            UpdatingRoutingStrategy(false);
        }
    }

    ISLState
    PowerLossModel::GetISLState() const
    {
        return m_state;
    }


    void
    PowerLossModel::ShutDownISL()
    {
        m_state = ISLState::SHUTDOWN;
        m_Device_a-> SetNewPacketLossRate (1.0);
        m_Device_b-> SetNewPacketLossRate (1.0);
    }

    void
    PowerLossModel::BootUpISL()
    {
        m_state = ISLState::WORK;
        m_Device_a-> SetNewPacketLossRate (0.0);
        m_Device_b-> SetNewPacketLossRate (0.0);
    }

    void
    PowerLossModel::UpdatingRoutingStrategy (bool LISLBreak)
    {
        Ptr <ReinforcementSingleForward> node_aSingleForward = m_Device_a->GetNode()->GetObject<Ipv4>()->GetRoutingProtocol()
                ->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>();
        Ptr <ReinforcementSingleForward> node_bSingleForward = m_Device_b->GetNode()->GetObject<Ipv4>()->GetRoutingProtocol()
                ->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>();
        if(node_aSingleForward!=NULL&&node_bSingleForward!=NULL)
        {
            node_aSingleForward->NotifyDisconnection(LISLBreak);
            node_bSingleForward->NotifyDisconnection(LISLBreak);
        }
    }

}

