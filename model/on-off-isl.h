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

#ifndef SATELLITE_NETWORK_ON_OFF_ISL_H
#define SATELLITE_NETWORK_ON_OFF_ISL_H

#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/command-line.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ptr.h"
#include "ns3/double.h"
#include "ns3/mobility-model.h"
#include <iostream>


namespace ns3 {
    class LaserChannel;
    class LaserNetDevice;
    enum class ISLState:unsigned int
    {
        WORK=1,     /**< ISL works properly. >*/
        SHUTDOWN=2     /**<Single point of failure.>*/
    };

    class FreeSpaceOpticsLossModel : public Object
    {
    public:

        //Pr= Pt L Gt Gr Lr Lt
        /**
         * \brief Get the type ID.
         * \return the object TypeId
         */
        static TypeId GetTypeId (void);
        FreeSpaceOpticsLossModel ();
        ~FreeSpaceOpticsLossModel();

        /**
         * \param waveLength (m)
         *
         * Set the carrier wavelength used in the FPO model
         * calculation.
         */
        void SetWavelength (double wavelength);


        /**
        * \param resolution (Rad)
        *
        * Set the resolution used in the FPO model
        * calculation.
        */
        void SetResolution (double resolution);

        /**
        * \param apertureDiameter (m)
        *
        * Set the aperture diameter used in the FPO model （m）
        * calculation.
        */
        void SetApertureDiameter (double apertureDiameter);

        /**
        * \param bandwidth (Hz)
        *
        * Set the bandwidth in the FPO model （Hz）
        * calculation.
        */
        void SetBandwidth (double bandwidth);

        /**
        *
        * Get the resolution (rad) used in the FPO model.
        */
        double GetResolution (void) const;

        /**
        *
        * Get the aperture diameter (m) used in the FPO model.
        */
        double GetApertureDiameter (void) const;

        /**
         * \returns the current waveLength (m)
         */
        double GetWavelength (void) const;

        /**
         * \returns the current bandwidth (Hz)
         */
        double GetBandwidth(void) const;

        /**
         * \returns the current channel capacity (bps)
         */
        double GetChannelCapacity (Ptr<MobilityModel> a, Ptr<MobilityModel> b, bool is_intraLISL);

    private:
        double DoCalcSNR (Ptr<MobilityModel> a, Ptr<MobilityModel> b);
        double m_lambda;        //!< the carrier wavelength
        double m_frequency;     //!< the carrier frequency
        double m_resolution;    //!< the resolution of FPO device.
        double m_apertureDiameter; //!< the apertureDiameter of FPO device.
        double m_bandwidth;  //!< the bandwidth of the laser/light.
        Ptr<UniformRandomVariable> m_uniformRandomVariable;
    };

    class PowerLossModel : public Object
    {
    public:
        static TypeId GetTypeId (void);
        PowerLossModel(Ptr<LaserChannel> laserChannel, double meanInterval, double meanDuration, bool usingSPOF, bool usingLoss);
        ~PowerLossModel();
        ISLState GetISLState() const;
        void SetIntraOrInterOrbitISL(bool intraOrbitISL);
        bool IsIntraOrbitISL() const;
        void UpdatingRoutingStrategy(bool disconnection);
    private:
        void SPOFEnd();
        void ScheduleSPOF();
        void SetISLSPOF(Time duration);
        void ShutDownISL();
        void BootUpISL();
        void CalculateChannelCapacity();
        Ptr<ExponentialRandomVariable> m_faultInterval;
        Ptr<NormalRandomVariable> m_faultDuration;
        ISLState m_state;
        Ptr<LaserNetDevice> m_Device_a;
        Ptr<LaserNetDevice> m_Device_b;
        Ptr<FreeSpaceOpticsLossModel> m_FPO_model;
        bool m_isIntraOrbitISL;
        bool m_usingSPOFModel;
        bool m_usingLossModel;
        double m_transmissionRate;
    };
}

#endif //SATELLITE_NETWORK_ON_OFF_ISL_H
