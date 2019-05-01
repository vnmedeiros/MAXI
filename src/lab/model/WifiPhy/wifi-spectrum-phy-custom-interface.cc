/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 CTTC
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 */

#include <ns3/ptr.h>
#include <ns3/object.h>
#include <ns3/net-device.h>
#include <ns3/mobility-model.h>
#include <ns3/wifi-phy.h>
#include <ns3/spectrum-phy.h>
#include <ns3/spectrum-signal-parameters.h>
#include <ns3/log.h>
#include <ns3/spectrum-value.h>
#include <ns3/antenna-model.h>

#include "wifi-spectrum-phy-custom-interface.h"
#include "spectrum-wifi-phy-custom.h"

NS_LOG_COMPONENT_DEFINE ("WifiSpectrumPhyCustomInterface");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (WifiSpectrumPhyCustomInterface);

TypeId
WifiSpectrumPhyCustomInterface::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::WifiSpectrumPhyCustomInterface")
    .SetParent<SpectrumPhy> ()
    .SetGroupName ("Wifi");
  return tid;
}

WifiSpectrumPhyCustomInterface::WifiSpectrumPhyCustomInterface ()
{
  NS_LOG_FUNCTION (this);
}

void
WifiSpectrumPhyCustomInterface::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_SpectrumWifiPhyCustom = 0;
  m_netDevice = 0;
  m_channel = 0;
}

void WifiSpectrumPhyCustomInterface::SetSpectrumWifiPhyCustom (Ptr<SpectrumWifiPhyCustom> SpectrumWifiPhyCustom)
{
  m_SpectrumWifiPhyCustom = SpectrumWifiPhyCustom;
}

Ptr<NetDevice>
WifiSpectrumPhyCustomInterface::GetDevice () const
{
  return m_netDevice;
}

Ptr<MobilityModel>
WifiSpectrumPhyCustomInterface::GetMobility ()
{
  return m_SpectrumWifiPhyCustom->GetMobility ();
}

void
WifiSpectrumPhyCustomInterface::SetDevice (Ptr<NetDevice> d)
{
  m_netDevice = d;
}

void
WifiSpectrumPhyCustomInterface::SetMobility (Ptr<MobilityModel> m)
{
  m_SpectrumWifiPhyCustom->SetMobility (m);
}

void
WifiSpectrumPhyCustomInterface::SetChannel (Ptr<SpectrumChannel> c)
{
  NS_LOG_FUNCTION (this << c);
  m_channel = c;
}

Ptr<const SpectrumModel>
WifiSpectrumPhyCustomInterface::GetRxSpectrumModel () const
{
  return m_SpectrumWifiPhyCustom->GetRxSpectrumModel ();
}

Ptr<AntennaModel>
WifiSpectrumPhyCustomInterface::GetRxAntenna (void)
{
  NS_LOG_FUNCTION (this);
  return m_SpectrumWifiPhyCustom->GetRxAntenna ();
}

void
WifiSpectrumPhyCustomInterface::StartRx (Ptr<SpectrumSignalParameters> params)
{
  m_SpectrumWifiPhyCustom->StartRx (params);
}


} //namespace ns3
