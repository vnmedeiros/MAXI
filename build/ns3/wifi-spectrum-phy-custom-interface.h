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
#ifndef WIFI_SPECTRUM_PHY_CUSTOM_INTERFACE_H
#define WIFI_SPECTRUM_PHY_CUSTOM_INTERFACE_H

#include <ns3/ptr.h>
#include <ns3/object.h>
#include <ns3/spectrum-phy.h>

namespace ns3 {

class SpectrumWifiPhyCustom;

/**
 * \ingroup wifi
 *
 * This class is an adaptor between class SpectrumWifiPhyCustom (which inherits
 * from WifiPhy) and class SpectrumChannel (which expects objects derived
 * from class SpectrumPhy to be connected to it).
 *
 * The adaptor is used only in the receive direction; in the transmit
 * direction, the class SpectrumWifiPhyCustom constructs signal parameters
 * and directly accesses the SpectrumChannel
 */
class WifiSpectrumPhyCustomInterface : public SpectrumPhy
{
public:
  static TypeId GetTypeId (void);
  WifiSpectrumPhyCustomInterface ();
  /**
   * Connect SpectrumWifiPhyCustom object
   * \param phy SpectrumWifiPhyCustom object to be connected to this object
   */
  void SetSpectrumWifiPhyCustom (Ptr<SpectrumWifiPhyCustom> phy);

  // Inherited from SpectrumPhy
  virtual Ptr<NetDevice> GetDevice () const;
  virtual void SetDevice (Ptr<NetDevice> d);
  virtual void SetMobility (Ptr<MobilityModel> m);
  virtual Ptr<MobilityModel> GetMobility ();
  virtual void SetChannel (Ptr<SpectrumChannel> c);
  virtual Ptr<const SpectrumModel> GetRxSpectrumModel () const;
  virtual Ptr<AntennaModel> GetRxAntenna ();
  virtual void StartRx (Ptr<SpectrumSignalParameters> params);

private:
  virtual void DoDispose (void);
  Ptr<SpectrumWifiPhyCustom> m_SpectrumWifiPhyCustom;
  Ptr<NetDevice> m_netDevice;
  Ptr<SpectrumChannel> m_channel;
};

} // namespace ns3

#endif  /* WIFI_SPECTRUM_PHY_CUSTOM_INTERFACE_H */
