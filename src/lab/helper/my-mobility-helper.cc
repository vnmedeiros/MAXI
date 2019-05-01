/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "ns3/my-mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/position-allocator.h"
#include "ns3/hierarchical-mobility-model.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/config.h"
#include "ns3/simulator.h"
#include "ns3/names.h"
#include "ns3/string.h"
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MyMobilityHelper");

MyMobilityHelper::MyMobilityHelper ()
{
  m_position = CreateObjectWithAttributes<RandomRectanglePositionAllocator> 
      ("X", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
      "Y", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  m_mobility.SetTypeId ("ns3::ConstantPositionMobilityModel");
  
  m_nNodes = 0;
  m_minDist = 100.0;
  m_maxDist = 150.0;
  m_currentNode = new int[1];
  m_currentNode[0] = 0;
}
MyMobilityHelper::~MyMobilityHelper ()
{
}
void
MyMobilityHelper::SetPositionAllocator (Ptr<PositionAllocator> allocator)
{
  m_position = allocator;
}

void
MyMobilityHelper::SetPositionAllocator (std::string type,
                                      std::string n1, const AttributeValue &v1,
                                      std::string n2, const AttributeValue &v2,
                                      std::string n3, const AttributeValue &v3,
                                      std::string n4, const AttributeValue &v4,
                                      std::string n5, const AttributeValue &v5,
                                      std::string n6, const AttributeValue &v6,
                                      std::string n7, const AttributeValue &v7,
                                      std::string n8, const AttributeValue &v8,
                                      std::string n9, const AttributeValue &v9)
{
  ObjectFactory pos;
  pos.SetTypeId (type);
  pos.Set (n1, v1);
  pos.Set (n2, v2);
  pos.Set (n3, v3);
  pos.Set (n4, v4);
  pos.Set (n5, v5);
  pos.Set (n6, v6);
  pos.Set (n7, v7);
  pos.Set (n8, v8);
  pos.Set (n9, v9);
  m_position = pos.Create ()->GetObject<PositionAllocator> ();
}

void 
MyMobilityHelper::SetMobilityModel (std::string type,
                                  std::string n1, const AttributeValue &v1,
                                  std::string n2, const AttributeValue &v2,
                                  std::string n3, const AttributeValue &v3,
                                  std::string n4, const AttributeValue &v4,
                                  std::string n5, const AttributeValue &v5,
                                  std::string n6, const AttributeValue &v6,
                                  std::string n7, const AttributeValue &v7,
                                  std::string n8, const AttributeValue &v8,
                                  std::string n9, const AttributeValue &v9)
{
  m_mobility.SetTypeId (type);
  m_mobility.Set (n1, v1);
  m_mobility.Set (n2, v2);
  m_mobility.Set (n3, v3);
  m_mobility.Set (n4, v4);
  m_mobility.Set (n5, v5);
  m_mobility.Set (n6, v6);
  m_mobility.Set (n7, v7);
  m_mobility.Set (n8, v8);
  m_mobility.Set (n9, v9);
}

void 
MyMobilityHelper::PushReferenceMobilityModel (Ptr<Object> reference)
{
  Ptr<MobilityModel> mobility = reference->GetObject<MobilityModel> ();
  m_mobilityStack.push_back (mobility);
}

void 
MyMobilityHelper::PushReferenceMobilityModel (std::string referenceName)
{
  Ptr<MobilityModel> mobility = Names::Find<MobilityModel> (referenceName);
  m_mobilityStack.push_back (mobility);
}

void 
MyMobilityHelper::PopReferenceMobilityModel (void)
{
  m_mobilityStack.pop_back ();
}


std::string 
MyMobilityHelper::GetMobilityModelType (void) const
{
  return m_mobility.GetTypeId ().GetName ();
}

/*
 * Method used to verify if an allocated position to a node is valid or not.
 */
bool IsPositionValid (Vector position, int *m_currentNode, int m_minDist, 
    int m_maxDist, Vector *m_allocatedPositions, int *m_matrix)
{
  bool result = false;
  
  if (m_currentNode[0] == 0)
    {
      result = true;
    }
  else
    {
      double euclidianDistance = 0;
      int count = 0;
      bool flag = false;
      for (int i = 0; i < m_currentNode[0]; i++)
        {
          euclidianDistance = sqrt(
              (position.x - m_allocatedPositions[i].x) * 
              (position.x - m_allocatedPositions[i].x) + 
              (position.y - m_allocatedPositions[i].y) * 
              (position.y - m_allocatedPositions[i].y));
          if (euclidianDistance > m_minDist)
            {
              count++;
            }
          if (euclidianDistance < m_maxDist)
            {
              flag = true;
            }
        }
      
      if (count == m_currentNode[0] && flag == true)
        {
          result = true;
        }
    }
  
  return result;
}

/*
 * Method used to allocate position to a node.
 */
bool
MyMobilityHelper::Install (Ptr<Node> node) const
{
  Ptr<Object> object = node;
  Ptr<MobilityModel> model = object->GetObject<MobilityModel> ();
  if (model == 0)
    {
      model = m_mobility.Create ()->GetObject<MobilityModel> ();
      if (model == 0)
        {
          NS_FATAL_ERROR ("The requested mobility model is not a mobility model: \""<< 
                          m_mobility.GetTypeId ().GetName ()<<"\"");
        }
      if (m_mobilityStack.empty ())
        {
          NS_LOG_DEBUG ("node="<<object<<", mob="<<model);
          object->AggregateObject (model);
        }
      else
        {
          // we need to setup a hierarchical mobility model
          Ptr<MobilityModel> parent = m_mobilityStack.back ();
          Ptr<MobilityModel> hierarchical = 
            CreateObjectWithAttributes<HierarchicalMobilityModel> ("Child", PointerValue (model),
                                                                   "Parent", PointerValue (parent));
          object->AggregateObject (hierarchical);
          NS_LOG_DEBUG ("node="<<object<<", mob="<<hierarchical);
        }
    }
  
  Vector position;
  int count = 0;
  bool result = true;
  do
    {
      position = m_position->GetNext ();
      
      if(IsPositionValid (position, m_currentNode, m_minDist, m_maxDist, 
          m_allocatedPositions, m_matrix) == true)
        {
		  for (int i = 0; i < m_currentNode[0]; i++)
		    {
              double euclidian_distance = sqrt(
                (position.x - m_allocatedPositions[i].x) * 
                (position.x - m_allocatedPositions[i].x) + 
                (position.y - m_allocatedPositions[i].y) * 
                (position.y - m_allocatedPositions[i].y));
              if (euclidian_distance < m_maxDist)
                {
                  m_matrix[m_currentNode[0] * m_nNodes + i] = 1;
                  m_matrix[i * m_nNodes + m_currentNode[0]] = 1;
                }
            }
			
          model->SetPosition (position);
          m_allocatedPositions[m_currentNode[0]] = position;
          m_currentNode[0]++;
          break;
        }
      else
        {
          count++;
          if (count > 100000)
            {
              m_currentNode[0] = 1;
              for (int i = 0; i < m_nNodes; i++)
                {
                  for (int j = 0; j < m_nNodes; j++)
                    {
                      m_matrix[i * m_nNodes + j] = 0;
                    }
                }
              result = false;
              break;
            }
        }
    }
  while (true);
  
  return result;
}

void
MyMobilityHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  Install (node);
}
/*
 * Method used to allocate positions to all nodes in NodeContainer c.
 */
void 
MyMobilityHelper::Install (NodeContainer c) const
{
  bool success;
  
  do
    {
      success = true;
      for (uint32_t i = 0; i < c.GetN(); i++)
        {
          if( Install (c.Get (i)) == false )
            {
              success = false;
              break;
            }
        }
    }
  while (success == false);
}

void 
MyMobilityHelper::InstallAll (void)
{
  Install (NodeContainer::GetGlobal ());
}
static double
DoRound (double v)
{
  if (v <= 1e-4 && v >= -1e-4)
    {
      return 0.0;
    }
  else if (v <= 1e-3 && v >= 0)
    {
      return 1e-3;
    }
  else if (v >= -1e-3 && v <= 0)
    {
      return -1e-3;
    }
  else
    {
      return v;
    }
}
void
MyMobilityHelper::CourseChanged (Ptr<OutputStreamWrapper> stream, Ptr<const MobilityModel> mobility)
{
  std::ostream* os = stream->GetStream ();
  Ptr<Node> node = mobility->GetObject<Node> ();
  *os << "now=" << Simulator::Now ()
      << " node=" << node->GetId ();
  Vector pos = mobility->GetPosition ();
  pos.x = DoRound (pos.x);
  pos.y = DoRound (pos.y);
  pos.z = DoRound (pos.z);
  Vector vel = mobility->GetVelocity ();
  vel.x = DoRound (vel.x);
  vel.y = DoRound (vel.y);
  vel.z = DoRound (vel.z);
  std::streamsize saved_precision = os->precision ();
  std::ios::fmtflags saved_flags = os->flags ();
  os->precision (3);
  os->setf (std::ios::fixed,std::ios::floatfield);
  *os << " pos=" << pos.x << ":" << pos.y << ":" << pos.z
      << " vel=" << vel.x << ":" << vel.y << ":" << vel.z
      << std::endl;
  os->flags (saved_flags);
  os->precision (saved_precision);
}

void 
MyMobilityHelper::EnableAscii (Ptr<OutputStreamWrapper> stream, uint32_t nodeid)
{
  std::ostringstream oss;
  oss << "/NodeList/" << nodeid << "/$ns3::MobilityModel/CourseChange";
  Config::ConnectWithoutContext (oss.str (), 
                                 MakeBoundCallback (&MyMobilityHelper::CourseChanged, stream));
}
void 
MyMobilityHelper::EnableAscii (Ptr<OutputStreamWrapper> stream, NodeContainer n)
{
  for (NodeContainer::Iterator i = n.Begin (); i != n.End (); ++i)
    {
      EnableAscii (stream, (*i)->GetId ());
    }
}
void 
MyMobilityHelper::EnableAsciiAll (Ptr<OutputStreamWrapper> stream)
{
  EnableAscii (stream, NodeContainer::GetGlobal ());
}
int64_t
MyMobilityHelper::AssignStreams (NodeContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<Node> node;
  Ptr<MobilityModel> mobility;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      node = (*i);
      mobility = node->GetObject<MobilityModel> ();
      if (mobility)
        {
          currentStream += mobility->AssignStreams (currentStream);
        }
    }
  return (currentStream - stream);
}

/*
 * Set the number of nodes.
 * Create the allocated positions vector with the number of nodes.
 * Create the connectivity matrix with the number of nodes.
 */
void
MyMobilityHelper::SetNNodes (int nNodes)
{
  m_nNodes = nNodes;
  
  m_allocatedPositions = new Vector[m_nNodes];
  
  m_matrix = new int[m_nNodes * m_nNodes];
  
  for(int i = 0; i < m_nNodes * m_nNodes; i++)
    {
      m_matrix[i] = 0;
    }
}

/*
 * Set the minimum distance
 */
void
MyMobilityHelper::SetMinDist (double minDist)
{
  m_minDist = minDist;
}

/*
 * Set the maximum distance
 */
void
MyMobilityHelper::SetMaxDist (double maxDist)
{
  m_maxDist = maxDist;
}

/*
 * Get the allocated positions from all nodes
 */
Vector *
MyMobilityHelper::GetAllocatedPositions ()
{
  return m_allocatedPositions;  
}

/*
 * Get the connectivity matrix
 */
int *
MyMobilityHelper::GetMatrix()
{
  return m_matrix;
}
} // namespace ns3
