//* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
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
#include "RandomRectanglePositionAllocatorMinMax.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include <cmath>

NS_LOG_COMPONENT_DEFINE ("RandomRectanglePositionAllocatorMinMax");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (RandomRectanglePositionAllocatorMinMax);

TypeId
RandomRectanglePositionAllocatorMinMax::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RandomRectanglePositionAllocatorMinMax")
    .SetParent<PositionAllocator> ()
    .SetGroupName ("Mobility")
    .AddConstructor<RandomRectanglePositionAllocatorMinMax> ()
    .AddAttribute ("X",
                   "A random variable which represents the x coordinate of a position in a random rectangle.",
                   StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
                   MakePointerAccessor (&RandomRectanglePositionAllocatorMinMax::m_x),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("Y",
                   "A random variable which represents the y coordinate of a position in a random rectangle.",
                   StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
                   MakePointerAccessor (&RandomRectanglePositionAllocatorMinMax::m_y),
                   MakePointerChecker<RandomVariableStream> ());
  return tid;
}

RandomRectanglePositionAllocatorMinMax::RandomRectanglePositionAllocatorMinMax ()
{
  m_inTheMiddle = new bool[1];
  m_inTheMiddle[0] = true;
  m_sideA = 1000;
  m_sideB = 1000;
  m_quadrantDivision = 3;
  m_quadrantSideA = m_sideA / (double)m_quadrantDivision;
  m_quadrantSideB = m_sideB / (double)m_quadrantDivision;
  m_quadrantI = 2;
  m_quadrantJ = 2;
}

RandomRectanglePositionAllocatorMinMax::RandomRectanglePositionAllocatorMinMax (double sizeArea, int quadrantDivision) : RandomRectanglePositionAllocatorMinMax() {
	Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
	x->SetAttribute("Min", DoubleValue (0.0));
	x->SetAttribute("Max", DoubleValue (sizeArea));
	Ptr<UniformRandomVariable> y = CreateObject<UniformRandomVariable>();
	y->SetAttribute("Min", DoubleValue(0.0));
	y->SetAttribute("Max", DoubleValue (sizeArea));
	this->SetX(x);
	this->SetY(y);
	this->SetSideA(sizeArea);
	this->SetSideB(sizeArea);
	this->SetQuadrantDivision(quadrantDivision);
}

RandomRectanglePositionAllocatorMinMax::~RandomRectanglePositionAllocatorMinMax ()
{
}

void
RandomRectanglePositionAllocatorMinMax::SetX (Ptr<RandomVariableStream> x)
{
  m_x = x;
}
void
RandomRectanglePositionAllocatorMinMax::SetY (Ptr<RandomVariableStream> y)
{
  m_y = y;
}
/*
 * Set the len of side A
 */
void
RandomRectanglePositionAllocatorMinMax::SetSideA (double sideA)
{
  m_sideA = sideA;
}
/*
 * Set the len of side B
 */
void
RandomRectanglePositionAllocatorMinMax::SetSideB (double sideB)
{
  m_sideB = sideB;
}
/*
 * Set the in the middle value
 */
void
RandomRectanglePositionAllocatorMinMax::SetInTheMiddle (bool inTheMiddle)
{
  m_inTheMiddle[0] = inTheMiddle;
}
/*
 * Set the quadrant division
 */
void RandomRectanglePositionAllocatorMinMax::SetQuadrantDivision (int quadrantDivision)
{
  m_quadrantDivision = quadrantDivision;
  m_quadrantSideA = m_sideA / (double)m_quadrantDivision;
  m_quadrantSideB = m_sideB / (double)m_quadrantDivision;
}
/*
 * Set the quadrant positions
 */
void RandomRectanglePositionAllocatorMinMax::SetQuadrants (int quadrantI, int quadrantJ)
{
  m_quadrantI = quadrantI;
  m_quadrantJ = quadrantJ;
}

/*
 * Get the next position.
 * The first position is the middle, the others are random.
 */
Vector
RandomRectanglePositionAllocatorMinMax::GetNext (void) const
{
  double x, y, deltaX, deltaY;

  deltaX = m_quadrantSideA * -1;
  for (int i = 1; i <= m_quadrantI; i++)
    {
	  deltaX += m_quadrantSideA;
    }
  deltaY = m_quadrantSideB * -1;
  for (int j = 1; j <= m_quadrantJ; j++)
    {
      deltaY += m_quadrantSideB;
    }

  if (m_inTheMiddle[0] == true)
    {
      //x = m_sideA / 2.0;
      //y = m_sideB / 2.0;
      x = m_quadrantSideA / 2.0 + deltaX;
      y = m_quadrantSideB / 2.0 + deltaY;
      m_inTheMiddle[0] = false;
    }
  else
    {
      x = m_x->GetValue ();
      y = m_y->GetValue ();
    }

  return Vector (x, y, 0.0);
}

int64_t
RandomRectanglePositionAllocatorMinMax::AssignStreams (int64_t stream)
{
  m_x->SetStream (stream);
  m_y->SetStream (stream + 1);
  return 2;
}

bool RandomRectanglePositionAllocatorMinMax::verifyAllocation(MyMobilityHelper mobility, uint32_t nNodes, double minimumDistance, double maximumDistance) {
	Vector *positions = mobility.GetAllocatedPositions();
	bool valid, result;
	int count;
	double euclidian_distance;

	result = true;
	for (uint32_t i = 0; i < nNodes; i++) {
		valid = true;
		count = 0;
		for (uint32_t j = 0; j < nNodes; j++) {
			euclidian_distance = sqrt(
					(positions[i].x - positions[j].x) * (positions[i].x - positions[j].x) +
					(positions[i].y - positions[j].y) * (positions[i].y - positions[j].y));
			if (i != j) {
				if (euclidian_distance < maximumDistance)
					count++;
				if (euclidian_distance < minimumDistance)
					valid = false;
			}
		}

		if (!(count >= 1 && valid == true)) {
			result = false;
		}
	}
	return result;
}

} // namespace ns3
