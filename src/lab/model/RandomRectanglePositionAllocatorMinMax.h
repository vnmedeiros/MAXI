/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
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

#ifndef POSITION_ALLOCATOR_RANDOM_RECTANGLE_H
#define POSITION_ALLOCATOR_RANDOM_RECTANGLE_H

#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"
#include "ns3/position-allocator.h"
#include "ns3/lab-module.h"

namespace ns3 {

	/**
	 * RandomRectanglePositionAllocatorMinMax
	 * \ingroup mobility
	 * \brief Allocate random positions within a rectangle according to a pair of random variables.
	 */
	class RandomRectanglePositionAllocatorMinMax : public PositionAllocator {
	public:
		static TypeId GetTypeId (void);
		RandomRectanglePositionAllocatorMinMax ();
		RandomRectanglePositionAllocatorMinMax (double sizeArea, int quadrantDivision);
		virtual ~RandomRectanglePositionAllocatorMinMax ();

		void SetX (Ptr<RandomVariableStream> x);
		void SetY (Ptr<RandomVariableStream> y);
		void SetSideA (double sideA); //Set the len of side A
		void SetSideB (double sideB);//Set the len of side B
		void SetInTheMiddle (bool inTheMiddle);//Set in the middle value
		void SetQuadrantDivision (int quadrantDivision);//Set the quadrant division
		void SetQuadrants (int quadrantI, int quadrantJ);//Set the quadrant positions
		bool verifyAllocation (MyMobilityHelper mobility, uint32_t nNodes, double minimumDistance, double maximumDistance); //Verify position nodes

		virtual Vector GetNext (void) const;
		virtual int64_t AssignStreams (int64_t stream);
	private:
		Ptr<RandomVariableStream> m_x;
		Ptr<RandomVariableStream> m_y;
		double m_sideA;//Len of side A
		double m_sideB;//Len of side B
		bool *m_inTheMiddle;//A node can be set in the middle or not
		int m_quadrantDivision;//Quadrand division
		double m_quadrantSideA;//Len of quadrant side A
		double m_quadrantSideB;//Len of quadrant side B
		int m_quadrantI;//Quadrant position i
		int m_quadrantJ;//Quadrant position j
	};

} // namespace ns3

#endif /* POSITION_ALLOCATOR_RANDOM_RECTANGLE_H */
