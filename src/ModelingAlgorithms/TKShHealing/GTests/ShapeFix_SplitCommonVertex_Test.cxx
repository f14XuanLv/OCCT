// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Plane.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_Sequence.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <ShapeExtend_WireData.hxx>
#include <ShapeFix_SplitCommonVertex.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include <gtest/gtest.h>

#include <initializer_list>

namespace
{
//=================================================================================================

TopoDS_Vertex makeVertex(const double theX, const double theY)
{
  TopoDS_Vertex aVertex;
  BRep_Builder  aBuilder;
  aBuilder.MakeVertex(aVertex, gp_Pnt(theX, theY, 0.0), Precision::Confusion());
  return aVertex;
}

//=================================================================================================
// Builds a closed polygonal wire through the given vertices (the last one is joined to the first).

TopoDS_Wire makeClosedPolygon(std::initializer_list<TopoDS_Vertex> theVertices)
{
  NCollection_Sequence<TopoDS_Vertex> aVertices;
  for (const TopoDS_Vertex& aVertex : theVertices)
  {
    aVertices.Append(aVertex);
  }
  BRepBuilderAPI_MakeWire aMakeWire;
  for (int anIndex = 1; anIndex <= aVertices.Length(); ++anIndex)
  {
    const TopoDS_Vertex& aNext = aVertices.Value(anIndex % aVertices.Length() + 1);
    aMakeWire.Add(BRepBuilderAPI_MakeEdge(aVertices.Value(anIndex), aNext).Edge());
  }
  return aMakeWire.Wire();
}

//=================================================================================================
// Builds a face on the XY plane bounded by the given wires, keeping their order.

TopoDS_Face makePlanarFace(std::initializer_list<TopoDS_Wire> theWires)
{
  BRep_Builder aBuilder;
  TopoDS_Face  aFace;
  aBuilder.MakeFace(aFace, new Geom_Plane(gp_Pln()), Precision::Confusion());
  for (const TopoDS_Wire& aWire : theWires)
  {
    aBuilder.Add(aFace, aWire);
  }
  return aFace;
}

//=================================================================================================

TopoDS_Face splitCommonVertices(const TopoDS_Face& theFace)
{
  ShapeFix_SplitCommonVertex aFixer;
  aFixer.Init(theFace);
  aFixer.Perform();
  return TopoDS::Face(aFixer.Shape());
}

//=================================================================================================

NCollection_Sequence<TopoDS_Wire> faceWires(const TopoDS_Face& theFace)
{
  NCollection_Sequence<TopoDS_Wire> aWires;
  for (TopoDS_Iterator anIter(theFace, false); anIter.More(); anIter.Next())
  {
    if (anIter.Value().ShapeType() == TopAbs_WIRE)
    {
      aWires.Append(TopoDS::Wire(anIter.Value()));
    }
  }
  return aWires;
}

//=================================================================================================
// Checks that in each wire of the face every edge ends at the vertex where the next one starts.

void expectClosedWires(const TopoDS_Face& theFace)
{
  const ShapeAnalysis_Edge                anAnalyzer;
  const NCollection_Sequence<TopoDS_Wire> aWires = faceWires(theFace);
  for (int aWireIndex = 1; aWireIndex <= aWires.Length(); ++aWireIndex)
  {
    const occ::handle<ShapeExtend_WireData> aWireData =
      new ShapeExtend_WireData(aWires.Value(aWireIndex));
    const int aNbEdges = aWireData->NbEdges();
    for (int anEdgeIndex = 1; anEdgeIndex <= aNbEdges; ++anEdgeIndex)
    {
      const TopoDS_Vertex aLast = anAnalyzer.LastVertex(aWireData->Edge(anEdgeIndex));
      const TopoDS_Vertex aNextFirst =
        anAnalyzer.FirstVertex(aWireData->Edge(anEdgeIndex % aNbEdges + 1));
      EXPECT_TRUE(aLast.IsSame(aNextFirst))
        << "wire " << aWireIndex << " is open after edge " << anEdgeIndex;
    }
  }
}

//=================================================================================================
// Checks that no vertex of the face belongs to more than one of its wires.

void expectNoVertexSharedBetweenWires(const TopoDS_Face& theFace)
{
  const NCollection_Sequence<TopoDS_Wire> aWires = faceWires(theFace);
  for (int aWireIndex1 = 1; aWireIndex1 < aWires.Length(); ++aWireIndex1)
  {
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aVertices1;
    TopExp::MapShapes(aWires.Value(aWireIndex1), TopAbs_VERTEX, aVertices1);
    for (int aWireIndex2 = aWireIndex1 + 1; aWireIndex2 <= aWires.Length(); ++aWireIndex2)
    {
      NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aVertices2;
      TopExp::MapShapes(aWires.Value(aWireIndex2), TopAbs_VERTEX, aVertices2);
      for (int aVertexIndex = 1; aVertexIndex <= aVertices2.Extent(); ++aVertexIndex)
      {
        EXPECT_FALSE(aVertices1.Contains(aVertices2.FindKey(aVertexIndex)))
          << "wires " << aWireIndex1 << " and " << aWireIndex2 << " share a vertex";
      }
    }
  }
}
} // namespace

//=================================================================================================

TEST(ShapeFix_SplitCommonVertexTest, Perform_NoCommonVertex_ReturnsSameFace)
{
  const TopoDS_Face aFace = makePlanarFace({makeClosedPolygon({makeVertex(0.0, 0.0),
                                                               makeVertex(100.0, 0.0),
                                                               makeVertex(100.0, 100.0),
                                                               makeVertex(0.0, 100.0)}),
                                            makeClosedPolygon({makeVertex(20.0, 20.0),
                                                               makeVertex(20.0, 40.0),
                                                               makeVertex(40.0, 40.0),
                                                               makeVertex(40.0, 20.0)})});

  // nothing to split: no replacement must be recorded and the input face is returned as is
  const TopoDS_Face aResult = splitCommonVertices(aFace);
  EXPECT_TRUE(aResult.IsSame(aFace));
}

//=================================================================================================

TEST(ShapeFix_SplitCommonVertexTest, Perform_TwoWiresShareVertex_SplitsVertex)
{
  // basic case: the inner wire touches the outer one at a single corner
  const TopoDS_Vertex aCorner = makeVertex(0.0, 0.0);
  const TopoDS_Face   aFace   = makePlanarFace(
    {makeClosedPolygon(
       {aCorner, makeVertex(100.0, 0.0), makeVertex(100.0, 100.0), makeVertex(0.0, 100.0)}),
     makeClosedPolygon(
       {aCorner, makeVertex(15.0, 35.0), makeVertex(40.0, 40.0), makeVertex(35.0, 15.0)})});

  const TopoDS_Face aResult = splitCommonVertices(aFace);
  expectClosedWires(aResult);
  expectNoVertexSharedBetweenWires(aResult);
}

//=================================================================================================

TEST(ShapeFix_SplitCommonVertexTest, Perform_EdgeEndsSharedWithTwoWires_KeepsWireClosed)
{
  // The outer wire comes last and the two ends of its bottom edge are shared with two
  // different inner wires, so this edge is modified while processing each of them.
  // The second modification must keep the vertex replaced by the first one, otherwise
  // the outer wire becomes open at the bottom-left corner.
  const TopoDS_Vertex aBottomLeft  = makeVertex(0.0, 0.0);
  const TopoDS_Vertex aBottomRight = makeVertex(100.0, 0.0);
  const TopoDS_Face   aFace        = makePlanarFace(
    {makeClosedPolygon({aBottomLeft, makeVertex(15.0, 35.0), makeVertex(40.0, 40.0)}),
     makeClosedPolygon({aBottomRight, makeVertex(65.0, 15.0), makeVertex(60.0, 40.0)}),
     makeClosedPolygon(
       {aBottomLeft, aBottomRight, makeVertex(100.0, 100.0), makeVertex(0.0, 100.0)})});

  const TopoDS_Face aResult = splitCommonVertices(aFace);
  expectClosedWires(aResult);
  expectNoVertexSharedBetweenWires(aResult);
}

//=================================================================================================

TEST(ShapeFix_SplitCommonVertexTest, Perform_EdgeEndsSharedWithOneWire_KeepsWireClosed)
{
  // both ends of the bottom edge of the outer wire are shared with the same triangle,
  // so this edge is modified twice while processing a single pair of wires
  const TopoDS_Vertex aBottomLeft  = makeVertex(0.0, 0.0);
  const TopoDS_Vertex aBottomRight = makeVertex(100.0, 0.0);
  const TopoDS_Face   aFace        = makePlanarFace(
    {makeClosedPolygon({aBottomLeft, makeVertex(50.0, 30.0), aBottomRight}),
     makeClosedPolygon(
       {aBottomLeft, aBottomRight, makeVertex(100.0, 100.0), makeVertex(0.0, 100.0)})});

  const TopoDS_Face aResult = splitCommonVertices(aFace);
  expectClosedWires(aResult);
  expectNoVertexSharedBetweenWires(aResult);
}

//=================================================================================================

TEST(ShapeFix_SplitCommonVertexTest, Perform_VertexSharedByThreeWires_SplitsVertexForEachWire)
{
  // three inner triangles meet at the face center; each of them must get its own vertex
  // there instead of two of them sharing the same new one
  const TopoDS_Vertex aCenter = makeVertex(50.0, 50.0);
  const TopoDS_Face   aFace =
    makePlanarFace({makeClosedPolygon({makeVertex(0.0, 0.0),
                                       makeVertex(100.0, 0.0),
                                       makeVertex(100.0, 100.0),
                                       makeVertex(0.0, 100.0)}),
                    makeClosedPolygon({aCenter, makeVertex(30.0, 20.0), makeVertex(20.0, 30.0)}),
                    makeClosedPolygon({aCenter, makeVertex(80.0, 70.0), makeVertex(70.0, 80.0)}),
                    makeClosedPolygon({aCenter, makeVertex(30.0, 80.0), makeVertex(20.0, 70.0)})});

  const TopoDS_Face aResult = splitCommonVertices(aFace);
  expectClosedWires(aResult);
  expectNoVertexSharedBetweenWires(aResult);
}

//=================================================================================================

TEST(ShapeFix_SplitCommonVertexTest, Perform_ClosedEdgeTouchesOtherWire_KeepsEdgeClosed)
{
  // A circular hole made of a single closed edge is tangent to the bottom side of the outer
  // wire at the vertex of that edge. Both ends of the circle must be replaced by the same
  // new vertex, so that the edge remains closed.
  const TopoDS_Vertex            aTangencyVertex = makeVertex(50.0, 0.0);
  const TopoDS_Wire              anOuterWire     = makeClosedPolygon({makeVertex(0.0, 0.0),
                                                                      aTangencyVertex,
                                                                      makeVertex(100.0, 0.0),
                                                                      makeVertex(100.0, 100.0),
                                                                      makeVertex(0.0, 100.0)});
  const occ::handle<Geom_Circle> aCircle =
    new Geom_Circle(gp_Ax2(gp_Pnt(50.0, 20.0, 0.0), gp::DZ(), gp_Dir(0.0, -1.0, 0.0)), 20.0);
  const TopoDS_Edge aCircleEdge =
    BRepBuilderAPI_MakeEdge(aCircle, aTangencyVertex, aTangencyVertex, 0.0, 2.0 * M_PI).Edge();
  const TopoDS_Face aFace =
    makePlanarFace({anOuterWire, BRepBuilderAPI_MakeWire(aCircleEdge).Wire()});

  const TopoDS_Face aResult = splitCommonVertices(aFace);
  expectClosedWires(aResult);
  expectNoVertexSharedBetweenWires(aResult);

  // wire order is preserved, so the second wire is the circle
  const NCollection_Sequence<TopoDS_Wire> aWires = faceWires(aResult);
  ASSERT_EQ(aWires.Length(), 2);
  const TopoDS_Edge aResultCircleEdge = TopoDS::Edge(TopoDS_Iterator(aWires.Value(2)).Value());
  TopoDS_Vertex     aFirstVertex, aLastVertex;
  TopExp::Vertices(aResultCircleEdge, aFirstVertex, aLastVertex);
  EXPECT_TRUE(aFirstVertex.IsSame(aLastVertex));
}
