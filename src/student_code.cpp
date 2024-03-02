#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    int n = points.size();
    if (n == 1) {
        return points;
    }

    std::vector<Vector2D> lerps;
    for (int i = 0; i < n - 1; i++) {
        lerps.push_back((1.0 - t) * points[i] + t * points[i + 1]);
    }
    return lerps;
    //return std::vector<Vector2D>();
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    int n = points.size();
    if (n == 1) {
      return points;
    }

    std::vector<Vector3D> lerps;
    for (int i = 0; i < n - 1; i++) {
      lerps.push_back((1.0 - t) * points[i] + t * points[i + 1]);
    }
    return lerps;
    //return std::vector<Vector3D>();
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    if (points.size() == 1) {
      return points[0];
    }
    return evaluate1D(evaluateStep(points, t), t);
    //return Vector3D();
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> points;
    for (int i = 0; i < controlPoints.size(); i++) {
        points.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(points, v);
    //return Vector3D();
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D result;
    HalfedgeCIter h = halfedge();
    do {
      if (!h->face()->isBoundary())
      {
        Vector3D v0 = position;
        Vector3D v1 = h->next()->vertex()->position;
        Vector3D v2 = h->next()->next()->vertex()->position;
        double face_area = (cross(v1 - v0, v2 - v0).norm()) / 2;
        result += face_area * h->face()->normal();
      }
      h = h->twin()->next();
    } while (h != halfedge());
    result = result.unit();
    return result;
    //return Vector3D();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()) return EdgeIter();

    // Inner Edges
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();

    // Outer Edges
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    // Vertices
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    // Edges
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    // Faces
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();


    h0->setNeighbors(h1, h3, v2, e0, f1);
    h1->setNeighbors(h2, h9, v3, e4, f1);
    h2->setNeighbors(h0, h6, v1, e1, f1);
    h3->setNeighbors(h4, h0, v3, e0, f2);
    h4->setNeighbors(h5, h7, v2, e2, f2);
    h5->setNeighbors(h3, h8, v0, e3, f2);

    h6->setNeighbors(h6->next(), h2, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h4, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h5, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h1, v1, e4, h9->face());

    v0->halfedge() = h5;
    v1->halfedge() = h2;
    v2->halfedge() = h4;
    v3->halfedge() = h1;

    e0->halfedge() = h0;
    e1->halfedge() = h2;
    e2->halfedge() = h4;
    e3->halfedge() = h5;
    e4->halfedge() = h1;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) return VertexIter();

    // Inner Edges
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();

    // Outer Edges
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    // Vertices
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    // Edges
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    // Faces
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();


    // New Mesh
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();

    VertexIter v = newVertex();

    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    h0->setNeighbors(h1, h3, v, e0, f0);
    h1->setNeighbors(h2, h6, v1, e1, f0);
    h2->setNeighbors(h0, h11, v2, e5, f0);
    h3->setNeighbors(h4, h0, v1, e0, f1);
    h4->setNeighbors(h5, h15, v, e7, f1);
    h5->setNeighbors(h3, h9, v3, e4, f1);
    h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h12, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h14, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
    h10->setNeighbors(h11, h13, v0, e6, f2);
    h11->setNeighbors(h12, h2, v, e5, f2);
    h12->setNeighbors(h10, h7, v2, e2, f2);
    h13->setNeighbors(h14, h10, v, e6, f3);
    h14->setNeighbors(h15, h8, v0, e3, f3);
    h15->setNeighbors(h13, h4, v3, e7, f3);

    v->position = 0.5 * (v0->position + v1->position);
    v->isNew = 1;        
    v0->halfedge() = h10;
    v1->halfedge() = h1;
    v2->halfedge() = h12;
    v3->halfedge() = h5;
    v->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h12;
    e3->halfedge() = h14;
    e4->halfedge() = h5;
    e5->halfedge() = h2;
    e6->halfedge() = h10;
    e7->halfedge() = h4;

    e0->isNew = 0;
    e5->isNew = 1;
    e6->isNew = 0;
    e7->isNew = 1;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h10;
    f3->halfedge() = h13;

    return v;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd();e++) {
      e->isNew = 0;

      // Inner Edges
      HalfedgeIter h0 = e->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      // Vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      e->newPosition = 3.0 / 8.0 * (v0->position + v1->position) + 1.0 / 8.0 * (v2->position + v3->position);
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = 0;

      HalfedgeIter h = v->halfedge();
      Vector3D vector_sum;
      do {
        vector_sum += h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      float degree = (float)v->degree();
      float a = (degree == 3.0) ? (3.0 / 16.0) : (3.0 / (8.0 * degree));

      v->newPosition = (1.0 - a * degree) * v->position + a * vector_sum;
    }
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v0 = e->halfedge()->vertex();
      VertexIter v1 = e->halfedge()->twin()->vertex();
      if (!(v0->isNew || v1->isNew)) {
        VertexIter v = mesh.splitEdge(e);
        v->newPosition = e->newPosition;
      }
    }
    
    // 4. Flip any new edge that connects an old and new vertex.

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v0 = e->halfedge()->vertex();
      VertexIter v1 = e->halfedge()->twin()->vertex();
      if (e->isNew && (v0->isNew + v1->isNew == 1)) {
        mesh.flipEdge(e);
      }
    }

    // 5. Copy the new vertex positions into final Vertex::position.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
      v->isNew = 0;
    }    

  }
}
