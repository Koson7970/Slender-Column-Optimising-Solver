using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using Grasshopper.Kernel.Geometry;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  private void RunScript(Curve boundary, int count, bool generate, ref object Cells, ref object edgeLength, ref object corners, ref object EccentricityRate)
  {
    BoundingBox bx = boundary.GetBoundingBox(true);
    Line[] edges = bx.GetEdges();
    Point3d[] bxCorners = bx.GetCorners();
    List<Point3d> cornerPts = new List<Point3d>();
    foreach(Point3d p in bxCorners)
    {
      cornerPts.Add(p);
    }

    Node2List nodes = new Node2List();
    Node2List outline = new Node2List();

    //vornoi algorithm
    if (generate || redo)
    {
      cellPts.Clear();
      lengthEdges.Clear();
      polys.Clear();
      generate = false;
      redo = false;

      for(int i = 0; i < edges.Length; i++)
      {
        if (edges[i] != null)
        {
          lengthEdges.Add(edges[i].Length);
        }
      }
      lengthEdges.Sort();
      lengthEdges.Reverse();

      GetVoronoi(count, boundary, nodes, bxCorners, outline);

    }
    geoCenter = new Point3d(0, 0, 0);
    foreach (Point3d p in cornerPts)
    {
      geoCenter += p;
    }
    geoCenter /= cornerPts.Count;
    averageCenter = new Point3d(0, 0, 0);
    foreach (Point3d p in cellPts)
    {
      averageCenter += p;
    }
    averageCenter /= cellPts.Count;
    eccentricity = geoCenter.DistanceTo(averageCenter) / lengthEdges[0];
    if(eccentricity > 0.05)
    {
      redo = true;
    }

    Cells = polys;
    edgeLength = lengthEdges[0];
    corners = cornerPts;
    EccentricityRate = eccentricity;
  }

  // <Custom additional code> 
  public static List<Point3d> cellPts = new List<Point3d>();
  public static Random rdn = new Random();

  public static List<double> lengthEdges = new List<double>();
  public static List<Polyline> polys = new List<Polyline>();


  public static Point3d geoCenter;
  public static Point3d averageCenter;
  public static double eccentricity;
  public static bool redo = false;

  public static void GetVoronoi(int count, Curve boundary, Node2List nodes, Point3d[] bxCorners, Node2List outline)
  {
    cellPts.Clear();
    polys.Clear();
    //create random points
    for (int i = 0; i < count; i++)
    {
      double x = rdn.NextDouble();
      double y = rdn.NextDouble();
      Point3d pt = new Point3d(x * lengthEdges[0], y * lengthEdges[0], 0);
      //check if point is inside the boundary
      if (boundary.Contains(pt, Rhino.Geometry.Plane.WorldXY, 0.001) == PointContainment.Inside)
      {
        cellPts.Add(pt);
      }
      else
      {
        i--;
      }
    }

    //referenced from >>https://www.grasshopper3d.com/forum/topics/feature-request-access-to-grasshopper-scripts-in-python-c?commentId=2985220%3AComment%3A678528
    //Script from Anders Holden Deleuran
    //Tranformed in C# by Laurent Delrieu
    foreach (Point3d p in cellPts)
    {
      Node2 n = new Node2(p.X, p.Y);
      nodes.Append(n);
    }

    foreach (Point3d p in bxCorners)
    {
      Node2 n = new Node2(p.X, p.Y);
      outline.Append(n);
    }
    //Calculate the delaunay triangulation
    var delaunay = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Connectivity(nodes, 0.1, false);

    //Calculate the voronoi diagram
    var voronoi = Grasshopper.Kernel.Geometry.Voronoi.Solver.Solve_Connectivity(nodes, delaunay, outline);

    foreach (var c in voronoi)
    {
      Polyline pl = c.ToPolyline();
      Brep bp = Brep.CreatePlanarBreps(pl.ToNurbsCurve(), 0.01)[0];
      if (bp.GetArea() > lengthEdges[0] * lengthEdges[1] / count * 0.5 && bp.GetArea() < lengthEdges[0] * lengthEdges[1] / count * 1.5)
      {
        polys.Add(pl);
      }
      else
      {
        redo = true;
        break;
      }
    }
  }
  // </Custom additional code> 
}
