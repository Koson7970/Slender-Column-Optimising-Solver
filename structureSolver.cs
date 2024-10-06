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
using System.IO;
using System.Linq;


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
  private void RunScript(List<Line> lns, double edgeRange, List<Line> borders, double maxDist, List<Point3d> corners, double siteArea, bool reset, bool calculate, string savePathCols, bool save, ref object GeometryCenter, ref object RigidCenter, ref object EccentricityRate, ref object CompressedColRatio, ref object CompressedCols, ref object TensionColRatio, ref object TensionCols, ref object DistributedLoad, ref object IntersectedBeams, ref object bestSample, ref object sampleNums, ref object status)
  {
    if (reset)
    {
      columns.Clear();
      testPts.Clear();
      renderCols.Clear();
      compressedCols.Clear();
      tensionCols.Clear();
      compressedPts.Clear();
      polys.Clear();
      cr.Clear();
      tr.Clear();
      testAreas.Clear();
      testBp.Clear();
      renderCompressedCols.Clear();
      renderTensionCols.Clear();
      intersectedLns.Clear();
      testOccup.Clear();
      colPts.Clear();
      eccentricity = 0;
      tempEccentricity = 0;
      possibleColumnsNum = 0;
      genoList.Clear();

      reset = false;
      redo = false;

      //generate grid
      GenerateGrids(edgeRange);
      CheckAllowabilityGrids(lns, borders, maxDist);
      for (int i = 0; i < grids.GetLength(0); i++)
      {
        for (int j = 0; j < grids.GetLength(1); j++)
        {
          if (grids[i, j].allowability == 1)
          {
            possibleColumnsNum++;
          }
        }
      }
    }

    if (calculate || redo)
    {
      redo = false;
      storeInList = false;

      columns.Clear();
      testPts.Clear();
      renderCols.Clear();
      compressedCols.Clear();
      tensionCols.Clear();
      compressedPts.Clear();
      polys.Clear();
      cr.Clear();
      tr.Clear();
      testAreas.Clear();
      testBp.Clear();
      renderCompressedCols.Clear();
      renderTensionCols.Clear();
      resultCols.Clear();
      intersectedLns.Clear();
      testOccup.Clear();
      colPts.Clear();
      tempEccentricity = 0;

      testColumnsNum = rnd.Next((int) (0.3 * possibleColumnsNum), possibleColumnsNum);
      GenerateCols(testColumnsNum, grids, edgeRange, lns, borders, maxDist);

      Node2List cNodes = new Node2List();
      Node2List cOutline = new Node2List();

      GetCompressedVoronoi(cNodes, corners, cOutline);

      //calculate the geometry center
      geoCenter = new Point3d(0, 0, 0);
      for (int i = 0; i < corners.Count; i++)
      {
        geoCenter += corners[i];
      }
      geoCenter /= corners.Count;

      //calculate the rigid center

      for (int i = 0; i < columns.Count; i++)
      {
        rigidCenterXLoc += columns[i].localWidth * Math.Abs(Math.Cos(columns[i].angle)) * columns[i].loc.X;
        rigidCenterXTotalWeight += columns[i].localWidth * Math.Abs(Math.Cos(columns[i].angle));
        rigidCenterYLoc += columns[i].localWidth * Math.Abs(Math.Sin(columns[i].angle)) * columns[i].loc.Y;
        rigidCenterYTotalWeight += columns[i].localWidth * Math.Abs(Math.Sin(columns[i].angle));
      }
      rigidCenter = new Point3d(rigidCenterXLoc / rigidCenterXTotalWeight, rigidCenterYLoc / rigidCenterYTotalWeight, 0);
      //checking eccentricity
      double rgDist = geoCenter.DistanceTo(rigidCenter);
      tempEccentricity = rgDist / edgeRange;
      if (tempEccentricity > 0.05)
      {
        redo = true;
      }

      Node2List nodes = new Node2List();
      Node2List outline = new Node2List();
      GetTensionVoronoi(nodes, corners, outline);

      if (!redo)
      {
        storeInList = true;
      }
    }


    if (storeInList)
    {
      //stored as genotypes
      eccentricity = tempEccentricity;
      double fitness = 0;
      for (int i = 0; i < columns.Count; i++)
      {
        fitness += columns[i].localWidth * columns[i].localThickness * 420;
      }

      Genotype g = new Genotype(columns.Count + 2);
      g.genes[0] = Math.Round(fitness, 2).ToString();
      g.genes[1] = Math.Round(eccentricity, 4).ToString();

      for (int i = 2; i < g.genes.Length; i++)
      {
        g.genes[i] = Math.Round(columns[i - 2].loc.X, 2).ToString() + ","
          + Math.Round(columns[i - 2].loc.Y, 2).ToString() + ","
          + (columns[i - 2].function).ToString() + ","
          + Math.Round(columns[i - 2].localWidth, 2).ToString() + ","
          + Math.Round(columns[i - 2].localThickness, 2).ToString() + ","
          + Math.Round(columns[i - 2].angle, 2).ToString() + ","
          + Math.Round(columns[i - 2].checkCompressedRatio, 2).ToString() + ","
          + Math.Round(columns[i - 2].checkTensionRatio, 2).ToString();
      }

      genoList.Add(g);
      genoList = genoList.OrderBy(o => double.Parse(o.genes[0])).ToList();
      storeInList = false;
    }


    //represent the best solution
    resultCols.Clear();
    for (int i = 2; i < genoList[0].genes.Length; i++)
    {
      string[] col = genoList[0].genes[i].Split(',');
      Point3d pt = new Point3d(double.Parse(col[0]), double.Parse(col[1]), 0);
      double width = double.Parse(col[3]);
      double thickness = double.Parse(col[4]);
      Column column = new Column(pt, width, thickness);
      column.function = int.Parse(col[2]);
      column.angle = double.Parse(col[5]);
      column.checkCompressedRatio = double.Parse(col[6]);
      column.checkTensionRatio = double.Parse(col[7]);
      resultCols.Add(column);
    }
    compressedCols.Clear();
    tensionCols.Clear();
    cr.Clear();
    tr.Clear();
    renderCompressedCols.Clear();
    renderTensionCols.Clear();
    for (int i = 0; i < resultCols.Count; i++)
    {
      if (resultCols[i].function == 0)
      {
        compressedCols.Add(resultCols[i]);
      }
      else
      {
        tensionCols.Add(resultCols[i]);
      }
    }

    //find the intersected lines
    intersectedLns.Clear();
    for (int i = 0; i < resultCols.Count; i++)
    {
      Point3d pt = resultCols[i].loc;
      var x = pt.X;
      var y = pt.Y;
      if (x % moduleSize != 0 && y % moduleSize != 0)
      {
        Point3d pa = new Point3d(Math.Floor(x / moduleSize) * moduleSize, y, 0);
        Point3d pb = new Point3d(Math.Ceiling(x / moduleSize) * moduleSize, y, 0);
        Point3d pc = new Point3d(x, Math.Floor(y / moduleSize) * moduleSize, 0);
        Point3d pd = new Point3d(x, Math.Ceiling(y / moduleSize) * moduleSize, 0);
        Line l1 = new Line(pa, pb);
        Line l2 = new Line(pc, pd);
        intersectedLns.Add(l1);
        intersectedLns.Add(l2);
      }
    }

    for (int i = 0; i < compressedCols.Count; i++)
    {
      renderCompressedCols.Add(compressedCols[i].GetColumn());
    }
    for (int i = 0; i < tensionCols.Count; i++)
    {
      renderTensionCols.Add(tensionCols[i].GetColumn());
    }
    for (int i = 0; i < resultCols.Count; i++)
    {
      if (resultCols[i].function == 1)
      {
        tr.Add(resultCols[i].checkTensionRatio);
      }
      else if (resultCols[i].function == 0)
      {
        cr.Add(resultCols[i].checkCompressedRatio);
      }
    }

    //save the data
    if (save)
    {
      SaveCSV(savePathCols, resultCols);
    }

    //showing status
    if (temp == testColumnsNum)
    {
      status = "Done";
    }
    else if (temp == 0)
    {
      status = "Initializing";
    }
    else
    {
      status = "Calculating";
    }
    temp = testColumnsNum;

    GeometryCenter = geoCenter;
    RigidCenter = rigidCenter;
    EccentricityRate = double.Parse(genoList[0].genes[1]);

    DistributedLoad = polys;
    CompressedColRatio = cr;
    TensionColRatio = tr;

    CompressedCols = renderCompressedCols;
    TensionCols = renderTensionCols;

    IntersectedBeams = intersectedLns;
    bestSample = genoList[0].genes;
    sampleNums = genoList.Count;
  }

  // <Custom additional code> 
  private static Random rnd = new Random();
  public static double moduleSize = 150;
  public static Grid[,] grids;

  public static int possibleColumnsNum;
  public static int testColumnsNum;
  public static int temp = 0;

  public static List<Column> columns = new List<Column>();
  public static List<Column> compressedCols = new List<Column>();
  public static List<Column> tensionCols = new List<Column>();
  public static List<Column> resultCols = new List<Column>();
  public static List<Point3d> compressedPts = new List<Point3d>();
  public static List<Point3d> colPts = new List<Point3d>();
  public static List<Point3d> testPts = new List<Point3d>();
  public static List<Box> renderCols = new List<Box>();
  public static List<Box> renderCompressedCols = new List<Box>();
  public static List<Box> renderTensionCols = new List<Box>();
  public static List<Polyline> polys = new List<Polyline>();
  public static List<Line> intersectedLns = new List<Line>();

  public static List<double> cr = new List<double>();
  public static List<double> tr = new List<double>();
  public static List<double> testAreas = new List<double>();
  public static List<Brep> testBp = new List<Brep>();

  public static Point3d geoCenter;
  public static Point3d rigidCenter;
  public static double rigidCenterXLoc;
  public static double rigidCenterXTotalWeight;
  public static double rigidCenterYLoc;
  public static double rigidCenterYTotalWeight;
  public static double eccentricity;
  public static double tempEccentricity;

  public static List<Genotype> genoList = new List<Genotype>();

  public static List<int> testOccup = new List<int>();

  public static bool redo = false;
  public static bool storeInList = false;

  public static int k;


  public void GenerateGrids(double edgeRange)
  {
    grids = new Grid[(int) (edgeRange / moduleSize), (int) (edgeRange / moduleSize)];
    for (int i = 0; i < (edgeRange / moduleSize); i++)
    {
      for (int j = 0; j < (edgeRange / moduleSize); j++)
      {
        grids[i, j] = new Grid(0);
        grids[i, j].centerPt = new Point3d((i + 0.5) * moduleSize, (j + 0.5) * moduleSize, 0);
        grids[i, j].onEdge = 0;
      }
    }
  }
  public void CheckAllowabilityGrids(List<Line> lns, List<Line> borders, double maxDist)
  {
    for (int i = 0; i < grids.GetLength(0); i++)
    {
      for (int j = 0; j < grids.GetLength(1); j++)
      {
        Point3d pt = grids[i, j].centerPt;
        //distance to the closest line
        List<double> dists = new List<double>();
        for (int k = 0; k < lns.Count; k++)
        {
          dists.Add(pt.DistanceTo(lns[k].ClosestPoint(pt, true)));
        }
        dists.Sort();
        //distance to the borders
        List<double> dists2 = new List<double>();
        for (int k = 0; k < borders.Count; k++)
        {
          dists2.Add(pt.DistanceTo(borders[k].ClosestPoint(pt, true)));
        }
        dists2.Sort();
        if (dists[0] < maxDist && dists2[0] > 0.5 * maxDist)
        {
          grids[i, j].allowability = 1;
        }
        else
        {
          grids[i, j].allowability = 0;
        }
      }
    }
  }

  public void GenerateCols(int testColumnsNum, Grid[,] grids, double edgeRange, List<Line> lns, List<Line> borders, double maxDist)
  {
    columns.Clear();

    int allowGridsNum = 0;
    for (int i = 0; i < grids.GetLength(0); i++)
    {
      for (int j = 0; j < grids.GetLength(1); j++)
      {
        if (grids[i, j].allowability == 1)
        {
          allowGridsNum++;
        }
      }
    }

    List<int> occupiedGrids = new List<int>();
    for (int i = 0; i < testColumnsNum; i++)
    {
      occupiedGrids.Add(1);
    }
    for (int i = 0; i < allowGridsNum - testColumnsNum; i++)
    {
      occupiedGrids.Add(0);
    }
    var shuffle_occupiedGrids = occupiedGrids.OrderBy(o => rnd.Next()).ToList();
    testOccup = shuffle_occupiedGrids;

    var a = 0;

    //asssign the occupied grids
    for (int i = 0; i < grids.GetLength(0); i++)
    {
      for (int j = 0; j < grids.GetLength(1); j++)
      {
        if (grids[i, j].allowability == 1)
        {
          grids[i, j].occupied = shuffle_occupiedGrids[a];
          a++;
        }
        else
        {
          grids[i, j].occupied = 0;
        }
      }
    }

    //create a list of points
    for (int i = 0; i < grids.GetLength(0); i++)
    {
      for (int j = 0; j < grids.GetLength(1); j++)
      {
        if (grids[i, j].occupied == 1)
        {
          var x = rnd.NextDouble();
          var y = rnd.NextDouble();
          if (x < 0.2)
          {
            x = 0;
            grids[i, j].onEdge = 1;
          }
          else if (x > 0.8)
          {
            x = 1;
            grids[i, j].onEdge = 1;
          }
          if (y < 0.2)
          {
            y = 0;
            grids[i, j].onEdge = 1;
          }
          else if (y > 0.8)
          {
            y = 1;
            grids[i, j].onEdge = 1;
          }

          Point3d pt = new Point3d((i + x) * moduleSize, (j + y) * moduleSize, 0);
          grids[i, j].innerPt = pt;
          if (grids[i, j].onEdge == 0)
          {
            Point3d pa = new Point3d(i * moduleSize, pt.Y, 0);
            Point3d pb = new Point3d((i + 1) * moduleSize, pt.Y, 0);
            Point3d pc = new Point3d(pt.X, j * moduleSize, 0);
            Point3d pd = new Point3d(pt.X, (j + 1) * moduleSize, 0);
            Line l1 = new Line(pa, pb);
            Line l2 = new Line(pc, pd);
            grids[i, j].intersectedLine = new Line[2];
            grids[i, j].intersectedLine[0] = l1;
            grids[i, j].intersectedLine[1] = l2;
          }

          var width = rnd.NextDouble() * 11 + 8;
          var thickness = rnd.NextDouble() * 6.4 + 1.6;
          Column col = new Column(pt, width, thickness);
          columns.Add(col);
          colPts.Add(pt);
          if (col.function == 0)
          {
            compressedCols.Add(col);
            compressedPts.Add(pt);
          }
          else
          {
            tensionCols.Add(col);
          }
        }
      }
    }
  }

  public static void GetCompressedVoronoi(Node2List nodes, List<Point3d> corners, Node2List outline)
  {
    foreach (Point3d pt in colPts)
    {
      Node2 n = new Node2(pt.X, pt.Y);
      nodes.Append(n);
    }

    foreach (Point3d pt in corners)
    {
      Node2 n = new Node2(pt.X, pt.Y);
      outline.Append(n);
    }

    //Calculate the delaunay triangulation
    var delaunay = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Connectivity(nodes, 0.1, false);
    //Calculate the voronoi diagram
    var voronoi = Grasshopper.Kernel.Geometry.Voronoi.Solver.Solve_Connectivity(nodes, delaunay, outline);

    for (int i = 0; i < voronoi.Count; i++)
    {
      Polyline pl = voronoi[i].ToPolyline();
      Brep bp = Brep.CreatePlanarBreps(pl.ToNurbsCurve(), 0.01)[0];
      //load combination : 1.2D+1.6L
      //Dead load : 50kg/m2
      //Live load : 60kg/m2
      //1.2*50+1.6*60 = 156
      double ratio = (bp.GetArea() * 0.0001 * 156) / (0.85 * Math.Pow(Math.PI, 2) * 2.04 * 1000000 * columns[i].localWidth * columns[i].localThickness / Math.Pow((0.8 * 420) / (0.288 * columns[i].localThickness), 2));
      if (ratio < 1.0)
      {
        columns[i].checkCompressedRatio = ratio;
        polys.Add(pl);
      }
      else
      {
        redo = true;
        break;
      }
    }
  }

  public static void GetTensionVoronoi(Node2List nodes, List<Point3d> corners, Node2List outline)
  {
    foreach (Point3d pt in colPts)
    {
      Node2 n = new Node2(pt.X, pt.Y);
      nodes.Append(n);
    }

    foreach (Point3d pt in corners)
    {
      Node2 n = new Node2(pt.X, pt.Y);
      outline.Append(n);
    }

    //Calculate the delaunay triangulation
    var delaunay = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Connectivity(nodes, 0.1, false);
    //Calculate the voronoi diagram
    var voronoi = Grasshopper.Kernel.Geometry.Voronoi.Solver.Solve_Connectivity(nodes, delaunay, outline);

    for (int i = 0; i < voronoi.Count; i++)
    {
      Polyline pl = voronoi[i].ToPolyline();
      Brep bp = Brep.CreatePlanarBreps(pl.ToNurbsCurve(), 0.01)[0];

      double ratioX = (bp.GetArea() * 0.0001 * 50 * 0.32) / (0.85 * 0.4 * (columns[i].localWidth * columns[i].localThickness * Math.Abs(Math.Cos(columns[i].angle))) * 2400);
      double ratioY = (bp.GetArea() * 0.0001 * 50 * 0.32) / (0.85 * 0.4 * (columns[i].localWidth * columns[i].localThickness * Math.Abs(Math.Sin(columns[i].angle))) * 2400);
      double ratio = Math.Max(ratioX, ratioY);
      if (ratio < 1.0)
      {
        columns[i].checkTensionRatio = ratio;
      }
      else
      {
        redo = true;
        break;
      }
    }
  }

  public void SaveCSV(string savePath, List<Column> columns)
  {
    string[] outputCols = new string[columns.Count];
    for (int i = 0; i < columns.Count; i++)
    {
      if (columns[i].function == 0)
      {
        outputCols[i] = (i + 1).ToString() + ","
          + columns[i].loc.X.ToString() + ","
          + columns[i].loc.Y.ToString() + ","
          + (i + 1).ToString() + ","
          + ((i + 1) + columns.Count).ToString() + ","
          + columns[i].localWidth.ToString() + ","
          + columns[i].localThickness.ToString() + ","
          + "BEAM" + ","
          + ((columns[i].angle) / Math.PI * 180).ToString();
      }
      else
      {
        outputCols[i] = (i + 1).ToString() + ","
          + columns[i].loc.X.ToString() + ","
          + columns[i].loc.Y.ToString() + ","
          + (i + 1).ToString() + ","
          + ((i + 1) + columns.Count).ToString() + ","
          + columns[i].localWidth.ToString() + ","
          + columns[i].localThickness.ToString() + ","
          + "TENS-TRUSS" + ","
          + ((columns[i].angle) / Math.PI * 180).ToString();
      }
    }
    File.WriteAllLines(savePath, outputCols);
  }

  public class Column
  {
    public Point3d loc;
    public int function; //0 = compression, 1 = tension
    public double localWidth;
    public double localThickness;
    public double angle;
    public double checkCompressedRatio;
    public double checkTensionRatio;

    public Column(Point3d loc, double localWidth, double localThickness)
    {
      this.loc = loc;
      this.localWidth = localWidth;
      this.localThickness = localThickness;
      //kl/r<200
      if ((0.8 * 420) / (0.288 * localThickness) < 200)
      {
        this.function = 0;
      }
      else
      {
        this.function = 1;
      }
      double r = rnd.NextDouble();
      this.angle = Math.PI * r;
    }

    public Box GetColumn()
    {
      Rhino.Geometry.Plane plane = new Rhino.Geometry.Plane(loc, Vector3d.ZAxis);
      Box box = new Box(plane, new Interval(-localWidth / 2, localWidth / 2), new Interval(-localThickness / 2, localThickness / 2), new Interval(0, 420));
      Transform xform = Transform.Rotation(angle, plane.Normal, loc);
      box.Transform(xform);
      return box;
    }
  }

  public class Grid
  {
    public int occupied; //0 = empty, 1 = occupied
    public Point3d innerPt;
    public Point3d centerPt;
    public int allowability; //0 = not allowed, 1 = allowed
    public int onEdge; //0 = not on edge, 1 = on edge
    public Line[] intersectedLine;
    public Grid(int occupied)
    {
      this.occupied = occupied;
    }
  }

  //genotype
  public class Genotype
  {
    public string[] genes;

    public Genotype(int size)
    {
      genes = new string[size];
    }
  }
  // </Custom additional code> 
}
