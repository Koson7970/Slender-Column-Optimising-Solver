using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;



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
  private void RunScript(List<string> results, int colNum, ref object Rects, ref object CenterPoints, ref object StringLine1, ref object StringLine2, ref object StringLine3)
  {
    locPts.Clear();
    widths.Clear();
    thickness.Clear();
    angles.Clear();
    rects.Clear();
    for(int i = 0; i < results.Count; i++)
    {
      //split the string into its components
      string[] parts = results[i].Split(',');
      //parse the parts into the correct data types
      Point3d pt = new Point3d(double.Parse(parts[0]), double.Parse(parts[1]), 0);
      double w = double.Parse(parts[3]);
      double t = double.Parse(parts[4]);
      double a = double.Parse(parts[5]);
      //add the data to the lists
      locPts.Add(pt);
      widths.Add(w);
      thickness.Add(t);
      angles.Add(a);
    }

    //draw the data
    for(int i = 0;i < results.Count;i++)
    {
      //create a plane
      Point3d o = new Point3d((i % colNum) * 50, (i / colNum) * 50 * -1, 0);
      Point3d textCenter = new Point3d(o.X - 10, o.Y - 20, 0);
      centerPts.Add(textCenter);
      Plane plane = new Plane(o, Vector3d.ZAxis);
      //create a rectangle
      Rectangle3d rect = new Rectangle3d(plane, thickness[i], widths[i]);
      Point3d rectCenter = rect.Center;
      //move to O
      rect.Transform(Transform.Translation(new Vector3d(o.X - rectCenter.X, o.Y - rectCenter.Y, 0)));
      //rotate the rectangle
      rect.Transform(Transform.Rotation(angles[i], Vector3d.ZAxis, o));
      rects.Add(rect);
      string line1 = "X: " + locPts[i].X + " Y: " + locPts[i].Y;
      string line2 = "Width: " + widths[i] + " Thickness: " + thickness[i];
      string line3 = "Angle: " + Math.Round(angles[i] / Math.PI, 2) * 180 + "°";
      strLine1.Add(line1);
      strLine2.Add(line2);
      strLine3.Add(line3);
    }

    Rects = rects;
    CenterPoints = centerPts;
    StringLine1 = strLine1;
    StringLine2 = strLine2;
    StringLine3 = strLine3;
  }

  // <Custom additional code> 
  public static List<Point3d> locPts = new List<Point3d>();
  public static List<double> widths = new List<double>();
  public static List<double> thickness = new List<double>();
  public static List<double> angles = new List<double>();
  public static List<Rectangle3d> rects = new List<Rectangle3d>();
  public static List<Point3d> centerPts = new List<Point3d>();
  public static List<string> strLine1 = new List<string>();
  public static List<string> strLine2 = new List<string>();
  public static List<string> strLine3 = new List<string>();
  // </Custom additional code> 
}
