package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MathTools;

/**
 * Box where base frame is in the center.
 */
public class Box3d extends Shape3d<Box3d>
{
   private final Point3D temporaryPoint = new Point3D();

   private final Size3d halfSize = new Size3d();
   private final Size3d size = new Size3d()
   {
      private static final long serialVersionUID = 3115155959997000188L;

      @Override
      public final void setX(double x)
      {
         super.setX(x);
         halfSize.setX(0.5 * x);
      }

      @Override
      public final void setY(double y)
      {
         super.setY(y);
         halfSize.setY(0.5 * y);
      }

      @Override
      public final void setZ(double z)
      {
         super.setZ(z);
         halfSize.setZ(0.5 * z);
      }
   };

   public Box3d()
   {
      this(1.0, 1.0, 1.0);
   }

   public Box3d(Box3d other)
   {
      set(other);
   }

   public Box3d(double lengthX, double widthY, double heightZ)
   {
      setSize(lengthX, widthY, heightZ);
   }

   public Box3d(Point3DReadOnly position, QuaternionReadOnly orientation, double length, double width, double height)
   {
      setPose(position, orientation);
      setSize(length, width, height);
   }

   public Box3d(Pose3D pose, double length, double width, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   public Box3d(RigidBodyTransform pose, double length, double width, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   public Box3d(RigidBodyTransform pose, double[] size)
   {
      this(pose, size[0], size[1], size[2]);
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || size.containsNaN();
   }

   @Override
   public boolean epsilonEquals(Box3d other, double epsilon)
   {
      return super.epsilonEqualsPose(other, epsilon) && size.epsilonEquals(other.size, epsilon);
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      boolean isInside = isInsideOrOnSurfaceShapeFrame(x, y, z, 0.0);

      if (isInside)
      {
         double dx = Math.abs(Math.abs(x) - halfSize.getX());
         double dy = Math.abs(Math.abs(y) - halfSize.getY());
         double dz = Math.abs(Math.abs(z) - halfSize.getZ());

         if (closestPointToPack != null)
         {
            closestPointToPack.set(x, y, z);

            if (dx < dy)
            {
               if (dx < dz)
                  closestPointToPack.setX(Math.copySign(halfSize.getX(), x));
               else
                  closestPointToPack.setZ(Math.copySign(halfSize.getZ(), z));
            }
            else
            {
               if (dy < dz)
                  closestPointToPack.setY(Math.copySign(halfSize.getY(), y));
               else
                  closestPointToPack.setZ(Math.copySign(halfSize.getZ(), z));
            }
         }

         if (normalToPack != null)
         {
            normalToPack.setToZero();

            if (dx < dy)
            {
               if (dx < dz)
                  normalToPack.setX(Math.copySign(1.0, x));
               else
                  normalToPack.setZ(Math.copySign(1.0, z));
            }
            else
            {
               if (dy < dz)
                  normalToPack.setY(Math.copySign(1.0, y));
               else
                  normalToPack.setZ(Math.copySign(1.0, z));
            }
         }

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {

         double xClamped = MathTools.clamp(x, halfSize.getX());
         double yClamped = MathTools.clamp(y, halfSize.getY());
         double zClamped = MathTools.clamp(z, halfSize.getZ());

         double dx = x - xClamped;
         double dy = y - yClamped;
         double dz = z - zClamped;

         double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClamped, yClamped, zClamped);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(1.0 / distance);
         }

         return distance;
      }
   }

   public void getBoundingBox3D(BoundingBox3D boundingBoxToPack)
   {
      boundingBoxToPack.setToNaN();

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
      {
         getVertex(vertexIndex, temporaryPoint);
         boundingBoxToPack.updateToIncludePoint(temporaryPoint);
      }
   }

   public void getCenter(Point3DBasics centerToPack)
   {
      getPosition(centerToPack);
   }

   public double getHeight()
   {
      return size.getHeight();
   }

   public double getLength()
   {
      return size.getLength();
   }

   public double getSizeX()
   {
      return size.getX();
   }

   public double getSizeY()
   {
      return size.getY();
   }

   public double getSizeZ()
   {
      return size.getZ();
   }

   public void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 8)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 7], was: " + vertexIndex);

      vertexToPack.setX((vertexIndex & 1) == 0 ? size.getX() : -size.getX());
      vertexToPack.setY((vertexIndex & 2) == 0 ? size.getY() : -size.getY());
      vertexToPack.setZ((vertexIndex & 4) == 0 ? size.getZ() : -size.getZ());
      vertexToPack.scale(0.5);
      transformToWorld(vertexToPack, vertexToPack);
   }

   public Point3D[] getVertices()
   {
      Point3D[] vertices = new Point3D[8];
      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   public void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new RuntimeException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   public double getWidth()
   {
      return size.getWidth();
   }

   public int intersectionWith(Line3D line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   public int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                               Point3DBasics secondIntersectionToPack)
   {
      double minX = -halfSize.getX();
      double minY = -halfSize.getY();
      double minZ = -halfSize.getZ();
      double maxX = halfSize.getX();
      double maxY = halfSize.getY();
      double maxZ = halfSize.getZ();

      double xLocal = TransformationTools.computeTransformedX(shapePose, true, pointOnLine);
      double yLocal = TransformationTools.computeTransformedY(shapePose, true, pointOnLine);
      double zLocal = TransformationTools.computeTransformedZ(shapePose, true, pointOnLine);

      double dxLocal = TransformationTools.computeTransformedX(shapePose, true, lineDirection);
      double dyLocal = TransformationTools.computeTransformedY(shapePose, true, lineDirection);
      double dzLocal = TransformationTools.computeTransformedZ(shapePose, true, lineDirection);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ, xLocal, yLocal, zLocal,
                                                                                                dxLocal, dyLocal, dzLocal, firstIntersectionToPack,
                                                                                                secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack, firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack, secondIntersectionToPack);
      return numberOfIntersections;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      return Math.abs(x) <= halfSize.getX() + epsilon && Math.abs(y) <= halfSize.getY() + epsilon && Math.abs(z) <= halfSize.getZ() + epsilon;
   }

   public void scale(double scale)
   {
      size.scale(scale);
   }

   @Override
   public void set(Box3d other)
   {
      setPose(other);
      size.set(other.size);
   }

   public void setSize(double lengthX, double widthY, double heightZ)
   {
      size.setLengthWidthHeight(lengthX, widthY, heightZ);
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      size.setToNaN();
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      size.setToZero();
   }

   @Override
   public String toString()
   {
      return "Box 3D: size = " + size + ", pose = " + getPoseString();
   }
}
